# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""
Run a bounded, latest-input MPC research process without flight command authority.

Navigation and complete attitude share the physical ROS/system sample epoch.
Only tracking predictions enter the request. All outputs are research JSON,
and asynchronous results undergo live input and whole-cycle admission again.
"""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import time

import rclpy
from px4_msgs.msg import TimesyncStatus, VehicleAttitude
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from uav_usv_interfaces.msg import MissionState, TargetPrediction, UavState

from uav_control.controllers.follow_mpc_seed import (
    FollowMpcSeed, MpcConfig, MpcRequest, PlanningContext,
)
from uav_control.controllers.mpc_shadow_inputs import ShadowInputAdapter
from uav_control.guidance.camera_visibility import body_frd_to_ned_from_quaternion
from uav_control.guidance.follow_profile import profiled


def _seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _gazebo_seconds(stamp):
    return float(stamp.sec) + float(stamp.nsec) * 1e-9


def _xyz(vector):
    return tuple(float(getattr(vector, axis)) for axis in 'xyz')


@dataclass(frozen=True)
class ShadowPrediction:
    """One immutable tracking prediction with original source and image epochs."""

    mission_id: int
    sequence_id: int
    source_stamp: float
    observation_stamp: float
    generated_stamp: float
    valid_until: float
    frame_id: str
    source: str
    model: str
    prediction_horizon: float
    prediction_times: tuple
    target_positions: tuple
    target_velocities: tuple


def prediction_from_message(message):
    """Reject invalid or forbidden target data before it can reach research geometry."""
    try:
        if not message.valid:
            raise ValueError('INVALID_PREDICTION')
        if message.source != 'tracking':
            raise ValueError('PREDICTION_SOURCE')
        if message.frame_id != 'local_ned':
            raise ValueError('PREDICTION_FRAME')
        result = ShadowPrediction(
            int(message.mission_id), int(message.sequence_id), _seconds(message.source_stamp),
            _seconds(message.observation_stamp), _seconds(message.generated_stamp),
            _seconds(message.valid_until), str(message.frame_id), str(message.source),
            str(message.model), float(message.prediction_horizon),
            tuple(_seconds(sample.relative_time) for sample in message.samples),
            tuple(_xyz(sample.position) for sample in message.samples),
            tuple(_xyz(sample.velocity) for sample in message.samples),
        )
        stamps = (result.source_stamp, result.observation_stamp, result.generated_stamp,
                  result.valid_until, result.prediction_horizon)
        values = (*stamps, *result.prediction_times,
                  *(value for vector in (*result.target_positions, *result.target_velocities)
                    for value in vector))
        if (result.mission_id < 1 or result.sequence_id < 0
                or not all(math.isfinite(value) for value in values)
                or not all(value > 0 for value in stamps)
                or result.observation_stamp > result.source_stamp + 1e-9
                or result.generated_stamp < result.source_stamp - 1e-9
                or result.valid_until <= result.source_stamp
                or len(result.prediction_times) < 2 or result.prediction_times[0] < 0
                or any(right <= left for left, right in zip(
                    result.prediction_times, result.prediction_times[1:]))
                or result.prediction_times[-1] > result.prediction_horizon + 1e-7):
            raise ValueError('INVALID_PREDICTION')
        return result
    except (AttributeError, TypeError, OverflowError) as error:
        raise ValueError('INVALID_PREDICTION') from error


@profiled('input_prepare')
def make_shadow_request(pose, prediction, mission_id, cycle_id, now, config,
                        execution_lead=None):
    """Preserve paired PVA/attitude and prediction epochs with strict horizon coverage."""
    if prediction.mission_id != mission_id:
        raise ValueError('MISSION_CHANGED')
    if prediction.source_stamp > now + 1e-9 or prediction.generated_stamp > now + 1e-9:
        raise ValueError('PREDICTION_FUTURE')
    if pose.stamp > now + 1e-9 or prediction.observation_stamp > now + 1e-9:
        raise ValueError('INPUT_FUTURE')
    if (max(now - pose.stamp, now - prediction.observation_stamp,
            now - prediction.source_stamp) > config.maximum_input_age + 1e-9
            or now >= prediction.valid_until):
        raise ValueError('STALE_INPUT')
    execution_epoch = pose.stamp if execution_lead is None else now + execution_lead
    first = execution_epoch - prediction.source_stamp
    end = first + config.dt * config.horizon_steps
    if (first < prediction.prediction_times[0] - 1e-8
            or end > prediction.prediction_times[-1] + 1e-8):
        raise ValueError('PREDICTION_HORIZON')
    rotation = body_frd_to_ned_from_quaternion(pose.quaternion_wxyz)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    context = PlanningContext(
        mission_id, cycle_id, pose.stamp, pose.stamp, pose.stamp,
        prediction.source_stamp, prediction.observation_stamp, prediction.sequence_id,
        prediction.valid_until, pose.clock_generation, prediction.frame_id, prediction.source,
    )
    request = MpcRequest(context, (*pose.position, *pose.velocity, *pose.acceleration, yaw),
                         prediction.prediction_times, prediction.target_positions,
                         prediction.target_velocities, tuple(map(tuple, rotation)), float(now))
    if execution_lead is not None:
        from uav_control.guidance.follow_problem import future_request
        request = future_request(request, execution_lead)
    return request


def completion_rejection(request, config, now, mission_id, generation, elapsed, follow):
    """Recheck live publication admission after preparation, solve, and JSON serialization."""
    context = request.context
    if not follow:
        return 'MISSION_NOT_FOLLOW'
    if mission_id != context.mission_id:
        return 'MISSION_CHANGED'
    if generation != context.clock_generation:
        return 'CLOCK_GENERATION_CHANGED'
    if not math.isfinite(now) or now < request.now_stamp - 1e-9:
        return 'PUBLICATION_CLOCK_RESET'
    if (max(now - context.navigation_stamp, now - context.observation_stamp,
            now - context.prediction_source_stamp) > config.maximum_input_age + 1e-9
            or now >= context.prediction_valid_until):
        return 'STALE_INPUT'
    if not math.isfinite(elapsed) or elapsed < 0 or elapsed > config.solve_budget:
        return 'CYCLE_DEADLINE_EXCEEDED'
    if (context.execution_start_stamp > context.navigation_stamp + 1e-9
            and now >= context.execution_start_stamp):
        return 'FUTURE_EXECUTION_MISSED'
    return ''


def _empty_output(reason):
    return {'valid': False, 'solver_status': 'SHADOW_REJECTED', 'reason': str(reason),
            'relative_times': [], 'positions': [], 'velocities': [], 'accelerations': [],
            'jerks': [], 'yaw_refs': [], 'yaw_rates': [], 'visibility_margins': []}


class ShadowResearchRunner:
    """Maintain one in-flight request; never queue optimization work behind it."""

    def __init__(self, config, adapter, executor, emit, rate_hz=1.0,
                 solver=None, request_factory=make_shadow_request):
        """Keep subscriptions responsive while one bounded solver uses a worker thread."""
        config.validate()
        if (config.allow_synthetic_predictions or config.maximum_input_age > .125
                or not math.isfinite(rate_hz) or rate_hz <= 0):
            raise ValueError('shadow requires tracking, <=125ms freshness, and positive rate')
        self.config, self.adapter, self.executor, self.emit = config, adapter, executor, emit
        self.solver = solver if solver is not None else FollowMpcSeed(config)
        self.request_factory = request_factory
        self.period = 1.0 / rate_hz
        self.mission = {'mission_id': 0, 'state': 0, 'completed': False, 'stamp': 0.0}
        self.mission_revision = 0
        self.prediction = None
        self.prediction_provenance = None
        self.prediction_revision = 0
        self.pending = None
        self.last_request = None
        self.last_pose = None
        self.last_prediction = None
        self.cycle_id = 0
        self.next_solve = -math.inf
        self.last_rejection = None
        self.clear_warm_pending = False

    def _follow(self):
        return (self.mission['mission_id'] > 0 and self.mission['state'] == MissionState.FOLLOW
                and not self.mission['completed'])

    def _invalidate_warm(self):
        if self.pending is None:
            self.solver.clear_warm_start()
        else:
            self.clear_warm_pending = True

    def update_mission(self, message, now, monotonic_now):
        """Invalidate candidate identity when the mission changes or exits FOLLOW."""
        incoming_stamp = _seconds(message.stamp)
        if not math.isfinite(incoming_stamp) or incoming_stamp <= 0 or incoming_stamp > now + 1e-9:
            self.reject('INVALID_MISSION_TIMESTAMP', now, monotonic_now)
            return
        if incoming_stamp < self.mission['stamp'] - 1e-9:
            self.reject('MISSION_OUT_OF_ORDER', now, monotonic_now)
            return
        previous = self.mission['mission_id']
        previous_identity = (previous, self.mission['state'], self.mission['completed'])
        self.mission = {'mission_id': int(message.mission_id), 'state': int(message.state),
                        'completed': bool(message.completed), 'stamp': incoming_stamp,
                        'state_name': str(message.state_name)}
        if previous_identity != (self.mission['mission_id'], self.mission['state'],
                                 self.mission['completed']):
            self.mission_revision += 1
        if previous != self.mission['mission_id']:
            self._invalidate_warm()
            self.reject('MISSION_CHANGED', now, monotonic_now)
        elif not self._follow():
            self._invalidate_warm()
            self.reject('MISSION_NOT_FOLLOW', now, monotonic_now)

    def update_prediction(self, message, now, monotonic_now):
        """Replace the latest snapshot, clearing candidates on an explicit invalid input."""
        self.prediction_provenance = {
            'valid': bool(message.valid), 'invalid_reason': str(message.invalid_reason),
            'source': str(message.source), 'frame_id': str(message.frame_id),
            'mission_id': int(message.mission_id), 'sequence_id': int(message.sequence_id),
            'source_stamp': _seconds(message.source_stamp),
            'observation_stamp': _seconds(message.observation_stamp),
            'generated_stamp': _seconds(message.generated_stamp),
            'valid_until': _seconds(message.valid_until), 'receipt_ros_stamp': now,
            'receipt_monotonic': monotonic_now,
        }
        try:
            prediction = prediction_from_message(message)
            if self.prediction is not None and (
                prediction.source_stamp < self.prediction.source_stamp - 1e-9
                or (prediction.mission_id == self.prediction.mission_id
                    and prediction.sequence_id < self.prediction.sequence_id)
            ):
                raise ValueError('PREDICTION_OUT_OF_ORDER')
            self.prediction = prediction
            self.prediction_provenance['snapshot'] = asdict(prediction)
        except ValueError as error:
            self.prediction = None
            self.prediction_revision += 1
            self._invalidate_warm()
            self.reject(str(error), now, monotonic_now)

    def event(self, output, now, monotonic_now, candidate=None, started=None):
        """Build a complete reproducible request/provenance/output event for JSONL."""
        request = self.pending['request'] if self.pending else self.last_request
        paired = self.pending['pose'] if self.pending else self.last_pose
        prepared_prediction = (self.pending['prediction'] if self.pending
                               else self.last_prediction)
        status = ('ADMITTED_RESEARCH' if output['valid'] else output.get('reason')
                  or 'CORE_' + output.get('solver_status', 'REJECTED'))
        return {
            'schema_version': 1, 'event': 'completion' if candidate is not None else 'reject',
            'research_mode': getattr(self.solver, 'mode', 'mpc_seed'),
            'event_stamp': now, 'accepted_by_tracker': False,
            'request': asdict(request) if request else None,
            'input_provenance': {'paired_navigation': asdict(paired) if paired else None,
                                 'prediction': self.prediction_provenance,
                                 'prepared_prediction': (asdict(prepared_prediction)
                                                         if prepared_prediction else None),
                                 'mission': dict(self.mission),
                                 'clock_generation': self.adapter.clock_generation,
                                 'model_config': {
                                     'mpc': asdict(self.config),
                                     'intrinsics': asdict(self.solver.intrinsics),
                                     'extrinsics': asdict(self.solver.extrinsics),
                                     'target': asdict(self.solver.target),
                                     'visibility': asdict(self.solver.visibility),
                                     'attitude': asdict(self.solver.attitude_config),
                                     'minco': (asdict(self.solver.optimizer_config)
                                               if hasattr(self.solver, 'optimizer_config')
                                               else None),
                                 }},
            'output': output, 'candidate_output': candidate, 'core_result': candidate,
            'admission_status': status,
            'whole_cycle_time': monotonic_now - started if started is not None else 0.0,
            'cycle_started_monotonic': started,
            'expires_at_ros_stamp': (min(
                request.context.navigation_stamp + self.config.maximum_input_age,
                request.context.observation_stamp + self.config.maximum_input_age,
                request.context.prediction_source_stamp + self.config.maximum_input_age,
                request.context.prediction_valid_until) if request else None),
            'cycle_deadline_monotonic': (started + self.config.solve_budget
                                         if started is not None else None),
        }

    def reject(self, reason, now, monotonic_now):
        """Clear the research trajectory on input failure without repetitive unchanged logs."""
        identity = (reason, self.mission['mission_id'], self.adapter.clock_generation)
        if identity != self.last_rejection:
            self.last_rejection = identity
            self.emit(self.event(_empty_output(reason), now, monotonic_now))

    def _prepare(self, now, monotonic_now):
        if not self._follow():
            raise ValueError('MISSION_NOT_FOLLOW')
        if self.prediction is None:
            raise ValueError('PREDICTION_UNAVAILABLE')
        pair = self.adapter.pair_latest(now, monotonic_now, self.prediction.observation_stamp)
        if pair.snapshot is None:
            raise ValueError(pair.status)
        if pair.snapshot.clock_generation != self.adapter.clock_generation:
            raise ValueError('CLOCK_GENERATION_CHANGED')
        request = self.request_factory(pair.snapshot, self.prediction, self.mission['mission_id'],
                                       self.cycle_id + 1, now, self.config)
        return request, pair.snapshot

    def _finish(self, now, monotonic_now):
        record = self.pending
        request = record['request']
        try:
            result = record['future'].result()
            candidate = asdict(result)
            reason = completion_rejection(request, self.config, now, self.mission['mission_id'],
                                          self.adapter.clock_generation,
                                          monotonic_now - record['started'], self._follow())
            if not reason and result.context != request.context:
                reason = 'SOLVER_CONTEXT_MISMATCH'
            if not reason and self.mission_revision != record['mission_revision']:
                reason = 'MISSION_STATE_CHANGED'
            if not reason and self.prediction_revision != record['prediction_revision']:
                reason = 'PREDICTION_INVALIDATED'
            if not reason:
                try:
                    self._prepare(now, monotonic_now)
                except ValueError as error:
                    reason = str(error)
            output = _empty_output(reason) if reason else candidate
        except Exception as error:
            candidate = {'exception_type': type(error).__name__, 'exception': str(error)}
            output = _empty_output('SOLVER_EXCEPTION')
        event = self.event(output, now, monotonic_now, candidate, record['started'])
        self.emit(event)
        self.pending = None
        self.last_rejection = None
        if self.clear_warm_pending or not event['output']['valid']:
            self.solver.clear_warm_start()
        self.clear_warm_pending = False

    def tick(self, now, monotonic_now):
        """Consume completions, reject stale inputs, and dispatch at most the latest request."""
        if self.pending is not None and self.pending['future'].done():
            self._finish(now, monotonic_now)
            return
        try:
            request, pose = self._prepare(now, monotonic_now)
        except ValueError as error:
            self._invalidate_warm()
            self.reject(str(error), now, monotonic_now)
            return
        if self.pending is not None or monotonic_now < self.next_solve:
            return
        self.cycle_id += 1
        self.last_request, self.last_pose = request, pose
        self.last_prediction = self.prediction
        self.last_rejection = None
        self.next_solve = monotonic_now + self.period
        self.pending = {'request': request, 'pose': pose, 'started': monotonic_now,
                        'prediction': self.prediction,
                        'mission_revision': self.mission_revision,
                        'prediction_revision': self.prediction_revision}
        self.pending['future'] = self.executor.submit(self.solver.solve, request)


def _json_safe(value):
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class FollowMpcShadowNode(Node):
    """Publish only research String JSON; never publish or alter flight/mission commands."""

    node_name = 'follow_mpc_shadow_node'
    config_class = MpcConfig

    def __init__(self):
        """Connect fixed navigation/tracking inputs and trusted Gazebo dual-clock anchors."""
        super().__init__(self.node_name)
        if self.get_parameter('use_sim_time').value:
            raise ValueError('shadow acquisition epochs require ROS/system clock')
        for name, value in asdict(MpcConfig()).items():
            self.declare_parameter(name, value)
        self.declare_parameter('shadow_rate_hz', 1.0)
        self.declare_parameter('gazebo_world_name', 'default')
        workspace = os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc')
        self.declare_parameter('log_directory', str(
            Path(workspace) / 'data/experiments/mpc_shadow'))
        values = {name: self.get_parameter(name).value for name in asdict(MpcConfig())}
        config = self.config_class(**values)
        self.adapter = ShadowInputAdapter()
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mpc-shadow-research')
        self.diagnostic_pub = self.create_publisher(String, '/research/mpc_seed/diagnostic', 1)
        self.trajectory_pub = self.create_publisher(String, '/research/mpc_seed/trajectory', 1)
        directory = Path(str(self.get_parameter('log_directory').value)).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.log_path = directory / f'mpc_seed_shadow_{time.time_ns()}_{os.getpid()}.jsonl'
        self.log_file = self.log_path.open('x', encoding='utf-8')
        self.runner = self.make_runner(config)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        mission_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_owned = (
            self.create_subscription(UavState, '/navigation/uav_state',
                                     self.navigation_callback, 10),
            self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                     self.attitude_callback, sensor_qos),
            self.create_subscription(TimesyncStatus, '/fmu/out/timesync_status',
                                     self.timesync_callback, sensor_qos),
            self.create_subscription(TargetPrediction, '/planning/target_prediction',
                                     self.prediction_callback, sensor_qos),
            self.create_subscription(MissionState, '/mission/state', self.mission_callback,
                                     mission_qos),
        )
        from gz.msgs10.clock_pb2 import Clock as GazeboClock
        from gz.transport13 import Node as GazeboTransportNode
        self.gz_node = GazeboTransportNode()
        world = str(self.get_parameter('gazebo_world_name').value).strip('/')
        self.gz_clock_topic = f'/world/{world}/clock'
        if not self.gz_node.subscribe(GazeboClock, self.gz_clock_topic,
                                      self.gazebo_clock_callback):
            raise RuntimeError(f'Cannot subscribe to trusted clock {self.gz_clock_topic}')
        self.timer = self.create_timer(.02, self.tick)
        self.get_logger().info(
            f'MPC SHADOW READY | 1 worker | research only | JSONL={self.log_path}')

    def make_runner(self, config):
        """Provide a solver injection point while retaining the original MPC default."""
        return ShadowResearchRunner(config, self.adapter, self.pool, self._publish_event,
                                    float(self.get_parameter('shadow_rate_hz').value))

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def navigation_callback(self, message):
        """Forward the original physical navigation message with separate receipt clocks."""
        self.adapter.add_navigation(message, self._ros_seconds(), time.monotonic())

    def attitude_callback(self, message):
        """Forward full native-sample PX4 attitude, never a receipt-derived orientation epoch."""
        self.adapter.add_attitude(message, self._ros_seconds(), time.monotonic())

    def timesync_callback(self, message):
        """Preserve source-epoch DDS clock offset history in the causal adapter."""
        self.adapter.add_timesync(message)

    def gazebo_clock_callback(self, message):
        """Pass trusted same-message simulation/system anchors to the locked adapter."""
        self.adapter.add_clock_anchor(
            _gazebo_seconds(message.sim), _gazebo_seconds(message.system),
            self._ros_seconds(), time.monotonic())

    def prediction_callback(self, message):
        """Replace the tracking prediction snapshot or immediately publish an invalid marker."""
        self.runner.update_prediction(message, self._ros_seconds(), time.monotonic())

    def mission_callback(self, message):
        """Observe mission identity/state without publishing mission transitions."""
        self.runner.update_mission(message, self._ros_seconds(), time.monotonic())

    def tick(self):
        """Service bounded research work without delaying subscription delivery."""
        self.runner.tick(self._ros_seconds(), time.monotonic())

    @profiled('ros_publication')
    def _publish_attempt(self, publisher, topic, message, event):
        """Preserve every actual publication flag and its call-time epoch/wall cost."""
        monotonic_now = time.monotonic()
        started = event['cycle_started_monotonic']
        event.setdefault('publication_attempts', []).append({
            'topic': topic, 'valid': bool(event['output']['valid']),
            'stamp': self._ros_seconds(), 'monotonic_stamp': monotonic_now,
            'elapsed': monotonic_now - started if started is not None else 0.0,
        })
        publisher.publish(message)

    def _publish_event(self, event):
        # Include JSON serialization and final publication checks in the wall budget.
        serialized = _serialize(event, separators=(',', ':'))
        request_data = event['request']
        now, monotonic_now = self._ros_seconds(), time.monotonic()
        if event['output']['valid'] and request_data:
            request = (self.runner.pending['request'] if self.runner.pending
                       else self.runner.last_request)
            elapsed = monotonic_now - event['cycle_started_monotonic']
            reason = completion_rejection(request, self.runner.config, now,
                                          self.runner.mission['mission_id'],
                                          self.adapter.clock_generation, elapsed,
                                          self.runner._follow())
            if reason:
                event['output'] = _empty_output(reason)
                event['admission_status'] = reason
                serialized = _serialize(event, separators=(',', ':'))
        message = String(data=serialized)
        self._publish_attempt(self.trajectory_pub, '/research/mpc_seed/trajectory', message, event)
        self._publish_attempt(self.diagnostic_pub, '/research/mpc_seed/diagnostic', message, event)
        finished = time.monotonic()
        if event['output']['valid'] and request_data:
            reason = completion_rejection(request, self.runner.config, self._ros_seconds(),
                                          self.runner.mission['mission_id'],
                                          self.adapter.clock_generation,
                                          finished - event['cycle_started_monotonic'],
                                          self.runner._follow())
            if reason:
                # A publisher which crosses the deadline must immediately clear its marker.
                event['output'] = _empty_output(reason)
                event['admission_status'] = reason
                event['publication_crossed_deadline'] = True
                clear_message = String(data=_serialize(
                    event, separators=(',', ':')))
                self._publish_attempt(self.trajectory_pub, '/research/mpc_seed/trajectory',
                                      clear_message, event)
                self._publish_attempt(self.diagnostic_pub, '/research/mpc_seed/diagnostic',
                                      clear_message, event)
                finished = time.monotonic()
        if event['cycle_started_monotonic'] is not None:
            event['whole_cycle_time'] = finished - event['cycle_started_monotonic']
        event['publish_stamp'] = self._ros_seconds()
        event['published_valid'] = any(
            attempt['valid'] for attempt in event['publication_attempts'])
        event['final_marker_valid'] = bool(event['output']['valid'])
        self.log_file.write(_serialize(event) + '\n')
        self.log_file.flush()

    def destroy_node(self):
        """Close only this research worker/log; never affect the original flight processes."""
        self.pool.shutdown(wait=True, cancel_futures=True)
        self.log_file.close()
        return super().destroy_node()


@profiled('json_serialization')
def _serialize(event, **kwargs):
    return json.dumps(_json_safe(event), allow_nan=False, **kwargs)


def main(args=None):
    """Run the standalone research node with ROS/system time and bounded shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = FollowMpcShadowNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
