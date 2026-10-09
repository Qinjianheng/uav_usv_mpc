"""ROS node for truth-only evaluation, event metrics, and run artifacts."""

import math
import statistics
import time
from collections import deque

from builtin_interfaces.msg import Time
import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool, Float32
from uav_usv_interfaces.msg import ControllerDiagnostic, InterceptResult
from uav_usv_interfaces.msg import MissionState, PlannerDiagnostic
from uav_usv_interfaces.msg import TargetPrediction, TargetState
from uav_usv_interfaces.msg import TargetObservation

from uav_control.tracking.prediction_error_tracker import (
    PredictionErrorTracker,
)

from .intercept_evaluator import ExperimentArtifactWriter
from .intercept_evaluator import InterceptEvaluatorCore, KinematicState
from .intercept_evaluator import PlannerEventAccumulator
from .intercept_evaluator import RuntimePerformanceAccumulator
from .intercept_evaluator import TimestampedStateHistory
from .intercept_evaluator import VisionMetricAccumulator
from .intercept_evaluator import synchronize_histories
from .intercept_evaluator import image_pose_diagnostics
from .run_metrics import RunMetricAccumulator
from .uav_heading_diagnostics import GazeboUavPoseTracker, heading_diagnostics
from .gazebo_entity_pose import entity_geometry_residuals
from .gazebo_entity_pose import GazeboEntityPoseTracker
from uav_control.common.runtime_performance import RateMeter


PREDICTION_HORIZONS = (0.5, 1.0, 2.0)


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _seconds_to_time(value):
    value = max(float(value), 0.0)
    seconds = int(value)
    nanoseconds = int(round((value - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def _finite_tuple(values, label):
    values = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'{label} contains non-finite values')
    return values


def truth_from_message(message):
    """Convert an explicitly truth-topic TargetState for evaluation only."""
    if not message.valid:
        raise ValueError('simulation truth state is invalid')
    return KinematicState(
        position=_finite_tuple((
            message.position.x,
            message.position.y,
            message.position.z,
        ), 'target position'),
        velocity=_finite_tuple((
            message.velocity.x,
            message.velocity.y,
            message.velocity.z,
        ), 'target velocity'),
    )


def uav_from_message(message):
    """Convert PX4 local state in unchanged NED coordinates."""
    return KinematicState(
        position=_finite_tuple(
            (message.x, message.y, message.z), 'UAV position'
        ),
        velocity=_finite_tuple(
            (message.vx, message.vy, message.vz), 'UAV velocity'
        ),
    )


def result_to_message(result, stamp_seconds, radius):
    """Serialize one terminal truth-evaluation result."""
    message = InterceptResult()
    message.stamp = _seconds_to_time(stamp_seconds)
    message.mission_id = int(result.mission_id)
    message.success = bool(result.success)
    message.outcome = str(result.outcome)
    message.reason = str(result.reason)
    message.detail = str(result.detail) or (
        'determined by simulation truth in evaluator only'
    )
    message.elapsed_time = float(result.elapsed_time)
    message.capture_radius = float(radius)
    message.minimum_distance = float(result.minimum_distance)
    message.horizontal_distance = float(result.horizontal_distance)
    message.vertical_error = float(result.vertical_error)
    message.relative_speed = float(result.relative_speed)
    message.closing_speed = float(result.closing_speed)
    message.max_horizontal_speed = float(result.maximum_horizontal_speed)
    message.max_vertical_speed = float(result.maximum_vertical_speed)
    message.max_horizontal_acceleration = float(
        result.maximum_horizontal_acceleration
    )
    message.max_vertical_acceleration = float(
        result.maximum_vertical_acceleration
    )
    return message


def _prediction_at(message, horizon):
    samples = list(message.samples)
    if not samples:
        raise ValueError('prediction has no samples')
    horizon = float(horizon)
    if horizon <= _duration_seconds(samples[0].relative_time):
        point = samples[0].position
        return float(point.x), float(point.y), float(point.z)
    for left, right in zip(samples, samples[1:]):
        left_time = _duration_seconds(left.relative_time)
        right_time = _duration_seconds(right.relative_time)
        if horizon <= right_time:
            scale = (horizon - left_time) / max(right_time - left_time, 1e-9)
            return tuple(
                float(getattr(left.position, axis))
                + scale * (
                    float(getattr(right.position, axis))
                    - float(getattr(left.position, axis))
                )
                for axis in ('x', 'y', 'z')
            )
    point = samples[-1].position
    return float(point.x), float(point.y), float(point.z)


class InterceptEvaluatorNode(Node):
    """Observe truth and diagnostics without feeding any control input."""

    def __init__(self):
        super().__init__('intercept_evaluator_node')
        self.declare_parameter('evaluation_rate_hz', 20.0)
        self.declare_parameter('evaluation_capture_radius', 0.50)
        self.declare_parameter('planned_capture_radius', 0.35)
        self.declare_parameter('sea_surface_z', 0.0)
        self.declare_parameter('enable_sea_contact_failure', True)
        self.declare_parameter('body_lower_extent', 0.23)
        self.declare_parameter('maximum_duration', 30.0)
        self.declare_parameter('detailed_diagnostics_enabled', False)
        self.declare_parameter('visual_evaluation_enabled', True)
        self.declare_parameter(
            'visual_observation_topic',
            '/perception/front/target_observation',
        )
        self.declare_parameter(
            'log_directory',
            'data/experiments/current',
        )
        self.declare_parameter('truth_topic', '/target/state')
        self.declare_parameter('truth_role', 'evaluation_only')
        self.declare_parameter(
            'shadow_prediction_topic',
            '/planning/shadow_target_prediction',
        )
        self.declare_parameter('gazebo_world_name', 'default')
        self.declare_parameter('gazebo_entity_diagnostics_enabled', False)
        self.declare_parameter('gazebo_uav_diagnostics_enabled', False)
        self.declare_parameter('gazebo_uav_entity_name', 'x500_mono_cam_0')
        self.declare_parameter('gazebo_target_entity_name', 'usv_target')
        self.declare_parameter('gazebo_visual_height_offset', 0.42)
        self.declare_parameter('gazebo_pause_timeout_ms', 250)
        self.declare_parameter('gazebo_pause_maximum_attempts', 2)
        self.declare_parameter('gazebo_pause_retry_delay', 0.05)
        self.declare_parameter('pause_gazebo_on_success', True)
        rate = float(self.get_parameter('evaluation_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('evaluation_rate_hz must be positive')
        self.evaluation_capture_radius = float(
            self.get_parameter('evaluation_capture_radius').value
        )
        self.planned_capture_radius = float(
            self.get_parameter('planned_capture_radius').value
        )
        self.evaluator = InterceptEvaluatorCore(
            capture_radius=self.evaluation_capture_radius,
            sea_surface_z=self.get_parameter('sea_surface_z').value,
            enable_sea_contact_failure=self.get_parameter(
                'enable_sea_contact_failure'
            ).value,
            maximum_duration=self.get_parameter('maximum_duration').value,
            body_lower_extent=self.get_parameter('body_lower_extent').value,
        )
        self.log_directory = str(
            self.get_parameter('log_directory').value
        )
        self.detailed_diagnostics_enabled = bool(
            self.get_parameter('detailed_diagnostics_enabled').value
        )
        self.visual_evaluation_enabled = bool(
            self.get_parameter('visual_evaluation_enabled').value
        )
        self.visual_observation_topic = str(
            self.get_parameter('visual_observation_topic').value
        )
        self.truth_topic = str(self.get_parameter('truth_topic').value)
        self.truth_role = str(self.get_parameter('truth_role').value)
        if self.truth_role != 'evaluation_only':
            raise ValueError('truth_role must be evaluation_only')
        self.shadow_prediction_topic = str(
            self.get_parameter('shadow_prediction_topic').value
        )
        self.gazebo_pauser = None
        self.gazebo_entity_tracker = None
        self.gazebo_uav_tracker = None
        self.gazebo_uav_diagnostics_enabled = bool(self.get_parameter(
            'gazebo_uav_diagnostics_enabled'
        ).value)
        self.gazebo_uav_entity_name = str(self.get_parameter(
            'gazebo_uav_entity_name'
        ).value)
        self.gazebo_entity_node = None
        self.gazebo_target_entity_name = str(
            self.get_parameter('gazebo_target_entity_name').value
        )
        self.gazebo_visual_height_offset = float(
            self.get_parameter('gazebo_visual_height_offset').value
        )
        world_name = str(
            self.get_parameter('gazebo_world_name').value
        ).strip().strip('/')
        self.pause_gazebo_on_success = bool(self.get_parameter(
            'pause_gazebo_on_success').value)
        if self.pause_gazebo_on_success:
            try:
                from .gazebo_terminal import GazeboTerminalPauser, GazeboWorldPauseClient

                pause_client = GazeboWorldPauseClient(
                    world_name, self.get_parameter('gazebo_pause_timeout_ms').value)
                self.gazebo_pauser = GazeboTerminalPauser(
                    pause_client.pause_world,
                    maximum_attempts=self.get_parameter(
                        'gazebo_pause_maximum_attempts').value,
                    retry_delay=self.get_parameter('gazebo_pause_retry_delay').value,
                )
            except (ImportError, RuntimeError, ValueError) as exc:
                self.get_logger().warn(f'Capture pause client unavailable: {exc}')
        self.gazebo_entity_diagnostics_enabled = bool(self.get_parameter(
            'gazebo_entity_diagnostics_enabled'
        ).value)
        if self.gazebo_entity_diagnostics_enabled or self.gazebo_uav_diagnostics_enabled:
            try:
                from gz.msgs10.clock_pb2 import Clock as GazeboClock
                from gz.msgs10.pose_v_pb2 import Pose_V
                from gz.transport13 import Node as GazeboTransportNode

                if self.gazebo_entity_diagnostics_enabled:
                    self.gazebo_entity_tracker = GazeboEntityPoseTracker()
                if self.gazebo_uav_diagnostics_enabled:
                    self.gazebo_uav_tracker = GazeboUavPoseTracker(
                        model_name=self.gazebo_uav_entity_name,
                    )
                self.gazebo_entity_node = GazeboTransportNode()
                clock_topic = f'/world/{world_name}/clock'
                pose_topic = f'/world/{world_name}/pose/info'
                if not self.gazebo_entity_node.subscribe(
                    GazeboClock,
                    clock_topic,
                    self.gazebo_entity_clock_callback,
                ):
                    raise RuntimeError(
                        f'could not subscribe to {clock_topic}'
                    )
                if not self.gazebo_entity_node.subscribe(
                    Pose_V,
                    pose_topic,
                    self.gazebo_entity_pose_callback,
                ):
                    raise RuntimeError(
                        f'could not subscribe to {pose_topic}'
                    )
            except (ImportError, RuntimeError, TypeError) as error:
                self.gazebo_entity_tracker = None
                self.gazebo_uav_tracker = None
                self.gazebo_entity_node = None
                self.get_logger().warn(
                    f'Gazebo entity diagnostics unavailable: {error}'
                )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        result_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.uav_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.uav_callback,
            sensor_qos,
        )
        self.truth_sub = self.create_subscription(
            TargetState,
            self.truth_topic,
            self.truth_callback,
            sensor_qos,
        )
        self.filtered_target_sub = self.create_subscription(
            TargetState,
            '/tracking/target_state',
            self.filtered_target_callback,
            sensor_qos,
        )
        self.visual_observation_sub = self.create_subscription(
            TargetObservation,
            self.visual_observation_topic,
            self.visual_observation_callback,
            sensor_qos,
        )
        self.prediction_sub = self.create_subscription(
            TargetPrediction,
            '/planning/target_prediction',
            self.prediction_callback,
            sensor_qos,
        )
        self.shadow_prediction_sub = self.create_subscription(
            TargetPrediction,
            self.shadow_prediction_topic,
            self.shadow_prediction_callback,
            sensor_qos,
        )
        self.planner_sub = self.create_subscription(
            PlannerDiagnostic,
            '/planning/diagnostic',
            self.planner_callback,
            event_qos,
        )
        self.controller_sub = self.create_subscription(
            ControllerDiagnostic,
            '/control/diagnostic',
            self.controller_callback,
            event_qos,
        )
        self.mission_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_callback,
            result_qos,
        )
        self.performance_values = {}
        performance_topics = {
            'gazebo_real_time_factor': '/simulation/gazebo/real_time_factor',
            'front_rgb_hz': '/diagnostics/front/rgb_frame_hz',
            'front_depth_hz': '/diagnostics/front/depth_frame_hz',
            'down_rgb_hz': '/diagnostics/down/rgb_frame_hz',
            'down_depth_hz': '/diagnostics/down/depth_frame_hz',
            'rgbd_localizer_compute_time': (
                '/diagnostics/rgbd_localizer/compute_time'
            ),
            'front_monitor_compute_time': (
                '/diagnostics/front/monitor_compute_time'
            ),
            'down_monitor_compute_time': (
                '/diagnostics/down/monitor_compute_time'
            ),
        }
        self.performance_subscriptions = [
            self.create_subscription(
                Float32,
                topic,
                lambda message, name=name: self._performance_callback(
                    name, message
                ),
                sensor_qos,
            )
            for name, topic in performance_topics.items()
        ]
        self.result_pub = self.create_publisher(
            InterceptResult,
            '/simulation/impact/result',
            result_qos,
        )
        self.hit_pub = self.create_publisher(
            Bool,
            '/simulation/impact/hit',
            result_qos,
        )
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.latest_uav = None
        self.latest_truth = None
        self.uav_history = TimestampedStateHistory(0.5)
        self.truth_history = TimestampedStateHistory(0.5)
        self.last_synchronized_stamp = None
        self.latest_mission = None
        self.latest_controller = None
        self.latest_prediction = None
        self.latest_planner_diagnostic = None
        self.latest_tracker_rejection_reason = ''
        self.event_metrics = PlannerEventAccumulator()
        self.prediction_tracker = PredictionErrorTracker(PREDICTION_HORIZONS)
        self.vision_metrics = VisionMetricAccumulator()
        self.pending_visual_observations = deque(maxlen=100)
        self.prediction_sequences = set()
        self.shadow_prediction_sequences = set()
        self.prediction_errors = {
            (model, horizon): []
            for model in (
                'guidance',
                'kf',
                'shadow_bctra',
            )
            for horizon in PREDICTION_HORIZONS
        }
        self.latest_prediction_error = {}
        self.controller_compute_times = []
        self.tracker_rate = RateMeter(window_seconds=1.0)
        self.runtime_performance = RuntimePerformanceAccumulator()
        self.terminal_pause_result = None
        self.writer = None
        self.run_started_at = None
        self.run_mission_id = None
        self.intercept_started_at = None
        self.intercept_result = None
        self.run_metrics = None
        self.last_run_sample_stamp = None
        self.last_run_paths = None
        self.last_run_summary = None
        self.result_published = False
        self.approach_phase = 'PREPARATION'
        self.terminal_approach_count = 0
        self.recovery_count = 0
        self.handover_jump_count = 0
        self.first_terminal_approach_result = ''
        self.minimum_body_clearance = math.inf
        self.truth_motion_regime = 'UNKNOWN'
        self.last_truth_heading = None
        self.last_truth_heading_stamp = None
        self.safety_event_histogram = {
            name: 0
            for name in ('WARNING', 'BRAKE', 'UNRECOVERABLE', 'SEA_CONTACT')
        }
        self.last_safety_state = ''
        self.get_logger().info(
            'Truth-only evaluator ready | truth='
            f'{self.truth_topic} | capture='
            f'{self.evaluation_capture_radius:.2f} m | '
            f'output={self.log_directory}'
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _gazebo_stamp(stamp):
        return float(stamp.sec) + float(stamp.nsec) * 1e-9

    def gazebo_entity_clock_callback(self, message):
        """Keep the evaluator-only entity history in ROS system time."""
        for tracker in (self.gazebo_entity_tracker, self.gazebo_uav_tracker):
            if tracker is not None:
                tracker.add_clock_anchor(
                    self._gazebo_stamp(message.sim),
                    self._gazebo_stamp(message.system),
                    self._now(), time.monotonic(),
                )

    def gazebo_entity_pose_callback(self, message):
        """Record the actual rendered target pose at its Gazebo sample time."""
        sim_stamp = self._gazebo_stamp(message.header.stamp)
        for pose in message.pose:
            name = str(pose.name)
            if self.gazebo_entity_tracker is not None and (
                name == self.gazebo_target_entity_name
                or name.rsplit('::', 1)[-1]
                == self.gazebo_target_entity_name
            ):
                self.gazebo_entity_tracker.add_pose(
                    sim_stamp,
                    (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    ),
                    self._now(),
                )
            if (self.gazebo_uav_tracker is not None
                    and self.gazebo_uav_tracker.matches_model(name)):
                self.gazebo_uav_tracker.add_pose(
                    sim_stamp,
                    (pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.w, pose.orientation.x,
                     pose.orientation.y, pose.orientation.z),
                    self._now(),
                )

    def uav_callback(self, message):
        try:
            state = uav_from_message(message)
        except ValueError:
            self.latest_uav = None
            return
        self.latest_uav = state
        self.uav_history.add(self._now(), state)

    def truth_callback(self, message):
        try:
            self.latest_truth = truth_from_message(message)
        except ValueError:
            self.latest_truth = None
            return
        truth_stamp = _stamp_seconds(message.stamp) or self._now()
        self.truth_history.add(truth_stamp, self.latest_truth)
        horizontal_speed = math.hypot(*self.latest_truth.velocity[:2])
        if horizontal_speed > 0.2:
            heading = math.atan2(
                self.latest_truth.velocity[1],
                self.latest_truth.velocity[0],
            )
            if (
                self.last_truth_heading is not None
                and self.last_truth_heading_stamp is not None
                and truth_stamp > self.last_truth_heading_stamp + 1e-6
            ):
                delta = math.atan2(
                    math.sin(heading - self.last_truth_heading),
                    math.cos(heading - self.last_truth_heading),
                )
                yaw_rate = delta / (
                    truth_stamp - self.last_truth_heading_stamp
                )
                self.truth_motion_regime = (
                    'TURNING' if abs(yaw_rate) > 0.1 else 'STRAIGHT'
                )
            self.last_truth_heading = heading
            self.last_truth_heading_stamp = truth_stamp
        self._drain_visual_observations()
        for model in (
            'guidance',
            'kf',
            'shadow_bctra',
        ):
            for horizon in PREDICTION_HORIZONS:
                evaluation = self.prediction_tracker.evaluate(
                    model,
                    horizon,
                    truth_stamp,
                    self.latest_truth.position,
                )
                if evaluation is not None:
                    self.prediction_errors[(model, horizon)].append(
                        evaluation.error
                    )
                    self.latest_prediction_error[(model, horizon)] = (
                        evaluation.error
                    )

    def filtered_target_callback(self, message):
        if not message.valid:
            return
        stamp = _stamp_seconds(message.stamp) or self._now()
        try:
            position = _finite_tuple((
                message.position.x,
                message.position.y,
                message.position.z,
            ), 'filtered target position')
            velocity = _finite_tuple((
                message.velocity.x,
                message.velocity.y,
                message.velocity.z,
            ), 'filtered target velocity')
        except ValueError:
            return
        truth = self.truth_history.state_at(stamp)
        if truth is not None:
            self.vision_metrics.observe_kf(
                stamp=stamp,
                position=position,
                velocity=velocity,
                truth_position=truth.position,
                truth_velocity=truth.velocity,
                source_stamp=(
                    _stamp_seconds(message.source_stamp)
                    if _stamp_seconds(message.source_stamp) > 0.0
                    else None
                ),
            )
        predictions = {
            horizon: tuple(
                value + rate * horizon
                for value, rate in zip(position, velocity)
            )
            for horizon in PREDICTION_HORIZONS
        }
        self.prediction_tracker.add('kf', stamp, predictions)

    def visual_observation_callback(self, message):
        """Queue one event for truth alignment without feeding control."""
        if not self.visual_evaluation_enabled or self.writer is None:
            return
        self.pending_visual_observations.append((message, self._now()))
        self._drain_visual_observations()

    def _drain_visual_observations(self, force=False):
        if self.writer is None:
            return
        pending = deque(maxlen=self.pending_visual_observations.maxlen)
        while self.pending_visual_observations:
            message, callback_receipt = (
                self.pending_visual_observations.popleft()
            )
            measurement_stamp = _stamp_seconds(message.stamp)
            receipt_stamp = (
                _stamp_seconds(message.received_stamp) or callback_receipt
            )
            truth = (
                self.truth_history.state_at(measurement_stamp)
                if measurement_stamp > 0.0 else None
            )
            if (message.valid and truth is None and not force
                    and self._now() - callback_receipt < 0.5):
                pending.append((message, callback_receipt))
                continue
            # Evaluation references may arrive after the online observation.
            # Wait a bounded interval for a causal bracket, never extrapolate.
            heading_query = (
                self.gazebo_uav_tracker.query_at(measurement_stamp)
                if self.gazebo_uav_tracker is not None else None
            )
            entity_query = (
                self.gazebo_entity_tracker.query_at(measurement_stamp)
                if self.gazebo_entity_tracker is not None else None
            )
            if (message.valid and message.geometry_diagnostics_enabled
                    and not force and self._now() - callback_receipt < 0.15
                    and any(query is not None and query.status in (
                        'EMPTY', 'AFTER_HISTORY', 'QUERY_GAP',
                    ) for query in (heading_query, entity_query))):
                pending.append((message, callback_receipt))
                continue
            estimate = (
                float(message.position.x),
                float(message.position.y),
                float(message.position.z),
            )
            truth_position = (
                truth.position if truth is not None else (math.nan,) * 3
            )
            target_range = float(message.target_range)
            distance_bin = (
                'UNKNOWN'
                if not math.isfinite(target_range) or target_range < 0.0
                else 'NEAR'
                if target_range < 5.0
                else 'MID'
                if target_range < 10.0
                else 'FAR'
            )
            self.vision_metrics.observe_raw(
                source=message.source,
                measurement_stamp=measurement_stamp,
                receipt_stamp=receipt_stamp,
                estimate=estimate,
                truth=truth_position,
                valid=bool(message.valid),
                truth_available=truth is not None,
                distance_bin=distance_bin,
                motion_regime=self.truth_motion_regime,
                approach_phase=self.approach_phase,
                rejection_reason=message.rejection_reason,
            )
            error = tuple(
                estimate[index] - truth_position[index]
                for index in range(3)
            )
            entity_center = (
                self.gazebo_entity_tracker.position_at(measurement_stamp)
                if self.gazebo_entity_tracker is not None
                and measurement_stamp > 0.0
                else None
            )
            entity_reference = (
                (
                    entity_center[0],
                    entity_center[1],
                    entity_center[2] + self.gazebo_visual_height_offset,
                )
                if entity_center is not None else (math.nan,) * 3
            )
            vision_to_entity = tuple(
                estimate[index] - entity_reference[index]
                for index in range(3)
            )
            entity_to_truth = tuple(
                entity_reference[index] - truth_position[index]
                for index in range(3)
            )
            entity_tracker = self.gazebo_entity_tracker
            geometry_residuals = None
            if (
                entity_center is not None
                and bool(message.geometry_diagnostics_enabled)
            ):
                try:
                    geometry_residuals = entity_geometry_residuals(
                        entity_center_ned=entity_center,
                        uav_position_ned=(
                            message.interpolated_uav_x,
                            message.interpolated_uav_y,
                            message.interpolated_uav_z,
                        ),
                        attitude_quaternion=(
                            message.interpolated_attitude_w,
                            message.interpolated_attitude_x,
                            message.interpolated_attitude_y,
                            message.interpolated_attitude_z,
                        ),
                        camera_translation_flu=(
                            message.camera_translation_x,
                            message.camera_translation_y,
                            message.camera_translation_z,
                        ),
                        camera_pitch_down=message.camera_pitch_down,
                        intrinsics=(
                            message.camera_fx,
                            message.camera_fy,
                            message.camera_cx,
                            message.camera_cy,
                        ),
                        observed_projection_center=(
                            message.projection_centroid_u,
                            message.projection_centroid_v,
                        ),
                        observed_center_camera=(
                            message.center_camera_x,
                            message.center_camera_y,
                            message.center_camera_z,
                        ),
                        observed_body_flu=(
                            message.target_body_flu_x,
                            message.target_body_flu_y,
                            message.target_body_flu_z,
                        ),
                    )
                except ValueError:
                    geometry_residuals = None
            expected_projection = (
                geometry_residuals.expected_projection
                if geometry_residuals else (math.nan,) * 2
            )
            projection_error = (
                geometry_residuals.pixel_error
                if geometry_residuals else (math.nan,) * 2
            )
            expected_camera = (
                geometry_residuals.expected_camera
                if geometry_residuals else (math.nan,) * 3
            )
            camera_error = (
                geometry_residuals.camera_error
                if geometry_residuals else (math.nan,) * 3
            )
            expected_body = (
                geometry_residuals.expected_body_flu
                if geometry_residuals else (math.nan,) * 3
            )
            body_error = (
                geometry_residuals.body_error
                if geometry_residuals else (math.nan,) * 3
            )
            heading_record = heading_diagnostics(
                message,
                heading_query,
                error_ned=error, entity_center_ned=entity_center,
            )
            if heading_record['heading_diagnostics_status'] != 'VALID':
                heading_record.pop('px4_heading', None)
            if self.gazebo_uav_diagnostics_enabled and self.gazebo_uav_tracker is None:
                heading_record.update(
                    gazebo_uav_diagnostics_enabled=True,
                    heading_diagnostics_status='REFERENCE_UNAVAILABLE',
                )
            covariance = list(message.covariance)
            self.writer.append_visual_event({
                'measurement_stamp': measurement_stamp,
                'receipt_stamp': receipt_stamp,
                'processed_stamp': _stamp_seconds(message.processed_stamp),
                'published_stamp': _stamp_seconds(message.published_stamp),
                'source': str(message.source),
                'valid': bool(message.valid),
                'observation_valid': bool(message.valid),
                'truth_available': truth is not None,
                'rejection_reason': str(message.rejection_reason),
                'rgb_raw_stamp': float(message.rgb_raw_stamp),
                'depth_raw_stamp': float(message.depth_raw_stamp),
                'rgb_receipt_stamp': float(message.rgb_receipt_stamp),
                'depth_receipt_stamp': float(message.depth_receipt_stamp),
                'rgb_mapped_stamp': float(message.rgb_mapped_stamp),
                'depth_mapped_stamp': float(message.depth_mapped_stamp),
                'image_measurement_stamp': float(
                    message.image_measurement_stamp
                ),
                'rgb_depth_acquisition_skew': float(
                    message.rgb_depth_acquisition_skew
                ),
                'pose_history_start_stamp': float(
                    message.pose_history_start_stamp
                ),
                'pose_history_end_stamp': float(
                    message.pose_history_end_stamp
                ),
                'position_source_stamp': float(
                    message.position_source_stamp
                ),
                'position_mapped_stamp': float(
                    message.position_mapped_stamp
                ),
                'attitude_source_stamp': float(
                    message.attitude_source_stamp
                ),
                'attitude_mapped_stamp': float(
                    message.attitude_mapped_stamp
                ),
                'position_history_start_stamp': float(
                    message.position_history_start_stamp
                ),
                'position_history_end_stamp': float(
                    message.position_history_end_stamp
                ),
                'attitude_history_start_stamp': float(
                    message.attitude_history_start_stamp
                ),
                'attitude_history_end_stamp': float(
                    message.attitude_history_end_stamp
                ),
                'image_clock_offset': float(message.image_clock_offset),
                'image_clock_mapping_mode': str(
                    message.image_clock_mapping_mode
                ),
                'image_clock_status': str(message.image_clock_status),
                'image_clock_reset_count': int(
                    message.image_clock_reset_count
                ),
                'image_clock_anchor_sim_stamp': float(
                    message.image_clock_anchor_sim_stamp
                ),
                'image_clock_anchor_system_stamp': float(
                    message.image_clock_anchor_system_stamp
                ),
                'image_clock_reference_age': float(
                    message.image_clock_reference_age
                ),
                'image_clock_sync_quality': float(
                    message.image_clock_sync_quality
                ),
                'image_measurement_time_source': str(
                    message.image_measurement_time_source
                ),
                'px4_clock_offset': float(message.px4_clock_offset),
                'px4_clock_reset_count': int(
                    message.px4_clock_reset_count
                ),
                'px4_clock_calibration_count': int(
                    message.px4_clock_calibration_count
                ),
                'px4_clock_recalibration_count': int(
                    message.px4_clock_recalibration_count
                ),
                'px4_clock_status': str(message.px4_clock_status),
                'approach_phase': self.approach_phase,
                'distance_bin': distance_bin,
                'motion_regime': self.truth_motion_regime,
                'confidence': float(message.confidence),
                'red_pixel_count': int(message.red_pixel_count),
                'valid_depth_ratio': float(message.valid_depth_ratio),
                'target_range': target_range,
                **image_pose_diagnostics(message),
                **heading_record,
                'view_angle': float(message.view_angle),
                'geometry_diagnostics_enabled': bool(
                    message.geometry_diagnostics_enabled
                ),
                'mask_centroid_u': float(message.mask_centroid_u),
                'mask_centroid_v': float(message.mask_centroid_v),
                'projection_centroid_u': float(
                    message.projection_centroid_u
                ),
                'projection_centroid_v': float(
                    message.projection_centroid_v
                ),
                'mask_bbox_left': int(message.mask_bbox_left),
                'mask_bbox_top': int(message.mask_bbox_top),
                'mask_bbox_right': int(message.mask_bbox_right),
                'mask_bbox_bottom': int(message.mask_bbox_bottom),
                'valid_depth_count': int(message.valid_depth_count),
                'depth_min': float(message.depth_min),
                'depth_median': float(message.depth_median),
                'depth_mad': float(message.depth_mad),
                'camera_fx': float(message.camera_fx),
                'camera_fy': float(message.camera_fy),
                'camera_cx': float(message.camera_cx),
                'camera_cy': float(message.camera_cy),
                'camera_translation_x': float(
                    message.camera_translation_x
                ),
                'camera_translation_y': float(
                    message.camera_translation_y
                ),
                'camera_translation_z': float(
                    message.camera_translation_z
                ),
                'camera_pitch_down': float(message.camera_pitch_down),
                'surface_camera_x': float(message.surface_camera_x),
                'surface_camera_y': float(message.surface_camera_y),
                'surface_camera_z': float(message.surface_camera_z),
                'center_camera_x': float(message.center_camera_x),
                'center_camera_y': float(message.center_camera_y),
                'center_camera_z': float(message.center_camera_z),
                'target_body_flu_x': float(message.target_body_flu_x),
                'target_body_flu_y': float(message.target_body_flu_y),
                'target_body_flu_z': float(message.target_body_flu_z),
                'interpolated_uav_x': float(message.interpolated_uav_x),
                'interpolated_uav_y': float(message.interpolated_uav_y),
                'interpolated_uav_z': float(message.interpolated_uav_z),
                'interpolated_attitude_w': float(
                    message.interpolated_attitude_w
                ),
                'interpolated_attitude_x': float(
                    message.interpolated_attitude_x
                ),
                'interpolated_attitude_y': float(
                    message.interpolated_attitude_y
                ),
                'interpolated_attitude_z': float(
                    message.interpolated_attitude_z
                ),
                'target_reference_z_offset': float(
                    message.target_reference_z_offset
                ),
                'gazebo_entity_available': entity_center is not None,
                'gazebo_entity_clock_status': (
                    entity_tracker.last_status
                    if entity_tracker else 'DISABLED'
                ),
                'gazebo_entity_clock_reset_count': (
                    entity_tracker.reset_count if entity_tracker else 0
                ),
                'gazebo_entity_raw_stamp': (
                    entity_tracker.last_raw_stamp
                    if entity_tracker else math.nan
                ),
                'gazebo_entity_mapped_stamp': (
                    entity_tracker.last_mapped_stamp
                    if entity_tracker else math.nan
                ),
                'gazebo_entity_center_x': (
                    entity_center[0] if entity_center else math.nan
                ),
                'gazebo_entity_center_y': (
                    entity_center[1] if entity_center else math.nan
                ),
                'gazebo_entity_center_z': (
                    entity_center[2] if entity_center else math.nan
                ),
                'gazebo_entity_reference_x': entity_reference[0],
                'gazebo_entity_reference_y': entity_reference[1],
                'gazebo_entity_reference_z': entity_reference[2],
                'entity_expected_projection_u': expected_projection[0],
                'entity_expected_projection_v': expected_projection[1],
                'projection_error_u': projection_error[0],
                'projection_error_v': projection_error[1],
                'entity_expected_camera_x': expected_camera[0],
                'entity_expected_camera_y': expected_camera[1],
                'entity_expected_camera_z': expected_camera[2],
                'camera_center_error_x': camera_error[0],
                'camera_center_error_y': camera_error[1],
                'camera_center_error_z': camera_error[2],
                'camera_center_error_3d': math.sqrt(sum(
                    value * value for value in camera_error
                )),
                'entity_expected_body_flu_x': expected_body[0],
                'entity_expected_body_flu_y': expected_body[1],
                'entity_expected_body_flu_z': expected_body[2],
                'body_flu_error_x': body_error[0],
                'body_flu_error_y': body_error[1],
                'body_flu_error_z': body_error[2],
                'body_flu_error_3d': math.sqrt(sum(
                    value * value for value in body_error
                )),
                'vision_to_entity_x': vision_to_entity[0],
                'vision_to_entity_y': vision_to_entity[1],
                'vision_to_entity_z': vision_to_entity[2],
                'vision_to_entity_3d': math.sqrt(sum(
                    value * value for value in vision_to_entity
                )),
                'entity_to_truth_x': entity_to_truth[0],
                'entity_to_truth_y': entity_to_truth[1],
                'entity_to_truth_z': entity_to_truth[2],
                'entity_to_truth_3d': math.sqrt(sum(
                    value * value for value in entity_to_truth
                )),
                'position_x': estimate[0],
                'position_y': estimate[1],
                'position_z': estimate[2],
                'covariance_xx': covariance[0],
                'covariance_yy': covariance[4],
                'covariance_zz': covariance[8],
                'truth_x': truth_position[0],
                'truth_y': truth_position[1],
                'truth_z': truth_position[2],
                'error_x': error[0],
                'error_y': error[1],
                'error_z': error[2],
                'horizontal_error': math.hypot(error[0], error[1]),
                'position_3d_error': math.sqrt(sum(v * v for v in error)),
                'observation_age': (
                    receipt_stamp - measurement_stamp
                    if measurement_stamp > 0.0 else math.nan
                ),
            })
        self.pending_visual_observations = pending

    def _queue_prediction(
        self,
        model,
        message,
        seen_sequences,
    ):
        key = (
            int(message.mission_id),
            int(message.sequence_id),
        )
        if (
            not message.valid
            or key in seen_sequences
        ):
            return

        seen_sequences.add(key)
        try:
            predictions = {
                horizon: _prediction_at(
                    message,
                    horizon,
                )
                for horizon in PREDICTION_HORIZONS
                if (
                    horizon
                    <= float(message.prediction_horizon)
                    + 1e-9
                )
            }
        except (TypeError, ValueError):
            return

        self.prediction_tracker.add(
            model,
            _stamp_seconds(message.source_stamp),
            predictions,
        )

    def prediction_callback(self, message):
        if self.writer is not None and int(message.mission_id) != self.run_mission_id:
            return
        if message.valid:
            self.latest_prediction = message

        self._queue_prediction(
            'guidance',
            message,
            self.prediction_sequences,
        )

    def shadow_prediction_callback(self, message):
        if self.writer is not None and int(message.mission_id) != self.run_mission_id:
            return
        self._queue_prediction(
            'shadow_bctra',
            message,
            self.shadow_prediction_sequences,
        )

    @staticmethod
    def _failure_name(message):
        names = {
            value: name
            for name, value in PlannerDiagnostic.__dict__.items()
            if name.isupper() and isinstance(value, int)
        }
        return names.get(
            int(message.failure_reason),
            str(message.failure_reason),
        )

    def planner_callback(self, message):
        if self.writer is not None and int(message.mission_id) != self.run_mission_id:
            return
        self.latest_planner_diagnostic = message
        is_new = self.event_metrics.observe_planner(
            mission_id=message.mission_id,
            plan_id=message.plan_id,
            success=message.result == PlannerDiagnostic.RESULT_SUCCESS,
            failure_reason=self._failure_name(message),
            compute_time=message.compute_time,
            generation_time=message.generation_time,
            completion_stamp=_stamp_seconds(message.generated_stamp),
            reachability_time=message.reachability_time,
            validation_time=message.validation_time,
            input_age_at_publish=message.input_age_at_publish,
            completion_to_publish_delay=(
                message.completion_to_publish_delay
            ),
            admission_wait=(
                message.result == PlannerDiagnostic.RESULT_IDLE
            ),
        )
        if is_new and self.writer is not None:
            self.writer.append_detail_event(
                'planner',
                f'{int(message.mission_id)}:{int(message.plan_id)}',
                {
                    'result': int(message.result),
                    'failure_reason': self._failure_name(message),
                    'failure_detail': message.failure_detail,
                    'rejection_stage': message.rejection_stage,
                    'rejection_detail': message.rejection_detail,
                    'planning_cycle_id': int(message.planning_cycle_id),
                    'prediction_sequence_id': int(
                        message.prediction_sequence_id
                    ),
                    'candidate_diagnostics': message.candidate_diagnostics,
                    'candidate_count': int(message.candidate_count),
                    'contact_recovery_reason': (
                        message.contact_recovery_reason
                    ),
                    'contact_delay': float(message.contact_delay),
                    'target_prediction_shift': float(
                        message.target_prediction_shift
                    ),
                    'candidate_published': bool(message.candidate_published),
                    'required_time': float(message.required_time),
                    'horizontal_min_time': float(
                        message.horizontal_min_time
                    ),
                    'vertical_min_time': float(message.vertical_min_time),
                    'sea_safe_min_time': float(message.sea_safe_min_time),
                    'search_min_time': float(message.search_min_time),
                    'search_max_time': float(message.search_max_time),
                    'available_prediction_duration': float(
                        message.available_prediction_duration
                    ),
                    'locked_remaining_t_go': float(
                        message.locked_remaining_t_go
                    ),
                    'terminal_admission': bool(message.terminal_admission),
                    'terminal_admission_reason': (
                        message.terminal_admission_reason
                    ),
                    'compute_time': float(message.compute_time),
                    'reachability_time': float(message.reachability_time),
                    'generation_time': float(message.generation_time),
                    'validation_time': float(message.validation_time),
                    'optimization_time': float(message.optimization_time),
                    'input_age_at_start': float(
                        message.input_age_at_start
                    ),
                    'input_age_at_finish': float(
                        message.input_age_at_finish
                    ),
                    'input_age_at_publish': float(
                        message.input_age_at_publish
                    ),
                    'completion_to_publish_delay': float(
                        message.completion_to_publish_delay
                    ),
                },
            )

    def _fresh_controller(self, now):
        """Expose a cached diagnostic only within its decision-age limit."""
        message = self.latest_controller
        if message is None:
            return None
        if (int(message.mission_id) != self.run_mission_id
                or not 0.0 <= now - _stamp_seconds(message.stamp) <= 0.125):
            return None
        return message

    def controller_callback(self, message):
        if (self.writer is not None
                and int(message.mission_id) != self.run_mission_id):
            return
        age = self._now() - _stamp_seconds(message.stamp)
        if not 0.0 <= age <= 0.125:
            self.latest_controller = None
            return
        if (self.latest_controller is not None
                and _stamp_seconds(message.stamp)
                < _stamp_seconds(self.latest_controller.stamp)):
            return
        self.tracker_rate.observe(self._now())
        self.latest_controller = message
        if self.writer is not None:
            stamp = _stamp_seconds(message.stamp)
            if 0.0 <= self._now() - stamp <= 0.125:
                self.run_metrics.observe(
                    self._now(), str(self.latest_mission.state_name),
                    visible=bool(message.target_visible),
                    locked=bool(message.target_locked),
                    control_status=str(message.status),
                )
        self.event_metrics.observe_controller(
            message.mission_id,
            message.plan_id,
            message.status,
            message.rejection_reason,
        )
        if message.status == 'PLAN_REJECTED':
            self.latest_tracker_rejection_reason = (
                str(message.rejection_reason)
            )
            if str(message.rejection_reason).startswith('REFERENCE_'):
                self.handover_jump_count += 1
        safety_state = str(message.safety_state)
        if (
            safety_state in self.safety_event_histogram
            and safety_state != self.last_safety_state
        ):
            self.safety_event_histogram[safety_state] += 1
        self.last_safety_state = safety_state
        if (
            self.writer is not None
            and message.status in (
                'PLAN_PENDING', 'PLAN_ACCEPTED', 'PLAN_REJECTED'
            )
        ):
            self.writer.append_detail_event(
                'tracker',
                f'{int(message.mission_id)}:{int(message.attempted_plan_id)}',
                {
                    'status': str(message.status),
                    'active_plan_id': int(message.plan_id),
                    'rejection_reason': str(message.rejection_reason),
                    'rejection_subreason': str(
                        message.rejection_subreason
                    ),
                    'trajectory_start_stamp': _stamp_seconds(
                        message.trajectory_start_stamp
                    ),
                    'planner_published_stamp': _stamp_seconds(
                        message.planner_published_stamp
                    ),
                    'tracker_receipt_stamp': _stamp_seconds(
                        message.tracker_receipt_stamp
                    ),
                    'tracker_state_stamp': _stamp_seconds(
                        message.tracker_state_stamp
                    ),
                    'source_age': float(message.source_age),
                    'state_age_at_receipt': float(
                        message.state_age_at_receipt
                    ),
                    'remaining_valid_time': float(
                        message.remaining_valid_time
                    ),
                    'trajectory_replaced': bool(message.trajectory_replaced),
                    'handover_position_error': float(
                        message.handover_position_error
                    ),
                    'handover_velocity_error': float(
                        message.handover_velocity_error
                    ),
                    'handover_acceleration_error': float(
                        message.handover_acceleration_error
                    ),
                },
            )
        self.controller_compute_times.append(
            max(float(message.callback_compute_time), 0.0)
        )

    def _performance_callback(self, name, message):
        value = float(message.data)
        if math.isfinite(value) and value >= 0.0:
            self.performance_values[str(name)] = value

    def _config_snapshot(self):
        return {
            'evaluation_capture_radius': self.evaluation_capture_radius,
            'planned_capture_radius': self.planned_capture_radius,
            'sea_surface_z': self.evaluator.sea_surface_z,
            'enable_sea_contact_failure': (
                self.evaluator.enable_sea_contact_failure
            ),
            # Body-water contact is tested body_lower_extent below the PX4
            # reference point, independently of any control-barrier state.
            'body_lower_extent': self.evaluator.body_lower_extent,
            'body_contact_z': self.evaluator.body_contact_z,
            'maximum_duration': self.evaluator.maximum_duration,
            'truth_topic': self.truth_topic,
            'truth_role': self.truth_role,
            'shadow_prediction_topic': (
                self.shadow_prediction_topic
            ),
            'detailed_diagnostics_enabled': (
                self.detailed_diagnostics_enabled
            ),
            'visual_evaluation_enabled': self.visual_evaluation_enabled,
            'visual_observation_topic': self.visual_observation_topic,
            'gazebo_entity_diagnostics_enabled': (
                self.gazebo_entity_diagnostics_enabled
            ),
            'gazebo_uav_diagnostics_enabled': self.gazebo_uav_diagnostics_enabled,
            'gazebo_uav_entity_name': self.gazebo_uav_entity_name,
            'gazebo_target_entity_name': self.gazebo_target_entity_name,
            'gazebo_visual_height_offset': self.gazebo_visual_height_offset,
        }

    def _start_run_artifacts(self, mission_id, now):
        """Start run logging on accepted takeoff, before interception."""
        if self.writer is not None:
            self._finalize_run('MISSION_RESET', now)
        self.evaluator.reset()
        self.run_started_at = float(now)
        self.run_mission_id = int(mission_id)
        self.intercept_started_at = None
        self.intercept_result = None
        self.run_metrics = RunMetricAccumulator(now)
        self.last_run_sample_stamp = None
        self.event_metrics = PlannerEventAccumulator()
        self.prediction_tracker.reset()
        self.vision_metrics = VisionMetricAccumulator()
        self.pending_visual_observations.clear()
        self.prediction_sequences.clear()
        self.shadow_prediction_sequences.clear()
        for values in self.prediction_errors.values():
            values.clear()
        self.latest_prediction_error.clear()
        self.controller_compute_times.clear()
        self.uav_history.clear()
        self.truth_history.clear()
        self.last_synchronized_stamp = None
        self.tracker_rate = RateMeter(window_seconds=1.0)
        self.runtime_performance = RuntimePerformanceAccumulator()
        self.terminal_pause_result = None
        self.latest_tracker_rejection_reason = ''
        self.latest_planner_diagnostic = None
        self.latest_prediction = None
        self.latest_controller = None
        self.approach_phase = 'PREPARATION'
        self.terminal_approach_count = 0
        self.recovery_count = 0
        self.handover_jump_count = 0
        self.first_terminal_approach_result = ''
        self.minimum_body_clearance = math.inf
        self.truth_motion_regime = 'UNKNOWN'
        self.last_truth_heading = None
        self.last_truth_heading_stamp = None
        self.safety_event_histogram = {
            name: 0
            for name in ('WARNING', 'BRAKE', 'UNRECOVERABLE', 'SEA_CONTACT')
        }
        self.last_safety_state = ''
        self.writer = ExperimentArtifactWriter(
            self.log_directory,
            mission_id,
            self._config_snapshot(),
            detailed_diagnostics_enabled=(
                self.detailed_diagnostics_enabled
            ),
            visual_evaluation_enabled=self.visual_evaluation_enabled,
        )
        self.result_published = False

    def _start_intercept_evaluation(self, mission_id, now):
        """Start the capture/timeout interval only after accepted Y."""
        if (self.writer is None or self.run_mission_id != int(mission_id)
                or self.intercept_started_at is not None):
            return False
        self.intercept_started_at = float(now)
        self.evaluator.begin(mission_id, now)
        self.last_synchronized_stamp = None
        self.result_published = False
        return True

    def mission_callback(self, message):
        previous_phase = (
            self.latest_mission.state_name if self.latest_mission else ''
        )
        now = self._now()
        state_name = str(message.state_name)
        mission_id = int(message.mission_id)
        if ((self.run_mission_id is not None and mission_id < self.run_mission_id)
                or (self.latest_mission is not None
                    and mission_id < int(self.latest_mission.mission_id))):
            return
        if self.writer is not None and mission_id != self.run_mission_id:
            if mission_id < self.run_mission_id:
                return
            self._finalize_run('MISSION_RESET', now)
        self.latest_mission = message
        if (state_name == 'TAKEOFF' and self.writer is None
                and mission_id != self.run_mission_id):
            self._start_run_artifacts(mission_id, now)
        if self.writer is not None and mission_id == self.run_mission_id:
            if message.intercept_requested:
                self._start_intercept_evaluation(mission_id, now)
            self.run_metrics.observe(now, state_name)
            if previous_phase != state_name:
                self._write_run_sample(now, transition=True)
            if message.completed or state_name in ('ABORTED', 'CAPTURE', 'FAILURE'):
                self._finalize_run(state_name, now)
        terminal_states = {
            'MINCO_READY', 'MINCO_TRACKING', 'TERMINAL_MINCO'
        }
        recovery_states = {'PLAN_RECOVERY', 'SAFE_WAIT', 'SAFE_RECOVERY', 'REACQUIRE'}
        if state_name in terminal_states:
            self.approach_phase = 'TERMINAL_APPROACH'
            if previous_phase not in terminal_states:
                self.terminal_approach_count += 1
        elif state_name in recovery_states:
            self.approach_phase = 'RECOVERY'
            if previous_phase not in recovery_states:
                self.recovery_count += 1
                if (
                    self.terminal_approach_count == 1
                    and not self.first_terminal_approach_result
                ):
                    self.first_terminal_approach_result = 'RECOVERY'
        elif state_name == 'FAR_GUIDANCE':
            self.approach_phase = 'PREPARATION'
        elif state_name in ('CAPTURE', 'FAILURE', 'ABORTED'):
            self.approach_phase = state_name

    def _sample_row(self, now, metrics_available=None):
        unavailable = KinematicState((math.nan,) * 3, (math.nan,) * 3)
        uav = self.latest_uav or unavailable
        truth = self.latest_truth or unavailable
        if metrics_available is None:
            metrics_available = self.latest_uav is not None and self.latest_truth is not None
        metrics = self.evaluator.instantaneous_metrics(
            uav, truth,
        ) if metrics_available else (math.nan,) * 6
        controller = self._fresh_controller(now)
        planner = self.latest_planner_diagnostic
        body_clearance = (
            self.evaluator.body_contact_z - uav.position[2]
        )
        self.minimum_body_clearance = min(
            self.minimum_body_clearance,
            body_clearance,
        )
        return {
            'time': now - self.run_started_at,
            'run_elapsed_time': now - self.run_started_at,
            'intercept_elapsed_time': (
                now - self.intercept_started_at
                if self.intercept_started_at is not None else ''
            ),
            'intercept_started': self.intercept_started_at is not None,
            'truth_available': bool(metrics_available),
            'uav_available': self.latest_uav is not None,
            'mission_id': self.run_mission_id,
            'phase': (
                self.latest_mission.state_name if self.latest_mission else ''
            ),
            'uav_x': uav.position[0],
            'uav_y': uav.position[1],
            'uav_z': uav.position[2],
            'uav_vx': uav.velocity[0],
            'uav_vy': uav.velocity[1],
            'uav_vz': uav.velocity[2],
            'target_x': truth.position[0],
            'target_y': truth.position[1],
            'target_z': truth.position[2],
            'target_vx': truth.velocity[0],
            'target_vy': truth.velocity[1],
            'target_vz': truth.velocity[2],
            'distance': metrics[0],
            'horizontal_distance': metrics[1],
            'vertical_error': metrics[2],
            'relative_speed': metrics[3],
            'closing_speed': metrics[4],
            'approach_phase': self.approach_phase,
            'first_terminal_approach': self.terminal_approach_count == 1,
            'controller_status': controller.status if controller else '',
            'bearing_approach_active': bool(
                controller and controller.status == 'BEARING_APPROACH'),
            'bearing_age_at_control': (
                float(controller.source_age)
                if controller and controller.status == 'BEARING_APPROACH' else math.inf),
            **{
                name: getattr(controller, name, default)
                for name, default in (
                    ('target_visible', False), ('target_locked', False),
                    ('search_state', ''), ('search_direction', 0),
                    ('last_valid_observation_age', math.inf),
                    ('last_valid_image_bearing', math.nan),
                    ('search_yaw_rate_command', 0.0),
                    ('consecutive_valid_frames', 0),
                    ('consecutive_lost_frames', 0),
                    ('kf_state_age', math.inf),
                    ('prediction_sample_age', math.inf),
                    ('planner_source_age', math.inf), ('yaw_owner', ''),
                )
            },
            'tracker_rejection_reason': (
                self.latest_tracker_rejection_reason
                if controller and controller.status == 'PLAN_REJECTED'
                else ''
            ),
            'plan_id': controller.plan_id if controller else 0,
            'planner_event_id': (
                f'{int(planner.mission_id)}:{int(planner.plan_id)}'
                if planner else ''
            ),
            'planner_result': int(planner.result) if planner else 0,
            'attempted_plan_id': (
                controller.attempted_plan_id if controller else 0
            ),
            'plan_prediction_sequence_id': (
                controller.prediction_sequence_id if controller else 0
            ),
            'latest_prediction_sequence_id': (
                self.latest_prediction.sequence_id
                if self.latest_prediction else 0
            ),
            'planner_failure_reason': (
                self._failure_name(self.latest_planner_diagnostic)
                if self.latest_planner_diagnostic else ''
            ),
            'terminal_admission': (
                planner.terminal_admission if planner else False
            ),
            'terminal_admission_reason': (
                planner.terminal_admission_reason if planner else ''
            ),
            'planner_failure_detail': (
                self.latest_planner_diagnostic.failure_detail
                if self.latest_planner_diagnostic else ''
            ),
            'planner_planning_cycle_id': (
                self.latest_planner_diagnostic.planning_cycle_id
                if self.latest_planner_diagnostic else 0
            ),
            'planner_rejection_stage': (
                self.latest_planner_diagnostic.rejection_stage
                if self.latest_planner_diagnostic else ''
            ),
            'planner_rejection_detail': (
                self.latest_planner_diagnostic.rejection_detail
                if self.latest_planner_diagnostic else ''
            ),
            'planner_contact_recovery_reason': (
                self.latest_planner_diagnostic.contact_recovery_reason
                if self.latest_planner_diagnostic else ''
            ),
            'planner_contact_delay': (
                self.latest_planner_diagnostic.contact_delay
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_target_prediction_shift': (
                self.latest_planner_diagnostic.target_prediction_shift
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_candidate_published': (
                self.latest_planner_diagnostic.candidate_published
                if self.latest_planner_diagnostic else False
            ),
            'planner_required_time': (
                self.latest_planner_diagnostic.required_time
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_horizontal_min_time': (
                self.latest_planner_diagnostic.horizontal_min_time
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_vertical_min_time': (
                self.latest_planner_diagnostic.vertical_min_time
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_sea_safe_min_time': (
                self.latest_planner_diagnostic.sea_safe_min_time
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_search_min_time': (
                self.latest_planner_diagnostic.search_min_time
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_search_max_time': (
                self.latest_planner_diagnostic.search_max_time
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_available_prediction_duration': (
                self.latest_planner_diagnostic
                .available_prediction_duration
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_locked_remaining_t_go': (
                self.latest_planner_diagnostic.locked_remaining_t_go
                if self.latest_planner_diagnostic else math.nan
            ),
            'planner_reachability_time': (
                self.latest_planner_diagnostic.reachability_time
                if self.latest_planner_diagnostic else 0.0
            ),
            'planner_generation_time': (
                self.latest_planner_diagnostic.generation_time
                if self.latest_planner_diagnostic else 0.0
            ),
            'planner_validation_time': (
                self.latest_planner_diagnostic.validation_time
                if self.latest_planner_diagnostic else 0.0
            ),
            'planner_candidate_diagnostics': (
                self.latest_planner_diagnostic.candidate_diagnostics
                if self.latest_planner_diagnostic else '[]'
            ),
            'planner_source_age_at_publish': (
                self.latest_planner_diagnostic.input_age_at_publish
                if self.latest_planner_diagnostic else 0.0
            ),
            'planner_completion_to_publish_delay': (
                self.latest_planner_diagnostic.completion_to_publish_delay
                if self.latest_planner_diagnostic else 0.0
            ),
            'selected_t_go': controller.selected_t_go if controller else 0.0,
            'contact_stamp': (
                _stamp_seconds(controller.contact_stamp) if controller else 0.0
            ),
            'remaining_t_go': controller.remaining_t_go if controller else 0.0,
            'terminal_mode': controller.terminal_mode if controller else False,
            'planned_capture_margin': (
                controller.planned_capture_margin if controller else 0.0
            ),
            'target_yaw': controller.target_yaw if controller else math.nan,
            'prediction_age': (
                controller.prediction_age if controller else 0.0
            ),
            'trajectory_age': (
                controller.trajectory_age if controller else 0.0
            ),
            'sea_safety_state': (
                controller.safety_state if controller else ''
            ),
            'body_clearance': body_clearance,
            'sea_safety_margin': (
                controller.safety_margin if controller else 0.0
            ),
            'prediction_0p5_error': self.latest_prediction_error.get(
                ('guidance', 0.5),
                '',
            ),
            'prediction_1p0_error': self.latest_prediction_error.get(
                ('guidance', 1.0),
                '',
            ),
            'prediction_2p0_error': self.latest_prediction_error.get(
                ('guidance', 2.0),
                '',
            ),
            'kf_prediction_0p5_error': (
                self.latest_prediction_error.get(
                    ('kf', 0.5),
                    '',
                )
            ),
            'kf_prediction_1p0_error': (
                self.latest_prediction_error.get(
                    ('kf', 1.0),
                    '',
                )
            ),
            'kf_prediction_2p0_error': (
                self.latest_prediction_error.get(
                    ('kf', 2.0),
                    '',
                )
            ),
            'shadow_bctra_prediction_0p5_error': (
                self.latest_prediction_error.get(
                    ('shadow_bctra', 0.5),
                    '',
                )
            ),
            'shadow_bctra_prediction_1p0_error': (
                self.latest_prediction_error.get(
                    ('shadow_bctra', 1.0),
                    '',
                )
            ),
            'shadow_bctra_prediction_2p0_error': (
                self.latest_prediction_error.get(
                    ('shadow_bctra', 2.0),
                    '',
                )
            ),
            **self.performance_values,
            'tracker_hz': self.tracker_rate.rate(now),
            'tracker_callback_time': (
                controller.callback_compute_time if controller else 0.0
            ),
            'planner_compute_time': (
                self.latest_planner_diagnostic.compute_time
                if self.latest_planner_diagnostic else 0.0
            ),
        }

    def _prediction_summary(self):
        summary = {}
        for (model, horizon), values in self.prediction_errors.items():
            label = str(horizon).replace('.', 'p')
            key = f'{model}_prediction_{label}'
            ordered = sorted(values)
            p95_index = max(math.ceil(0.95 * len(ordered)) - 1, 0)
            summary[key] = {
                'count': len(values),
                'rmse': (
                    math.sqrt(
                        sum(value * value for value in values) / len(values)
                    )
                    if values else 0.0
                ),
                'p95': ordered[p95_index] if ordered else 0.0,
            }
        return summary

    def _summary(self, result):
        if (
            self.terminal_approach_count == 1
            and not self.first_terminal_approach_result
        ):
            self.first_terminal_approach_result = result.outcome
        if result.reason == 'SEA_CONTACT':
            self.safety_event_histogram['SEA_CONTACT'] += 1
        summary = {
            'outcome': result.outcome,
            'failure_reason': '' if result.success else result.reason,
            'elapsed_time': result.elapsed_time,
            'evaluation_capture_radius': self.evaluation_capture_radius,
            'planned_capture_radius': self.planned_capture_radius,
            'minimum_distance': result.minimum_distance,
            'minimum_body_clearance': (
                self.minimum_body_clearance
                if math.isfinite(self.minimum_body_clearance) else 0.0
            ),
            'terminal_approach_count': self.terminal_approach_count,
            'recovery_count': self.recovery_count,
            'handover_jump_count': self.handover_jump_count,
            'first_terminal_approach_result': (
                self.first_terminal_approach_result
            ),
            'first_terminal_approach_hit': bool(
                result.success
                and self.terminal_approach_count == 1
                and self.recovery_count == 0
            ),
            'safety_event_histogram': dict(self.safety_event_histogram),
            'horizontal_distance': result.horizontal_distance,
            'vertical_error': result.vertical_error,
            'relative_speed': result.relative_speed,
            'closing_speed': result.closing_speed,
            'max_horizontal_speed': result.maximum_horizontal_speed,
            'max_vertical_speed': result.maximum_vertical_speed,
            'max_horizontal_acceleration': (
                result.maximum_horizontal_acceleration
            ),
            'max_vertical_acceleration': result.maximum_vertical_acceleration,
            'truth_topic': self.truth_topic,
            'truth_role': self.truth_role,
            'controller_callback_p95': 0.0,
        }
        if self.terminal_pause_result is not None:
            summary.update({
                'terminal_event': self.terminal_pause_result.terminal_event,
                'gazebo_pause_requested': (
                    self.terminal_pause_result.gazebo_pause_requested
                ),
                'gazebo_pause_succeeded': (
                    self.terminal_pause_result.gazebo_pause_succeeded
                ),
                'gazebo_pause_attempts': self.terminal_pause_result.attempts,
            })
        if self.controller_compute_times:
            ordered = sorted(self.controller_compute_times)
            summary['controller_callback_p95'] = ordered[
                max(math.ceil(0.95 * len(ordered)) - 1, 0)
            ]
            summary['controller_callback_p50'] = statistics.median(ordered)
            summary['controller_callback_max'] = ordered[-1]
        else:
            summary['controller_callback_p50'] = 0.0
            summary['controller_callback_max'] = 0.0
        summary.update(self.event_metrics.summary(result.elapsed_time))
        summary['vision'] = self.vision_metrics.summary()
        return summary

    def _write_run_sample(self, now, transition=False, synchronized=None):
        """Write phase events and periodic rows even without target truth."""
        if self.writer is None:
            return
        if (not transition and self.last_run_sample_stamp is not None
                and now <= self.last_run_sample_stamp):
            return
        original_uav, original_truth = self.latest_uav, self.latest_truth
        if synchronized is not None:
            _, self.latest_uav, self.latest_truth = synchronized
        sample = self._sample_row(now, metrics_available=synchronized is not None)
        sample['sample_kind'] = 'PHASE_TRANSITION' if transition else 'PERIODIC'
        self.writer.append_sample(sample)
        # Distance statistics use periodic rows, not duplicated phase events.
        self.run_metrics.observe(
            now, sample['phase'],
            position=self.latest_uav.position if self.latest_uav else None,
            estimated_distance=(
                getattr(self._fresh_controller(now), 'target_distance', None)
                if not transition else None
            ),
            evaluation_distance=sample['distance'] if not transition else None,
            control_status=sample['controller_status'],
        )
        self.runtime_performance.observe(sample['distance'], {
            **self.performance_values,
            'tracker_hz': sample['tracker_hz'],
            'tracker_callback_time': sample['tracker_callback_time'],
            'planner_compute_time': sample['planner_compute_time'],
        })
        self.latest_uav, self.latest_truth = original_uav, original_truth
        self.last_run_sample_stamp = now

    def _finalize_run(self, reason, now=None):
        """Close one run once on reset/end/shutdown, including X-only runs."""
        if self.writer is None:
            return self.last_run_paths
        now = self._now() if now is None else float(now)
        self._drain_visual_observations(force=True)
        result = self.intercept_result
        summary = self._summary(result) if result is not None else {
            'outcome': 'ABORTED', 'failure_reason': str(reason),
            'elapsed_time': (
                now - self.intercept_started_at
                if self.intercept_started_at is not None else 0.0
            ),
            'vision': self.vision_metrics.summary(),
            **self.event_metrics.summary(0.0),
        }
        summary.update({
            'run_started_at': self.run_started_at,
            'intercept_started_at': self.intercept_started_at,
            'intercept_started': self.intercept_started_at is not None,
            'run_elapsed_time': max(now - self.run_started_at, 0.0),
            'intercept_elapsed_time': (
                max(now - self.intercept_started_at, 0.0)
                if self.intercept_started_at is not None else None
            ),
            'run_end_reason': str(reason),
            'truth_role': 'evaluation_only',
            'run_metrics': self.run_metrics.summary(now),
            'prediction_errors': self._prediction_summary(),
            'runtime_performance_by_distance': self.runtime_performance.summary(),
        })
        self.last_run_summary = summary
        self.last_run_paths = self.writer.finalize(summary)
        self.writer = None
        self.get_logger().info(
            f'Run finalized: {reason} | artifacts={self.last_run_paths.csv_path}'
        )
        return self.last_run_paths

    def timer_callback(self):
        if self.writer is None:
            return
        run_now = self._now()
        self._drain_visual_observations()
        synchronized = synchronize_histories(
            self.uav_history,
            self.truth_history,
        )
        if (synchronized is not None
                and not 0.0 <= run_now - synchronized[0] <= 0.25):
            synchronized = None
        self._write_run_sample(run_now, synchronized=synchronized)
        if self.evaluator.started_at is None or self.result_published:
            return
        if synchronized is None:
            return
        now, uav, truth = synchronized
        if now < self.intercept_started_at:
            return
        if (
            self.last_synchronized_stamp is not None
            and now <= self.last_synchronized_stamp + 1e-9
        ):
            return
        self.last_synchronized_stamp = now
        result = self.evaluator.update(
            now,
            uav,
            truth,
        )
        if result is None:
            return
        self.result_pub.publish(
            result_to_message(result, now, self.evaluation_capture_radius)
        )
        hit = Bool()
        hit.data = bool(result.success)
        self.hit_pub.publish(hit)
        from .gazebo_terminal import GazeboPauseResult
        self.terminal_pause_result = GazeboPauseResult(
            terminal_event=result.reason,
            gazebo_pause_requested=False,
            gazebo_pause_succeeded=False,
            attempts=0,
        )
        if self.writer is not None:
            self.writer.append_detail_event(
                'run_detail',
                f'{self.evaluator.mission_id}:final',
                {
                    'prediction_errors': self._prediction_summary(),
                    'runtime_performance_by_distance': (
                        self.runtime_performance.summary()
                    ),
                },
            )
            self.get_logger().info(
                f'{result.outcome}: {result.reason} | '
                f'minimum distance={result.minimum_distance:.3f} m | '
                f'run artifacts: {self.writer.paths.csv_path}'
            )
        self.intercept_result = result
        self.result_published = True
        if result.success and getattr(self, 'pause_gazebo_on_success', False):
            # Only end the simulated experiment. No evaluation state or hit
            # event is fed into the online mission/planner/controller graph.
            if self.gazebo_pauser is not None:
                self.terminal_pause_result = self.gazebo_pauser.pause(result.reason)
            else:
                self.terminal_pause_result = GazeboPauseResult(
                    result.reason, True, False, 0)
            paused = self.terminal_pause_result.gazebo_pause_succeeded
            if paused:
                self.get_logger().info('Capture complete | Gazebo world paused')
            else:
                self.get_logger().warn(
                    'Capture complete | Gazebo pause FAILED; inspect world control service')
            self._finalize_run(result.reason, run_now)


def main(args=None):
    rclpy.init(args=args)
    node = InterceptEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._finalize_run('NODE_SHUTDOWN')
        node.destroy_node()
        rclpy.shutdown()
