#!/usr/bin/env python3
"""Observe actual stage B/C/D, issue only task-authorized X/Y, never flight setpoints."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, String
from uav_usv_interfaces.msg import (ControllerDiagnostic, InterceptResult, NativeAttitude,
                                    TargetImageDetection, TargetImageState, TargetState, UavState)

from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_phase import phase_qos
from uav_control.common.p76_readiness import source_fingerprint, sha256
from uav_control.common.p76_native import NativePoseHistory


def message_dict(value):
    """Retain all source stamps and fields in observed ROS messages."""
    if hasattr(value, 'get_fields_and_field_types'):
        return {key: message_dict(getattr(value, key))
                for key in value.get_fields_and_field_types()}
    if hasattr(value, 'tolist'):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return [message_dict(x) for x in value]
    return value


def stamp(value):
    """Read the source epoch, retaining its original ROS meaning."""
    return value.sec+value.nanosec*1e-9


def bodyrate_output_fresh(now, runtime, px4_mode, phase, owner):
    """Require fresh sole-outlet publication and native mode, using their distinct age bounds."""
    runtime, px4_mode = runtime or {}, px4_mode or {}
    output = runtime.get('output') or {}
    flags = px4_mode.get('flags', {})
    try:
        rates = np.asarray(output.get('rates'), dtype=float)
        if not (runtime.get('schema_version') == 2
                and runtime.get('phase') == phase
                and runtime.get('guidance_owner') == owner
                and runtime.get('px4_interface') == 'BODY_RATE'
                and 0 <= now-float(runtime.get('stamp', 0)) <= .125
                and 0 <= now-float(output.get('stamp', 0)) <= .125
                and rates.shape == (3,) and np.isfinite(rates).all()
                and -1 <= float(output.get('thrust', 0)) < 0
                and 0 <= now-float(px4_mode.get('stamp', 0)) <= .65
                and flags.get('offboard') is True and flags.get('rates') is True
                and all(flags.get(key) is False for key in (
                    'position', 'velocity', 'attitude', 'altitude', 'climb_rate'))):
            return False
    except (TypeError, ValueError):
        return False
    return True


def follow_authority_fresh(now, diagnostic, target, acks, receipts, mode,
                           runtime=None, px4_mode=None):
    """Require fresh actual Body-rate output; a supervisor diagnostic cannot prove ownership."""
    if not bodyrate_output_fresh(now, runtime, px4_mode, 'FOLLOW_ACTIVE', 'MINCO_FOLLOW'):
        return False
    age = now-stamp(diagnostic.stamp)
    if not (0 <= age <= .125 and all(0 <= now-t <= .125 for t in receipts)
            and diagnostic.status == 'MINCO_FOLLOW' and diagnostic.plan_id > 0
            and diagnostic.remaining_time > age
            and 0 <= diagnostic.source_age+age <= .125
            and 0 <= diagnostic.last_valid_observation_age+age <= .125
            and target.valid and 0 <= now-stamp(target.stamp) <= .125
            and 0 <= now-stamp(target.source_stamp) <= .125
            and mode.get('phase') == 'FOLLOW_ACTIVE'):
        return False
    for name, authorized in (('spatial', True), ('image', False)):
        ack = acks.get(name, {})
        if not (ack.get('schema_version') == 2 and ack.get('active') is True
                and ack.get('ready') is True
                and ack.get('control_authorized') is authorized
                and ack.get('epoch') == mode.get('epoch')
                and ack.get('phase') == 'FOLLOW_ACTIVE'
                and 0 <= now-ack.get('source_stamp', 0) <= .125
                and 0 <= now-ack.get('stamp', 0) <= .125):
            return False
    return True


def intercept_authority_fresh(now, acks, mode, runtime, px4_mode):
    """After Y inspect estimator ACKs and actual image guidance, never the 3D target state."""
    if (mode.get('phase') != 'PAPER_INTERCEPT_ACTIVE'
            or not bodyrate_output_fresh(
                now, runtime, px4_mode, 'PAPER_INTERCEPT_ACTIVE', 'PAPER')):
        return False
    for name, authorized in (('spatial', False), ('image', True)):
        ack = acks.get(name, {})
        if not (ack.get('schema_version') == 2 and ack.get('active') is True
                and ack.get('control_authorized') is authorized
                and ack.get('phase') == mode['phase'] and ack.get('epoch') == mode.get('epoch')
                and 0 <= now-ack.get('stamp', 0) <= .125):
            return False
    image = acks['image']
    return bool(image.get('ready') is True and 0 <= now-image.get('source_stamp', 0) <= .125)


def bodyrate_graph_qualified(graph):
    """Require the sole named Tracker outlet and no velocity-setpoint publisher."""
    return bool(graph.get('/perception/target_detection') == ['yolo_target_detector']
                and len(graph.get('/tracking/target_state', [])) == 1
                and graph.get('/fmu/in/offboard_control_mode') == ['trajectory_tracker_node']
                and graph.get('/fmu/in/vehicle_rates_setpoint') == ['trajectory_tracker_node']
                and graph.get('/fmu/in/trajectory_setpoint') == [])


class BPhysicalAcceptance:
    """Independent native acceptance, never an assumption that commands were executed.

    The 0.15 rad/s per-axis RMSE limit comes from A. P/V/A norm errors of
    1 m, 1 m/s, 1.5 m/s2 and collective error 0.08 are explicit unqualified
    engineering acceptance candidates. Native/command saturation rejects B.
    Causal pairing uses command <= native sample with at most a 50 ms gap;
    no measured response delay is subtracted from a native acquisition time.
    """

    bounds = dict(position=1., velocity=1., acceleration=1.5, collective=.08,
                  rate_rmse=.15, rate_peak=.30, source_age=.125, command_gap=.05,
                  saturation_fraction=.98)

    def __init__(self, rate_limits=(1.5, 1.5, 1.2)):
        self.rate_limits = np.asarray(rate_limits, dtype=float)
        self.outputs, self.navigation, self.thrust = (deque(maxlen=1024) for _ in range(3))
        self.poses = NativePoseHistory(.05)
        self.window = deque(maxlen=2048)
        self.last_navigation, self.last_thrust = 0., 0.

    @staticmethod
    def vector(value):
        result = np.asarray(value, dtype=float)
        if result.shape != (3,) or not np.isfinite(result).all():
            raise ValueError('B_PHYSICS_NONFINITE_VECTOR')
        return result

    @staticmethod
    def append(history, row):
        if not history or row['stamp'] > history[-1]['stamp']:
            history.append(row)

    def observe(self, kind, message, receipt):
        """Keep original source epochs; missing raw native identity is never physical evidence."""
        try:
            if kind == 'paper':
                packet = json.loads(message.data)
                if (packet.get('schema_version') != 2 or packet.get('phase') != 'FOLLOW_ACTIVE'
                        or packet.get('guidance_owner') != 'MINCO_FOLLOW'):
                    return
                for row in packet.get('output_history', []):
                    source, reference = float(row['stamp']), row['reference']
                    epoch = float(reference['stamp'])
                    if not 0 < epoch <= source <= receipt or receipt-source > .125:
                        continue
                    self.append(self.outputs, dict(
                        stamp=source, rates=self.vector(row['rates']), thrust=float(row['thrust']),
                        reference=dict(stamp=epoch, position=self.vector(reference['position']),
                                       velocity=self.vector(reference['velocity']),
                                       acceleration=self.vector(reference['acceleration']))))
            elif kind in ('navigation', 'pose'):
                source = stamp(message.stamp)
                if not (message.valid and message.px4_timestamp_sample > 0
                        and message.native_timestamp_sample > 0 and 0 < source <= receipt):
                    return
                if kind == 'pose':
                    self.poses.add(source, message.quaternion_wxyz, message.angular_velocity_frd,
                                   message.orientation_variance)
                elif message.frame_id == 'local_ned':
                    self.append(self.navigation, dict(stamp=source, **{
                        name: self.vector([getattr(getattr(message, name), a) for a in 'xyz'])
                        for name in ('position', 'velocity', 'acceleration')}))
            elif kind == 'native_thrust':
                packet = json.loads(message.data)
                source, xyz = float(packet['stamp']), self.vector(packet['xyz'])
                if (packet['native_timestamp'] > 0 and packet['px4_timestamp'] > 0
                        and 0 < source <= receipt and max(abs(xyz[:2])) <= 1e-6):
                    self.append(self.thrust, dict(stamp=source, thrust=float(xyz[2])))
        except (KeyError, ValueError, TypeError, AttributeError):
            return

    def reset_window(self):
        """A failed prerequisite invalidates all accumulated continuous-window metrics."""
        self.window.clear()

    def command_at(self, source):
        """Use only the actual command already emitted at the native acquisition epoch."""
        for row in reversed(self.outputs):
            if row['stamp'] <= source:
                if source-row['stamp'] <= self.bounds['command_gap']+1e-9:
                    return row
                break
        raise ValueError('B_PHYSICS_NO_CAUSAL_COMMAND')

    def evaluate(self, now):
        """Consume a new causal native pair, rejecting missing, stale or saturated execution."""
        try:
            paired = next(((native, output['reference']) for native in reversed(self.navigation)
                           if 0 <= now-native['stamp'] <= .125
                           for output in reversed(self.outputs)
                           if abs(native['stamp']-output['reference']['stamp']) <= 1e-6), None)
            if paired is None or not self.thrust:
                raise ValueError('B_PHYSICS_MISSING_NATIVE_OR_REFERENCE')
            native, reference = paired
            executed = self.thrust[-1]
            epoch = native['stamp']
            if not (0 <= now-epoch <= .125 and 0 <= now-executed['stamp'] <= .125):
                raise ValueError('B_PHYSICS_NATIVE_STALE')
            if epoch <= self.last_navigation or executed['stamp'] <= self.last_thrust:
                raise ValueError('B_PHYSICS_REPEATED_NATIVE_SAMPLE')
            pose = self.poses.at(epoch)
            rates, collective = self.command_at(epoch), self.command_at(executed['stamp'])
            fraction = self.bounds['saturation_fraction']
            if (np.any(np.abs(pose.omega) >= fraction*self.rate_limits)
                    or np.any(np.abs(rates['rates']) >= fraction*self.rate_limits)
                    or not -.98 < executed['thrust'] < -.02
                    or not -.98 < collective['thrust'] < -.02):
                raise ValueError('B_PHYSICS_SATURATED_NATIVE_OR_COMMAND')
            error = dict(navigation_stamp=epoch, thrust_stamp=executed['stamp'],
                         rate_command_stamp=rates['stamp'],
                         collective_command_stamp=collective['stamp'],
                         rate_error=(pose.omega-rates['rates']).tolist(),
                         collective_error=abs(executed['thrust']-collective['thrust']))
            for name in ('position', 'velocity', 'acceleration'):
                error[name+'_error'] = float(np.linalg.norm(native[name]-reference[name]))
                if error[name+'_error'] > self.bounds[name]:
                    raise ValueError('B_PHYSICS_'+name.upper()+'_TRACKING')
            if error['collective_error'] > self.bounds['collective']:
                raise ValueError('B_PHYSICS_COLLECTIVE_TRACKING')
            recent = [row['rate_error'] for row in self.window
                      if epoch-row['navigation_stamp'] <= .5]+[error['rate_error']]
            if (max(abs(np.asarray(error['rate_error']))) > self.bounds['rate_peak']
                    or max(np.sqrt(np.mean(np.square(recent), axis=0)))
                    > self.bounds['rate_rmse']):
                raise ValueError('B_PHYSICS_RATE_RMSE')
            self.last_navigation, self.last_thrust = epoch, executed['stamp']
            self.window.append(error)
            return True, error
        except (ValueError, TypeError, KeyError) as error:
            self.reset_window()
            return False, dict(reason=str(error))

    def metrics(self):
        """Report only the current uninterrupted physical acceptance window."""
        result = dict(count=len(self.window), bounds=dict(self.bounds),
                      candidate_status='UNQUALIFIED_ENGINEERING_ACCEPTANCE_BOUNDS')
        if self.window:
            result.update(source_start=self.window[0]['navigation_stamp'],
                          source_end=self.window[-1]['navigation_stamp'],
                          source_seconds=self.window[-1]['navigation_stamp']
                          - self.window[0]['navigation_stamp'],
                          rate_rmse=np.sqrt(np.mean(np.square(
                              [r['rate_error'] for r in self.window]), axis=0)).tolist(),
                          **{name+'_error_max': max(r[name+'_error'] for r in self.window)
                             for name in ('position', 'velocity', 'acceleration', 'collective')})
        return result

    def qualified_window(self):
        """A 25 s observer window also needs fresh native coverage and enough distinct pairs."""
        metrics = self.metrics()
        return bool(metrics['count'] >= 450 and metrics.get('source_seconds', 0.) >= 24.9
                    and max(metrics.get('rate_rmse', [float('inf')])) <= self.bounds['rate_rmse'])


def atomic_json(path, data):
    """Publish a completion marker only after every dependent file is complete."""
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, indent=2)+'\n')
    temporary.replace(path)


class Validation(Node):
    """Stage B collects qualification; C/D depend on it and accept native evaluator results."""

    def __init__(self, output, stage):
        super().__init__('p761_validation_observer')
        import os
        from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
        require_isolated_sitl()
        self.output, self.stage = output, stage
        self.contract = load_contract(os.environ['UAV_USV_P76_CONTRACT'])
        from uav_control.common.p76_body_geometry import base_reference_flu
        self.base_offset = base_reference_flu(os.environ['UAV_USV_P76_MODEL_ROOT'])[:3, 3]
        self.started = time.monotonic()
        self.raw = (output/'validation.jsonl').open('x')
        self.latest, self.acks, self.detections, self.graph = {}, {}, [], []
        self.graph_times = []
        limits = self.contract['candidate']
        self.physics = BPhysicalAcceptance((limits['maximum_body_roll_pitch_rate'],
                                            limits['maximum_body_roll_pitch_rate'],
                                            limits['maximum_body_yaw_rate']))
        self.ack_receipts = {}
        self.x_times, self.y_times = [], []
        atomic_json(output/'validation_identity.json', dict(
            sources=source_fingerprint(Path(__file__).resolve().parents[1]),
            contract_sha256=sha256(self.contract['_path']),
            model_manifest_sha256=sha256(
                Path(os.environ['UAV_USV_P76_MODEL_ROOT'])/'p76_model_manifest.json')))
        self.follow_since, self.first_follow, self.y_sent = None, None, None
        self.last_graph, self.done, self.success, self.reason = 0., False, False, ''
        self.publisher = self.create_publisher(String, '/simulation/impact/command', 10)
        self.subs = []
        for name, topic, cls in (
            ('ready', '/simulation/impact/flight_ready', Bool),
            ('control', '/control/diagnostic', ControllerDiagnostic),
            ('navigation', '/navigation/uav_state', UavState),
            ('pose', '/navigation/native_attitude', NativeAttitude),
            ('target', '/tracking/target_state', TargetState),
            ('detection', '/perception/target_detection', TargetImageDetection),
            ('image', '/tracking/image_state', TargetImageState),
            ('filter', '/tracking/p76_filter_ack', String),
            ('paper', '/control/p76_paper', String),
            ('px4_mode', '/navigation/px4_control_mode', String),
            ('native_thrust', '/navigation/native_thrust', String),
            ('reference', '/navigation/native_local_reference', String),
            ('result', '/simulation/impact/result', InterceptResult),
        ):
            qos = qos_profile_sensor_data if name in ('navigation', 'pose') else 100
            self.subs.append(
                self.create_subscription(
                    cls,
                    topic,
                    lambda message,
                    key=name: self.receive(
                        key,
                        message),
                    qos))
        self.subs.append(self.create_subscription(String, '/control/p76_mode',
                                                  lambda m: self.receive('mode', m), phase_qos()))
        self.timer = self.create_timer(.05, self.tick)

    def receive(self, name, message):
        now = self.get_clock().now().nanoseconds*1e-9
        value = message_dict(message)
        self.raw.write(json.dumps(dict(kind=name, receipt=now, message=value))+'\n')
        self.latest[name] = (now, message)
        if name in ('paper', 'navigation', 'pose', 'native_thrust'):
            self.physics.observe(name, message, now)
        if name == 'filter':
            packet = json.loads(message.data)
            self.acks[packet['estimator']] = packet
            self.ack_receipts[packet['estimator']] = now
        elif name == 'detection':
            stamp = message.stamp.sec+message.stamp.nanosec*1e-9
            self.detections.append(dict(stamp=stamp, age=now-stamp, valid=message.valid,
                                        model=message.model_sha256))
        elif name == 'result':
            self.success = bool(message.success)
            self.reason = message.reason
            # Evaluator finalizes and pauses after publishing; allow its bounded pause RPC.
            self.terminal_seen = time.monotonic()

    def finish(self, success, reason):
        self.success, self.reason, self.done = success, reason, True
        self.raw.flush()

    def tick(self):
        wall = time.monotonic()
        now = self.get_clock().now().nanoseconds*1e-9
        if hasattr(self, 'terminal_seen'):
            if wall-self.terminal_seen > 1.:
                summaries = [json.loads(p.read_text()) for p in self.output.glob('*_summary.json')]
                paused = any(s.get('gazebo_pause_succeeded') for s in summaries)
                self.finish(self.success and paused,
                            self.reason if paused else 'PAUSE_NOT_CONFIRMED')
            return
        if wall-self.started > 190:
            self.finish(False, 'VALIDATION_WATCHDOG')
            return
        if wall-self.last_graph > 1.:
            self.last_graph = wall
            topics = ('/perception/target_detection', '/tracking/target_state',
                      '/fmu/in/offboard_control_mode', '/fmu/in/vehicle_rates_setpoint',
                      '/fmu/in/trajectory_setpoint')
            graph = {
                t: [i.node_name for i in self.get_publishers_info_by_topic(t)] for t in topics}
            self.graph.append(graph)
            self.graph_times.append((wall, graph))
            self.raw.write(json.dumps(dict(kind='graph', receipt=now, publishers=graph))+'\n')
            outlets = ('/fmu/in/offboard_control_mode', '/fmu/in/vehicle_rates_setpoint')
            if (any(len(names) > 1 for names in graph.values())
                    or graph['/fmu/in/trajectory_setpoint']
                    or any(graph[t] and graph[t] != ['trajectory_tracker_node'] for t in outlets)):
                self.finish(False, 'PX4_PUBLISHER_GRAPH_NOT_EXCLUSIVE_BODYRATE')
                return
        ready = self.latest.get('ready')
        if not self.x_times:
            if ready and ready[1].data and self.publisher.get_subscription_count() >= 2:
                self.x_times = [wall, wall+.1, wall+.2]
        if self.x_times and wall >= self.x_times[0]:
            self.publisher.publish(String(data='X'))
            self.x_times.pop(0)
            if not self.x_times:
                self.x_times = [float('inf')]
        if self.y_times and wall >= self.y_times[0]:
            self.publisher.publish(String(data='Y'))
            self.y_times.pop(0)
        if self.y_sent is not None:
            # After Y this observer does not read the 3D target for decisions.
            mode = json.loads(self.latest['mode'][1].data) if 'mode' in self.latest else {}
            if mode.get('phase') == 'SAFE_RECOVERY':
                self.finish(False, 'PAPER_SAFE_RECOVERY')
            elif wall-self.y_sent > self.contract['timing']['handover_timeout']:
                runtime = (json.loads(self.latest['paper'][1].data)
                           if 'paper' in self.latest else {})
                native = (json.loads(self.latest['px4_mode'][1].data)
                          if 'px4_mode' in self.latest else {})
                if not intercept_authority_fresh(now, self.acks, mode, runtime, native):
                    self.finish(False, 'PAPER_ACTUAL_IMAGE_AUTHORITY_NOT_FRESH')
            return
        diagnostic = self.latest.get('control')
        navigation = self.latest.get('navigation')
        target = self.latest.get('target')
        if not all((diagnostic, navigation, target)):
            return
        control, state, tracking = diagnostic[1], navigation[1], target[1]
        ns = state.stamp.sec+state.stamp.nanosec*1e-9
        physical_height = None
        if 'reference' in self.latest and 'pose' in self.latest:
            reference = json.loads(self.latest['reference'][1].data)
            pose = self.latest['pose'][1]
            rotation = Rotation.from_quat(np.asarray(pose.quaternion_wxyz)[[1, 2, 3, 0]])
            base_ned = self.base_offset*np.array([1., -1., -1.])
            model_z = state.position.z-rotation.apply(base_ned)[2]
            physical_height = reference['ref_alt'] - \
                self.contract['candidate']['sea_altitude_msl']-model_z
        mode = json.loads(self.latest['mode'][1].data) if 'mode' in self.latest else {}
        runtime = json.loads(self.latest['paper'][1].data) if 'paper' in self.latest else {}
        native = json.loads(self.latest['px4_mode'][1].data) if 'px4_mode' in self.latest else {}
        receipts = [diagnostic[0], target[0]]+list(self.ack_receipts.values())
        if 'paper' in self.latest:
            receipts.append(self.latest['paper'][0])
        authority = follow_authority_fresh(
            now, control, tracking, self.acks, receipts, mode, runtime, native)
        physical, evidence = self.physics.evaluate(now)
        self.raw.write(json.dumps(dict(kind='physical_acceptance', receipt=now,
                                       passed=physical, evidence=evidence))+'\n')
        graph_ok = bool(self.graph_times and wall-self.graph_times[-1][0] <= 1.25
                        and bodyrate_graph_qualified(self.graph_times[-1][1]))
        valid = (authority and physical and graph_ok and control.status == 'MINCO_FOLLOW'
                 and control.target_locked and control.target_visible
                 and state.valid and 0 <= now-ns <= .125
                 and 'pose' in self.latest
                 and 0 <= now-stamp(self.latest['pose'][1].stamp) <= .125
                 and abs(np.hypot(state.position.x-tracking.position.x,
                                  state.position.y-tracking.position.y)-5.) <= 1.
                 and physical_height is not None and abs(physical_height-4.) <= .5)
        if not valid:
            self.physics.reset_window()
        self.follow_since = (self.follow_since or wall) if valid else None
        if control.status in ('FOLLOW', 'MINCO_FOLLOW') and self.first_follow is None:
            self.first_follow = wall
        if (self.first_follow is not None and wall-self.first_follow > 65
                and self.follow_since is None):
            self.finish(False, 'NO_CONTINUOUS_QUALIFIED_FOLLOW_WINDOW')
        if (self.follow_since is not None and wall-self.follow_since >= 25.
                and self.physics.qualified_window()):
            if self.stage == 'B':
                self.finish(True, '25S_ACTUAL_MINCO_FOLLOW')
            else:
                self.y_sent = wall
                self.y_times = [wall, wall+.1, wall+.2]

    def save(self):
        """B manifests derive from observed ownership, graph and actual acquisition authority."""
        self.raw.flush()
        result = dict(success=self.success, reason=self.reason, stage=self.stage,
                      elapsed=time.monotonic()-self.started, y_sent=self.y_sent,
                      physical_acceptance=self.physics.metrics())
        if self.stage != 'B' or not self.success:
            atomic_json(self.output/'run_result.json', result)
            return
        valid = [d for d in self.detections if d['valid']]
        full_graphs = [g for t, g in self.graph_times if t >= self.follow_since]
        graph_ok = bool(full_graphs) and all(bodyrate_graph_qualified(g) for g in full_graphs)
        detector_ok = (graph_ok and len(valid) >= 450 and all(
            0 <= d['age'] <= .125 and d['model'] == self.contract['detector']['model_sha256']
            for d in valid))
        if not detector_ok or not self.physics.qualified_window():
            self.success, self.reason = False, 'DETECTOR_OR_GRAPH_QUALIFICATION_FAILED'
            result.update(success=False, reason=self.reason)
            atomic_json(self.output/'run_result.json', result)
            return
        sources = source_fingerprint(Path(__file__).resolve().parents[1])
        raw = {str(self.output/'validation.jsonl'): sha256(self.output/'validation.jsonl')}
        data = dict(schema_version=2, architecture='parallel_dkf_unified_bodyrate',
                    px4_interface='BODY_RATE', qualified=detector_ok, valid_follow_seconds=25.,
                    body_rate_output_verified=True, native_body_rate_mode_verified=True,
                    native_physical_execution_verified=True,
                    physical_acceptance=self.physics.metrics(),
                    both_estimators_active_fresh=True, image_follow_control_authorized=False,
                    actual_minco_ownership_verified=True, sources=sources,
                    contract_sha256=sha256(self.contract['_path']), raw_evidence=raw)
        path = Path(self.contract['candidate']['follow_qualification_manifest'])
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            raise ValueError('NEVER_OVERWRITE_FOLLOW_QUALIFICATION')
        atomic_json(path, data)
        path = Path(self.contract['detector']['qualification_manifest'])
        if path.exists():
            raise ValueError('NEVER_OVERWRITE_DETECTOR_QUALIFICATION')
        atomic_json(path, dict(schema_version=2, architecture='parallel_dkf_unified_bodyrate',
                               qualified=detector_ok, sources=sources,
                               model_sha256=self.contract['detector']['model_sha256'],
                               contract_sha256=sha256(self.contract['_path']), raw_evidence=raw))
        atomic_json(self.output/'run_result.json', result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=('B', 'C', 'D'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Validation(args.output, args.stage)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.save()
        node.raw.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.success else 2


if __name__ == '__main__':
    raise SystemExit(main())
