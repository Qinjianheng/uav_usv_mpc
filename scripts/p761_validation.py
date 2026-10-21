#!/usr/bin/env python3
"""Observe actual stage B/C/D, issue only task-authorized X/Y, never flight setpoints."""
import argparse
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


def follow_authority_fresh(now, diagnostic, target, acks, receipts, mode):
    """A held diagnostic or ACK cannot accumulate actual ownership qualification."""
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
    for name, active in (('spatial', True), ('image', False)):
        ack = acks.get(name, {})
        if not (ack.get('active') is active and ack.get('epoch') == mode.get('epoch')
                and ack.get('phase') == 'FOLLOW_ACTIVE' and 0 <= now-ack.get('stamp', 0) <= .125):
            return False
    spatial = acks['spatial']
    return bool(spatial.get('ready') and 0 <= now-spatial.get('source_stamp', 0) <= .125)


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
                      '/fmu/in/vehicle_rates_setpoint', '/fmu/in/trajectory_setpoint')
            graph = {
                t: [i.node_name for i in self.get_publishers_info_by_topic(t)] for t in topics}
            self.graph.append(graph)
            self.raw.write(json.dumps(dict(kind='graph', receipt=now, publishers=graph))+'\n')
            if any(len(names) > 1 for names in graph.values()):
                self.finish(False, 'MULTIPLE_PUBLISHERS')
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
            mode = self.latest.get('mode')
            if mode and json.loads(mode[1].data)['phase'] == 'SAFE_RECOVERY':
                self.finish(False, 'PAPER_SAFE_RECOVERY')
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
        authority = follow_authority_fresh(
            now, control, tracking, self.acks,
            [diagnostic[0], target[0]]+list(self.ack_receipts.values()), mode)
        valid = (authority and control.status == 'MINCO_FOLLOW'
                 and control.target_locked and control.target_visible
                 and state.valid and 0 <= now-ns <= .125
                 and 'pose' in self.latest
                 and 0 <= now-stamp(self.latest['pose'][1].stamp) <= .125
                 and abs(np.hypot(state.position.x-tracking.position.x,
                                  state.position.y-tracking.position.y)-5.) <= 1.
                 and not self.acks.get('image', {}).get('active', True)
                 and self.acks.get('spatial', {}).get('active', False)
                 and physical_height is not None and abs(physical_height-4.) <= .5)
        self.follow_since = (self.follow_since or wall) if valid else None
        if control.status in ('FOLLOW', 'MINCO_FOLLOW') and self.first_follow is None:
            self.first_follow = wall
        if (self.first_follow is not None and wall-self.first_follow > 65
                and self.follow_since is None):
            self.finish(False, 'NO_CONTINUOUS_QUALIFIED_FOLLOW_WINDOW')
        if self.follow_since is not None and wall-self.follow_since >= 25.:
            if self.stage == 'B':
                self.finish(True, '25S_ACTUAL_MINCO_FOLLOW')
            else:
                self.y_sent = wall
                self.y_times = [wall, wall+.1, wall+.2]

    def save(self):
        """B manifests derive from observed ownership, graph and actual acquisition authority."""
        self.raw.flush()
        result = dict(success=self.success, reason=self.reason, stage=self.stage,
                      elapsed=time.monotonic()-self.started, y_sent=self.y_sent)
        if self.stage != 'B' or not self.success:
            atomic_json(self.output/'run_result.json', result)
            return
        valid = [d for d in self.detections if d['valid']]
        full_graphs = [g for g in self.graph if g['/tracking/target_state']]
        graph_ok = bool(full_graphs) and all(
            g['/perception/target_detection'] == ['yolo_target_detector']
            and len(g['/tracking/target_state']) == 1 for g in full_graphs)
        detector_ok = (graph_ok and len(valid) >= 450 and all(
            0 <= d['age'] <= .125 and d['model'] == self.contract['detector']['model_sha256']
            for d in valid))
        if not detector_ok:
            self.success, self.reason = False, 'DETECTOR_OR_GRAPH_QUALIFICATION_FAILED'
            result.update(success=False, reason=self.reason)
            atomic_json(self.output/'run_result.json', result)
            return
        sources = source_fingerprint(Path(__file__).resolve().parents[1])
        raw = {str(self.output/'validation.jsonl'): sha256(self.output/'validation.jsonl')}
        data = dict(qualified=detector_ok, valid_follow_seconds=25.,
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
        atomic_json(path, dict(qualified=detector_ok, sources=sources,
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
