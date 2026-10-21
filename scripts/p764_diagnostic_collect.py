#!/usr/bin/env python3
"""One unarmed real-input DKF executor and disconnected 200 Hz engineering shadow."""
import argparse
from bisect import bisect_left
from collections import defaultdict, deque
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus
from uav_usv_interfaces.msg import (
    NativeAttitude, TargetImageDetection, TargetImageState, TargetObservation,
    TargetState, UavState,
)

from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_native import NativePoseHistory
from uav_control.common.p76_phase import phase_qos
from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker
from uav_control.control.p763_calibration_control import load_bootstrap
from uav_control.perception.rgbd_target_localizer import aligned_camera_qos
from uav_control.perception.yolo_detection_contract import stamp_seconds
from uav_control.tracking.image_dkf_node import ImageDkfNode
from uav_control.tracking.spatial_dkf_node import SpatialDkfNode

ROOT = Path(__file__).resolve().parents[1]
COMMAND_TOPICS = ('/fmu/in/vehicle_command', '/fmu/in/offboard_control_mode',
                  '/fmu/in/vehicle_rates_setpoint', '/fmu/in/trajectory_setpoint')


def plain(value):
    """Retain complete typed packets, including RLE mask and raw acquisition identity."""
    if hasattr(value, 'get_fields_and_field_types'):
        return plain(message_to_ordereddict(value))
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if hasattr(value, 'tolist'):
        return plain(value.tolist())
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def stats(values):
    if len(values) == 0:
        return dict(count=0)
    return dict(count=len(values), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)),
                maximum=float(max(values)))


def instrument(cls, names, timings):
    """Wrap actual callbacks before ROS subscriptions/timers bind them."""
    for name in names:
        original = getattr(cls, name)

        def wrapped(self, *args, _method=original, _key=cls.__name__+'.'+name, **kwargs):
            began = time.perf_counter()
            depth = getattr(self, 'p764_callback_depth', 0)
            if depth == 0:
                if not hasattr(self, 'p764_callback_events'):
                    self.p764_callback_events = []
                self.p764_callback_events.append(dict(
                    node=cls.__name__, method=_method.__name__,
                    receipt=self.get_clock().now().nanoseconds*1e-9,
                    arguments=[plain(arg) for arg in args], phase=self.p76_phase,
                    epoch=self.p76_epoch, y_stamp=self.p76_y_stamp))
            self.p764_callback_depth = depth+1
            transition = _method.__name__ == 'mode_changed'

            def snapshot():
                core = self.filter.filter if hasattr(self.filter, 'filter') else self.filter
                return hashlib.sha256(json.dumps(plain(dict(
                    anchor=core.anchor, checkpoints=core.checkpoints,
                    source_stamp=core.source_stamp,
                    camera_anchor=getattr(self.filter, 'anchor_rotation', None))),
                    sort_keys=True).encode()).hexdigest()
            before = snapshot() if transition else None
            try:
                return _method(self, *args, **kwargs)
            finally:
                timings[_key].append(time.perf_counter()-began)
                self.p764_callback_depth = depth
                if transition:
                    if not hasattr(self, 'p764_transitions'):
                        self.p764_transitions = []
                    self.p764_transitions.append(dict(phase=self.p76_phase, epoch=self.p76_epoch,
                                                      before=before, after=snapshot()))
        setattr(cls, name, wrapped)


class Collector(Node):
    """Only publishes shadow phase labels; never creates a PX4 input publisher."""

    def __init__(self, output, contract, spatial, image, timings, duration):
        super().__init__('p764_unarmed_diagnostic')
        self.output, self.c, self.filters = output, contract, (spatial, image)
        self.timings, self.duration = timings, duration
        self.began, self.cpu_began = time.monotonic(), time.process_time()
        self.raw = gzip.open(output/'synchronized_packets.jsonl.gz', 'xt')
        self.records, self.phases, self.graph = [], [], []
        self.phase, self.epoch, self.failure = 'FOLLOW_ACTIVE', 0, ''
        self.latest, self.navigation = {}, deque(maxlen=64)
        self.poses = NativePoseHistory()
        self.timer_times, self.shadow_costs, self.shadow_epochs = [], [], []
        self.shadow_rejections, self.rss = defaultdict(int), []
        self.mode_pub = self.create_publisher(String, '/control/p76_mode', phase_qos())
        self.subscriptions_owned = []
        topics = (
            ('detection', '/perception/target_detection', TargetImageDetection),
            ('observation', '/perception/front/target_observation', TargetObservation),
            ('navigation', '/navigation/uav_state', UavState),
            ('pose', '/navigation/native_attitude', NativeAttitude),
            ('spatial', '/tracking/target_state', TargetState),
            ('image', '/tracking/image_state', TargetImageState),
            ('ack', '/tracking/p76_filter_ack', String),
            ('reference', '/navigation/native_local_reference', String),
            ('thrust', '/navigation/native_thrust', String),
            ('yolo', '/diagnostics/p76_yolo', String),
            ('status', '/fmu/out/vehicle_status_v1', VehicleStatus),
        )
        for name, topic, cls in topics:
            if name == 'status':
                version = getattr(cls, 'MESSAGE_VERSION', 0)
                topic = '/fmu/out/vehicle_status'+(f'_v{version}' if version else '')
            qos = qos_profile_sensor_data if name in ('pose', 'navigation', 'status') else 100
            self.subscriptions_owned.append(self.create_subscription(
                cls, topic, lambda m, key=name: self.receive(key, m), qos))
        for name, topic in (('rgb_frame', '/camera/front/image_raw'),
                            ('depth_frame', '/camera/front/depth/image_raw')):
            self.subscriptions_owned.append(self.create_subscription(
                Image, topic, lambda m, key=name: self.frame(key, m), aligned_camera_qos()))
        prior = ROOT/'data/experiments/20261021_p761_01/session_A1_03'
        old = json.loads((prior/'identity.json').read_text())
        model = json.loads((Path(os.environ['UAV_USV_P76_MODEL_ROOT'])
                            / 'p76_model_manifest.json').read_text())
        bootstrap = load_bootstrap(prior, dict(model=model, px4=old['px4']))
        self.outer = BodyrateTracker(
            contract, model['total_mass'], model['maximum_thrust'], bootstrap)
        self.outer.initialize(np.zeros(3), -bootstrap.hover)
        self.timer = self.create_timer(.005, self.shadow)
        self.supervisor = self.create_timer(.1, self.supervise)
        self.done = False
        self.publish_phase()

    def record(self, kind, message):
        now = self.get_clock().now().nanoseconds*1e-9
        row = dict(kind=kind, receipt=now, monotonic=time.monotonic(), message=plain(message))
        self.raw.write(json.dumps(row)+'\n')
        self.records.append(row)
        return row

    def receive(self, name, message):
        row = self.record(name, message)
        self.latest[name] = row
        if name == 'navigation' and message.valid:
            self.navigation.append(message)
        elif name == 'pose' and message.valid:
            self.poses.add(stamp_seconds(message.stamp), message.quaternion_wxyz,
                           message.angular_velocity_frd, message.orientation_variance)
        elif name == 'status' and message.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            self.failure = 'UNEXPECTED_ARMED_STATE'
            self.done = True

    def frame(self, name, message):
        # Store exact RGB-D header identities; detector RLE is stored in full separately.
        self.record(name, dict(header=plain(message.header), width=message.width,
                               height=message.height, encoding=message.encoding,
                               step=message.step))

    def publish_phase(self):
        now = self.get_clock().now().nanoseconds*1e-9
        packet = dict(schema_version=2, phase=self.phase, epoch=self.epoch,
                      y_stamp=self.y_stamp if hasattr(self, 'y_stamp') else 0.,
                      diagnostic_only=True, physical_guidance_owner='NONE')
        self.mode_pub.publish(String(data=json.dumps(packet)))
        self.phases.append(dict(stamp=now, **packet))

    def shadow(self):
        began = time.perf_counter()
        self.timer_times.append(began)
        now = self.get_clock().now().nanoseconds*1e-9
        try:
            state, pose = None, None
            for candidate in reversed(self.navigation):
                epoch = stamp_seconds(candidate.stamp)
                if 0 <= now-epoch <= .125:
                    try:
                        pose, state = self.poses.at(epoch), candidate
                        break
                    except ValueError:
                        continue
            if state is None:
                raise ValueError('NATIVE_PAIR_NOT_AVAILABLE')
            p, v = [np.array([getattr(getattr(state, k), a) for a in 'xyz'])
                    for k in ('position', 'velocity')]
            ref = np.r_[p, v, np.zeros(3), math.atan2(
                pose.rotation[1, 0], pose.rotation[0, 0]), 0.]
            # A native-state hold workload, not a fictitious admitted MINCO trajectory or flight.
            dt = self.timer_times[-1]-self.timer_times[-2] if len(self.timer_times) > 1 else .005
            self.outer.step(ref, p, v, pose.rotation, dt, pose.omega)
            self.shadow_epochs.append(pose.stamp)
        except (ValueError, np.linalg.LinAlgError) as error:
            self.shadow_rejections[str(error)] += 1
        self.shadow_costs.append(time.perf_counter()-began)

    def supervise(self):
        elapsed = time.monotonic()-self.began
        if elapsed >= 12. and self.phase == 'FOLLOW_ACTIVE':
            self.phase, self.epoch = 'Y_HANDOVER', 1
            self.y_stamp = self.get_clock().now().nanoseconds*1e-9
        elif elapsed >= 12.3 and self.phase == 'Y_HANDOVER':
            self.phase = 'PAPER_INTERCEPT_ACTIVE'
        self.publish_phase()
        graph = {topic: [i.node_name for i in self.get_publishers_info_by_topic(topic)]
                 for topic in COMMAND_TOPICS+('/perception/target_detection',)}
        self.graph.append(graph)
        if any(graph[t] for t in COMMAND_TOPICS):
            self.failure, self.done = 'PX4_COMMAND_PUBLISHER_IN_NONFLIGHT_DIAGNOSTIC', True
        self.rss.append(int(next(line.split()[1] for line in
                                 Path('/proc/self/status').read_text().splitlines()
                                 if line.startswith('VmRSS:')))*1024)
        if elapsed >= self.duration:
            self.done = True

    def save(self):
        self.raw.close()
        events = sorted((row for node in self.filters
                         for row in getattr(node, 'p764_callback_events', [])),
                        key=lambda row: row['receipt'])
        with gzip.open(self.output/'actual_callback_inputs.jsonl.gz', 'xt') as stream:
            for row in events:
                stream.write(json.dumps(row)+'\n')
        elapsed, cpu = time.monotonic()-self.began, time.process_time()-self.cpu_began
        intervals = np.diff(self.timer_times)
        missed = sum(max(0, int(round(x/.005))-1) for x in intervals)
        shadow = dict(evidence_class='ACTUAL_ROS_EXECUTOR_DISCONNECTED_SHADOW',
                      period_seconds=.005, interval_seconds=stats(intervals),
                      compute_seconds=stats(self.shadow_costs), missed_cycles=missed,
                      compute_deadline_misses=sum(x > .005 for x in self.shadow_costs),
                      successful_native_computations=len(self.shadow_epochs),
                      rejections=dict(self.shadow_rejections),
                      qualified=bool(len(self.shadow_epochs) >= 200 and not missed
                                     and max(self.shadow_costs, default=1.) <= .005),
                      workload='Real BodyrateTracker, native hold PVA; no admitted MINCO or PNG',
                      px4_outputs=False, flight_qualified=False)
        coverage = analyze_coverage(self.records, self.phases, self.graph)
        transitions = [r for node in self.filters
                       for r in getattr(node, 'p764_transitions', [])]
        continuity = bool(transitions) and all(r['before'] == r['after'] for r in transitions)
        coverage.update(posterior_and_covariance_preserved=continuity,
                        transitions=transitions)
        coverage['qualified'] &= continuity
        coverage.update(failure=self.failure,
                        source=str(self.output/'synchronized_packets.jsonl.gz'))
        perf = dict(callback_seconds={k: stats(v) for k, v in self.timings.items()},
                    process_cpu_seconds=cpu, wall_seconds=elapsed,
                    process_cpu_core_fraction=cpu/elapsed, rss_bytes=stats(self.rss),
                    scope='Two DKFs, recorder and shadow share one single-thread executor')
        for filename, result in (('e1_real_data_coverage.json', coverage),
                                 ('e1_dkf_performance.json', perf),
                                 ('e1_tracker_shadow_timing.json', shadow)):
            (self.output/filename).write_text(json.dumps(result, indent=2)+'\n')
        (self.output/'phase_labels.json').write_text(json.dumps(self.phases, indent=2)+'\n')
        success = coverage['qualified'] and shadow['qualified'] and not self.failure
        (self.output/'result.json').write_text(json.dumps(dict(
            success=bool(success), unarmed=coverage['unarmed'], flight_qualified=False,
            reason=self.failure or (
                'E1_SOFTWARE_PASS' if success else 'E1_COVERAGE_OR_TIMING_FAILED'),
            graph=self.graph[-1] if self.graph else {}), indent=2)+'\n')


def analyze_coverage(records, phases, graphs):
    """Require real same-frame detection/observation/native coverage across shadow Y labels."""
    def seconds(d):
        return d['sec']+d['nanosec']*1e-9
    by_kind = defaultdict(list)
    for row in records:
        by_kind[row['kind']].append(row)
    valid = [r for r in by_kind['detection'] if r['message']['valid']]
    observations = {round(seconds(r['message']['stamp'])*1e9): r
                    for r in by_kind['observation'] if r['message']['valid']}
    rgb = {json.dumps(r['message']['header']['stamp'], sort_keys=True)
           for r in by_kind['rgb_frame']}
    depth = {json.dumps(r['message']['header']['stamp'], sort_keys=True)
             for r in by_kind['depth_frame']}
    paired = [r for r in valid if round(seconds(r['message']['stamp'])*1e9) in observations
              and json.dumps(r['message']['raw_stamp'], sort_keys=True) in rgb & depth
              and r['message']['mask_counts']]

    def bracket(times, source):
        index = bisect_left(times, source)
        return ((index < len(times) and abs(times[index]-source) <= 1e-9)
                or 0 < index < len(times) and times[index]-times[index-1] <= .05+1e-9)
    native_pairs = []
    for row in paired:
        source = seconds(row['message']['stamp'])
        observed = observations[round(source*1e9)]['receipt']
        times = []
        for kind in ('pose', 'navigation'):
            times.append(sorted(seconds(r['message']['stamp']) for r in by_kind[kind]
                                if r['receipt'] <= observed and r['message']['valid']
                                and r['message']['px4_timestamp_sample'] > 0
                                and r['message']['native_timestamp_sample'] > 0))
        if all(bracket(t, source) for t in times):
            native_pairs.append(row)
    ages = [r['receipt']-seconds(r['message']['stamp']) for r in valid]
    spatial = [r for r in by_kind['spatial'] if r['message']['valid']]
    image = [r for r in by_kind['image'] if r['message']['valid']]
    y = next((p['stamp'] for p in phases if p['phase'] == 'Y_HANDOVER'), float('inf'))
    before = [r for r in image if r['receipt'] < y]
    after = [r for r in image if r['receipt'] >= y]
    after_spatial = [r for r in spatial if r['receipt'] >= y]
    illegal = 0
    for row in by_kind['ack']:
        ack = json.loads(row['message']['data'])
        permitted = ((ack['estimator'] == 'spatial') == (ack['phase'] == 'FOLLOW_ACTIVE'))
        illegal += bool(ack['control_authorized']) and not permitted
    no_output = bool(graphs) and all(not g[t] for g in graphs for t in COMMAND_TOPICS)
    statuses = by_kind['status']
    unarmed = (len(statuses) >= 10 and all(r['message']['arming_state'] == 1 for r in statuses))
    discovered = [g['/perception/target_detection'] for g in graphs
                  if g['/perception/target_detection']]
    one_yolo = bool(discovered) and all(names == ['yolo_target_detector'] for names in discovered)
    qualified = (len(native_pairs) >= 50 and len(before) >= 10 and len(after) >= 10
                 and len(after_spatial) >= 10 and ages and max(ages) <= .125
                 and not illegal and no_output and one_yolo and unarmed)
    return dict(evidence_class='REAL_SYNCHRONIZED_UNARMED_SESSION',
                counts={k: len(v) for k, v in by_kind.items()}, valid_detections=len(valid),
                same_rgb_depth_detection_observation_frames=len(paired),
                causal_native_pose_navigation_brackets=len(native_pairs),
                image_valid_before_y=len(before), image_valid_after_y=len(after),
                spatial_valid_after_y=len(after_spatial), illegal_authorizations=int(illegal),
                valid_detection_age_seconds=stats(ages), one_yolo=one_yolo,
                no_px4_command_publishers=no_output, qualified=bool(qualified),
                unarmed=bool(unarmed),
                flight_qualified=False, limits=dict(source_age=.125, pose_bracket=.05),
                scope='Stationary ground scene and shadow labels; no flight or capture')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=30.)
    args, ros_args = parser.parse_known_args()
    if not 20 <= args.duration <= 40:
        raise ValueError('P764_DIAGNOSTIC_DURATION_20_TO_40_SECONDS')
    from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
    require_isolated_sitl()
    c = load_contract(os.environ['UAV_USV_P76_CONTRACT'])
    timings = defaultdict(list)
    instrument(SpatialDkfNode, ('receive', 'tick', 'mode_changed'), timings)
    instrument(ImageDkfNode, ('receive', 'pose', 'navigation', 'tick', 'mode_changed'), timings)
    rclpy.init(args=ros_args)
    spatial, image = SpatialDkfNode(), ImageDkfNode()
    collector = Collector(args.output, c, spatial, image, timings, args.duration)
    executor = SingleThreadedExecutor()
    for node in (spatial, image, collector):
        executor.add_node(node)
    try:
        while rclpy.ok() and not collector.done:
            executor.spin_once(timeout_sec=.01)
    finally:
        collector.save()
        executor.shutdown()
        for node in (spatial, image, collector):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
