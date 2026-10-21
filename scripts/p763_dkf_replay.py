#!/usr/bin/env python3
"""Audit real input coverage and compare actual old/new DKF callbacks on a labelled fixture."""
import argparse
from collections import defaultdict, deque
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from types import ModuleType, SimpleNamespace as Row

import numpy as np
from rclpy.time import Time
from uav_usv_interfaces.msg import (
    NativeAttitude, TargetImageDetection, TargetObservation, UavState,
)

from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_native import NativePoseHistory
from uav_control.perception.yolo_detection_contract import stamp_seconds
from uav_control.tracking.delayed_image_kalman import DelayedImageKalman
from uav_control.tracking.delayed_spatial_kalman import DelayedSpatialKalman
from uav_control.tracking.image_dkf_node import ImageDkfNode
from uav_control.tracking.spatial_dkf_node import SpatialDkfNode

ROOT = Path(__file__).resolve().parents[1]
LEGACY_REF = '718295a920a7eddbd3ffc5224ec0e38662baf6cc'
CONTRACT = ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'


def digest(path):
    """Identify an input without modifying or recompressing it."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    """Read original recording epochs without offsets or resampling."""
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def audit_real_inputs(detector_path, navigation_path):
    """Missing full packets or matched native epochs remain a real-data blocker."""
    packets, navigation = rows(detector_path), rows(navigation_path)
    poses = [r for r in navigation if r.get('kind') == 'pose']
    states = [r for r in navigation if r.get('kind') == 'state']
    sources = [r['source_stamp'] for r in packets]
    pose_epochs, state_epochs = [[r['stamp'] for r in samples] for samples in (poses, states)]
    overlap = sum(bool(pose_epochs and state_epochs
                       and min(pose_epochs) <= source <= max(pose_epochs)
                       and min(state_epochs) <= source <= max(state_epochs)) for source in sources)
    full_packets = bool(packets) and all(
        all(key in r for key in ('mask_counts', 'raw_stamp', 'frame_sequence')) for r in packets)
    blockers = []
    if not overlap:
        blockers.append('NO_SYNCHRONIZED_NATIVE_NAVIGATION')
    if not full_packets:
        blockers.append('DETECTION_SUMMARIES_LACK_ORIGINAL_MASK_AND_RAW_IDENTITY')
    # Epoch overlap alone cannot certify per-frame causal brackets or same-session provenance.
    blockers.append('MATCHED_FULL_PACKET_NATIVE_REPLAY_NOT_AVAILABLE')
    return dict(
        detector_packets=len(packets),
        valid_detector_packets=sum(bool(r.get('valid')) for r in packets),
        detector_epoch_span=[min(sources), max(sources)] if sources else [],
        native_pose_samples=len(poses), native_navigation_samples=len(states),
        native_epoch_span=[min(pose_epochs), max(pose_epochs)] if pose_epochs else [],
        overlapping_source_epochs=overlap, full_detection_packets_available=full_packets,
        matched_real_replay_qualified=False, blockers=blockers,
        inputs={str(path): digest(path) for path in (detector_path, navigation_path)})


def legacy_classes():
    """Run the frozen old callbacks, using their old ACK serializer and current unchanged math."""
    def module(relative, package):
        source = subprocess.run(['git', 'show', LEGACY_REF+':'+relative], cwd=ROOT,
                                capture_output=True, text=True, check=True).stdout
        result = ModuleType('p763_legacy_'+Path(relative).stem)
        result.__package__ = package
        exec(compile(source, LEGACY_REF+':'+relative, 'exec'), result.__dict__)
        return result
    prefix = 'src/uav_control/uav_control/'
    phase = module(prefix+'common/p76_phase.py', 'uav_control.common')
    spatial = module(prefix+'tracking/spatial_dkf_node.py', 'uav_control.tracking')
    image = module(prefix+'tracking/image_dkf_node.py', 'uav_control.tracking')
    spatial.publish_ack = image.publish_ack = phase.publish_ack
    return spatial.SpatialDkfNode, image.ImageDkfNode


def ros_stamp(value):
    """Encode the fixture's own epoch; never convert an unrelated recording."""
    return Time(nanoseconds=round(value*1e9)).to_msg()


def fixture_nodes(classes, clock, contract):
    """Substitute clock/publication transport only; run real node callbacks and DKF kernels."""
    nodes = []
    for cls in classes:
        node = cls.__new__(cls)
        node.get_clock = lambda: Row(now=lambda: Row(nanoseconds=round(clock.value*1e9)))
        node.last_state, node.last_ack = None, {}
        node.state_pub = Row(publish=lambda message, n=node: setattr(n, 'last_state', message))
        node.ack_pub = Row(publish=lambda message, n=node:
                           setattr(n, 'last_ack', json.loads(message.data)))
        node.p76_phase, node.p76_epoch, node.p76_y_stamp = 'FOLLOW_ACTIVE', 0, 0.
        node.navigation_frame_fault, node.last_now = '', None
        nodes.append(node)
    spatial, image = nodes
    spatial.filter = DelayedSpatialKalman()
    image.contract = contract
    camera = contract['camera']
    focal = camera['width']/(2*math.tan(camera['horizontal_fov']/2))
    image.calibration = (camera['width']/2, camera['height']/2, focal, focal)
    pitch = math.radians(camera['pitch_down_degrees'])
    c, s = math.cos(pitch), math.sin(pitch)
    image.camera_to_body = np.diag((1., -1., -1.))@np.array(
        ((c, 0., s), (0., 1., 0.), (-s, 0., c)))@np.array(
            ((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)))
    image.filter = DelayedImageKalman(image.calibration)
    image.history = NativePoseHistory()
    image.pending, image.cached = deque(maxlen=12), deque(maxlen=12)
    image.seen_detections = deque(maxlen=256)
    image.latest_packet, image.last_reason, image.filter_epoch = None, 'UNINITIALIZED', 0
    image.navigation_stamp = 0.
    return spatial, image


def percentile(values):
    """Report callback wall duration, including callback math and serialization."""
    return dict(count=len(values), p95_ms=float(np.percentile(values, 95))*1e3,
                p99_ms=float(np.percentile(values, 99))*1e3, maximum_ms=max(values)*1e3)


def run_fixture(classes, seconds, contract, parallel):
    """Identical synthetic 20 Hz image/50 Hz navigation inputs, with a midpoint Y transition."""
    clock = Row(value=100.)
    spatial, image = fixture_nodes(classes, clock, contract)
    durations, ages = defaultdict(list), []
    states_seen = defaultdict(int)
    image_valid_at_y, jump = False, None
    spatial_updates_after_y, illegal, overrun = 0, 0, 0
    last_spatial_source = 0.
    events = []

    def call(label, callback, *args):
        started = time.perf_counter()
        callback(*args)
        durations[label].append(time.perf_counter()-started)

    start_wall, start_cpu = time.perf_counter(), time.process_time()
    start_thread = time.thread_time()
    steps, handover = round(seconds*100), round(seconds*50)
    for index in range(steps+1):
        clock.value = 100.+index*.01
        iteration_start = time.perf_counter()
        if index % 2 == 0:
            epoch = clock.value-.01  # An explicitly generated native delivery delay of 10 ms.
            pose = NativeAttitude(stamp=ros_stamp(epoch), quaternion_wxyz=[1., 0., 0., 0.],
                                  angular_velocity_frd=[0., 0., 0.],
                                  orientation_variance=[1e-6]*3, valid=True)
            navigation = UavState(stamp=ros_stamp(epoch), frame_id='local_ned', valid=True)
            call('image.pose', image.pose, pose)
            call('image.navigation', image.navigation, navigation)
        if index == handover:
            call('image.tick', image.tick)
            before = image.last_state
            for node in (spatial, image):
                node.p76_phase, node.p76_epoch, node.p76_y_stamp = 'Y_HANDOVER', 1, clock.value
            call('image.mode_changed', image.mode_changed)
            call('image.tick', image.tick)
            image_valid_at_y = image.last_state.valid
            if before and before.valid and image.last_state.valid:
                jump = float(np.linalg.norm(np.array(before.state)-image.last_state.state))
        if index >= 5 and index % 5 == 0:
            source = clock.value-.03
            packet = TargetImageDetection(
                stamp=ros_stamp(source), raw_stamp=ros_stamp(source),
                detection_stamp=ros_stamp(clock.value-.005), frame_sequence=index//5,
                image_width=640, image_height=480,
                center_uv=[320.+10.*math.sin(index*.01), 240.+2.*math.cos(index*.01)],
                mask_counts=[0, 3, 640*480-3], confidence=.9, valid=True,
                class_name='red_ball', model_sha256=contract['detector']['model_sha256'])
            measurement = TargetObservation(stamp=ros_stamp(source), frame_id='local_ned',
                                            covariance=(np.eye(3)*.01).reshape(-1).tolist(),
                                            valid=True)
            measurement.position.x = 5.+.1*index*.01
            call('spatial.receive', spatial.receive, measurement)
            call('image.receive', image.receive, packet)
            ages.append(clock.value-source)
        if index % 2 == 0:
            call('spatial.tick', spatial.tick)
            call('image.tick', image.tick)
            for name, node in (('spatial', spatial), ('image', image)):
                ack = node.last_ack
                states_seen[name+'_active_ticks'] += bool(ack.get('active'))
                states_seen[name+'_ready_ticks'] += bool(ack.get('ready'))
                if parallel:
                    expected = (name == 'spatial') == (index < handover)
                    illegal += bool(ack.get('control_authorized')) and not expected
            if spatial.last_state is not None:
                latest = stamp_seconds(spatial.last_state.source_stamp)
                spatial_updates_after_y += index > handover and latest > last_spatial_source
                last_spatial_source = latest
            events.append(dict(t=clock.value, phase=image.p76_phase,
                               spatial_ack=spatial.last_ack, image_ack=image.last_ack))
        overrun += time.perf_counter()-iteration_start > .02
    cpu, wall = time.process_time()-start_cpu, time.perf_counter()-start_wall
    thread_cpu = time.thread_time()-start_thread
    return dict(callbacks={name: percentile(values) for name, values in durations.items()},
                logical_seconds=seconds, cpu_seconds=cpu, wall_seconds=wall,
                process_cpu_seconds_all_threads=cpu, calling_thread_cpu_seconds=thread_cpu,
                process_cpu_fraction_at_realtime=cpu/seconds,
                calling_thread_cpu_fraction_at_realtime=thread_cpu/seconds,
                image_valid_at_y=bool(image_valid_at_y), image_same_epoch_y_jump_pixels=jump,
                spatial_updates_after_y=spatial_updates_after_y,
                illegal_authorizations=int(illegal), timer_ticks_50hz=steps//2+1,
                image_source_age_ms=percentile(ages), iterations_exceeding_20ms=overrun,
                **states_seen), events


def benchmark_fixture(seconds=4., include_events=False):
    """A callback fixture comparison cannot qualify ROS executor/Tracker scheduling."""
    if not math.isfinite(seconds) or not .4 <= seconds <= 30.:
        raise ValueError('FIXTURE_DURATION_REQUIRES_0_4_TO_30_SECONDS')
    contract = load_contract(CONTRACT)
    old, old_events = run_fixture(legacy_classes(), seconds, contract, False)
    new, new_events = run_fixture((SpatialDkfNode, ImageDkfNode), seconds, contract, True)
    result = dict(evidence_class='SYNTHETIC_CALLBACK_FIXTURE', legacy_ref=LEGACY_REF,
                  fixture='20Hz synthetic RGB-D/YOLO; 50Hz synthetic pose/nav; identity attitude',
                  thread_environment={k: v for k, v in os.environ.items()
                                      if k.endswith('NUM_THREADS')},
                  process_thread_count=len(list(Path('/proc/self/task').iterdir())),
                  legacy_exclusive=old, parallel=new, tracker_scheduling_qualified=False,
                  limitations=['No ROS DDS/executor transport or 200Hz Tracker in this fixture',
                               'No inference or RGB-D localization cost in callback timings',
                               'Serial replay CPU fraction estimates cost, not real runtime load'])
    if include_events:
        result['events'] = dict(legacy_exclusive=old_events, parallel=new_events)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--detector-log', type=Path, default=ROOT /
                        ('data/baselines/20261021_p761_completion_01/detector/live_probe/'
                         'packets.jsonl.gz'))
    parser.add_argument('--navigation-log', type=Path, default=ROOT /
                        ('data/baselines/20261021_p762_calibration_01/session_A1_03/'
                         'calibration.jsonl.gz'))
    parser.add_argument('--fixture-seconds', type=float, default=4.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    coverage = audit_real_inputs(args.detector_log, args.navigation_log)
    comparison = benchmark_fixture(args.fixture_seconds, include_events=True)
    events = comparison.pop('events')
    with (args.output/'fixture_events.jsonl').open('x') as stream:
        for architecture, samples in events.items():
            for event in samples:
                stream.write(json.dumps(dict(architecture=architecture, **event))+'\n')
    tracking = ROOT/'src/uav_control/uav_control/tracking'
    inputs = [Path(__file__), CONTRACT]+list(tracking.glob('*dkf_node.py'))+[
        ROOT/'src/uav_control/uav_control/common/p76_phase.py',
        tracking/'delayed_image_kalman.py', tracking/'delayed_spatial_kalman.py']
    result = dict(schema_version=1, e1_matched_real_replay_passed=False,
                  coverage=coverage, fixture=comparison,
                  source_sha256={str(path): digest(path) for path in inputs})
    (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    hashes = {str(path): digest(path) for path in args.output.iterdir() if path.is_file()}
    checksums = ''.join(value+'  '+name+'\n' for name, value in sorted(hashes.items()))
    (args.output/'SHA256SUMS').write_text(checksums)
    print(json.dumps(dict(output=str(args.output),
                          e1_matched_real_replay_passed=False, blockers=coverage['blockers'],
                          fixture_image_valid_at_y=comparison['parallel']['image_valid_at_y'])))


if __name__ == '__main__':
    main()
