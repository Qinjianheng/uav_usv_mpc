#!/usr/bin/env python3
"""Evaluation-only ROS topic and process-load sample for render A/B runs."""

import argparse
from collections import defaultdict
import json
from functools import partial
import math
from pathlib import Path
import statistics
import time

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from uav_usv_interfaces.msg import TargetObservation


IMAGE_TOPICS = (
    '/camera/front/image_raw', '/camera/front/depth/image_raw',
    '/camera/down/image_raw', '/camera/down/depth/image_raw',
)


def describe(values):
    if not values:
        return {'count': 0}
    ordered = sorted(values)

    def percentile(q):
        location = (len(ordered) - 1) * q
        low = int(location)
        high = min(low + 1, len(ordered) - 1)
        return ordered[low] + (ordered[high] - ordered[low]) * (location - low)
    return {'count': len(values), 'min': ordered[0], 'p05': percentile(.05),
            'p50': percentile(.50), 'p95': percentile(.95), 'max': ordered[-1],
            'mean': statistics.fmean(values),
            'std': statistics.pstdev(values)}


def low_rtf_episodes(samples, threshold):
    durations = []
    current = 0.0
    for (start, value), (end, _) in zip(samples, samples[1:]):
        if value < threshold:
            current += min(max(end - start, 0.0), 1.0)
        elif current:
            durations.append(current)
            current = 0.0
    if current:
        durations.append(current)
    return {'sample_count': sum(value < threshold for _, value in samples),
            'episode_count': len(durations),
            'duration_s': sum(durations), 'episodes_s': durations}


class PerfCapture(Node):
    def __init__(self):
        super().__init__('sim_render_perf_capture')
        self.start = time.monotonic()
        self.rtf = []
        self.images = defaultdict(list)
        self.image_meta = {}
        self.compute = defaultdict(list)
        self.valid = defaultdict(int)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Float32, '/simulation/gazebo/real_time_factor',
            self.on_rtf, 10)
        for topic in IMAGE_TOPICS:
            self.create_subscription(
                Image, topic, partial(self.on_image, topic), qos)
        for camera in ('front', 'down'):
            topic = f'/diagnostics/{camera}/monitor_compute_time'
            self.create_subscription(
                Float32, topic, partial(self.on_compute, camera), 10)
        self.create_subscription(
            TargetObservation, '/perception/front/target_observation',
            self.on_observation, 10)

    def on_rtf(self, message):
        self.rtf.append((time.monotonic(), float(message.data)))

    def on_compute(self, camera, message):
        self.compute[camera].append(float(message.data))

    def on_observation(self, message):
        key = 'valid' if message.valid else 'invalid'
        self.valid[key] += 1

    def on_image(self, topic, message):
        now = time.monotonic()
        stamp = (message.header.stamp.sec
                 + message.header.stamp.nanosec * 1e-9)
        self.images[topic].append((now, stamp))
        self.image_meta[topic] = {
            'width': int(message.width), 'height': int(message.height),
            'step': int(message.step), 'encoding': message.encoding,
            'frame_id': message.header.frame_id,
        }


def process_load():
    rows = []
    for process in psutil.process_iter(
            ('pid', 'name', 'cmdline', 'cpu_percent')):
        try:
            command = ' '.join(process.info['cmdline'] or [])
            name = process.info['name'] or ''
            if any(marker in command or marker in name for marker in (
                    'gz sim', 'ruby .*gz', 'ros_gz_image',
                    'front_tof_monitor', 'rgbd_target_localizer',
                    'QGroundControl', 'px4')):
                rows.append({'pid': process.info['pid'], 'name': name,
                             'command': command[:250],
                             'cpu_percent': (
                                 process.info['cpu_percent'] or 0.0)})
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=40)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase', required=True)
    args = parser.parse_args()
    if args.seconds <= 0 or args.output.exists():
        parser.error('seconds must be positive and output must not exist')
    rclpy.init()
    node = PerfCapture()
    process_load()
    loads = []
    next_load = time.monotonic()
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() >= next_load:
                loads.append(process_load())
                next_load += 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    images = {}
    for topic in IMAGE_TOPICS:
        pairs = node.images[topic]
        arrival = [b[0] - a[0] for a, b in zip(pairs, pairs[1:])]
        stamps = [b[1] - a[1] for a, b in zip(pairs, pairs[1:])]
        images[topic] = {
            'frames': len(pairs), 'arrival_period_s': describe(arrival),
            'stamp_period_s': describe(stamps),
            'average_hz': (
                (len(pairs) - 1) / (pairs[-1][0] - pairs[0][0])
                if len(pairs) > 1 else math.nan),
            'metadata': node.image_meta.get(topic),
        }
    output = {'phase': args.phase, 'seconds': args.seconds,
              'rtf': describe([value for _, value in node.rtf]),
              'rtf_below_095': low_rtf_episodes(node.rtf, .95),
              'rtf_below_090': low_rtf_episodes(node.rtf, .90),
              'images': images,
              'monitor_compute_s': {name: describe(values) for name, values
                                    in node.compute.items()},
              'observation_counts': dict(node.valid), 'process_load': loads}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2, allow_nan=True)
    print(json.dumps({key: output[key] for key in
                      ('phase', 'rtf', 'rtf_below_095', 'rtf_below_090',
                       'observation_counts')}, indent=2))


if __name__ == '__main__':
    main()
