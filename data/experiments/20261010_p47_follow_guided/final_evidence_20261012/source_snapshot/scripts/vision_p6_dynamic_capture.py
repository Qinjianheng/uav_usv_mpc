#!/usr/bin/env python3
"""Reuse static P6 capture and save KF states for offline paired audit."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import rclpy
from uav_usv_interfaces.msg import TargetState

from vision_static_capture import StaticVisionCapture, stamp_seconds


def kf_record(message, receipt_stamp):
    return {
        'stamp': stamp_seconds(message.stamp),
        'source_stamp': stamp_seconds(message.source_stamp),
        'receipt_stamp': receipt_stamp,
        'valid': bool(message.valid),
        'frame_id': str(message.frame_id),
        'position_ned': [message.position.x, message.position.y,
                         message.position.z],
        'velocity_ned': [message.velocity.x, message.velocity.y,
                         message.velocity.z],
        'covariance': list(message.covariance),
    }


class DynamicVisionCapture(StaticVisionCapture):
    def __init__(self, output_path):
        super().__init__(output_path, visual_height_offset=0.42)
        self.kf_path = output_path.with_name('kf_states.jsonl')
        self.kf_stream = self.kf_path.open('x', encoding='utf-8', buffering=1)
        self.create_subscription(
            TargetState, '/tracking/target_state', self.kf_callback, 10,
        )

    def kf_callback(self, message):
        receipt = self.get_clock().now().nanoseconds * 1e-9
        self.kf_stream.write(json.dumps(kf_record(message, receipt)) + '\n')
        self.counts['kf_state'] += 1

    def close(self):
        super().close()
        self.kf_stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--output-directory', type=Path, required=True)
    args = parser.parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output = args.output_directory / f'vision_dynamic_capture_{stamp}.csv'
    rclpy.init()
    node = DynamicVisionCapture(output)
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(output)
    print(dict(node.counts))


if __name__ == '__main__':
    main()
