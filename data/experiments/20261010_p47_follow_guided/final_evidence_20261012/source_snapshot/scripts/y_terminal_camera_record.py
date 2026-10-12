#!/usr/bin/env python3
"""Read-only paired terminal RGB-D capture, with original sensor timestamps."""

import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from uav_usv_interfaces.msg import MissionState, InterceptResult


class CameraRecorder(Node):
    def __init__(self, output):
        super().__init__('terminal_camera_evidence')
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        self.pending = {}
        self.count = 0
        self.started = time.monotonic()
        self.active = False
        self.seen_terminal = False
        self.finished = False
        qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_ = [
            self.create_subscription(MissionState, '/mission/state', self.phase, qos),
            self.create_subscription(InterceptResult, '/simulation/impact/result',
                                     lambda _: setattr(self, 'finished', True), qos),
        ]
        for kind, topic in (('rgb', '/camera/front/image_raw'),
                            ('depth', '/camera/front/depth/image_raw')):
            self.subscriptions_.append(self.create_subscription(
                Image, topic, lambda msg, kind=kind: self.receive(kind, msg), qos,
            ))
        self.create_timer(.5, self.tick)

    def phase(self, message):
        self.active = message.state_name in ('MINCO_TRACKING', 'TERMINAL_MINCO')
        self.seen_terminal |= self.active

    def receive(self, kind, message):
        if not self.active or self.count >= 100:
            return
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        self.pending.setdefault(key, {})[kind] = message
        if len(self.pending) > 4:
            self.pending.pop(next(iter(self.pending)))
        pair = self.pending.get(key, {})
        if len(pair) != 2:
            return
        self.pending.pop(key)
        color, depth = pair['rgb'], pair['depth']
        if color.encoding not in ('rgb8', 'bgr8') or depth.encoding != '32FC1':
            return
        rgb = np.frombuffer(color.data, np.uint8).reshape(color.height, color.step)
        rgb = rgb[:, :color.width * 3].reshape(color.height, color.width, 3)
        if color.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1]
        dtype = '>f4' if depth.is_bigendian else '<f4'
        pixels = np.frombuffer(depth.data, dtype).reshape(depth.height, depth.step // 4)
        pixels = pixels[:, :depth.width]
        np.savez_compressed(
            self.output / f'frame_{self.count:03d}.npz', rgb=rgb, depth=pixels,
            raw_stamp=np.array(key),
            receipt=self.get_clock().now().nanoseconds * 1e-9,
        )
        self.count += 1

    def tick(self):
        if self.finished or time.monotonic() - self.started > 180:
            (self.output / 'capture.json').write_text(json.dumps({
                'count': self.count, 'role': 'OFFLINE_CAMERA_EVIDENCE',
                'paired_by_original_sensor_stamp': True,
            }, indent=2) + '\n')
            raise SystemExit(0)


def main():
    rclpy.init()
    node = CameraRecorder(Path(sys.argv[1]))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
