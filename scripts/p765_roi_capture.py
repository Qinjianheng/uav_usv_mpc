#!/usr/bin/env python3
"""Prepare one bounded raw ROI recorder; starting a live session needs separate approval."""
import argparse
import hashlib
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--max-samples', type=int, default=60)
    parser.add_argument('--max-per-stratum', type=int, default=10)
    parser.add_argument('--max-bytes', type=int, default=64*1024*1024)
    parser.add_argument('--capacity', type=int, default=24)
    args = parser.parse_args()
    if not (0 < args.duration <= 60 and 1 <= args.capacity <= 64
            and 1 <= args.max_samples <= 200 and 1 <= args.max_per_stratum <= 100
            and 0 < args.max_bytes <= 256*1024*1024):
        parser.error('bounded capture limits exceeded')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleStatus
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from uav_usv_interfaces.msg import TargetImageDetection
    from uav_control.perception.p765_roi_capture import (
        BoundedRoiWriter, RoiCaptureMatcher, image_array, runtime_identity,
    )
    from uav_control.perception.rgbd_target_localizer import aligned_camera_qos

    class Capture(Node):
        """Own only subscriptions and disk I/O, with bounded acquisition-keyed queues."""

        def __init__(self):
            super().__init__('p765_raw_roi_capture')
            identity = runtime_identity()
            identity['capture_script'] = dict(
                path=str(Path(__file__).resolve()),
                sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
            writer = BoundedRoiWriter(args.output, runtime_identity=identity,
                                      max_samples=args.max_samples,
                                      max_per_stratum=args.max_per_stratum,
                                      max_bytes=args.max_bytes)
            self.matcher, self.failure = RoiCaptureMatcher(writer, args.capacity), ''
            self.subscriptions_owned = [
                self.create_subscription(Image, '/camera/front/image_raw',
                                         lambda msg: self.frame('rgb', msg), aligned_camera_qos()),
                self.create_subscription(
                    Image, '/camera/front/depth/image_raw',
                    lambda msg: self.frame('depth', msg), aligned_camera_qos()),
                self.create_subscription(TargetImageDetection, '/perception/target_detection',
                                         self.detection, 100),
                self.create_subscription(
                    String, '/diagnostics/p765_sphere_fit', self.diagnostic, 100),
                self.create_subscription(
                    String, '/diagnostics/p76_yolo', self.detector_status, 100),
            ]
            version = getattr(VehicleStatus, 'MESSAGE_VERSION', 0)
            topic = '/fmu/out/vehicle_status'+(f'_v{version}' if version else '')
            self.subscriptions_owned.append(self.create_subscription(
                VehicleStatus, topic, self.status, qos_profile_sensor_data))

        def status(self, message):
            if message.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
                self.failure = 'UNEXPECTED_ARMED_STATE'

        def frame(self, name, message):
            receipt = self.get_clock().now().nanoseconds
            header = message_to_ordereddict(message.header)
            metadata = dict(header=header, receipt_ns=receipt, width=message.width,
                            height=message.height, encoding=message.encoding, step=message.step,
                            is_bigendian=message.is_bigendian)
            try:
                self.matcher.add_frame(name, header['stamp'], image_array(message), metadata)
            except ValueError as error:
                self.matcher.writer.note(metadata, reason=str(error))

        def detection(self, message):
            self.matcher.add_detection(message_to_ordereddict(message),
                                       receipt_ns=self.get_clock().now().nanoseconds)

        def diagnostic(self, message):
            self.matcher.add_diagnostic(json.loads(message.data),
                                        receipt_ns=self.get_clock().now().nanoseconds)

        def detector_status(self, message):
            self.matcher.add_detector_status(json.loads(message.data),
                                             receipt_ns=self.get_clock().now().nanoseconds)

    rclpy.init()
    node = Capture()
    began = time.monotonic()
    try:
        while (rclpy.ok() and not node.failure and not node.matcher.failure
               and time.monotonic()-began < args.duration
               and node.matcher.writer.events_count < node.matcher.writer.event_limit):
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        node.failure = 'INTERRUPTED'
    finally:
        elapsed = time.monotonic()-began
        node.failure = node.failure or node.matcher.failure
        node.matcher.close(terminal_metadata=dict(duration_s=elapsed, failure=node.failure))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.failure:
        raise SystemExit(node.failure)


if __name__ == '__main__':
    main()
