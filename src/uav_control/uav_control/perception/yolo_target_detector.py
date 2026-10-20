"""One acquisition-timed ROS detector for the explicit P76 formal perception chain."""
from dataclasses import replace
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from uav_usv_interfaces.msg import TargetImageDetection

from uav_control.common.p76_contract import load_contract
from uav_control.guidance.camera_visibility import CameraIntrinsics
from .rgbd_target_localizer import GazeboImageClockMapper, aligned_camera_qos
from .yolo_inference import Detection, YoloInference


def time_message(seconds):
    """Serialize an actual supplied epoch, never invent a missing source timestamp."""
    nanoseconds = max(0, round(seconds*1e9))
    return Time(sec=nanoseconds//1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


class YoloTargetDetector(Node):
    """CPU inference with a latest-frame queue and the original trusted image clock mapping."""

    def __init__(self):
        super().__init__('yolo_target_detector')
        self.declare_parameter('contract_file', '')
        self.contract = load_contract(self.get_parameter('contract_file').value)
        detector = self.contract['detector']
        self.inference = YoloInference(detector['model_file'], detector['model_sha256'],
                                       detector['input_size'], detector['confidence_threshold'],
                                       detector['nms_iou'], detector['cpu_threads'])
        self.publisher = self.create_publisher(
            TargetImageDetection, '/perception/target_detection', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/front/camera_info',
                                              aligned_camera_qos())
        self.status_pub = self.create_publisher(String, '/diagnostics/p76_yolo', 10)
        self.processing_guard = self.create_guard_condition(self.process)
        self.subscription = self.create_subscription(Image, '/camera/front/image_raw',
                                                     self.image_callback, aligned_camera_qos())
        self.clock = GazeboImageClockMapper(maximum_reference_age=.5, history_duration=2.,
                                            maximum_system_clock_step=.25)
        self.lock = threading.RLock()
        self.pending = None
        self.sequence = 0
        self.last_raw = 0
        self.clock_resets = 0
        self.waiting_clock = False
        from gz.msgs10.clock_pb2 import Clock
        from gz.transport13 import Node as GazeboNode
        self.gz = GazeboNode()
        if not self.gz.subscribe(Clock, '/world/default/clock', self.clock_callback):
            raise RuntimeError('YOLO_TRUSTED_CLOCK_UNAVAILABLE')
        self.timer = self.create_timer(1./self.contract['paper']['camera_rate_hz'], self.process)

    def now(self):
        """Return the ROS/system epoch for age checks only."""
        return self.get_clock().now().nanoseconds*1e-9

    def clock_callback(self, clock):
        """Maintain the original server clock correspondence and clear reset epochs."""
        with self.lock:
            before = self.clock.reset_count
            self.clock.add_anchor(clock.sim.sec+clock.sim.nsec*1e-9,
                                  clock.system.sec+clock.system.nsec*1e-9,
                                  self.now(), time.monotonic())
            if self.clock.reset_count != before:
                self.pending, self.last_raw = None, 0
                self.clock_resets += 1
            elif self.waiting_clock and self.pending is not None:
                self.processing_guard.trigger()

    def image_callback(self, image):
        """Drop duplicate/out-of-order frames without doing inference in DDS callbacks."""
        raw = image.header.stamp.sec*1_000_000_000+image.header.stamp.nanosec
        with self.lock:
            if raw <= self.last_raw:
                return
            self.last_raw = raw
            self.sequence += 1
            self.pending = (image, self.sequence, self.clock_resets, time.monotonic())
            self.processing_guard.trigger()

    def process(self):
        """Publish one authoritative valid/rejected result for each selected original frame."""
        with self.lock:
            if self.pending is None:
                return
            image, sequence, reset, received = self.pending
            source, reason = self.clock.map_time(image.header.stamp.sec
                                                 + image.header.stamp.nanosec*1e-9, self.now())
            if source is None and time.monotonic()-received <= .125:
                self.waiting_clock = True
                return
            self.pending = None
            self.waiting_clock = False
        result = Detection(reason=reason or 'INVALID_IMAGE')
        if source is not None and 0 <= self.now()-source <= .125:
            try:
                channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}[image.encoding.lower()]
                pixels = np.frombuffer(image.data, np.uint8).reshape(image.height, image.step)
                rgb = pixels[:, :image.width*channels].reshape(image.height, image.width, channels)
                rgb = rgb[:, :, :3]
                if image.encoding.lower().startswith('bgr'):
                    rgb = rgb[:, :, ::-1]
                result = self.inference.infer(np.ascontiguousarray(rgb))
            except (ValueError, KeyError, RuntimeError) as exception:
                result = Detection(reason='INFERENCE_ERROR:'+type(exception).__name__)
        completed = self.now()
        if source is not None and completed-source > .125:
            result = replace(result, valid=False, reason='YOLO_SOURCE_STALE_AFTER_INFERENCE')
        with self.lock:
            if reset != self.clock_resets:
                result = replace(result, valid=False, reason='CLOCK_RESET')
        message = TargetImageDetection()
        message.stamp = time_message(source or 0.)
        message.raw_stamp = image.header.stamp
        message.detection_stamp = time_message(completed)
        message.frame_sequence = sequence
        message.image_width, message.image_height = image.width, image.height
        message.center_uv, message.bounding_box = list(result.center_uv), list(result.bounding_box)
        message.mask_counts = list(result.mask_counts)
        message.confidence, message.valid = result.confidence, result.valid
        message.class_name, message.rejection_reason = 'red_ball', result.reason
        message.model_sha256 = self.contract['detector']['model_sha256']
        self.publisher.publish(message)
        camera = self.contract['camera']
        intr = CameraIntrinsics.from_horizontal_fov(camera['width'], camera['height'],
                                                    camera['horizontal_fov'])
        info = CameraInfo()
        info.header = image.header
        info.header.frame_id = 'front_camera_optical_frame'
        info.width, info.height = camera['width'], camera['height']
        info.k = [intr.fx, 0., intr.cx, 0., intr.fy, intr.cy, 0., 0., 1.]
        self.info_pub.publish(info)
        import json
        status = String()
        status.data = json.dumps(dict(source_stamp=source, completed=completed, frame=sequence,
                                      valid=result.valid, reason=result.reason,
                                      inference_seconds=result.inference_seconds,
                                      source_age=completed-source if source else None))
        self.status_pub.publish(status)


def main(args=None):
    """Run only a pinned real model; a missing model is an explicit startup failure."""
    rclpy.init(args=args)
    node = YoloTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
