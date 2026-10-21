"""Continuous delayed pixel filter using shared YOLO and native acquisition-pose brackets."""
from collections import deque
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from uav_usv_interfaces.msg import NativeAttitude, TargetImageDetection, TargetImageState, UavState

from uav_control.common.navigation_frame import install_navigation_frame_guard
from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_native import NativePoseHistory
from uav_control.common.p76_phase import estimator_active, install_phase_gate, publish_ack
from uav_control.perception.yolo_detection_contract import detection_mask, raw_key, stamp_seconds
from .delayed_image_kalman import DelayedImageKalman


class ImageDkfNode(Node):
    """Warm in FOLLOW, preserve the posterior at Y and authorize only image guidance."""

    def __init__(self):
        super().__init__('image_dkf_node')
        self.declare_parameter('contract_file', '')
        self.contract = load_contract(self.get_parameter('contract_file').value)
        camera = self.contract['camera']
        focal = camera['width']/(2*math.tan(camera['horizontal_fov']/2))
        self.calibration = (camera['width']/2, camera['height']/2, focal, focal)
        pitch = math.radians(camera['pitch_down_degrees'])
        c, s = math.cos(pitch), math.sin(pitch)
        self.camera_to_body = np.diag((1., -1., -1.))@np.array(
            ((c, 0., s), (0., 1., 0.), (-s, 0., c)))@np.array(
            ((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)))
        filters, timing = self.contract['filters'], self.contract['timing']
        self.filter = DelayedImageKalman(
            self.calibration, filters['image_acceleration_std'],
            filters['image_initial_velocity_std'], timing['history_seconds'],
            timing['maximum_source_age'])
        self.history = NativePoseHistory(timing['maximum_pose_bracket'])
        self.pending, self.cached = deque(maxlen=12), deque(maxlen=12)
        self.seen_detections = deque(maxlen=256)
        self.latest_packet, self.last_now, self.last_reason = None, None, 'UNINITIALIZED'
        self.filter_epoch = 0
        self.navigation_stamp = 0.
        self.state_pub = self.create_publisher(TargetImageState, '/tracking/image_state', 10)
        self.ack_pub = self.create_publisher(String, '/tracking/p76_filter_ack', 10)
        self.declare_parameter('p76_mode_topic', '/control/p76_mode')
        install_phase_gate(self, self.mode_changed)
        install_navigation_frame_guard(self, self.frame_failed)
        self.image_sub = self.create_subscription(TargetImageDetection,
                                                  '/perception/target_detection', self.receive, 10)
        self.pose_sub = self.create_subscription(NativeAttitude, '/navigation/native_attitude',
                                                 self.pose, 16)
        self.navigation_sub = self.create_subscription(UavState, '/navigation/uav_state',
                                                       self.navigation, 16)
        self.timer = self.create_timer(1/self.contract['paper']['dkf_rate_hz'], self.tick)

    def frame_failed(self, reason):
        """Reject frame resets even when a fresh target detection arrives."""
        self.history.clear()
        self.filter.reset()
        self.pending.clear()
        self.cached.clear()
        self.seen_detections.clear()
        self.latest_packet, self.last_reason = None, str(reason)
        self.navigation_stamp = 0.

    def mode_changed(self):
        """Relabel normal authority epochs without resetting any state, covariance or history."""
        if not estimator_active(self):
            self.frame_failed('IMAGE_DKF_PHASE_INACTIVE')
        self.filter_epoch = self.p76_epoch

    def check_clock(self, now):
        """Clear histories before any callback admits data after a clock rollback."""
        if self.last_now is not None and now < self.last_now:
            self.frame_failed('IMAGE_DKF_CLOCK_RESET')
        self.last_now = now

    def receive(self, message):
        """Consume each original detector frame once, including across Y relabelling."""
        self.check_clock(self.get_clock().now().nanoseconds*1e-9)
        if not estimator_active(self) or getattr(self, 'navigation_frame_fault', ''):
            return
        identity = (raw_key(message.raw_stamp), int(message.frame_sequence))
        if any(identity[0] == raw or identity[1] == frame
               for raw, frame in self.seen_detections):
            self.last_reason = 'IMAGE_DKF_DUPLICATE_DETECTION'
            return
        self.seen_detections.append(identity)
        self.cached.append(message)
        self.pending.append(message)
        self.tick()

    def pose(self, message):
        """Store native attitudes independently of the 3D target filter."""
        self.check_clock(self.get_clock().now().nanoseconds*1e-9)
        if not message.valid or getattr(self, 'navigation_frame_fault', ''):
            return
        try:
            self.history.add(stamp_seconds(message.stamp), message.quaternion_wxyz,
                             message.angular_velocity_frd, message.orientation_variance)
        except ValueError as error:
            self.last_reason = str(error)

    def navigation(self, message):
        """Bound projection by actual navigation availability, without reading any 3D target."""
        from uav_control.common.navigation_state import navigation_components
        now = self.get_clock().now().nanoseconds*1e-9
        self.check_clock(now)
        if getattr(self, 'navigation_frame_fault', ''):
            return
        try:
            stamp, _, _, _ = navigation_components(message, now)
            self.navigation_stamp = max(self.navigation_stamp, stamp)
        except ValueError:
            return

    def tick(self):
        """Replay historical updates and project at a real delivered pose, never ROS-now pose."""
        now = self.get_clock().now().nanoseconds*1e-9
        self.check_clock(now)
        active = estimator_active(self)
        if getattr(self, 'navigation_frame_fault', ''):
            active = False
        if not active:
            publish_ack(self, 'image', False)
            return
        remaining = deque(maxlen=12)
        for packet in self.pending:
            try:
                detection_mask(packet, now, self.contract['detector']['model_sha256'])
                source = stamp_seconds(packet.stamp)
                if source > self.navigation_stamp:
                    raise ValueError('IMAGE_DKF_NATIVE_POSE_BRACKET')
                pose = self.history.at(source)
                rotation = pose.rotation@self.camera_to_body
                covariance = np.eye(2)*self.contract['filters']['image_measurement_std']**2
                if self.filter.update(packet.center_uv, covariance, source, now, rotation):
                    if (self.latest_packet is None or source
                            > stamp_seconds(self.latest_packet.stamp)):
                        self.latest_packet = packet
            except ValueError as error:
                self.last_reason = str(error)
                if 'BRACKET' in str(error) and now-stamp_seconds(packet.stamp) <= .125:
                    remaining.append(packet)
        self.pending = remaining
        ready, source = False, 0.
        result = TargetImageState(mode_epoch=self.p76_epoch, rejection_reason=self.last_reason)
        try:
            if not self.history.samples or self.latest_packet is None:
                raise ValueError('IMAGE_DKF_WAITING_NATIVE_POSE_OR_IMAGE')
            pose = self.history.at(min(self.history.samples[-1][0], self.navigation_stamp))
            source = stamp_seconds(self.latest_packet.stamp)
            if not 0 <= now-pose.stamp <= .125 or not 0 <= now-source <= .125:
                raise ValueError('IMAGE_DKF_STALE')
            if pose.stamp < source:
                raise ValueError('IMAGE_DKF_POSE_BEFORE_SOURCE')
            estimate = self.filter.predict(
                pose.stamp, pose.rotation@self.camera_to_body,
                self.camera_to_body.T@pose.omega,
                self.camera_to_body.T@np.diag(pose.variance)@self.camera_to_body)
            result.stamp = Time(nanoseconds=round(pose.stamp*1e9)).to_msg()
            result.pose_stamp = result.stamp
            result.source_stamp, result.raw_source_stamp = (
                self.latest_packet.stamp, self.latest_packet.raw_stamp)
            result.state = estimate.state.tolist()
            result.covariance = estimate.covariance.reshape(-1).tolist()
            result.body_quaternion.w = float(pose.quaternion[0])
            result.body_quaternion.x, result.body_quaternion.y, result.body_quaternion.z = (
                map(float, pose.quaternion[1:]))
            result.frame_sequence = self.latest_packet.frame_sequence
            result.model_sha256 = self.latest_packet.model_sha256
            result.valid = bool(estimate.valid)
            ready = result.valid
            result.rejection_reason = '' if ready else estimate.reason
        except ValueError as error:
            result.rejection_reason = str(error)
        self.state_pub.publish(result)
        publish_ack(self, 'image', True, ready, source, result.rejection_reason)


def main(args=None):
    """Run the opt-in image estimator without a 3D target-state subscription."""
    rclpy.init(args=args)
    node = ImageDkfNode()
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
