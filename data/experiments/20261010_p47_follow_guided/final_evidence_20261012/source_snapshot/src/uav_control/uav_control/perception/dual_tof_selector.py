"""Hysteretic selection of forward and downward ToF diagnostics."""

from dataclasses import dataclass
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, String, UInt64


@dataclass
class CameraObservation:
    visible: bool = False
    tof_valid: bool = False
    target_range: float = math.nan
    depth_valid_ratio: float = 0.0
    visibility_rate: float = 0.0
    tof_valid_rate: float = 0.0
    frame_change: float = 0.0
    stream_alive: bool = False
    frame_count: int = 0
    red_pixel_count: int = 0
    horizontal_angle: float = math.nan
    vertical_angle: float = math.nan
    truth_in_fov: bool = False

    @property
    def available(self):
        return self.visible or self.tof_valid


class HystereticCameraSelector:
    """Prefer front view, but retain a usable camera through brief dropouts."""

    def __init__(
        self,
        loss_frames=3,
        front_recovery_frames=10,
        allow_down_fallback=True,
    ):
        self.loss_frames = max(int(loss_frames), 1)
        self.front_recovery_frames = max(
            int(front_recovery_frames),
            1,
        )
        self.allow_down_fallback = bool(allow_down_fallback)
        self.active = 'front'
        self.loss_count = 0
        self.front_recovery_count = 0

    def update(self, front_available, down_available):
        front_available = bool(front_available)
        down_available = bool(down_available)
        previous = self.active

        if self.active == 'front':
            self.front_recovery_count = 0
            if front_available:
                self.loss_count = 0
            else:
                self.loss_count += 1
                if (
                    self.allow_down_fallback
                    and self.loss_count >= self.loss_frames
                    and down_available
                ):
                    self.active = 'down'
                    self.loss_count = 0
        else:
            if down_available:
                self.loss_count = 0
            else:
                self.loss_count += 1
                if self.loss_count >= self.loss_frames and front_available:
                    self.active = 'front'
                    self.loss_count = 0

            if self.active == 'down' and front_available:
                self.front_recovery_count += 1
                if (
                    self.front_recovery_count
                    >= self.front_recovery_frames
                ):
                    self.active = 'front'
                    self.loss_count = 0
                    self.front_recovery_count = 0
            else:
                self.front_recovery_count = 0

        return self.active, self.active != previous


class DualTofSelector(Node):
    """Expose a stable aggregate view while retaining per-camera diagnostics."""

    def __init__(self):
        super().__init__('dual_tof_selector')
        self.declare_parameter('switch_loss_frames', 3)
        self.declare_parameter('front_recovery_frames', 10)
        self.declare_parameter('allow_down_fallback', False)
        self.declare_parameter('selection_rate_hz', 10.0)

        self.selector = HystereticCameraSelector(
            self.get_parameter('switch_loss_frames').value,
            self.get_parameter('front_recovery_frames').value,
            self.get_parameter('allow_down_fallback').value,
        )
        self.observations = {
            'front': CameraObservation(),
            'down': CameraObservation(),
        }
        self._camera_subscriptions = []

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for camera in self.observations:
            prefix = f'/perception/{camera}'
            self._camera_subscriptions.append(self.create_subscription(
                Bool,
                prefix + '/usv_visible',
                self._bool_callback(camera, 'visible'),
                status_qos,
            ))
            self._camera_subscriptions.append(self.create_subscription(
                Bool,
                prefix + '/usv_tof_valid',
                self._bool_callback(camera, 'tof_valid'),
                status_qos,
            ))
            self._camera_subscriptions.append(self.create_subscription(
                Float32,
                prefix + '/usv_range',
                self._float_callback(camera, 'target_range'),
                status_qos,
            ))
            self._camera_subscriptions.append(self.create_subscription(
                Float32,
                prefix + '/usv_depth_valid_ratio',
                self._float_callback(camera, 'depth_valid_ratio'),
                status_qos,
            ))
            self._camera_subscriptions.append(self.create_subscription(
                Float32,
                prefix + '/usv_visibility_rate',
                self._float_callback(camera, 'visibility_rate'),
                status_qos,
            ))
            self._camera_subscriptions.append(self.create_subscription(
                Float32,
                prefix + '/usv_tof_valid_rate',
                self._float_callback(camera, 'tof_valid_rate'),
                status_qos,
            ))
            for suffix, field in (
                ('camera_frame_change', 'frame_change'),
                ('target_horizontal_angle', 'horizontal_angle'),
                ('target_vertical_angle', 'vertical_angle'),
            ):
                self._camera_subscriptions.append(self.create_subscription(
                    Float32,
                    prefix + '/' + suffix,
                    self._float_callback(camera, field),
                    status_qos,
                ))
            for suffix, field in (
                ('camera_stream_alive', 'stream_alive'),
                ('target_truth_in_fov', 'truth_in_fov'),
            ):
                self._camera_subscriptions.append(self.create_subscription(
                    Bool,
                    prefix + '/' + suffix,
                    self._bool_callback(camera, field),
                    status_qos,
                ))
            for suffix, field in (
                ('camera_frame_count', 'frame_count'),
                ('usv_red_pixel_count', 'red_pixel_count'),
            ):
                self._camera_subscriptions.append(self.create_subscription(
                    UInt64,
                    prefix + '/' + suffix,
                    self._integer_callback(camera, field),
                    status_qos,
                ))

        self.visible_pub = self.create_publisher(
            Bool, '/perception/usv_visible', status_qos
        )
        self.tof_valid_pub = self.create_publisher(
            Bool, '/perception/usv_tof_valid', status_qos
        )
        self.active_camera_pub = self.create_publisher(
            String, '/perception/active_camera', status_qos
        )
        self.range_pub = self.create_publisher(
            Float32, '/perception/usv_range', status_qos
        )
        self.depth_ratio_pub = self.create_publisher(
            Float32, '/perception/usv_depth_valid_ratio', status_qos
        )
        self.visibility_rate_pub = self.create_publisher(
            Float32, '/perception/usv_visibility_rate', status_qos
        )
        self.tof_valid_rate_pub = self.create_publisher(
            Float32, '/perception/usv_tof_valid_rate', status_qos
        )
        self.frame_change_pub = self.create_publisher(
            Float32, '/perception/camera_frame_change', status_qos
        )
        self.stream_alive_pub = self.create_publisher(
            Bool, '/perception/camera_stream_alive', status_qos
        )
        self.frame_count_pub = self.create_publisher(
            UInt64, '/perception/camera_frame_count', status_qos
        )
        self.red_pixel_count_pub = self.create_publisher(
            UInt64, '/perception/usv_red_pixel_count', status_qos
        )
        self.horizontal_angle_pub = self.create_publisher(
            Float32, '/perception/target_horizontal_angle', status_qos
        )
        self.vertical_angle_pub = self.create_publisher(
            Float32, '/perception/target_vertical_angle', status_qos
        )
        self.truth_in_fov_pub = self.create_publisher(
            Bool, '/perception/target_truth_in_fov', status_qos
        )

        rate = max(
            float(self.get_parameter('selection_rate_hz').value),
            1.0,
        )
        self.last_status = None
        self.create_timer(1.0 / rate, self.publish_selection)
        self.get_logger().info(
            'DUAL TOF SELECTOR READY | front preferred | '
            f'loss={self.selector.loss_frames} frames | '
            'front recovery='
            f'{self.selector.front_recovery_frames} frames | '
            f'down fallback={self.selector.allow_down_fallback}'
        )

    def _bool_callback(self, camera, field):
        def callback(message):
            setattr(self.observations[camera], field, bool(message.data))
        return callback

    def _float_callback(self, camera, field):
        def callback(message):
            setattr(self.observations[camera], field, float(message.data))
        return callback

    def _integer_callback(self, camera, field):
        def callback(message):
            setattr(self.observations[camera], field, int(message.data))
        return callback

    @staticmethod
    def _publish_bool(publisher, value):
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    @staticmethod
    def _publish_float(publisher, value):
        message = Float32()
        message.data = float(value)
        publisher.publish(message)

    @staticmethod
    def _publish_integer(publisher, value):
        message = UInt64()
        message.data = int(value)
        publisher.publish(message)

    def publish_selection(self):
        front = self.observations['front']
        down = self.observations['down']
        active, switched = self.selector.update(
            front.available,
            down.available,
        )
        observation = self.observations[active]

        self._publish_bool(self.visible_pub, observation.visible)
        self._publish_bool(self.tof_valid_pub, observation.tof_valid)
        self._publish_float(self.range_pub, observation.target_range)
        self._publish_float(
            self.depth_ratio_pub,
            observation.depth_valid_ratio,
        )
        self._publish_float(
            self.visibility_rate_pub,
            observation.visibility_rate,
        )
        self._publish_float(
            self.tof_valid_rate_pub,
            observation.tof_valid_rate,
        )
        self._publish_float(
            self.frame_change_pub,
            observation.frame_change,
        )
        self._publish_bool(
            self.stream_alive_pub,
            observation.stream_alive,
        )
        self._publish_integer(
            self.frame_count_pub,
            observation.frame_count,
        )
        self._publish_integer(
            self.red_pixel_count_pub,
            observation.red_pixel_count,
        )
        self._publish_float(
            self.horizontal_angle_pub,
            observation.horizontal_angle,
        )
        self._publish_float(
            self.vertical_angle_pub,
            observation.vertical_angle,
        )
        self._publish_bool(
            self.truth_in_fov_pub,
            observation.truth_in_fov,
        )
        active_message = String()
        if observation.tof_valid:
            active_message.data = f'{active}_tof'
        elif observation.visible:
            active_message.data = f'{active}_rgb_only'
        else:
            active_message.data = 'none'
        self.active_camera_pub.publish(active_message)

        status = (
            active,
            observation.visible,
            observation.tof_valid,
        )
        if switched or status != self.last_status:
            message = (
                f'ACTIVE CAMERA={active_message.data} | '
                f'front RGB/ToF={front.visible}/{front.tof_valid} | '
                f'down RGB/ToF={down.visible}/{down.tof_valid}'
            )
            # rclpy identifies a logging call site by source location and
            # rejects changing its severity between calls.  Keep INFO and
            # WARN on distinct call sites instead of selecting a bound method.
            if observation.available:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
        self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = DualTofSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
