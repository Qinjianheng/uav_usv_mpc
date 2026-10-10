"""Validate front ToF ranging and camera-to-target viewing geometry."""

from collections import deque
import math
import threading
import time

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String, UInt64

from uav_control.common.runtime_performance import RateMeter
from .tof_depth_model import decode_depth_image


COLOR_PIXEL_FORMATS = {
    3: ('rgb8', 3, 0, 1, 2),
    4: ('rgba8', 4, 0, 1, 2),
    5: ('bgra8', 4, 2, 1, 0),
    8: ('bgr8', 3, 2, 1, 0),
}
DEPTH_FLOAT32_PIXEL_FORMAT = 13


def camera_intrinsics(width, height, horizontal_fov):
    """Calculate pinhole intrinsics with square pixels."""
    width = int(width)
    height = int(height)
    horizontal_fov = float(horizontal_fov)
    if width <= 0 or height <= 0:
        raise ValueError('Camera image dimensions must be positive.')
    if not 0.0 < horizontal_fov < math.pi:
        raise ValueError('Horizontal field of view must be in (0, pi).')
    focal_length = width / (2.0 * math.tan(horizontal_fov / 2.0))
    return focal_length, focal_length, width / 2.0, height / 2.0


def vertical_field_of_view(width, height, horizontal_fov):
    """Calculate vertical field of view for square camera pixels."""
    width = int(width)
    height = int(height)
    horizontal_fov = float(horizontal_fov)
    if width <= 0 or height <= 0:
        raise ValueError('Camera image dimensions must be positive.')
    if not 0.0 < horizontal_fov < math.pi:
        raise ValueError('Horizontal field of view must be in (0, pi).')
    return 2.0 * math.atan(
        math.tan(horizontal_fov / 2.0) * height / width
    )


def rotate_ned_to_body_frd(quaternion, vector):
    """Rotate a NED vector into body FRD using PX4's body-to-NED q."""
    q = np.asarray(quaternion, dtype=float)
    vector = np.asarray(vector, dtype=float)
    if q.shape != (4,) or vector.shape != (3,):
        raise ValueError('Quaternion and vector must have lengths 4 and 3.')
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError('Attitude quaternion must be finite and nonzero.')
    w, x, y, z = q / norm
    body_to_ned = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])
    return body_to_ned.T @ vector


def target_camera_angles(
    uav_position,
    target_position,
    attitude_quaternion,
    camera_pitch_down,
    horizontal_fov,
    width,
    height,
):
    """Return camera horizontal/down angles and whether truth is in view."""
    target_vector_ned = np.asarray(target_position, dtype=float) - np.asarray(
        uav_position,
        dtype=float,
    )
    if target_vector_ned.shape != (3,) or not np.all(
        np.isfinite(target_vector_ned)
    ):
        raise ValueError('UAV and target positions must be finite 3-vectors.')
    body_frd = rotate_ned_to_body_frd(
        attitude_quaternion,
        target_vector_ned,
    )
    # Gazebo camera link uses FLU. A positive SDF pitch tilts +X downward.
    body_flu = np.array([body_frd[0], -body_frd[1], -body_frd[2]])
    pitch = float(camera_pitch_down)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    camera_x = cos_pitch * body_flu[0] - sin_pitch * body_flu[2]
    camera_y = body_flu[1]
    camera_z = sin_pitch * body_flu[0] + cos_pitch * body_flu[2]
    horizontal_angle = math.atan2(camera_y, camera_x)
    vertical_down_angle = math.atan2(-camera_z, camera_x)
    vertical_fov = vertical_field_of_view(width, height, horizontal_fov)
    in_fov = (
        camera_x > 0.0
        and abs(horizontal_angle) <= float(horizontal_fov) / 2.0
        and abs(vertical_down_angle) <= vertical_fov / 2.0
    )
    return horizontal_angle, vertical_down_angle, bool(in_fov)


def red_pixel_mask(data, width, height, step, encoding):
    """Return a mask of strongly red pixels from a packed 8-bit image."""
    formats = {
        value[0]: value[1:]
        for value in COLOR_PIXEL_FORMATS.values()
    }
    if encoding not in formats:
        return None
    channel_count, red_index, green_index, blue_index = formats[encoding]
    width = int(width)
    height = int(height)
    step = int(step)
    minimum_step = width * channel_count
    if width <= 0 or height <= 0 or step < minimum_step:
        return None
    raw = np.frombuffer(data, dtype=np.uint8)
    required_size = height * step
    if raw.size < required_size:
        return None
    rows = raw[:required_size].reshape(height, step)
    pixels = rows[:, :minimum_step].reshape(
        height,
        width,
        channel_count,
    )
    red = pixels[:, :, red_index].astype(np.uint16)
    green = pixels[:, :, green_index].astype(np.uint16)
    blue = pixels[:, :, blue_index].astype(np.uint16)
    return (
        # The validation sphere's shadow is still saturated red (R=94 in
        # native terminal frames). A lit-surface cutoff drops the whole
        # target before it leaves view. Keep chromatic dominance and the
        # absolute contrast gate while admitting that visible dark surface.
        (red >= 90)
        & (red >= 3 * green // 2)
        & (red >= 3 * blue // 2)
        & (red - np.minimum(green, blue) >= 60)
    )


def count_red_pixels(data, width, height, step, encoding):
    """Count strongly red pixels in a supported packed 8-bit image."""
    mask = red_pixel_mask(data, width, height, step, encoding)
    return 0 if mask is None else int(np.count_nonzero(mask))


def decode_float32_depth(data, width, height, step):
    """Decode a row-padded little-endian R_FLOAT32 depth image."""
    width = int(width)
    height = int(height)
    step = int(step)
    minimum_step = width * 4
    if width <= 0 or height <= 0 or step < minimum_step:
        return None
    raw = np.frombuffer(data, dtype=np.uint8)
    required_size = height * step
    if raw.size < required_size:
        return None
    rows = raw[:required_size].reshape(height, step)
    packed = np.ascontiguousarray(rows[:, :minimum_step])
    return packed.view('<f4').reshape(height, width)


def target_depth_statistics(depth, target_mask, minimum, maximum):
    """Return median target range and valid-depth ratio inside its mask."""
    if depth is None or target_mask is None:
        return math.nan, 0.0
    if depth.shape != target_mask.shape:
        return math.nan, 0.0
    target_count = int(np.count_nonzero(target_mask))
    if target_count == 0:
        return math.nan, 0.0
    target_depth = depth[target_mask]
    valid = (
        np.isfinite(target_depth)
        & (target_depth >= float(minimum))
        & (target_depth <= float(maximum))
    )
    valid_count = int(np.count_nonzero(valid))
    valid_ratio = valid_count / target_count
    if valid_count == 0:
        return math.nan, valid_ratio
    return float(np.median(target_depth[valid])), valid_ratio


def target_renderable_in_rgb(
    angular_in_fov,
    distance,
    minimum_distance,
    maximum_distance,
):
    """Return whether truth lies inside the RGB angular and clip volume."""
    distance = float(distance)
    return bool(
        angular_in_fov
        and math.isfinite(distance)
        and float(minimum_distance) <= distance <= float(maximum_distance)
    )


class FrontTofMonitor(Node):
    """Publish ToF visibility and camera geometry diagnostics."""

    def __init__(self):
        super().__init__('front_tof_monitor')
        self.declare_parameter('camera_name', 'front')
        self.declare_parameter('diagnostic_prefix', '/perception/front')
        self.declare_parameter(
            'camera_frame_id',
            'front_camera_optical_frame',
        )
        self.declare_parameter(
            'color_gazebo_topic',
            '/uav/camera/front/image',
        )
        self.declare_parameter(
            'depth_gazebo_topic',
            '/uav/camera/front/depth_image',
        )
        self.declare_parameter(
            'color_ros_topic',
            '/camera/front/image_raw',
        )
        self.declare_parameter(
            'depth_ros_topic',
            '/camera/front/depth/image_raw',
        )
        self.declare_parameter('horizontal_fov', 1.74)
        self.declare_parameter('minimum_red_pixels', 20)
        self.declare_parameter('minimum_target_depth_ratio', 0.5)
        self.declare_parameter('image_timeout', 0.5)
        self.declare_parameter('maximum_rgb_depth_skew', 0.1)
        self.declare_parameter('minimum_rgb_distance', 0.05)
        self.declare_parameter('maximum_rgb_distance', 60.0)
        self.declare_parameter('minimum_depth', 0.05)
        self.declare_parameter('maximum_depth', 25.0)
        self.declare_parameter('evaluation_window_seconds', 5.0)
        self.declare_parameter('camera_pitch_down', 0.4886921905584123)
        self.declare_parameter('target_visual_height_offset', 0.42)
        self.declare_parameter('analysis_rate_hz', 10.0)
        self.declare_parameter('gazebo_world_name', 'default')
        self.declare_parameter('publish_gazebo_rtf', True)
        self.declare_parameter('depth_input_ros', False)

        self.camera_name = str(
            self.get_parameter('camera_name').value
        ).strip() or 'camera'
        self.diagnostic_prefix = str(
            self.get_parameter('diagnostic_prefix').value
        ).rstrip('/')
        if not self.diagnostic_prefix.startswith('/'):
            self.diagnostic_prefix = '/' + self.diagnostic_prefix
        self.camera_frame_id = str(
            self.get_parameter('camera_frame_id').value
        )

        self.color_gazebo_topic = str(
            self.get_parameter('color_gazebo_topic').value
        )
        self.depth_gazebo_topic = str(
            self.get_parameter('depth_gazebo_topic').value
        )
        self.color_ros_topic = str(
            self.get_parameter('color_ros_topic').value
        )
        self.depth_ros_topic = str(
            self.get_parameter('depth_ros_topic').value
        )
        self.horizontal_fov = float(
            self.get_parameter('horizontal_fov').value
        )
        self.minimum_red_pixels = max(
            int(self.get_parameter('minimum_red_pixels').value),
            1,
        )
        self.minimum_target_depth_ratio = min(
            max(
                float(
                    self.get_parameter(
                        'minimum_target_depth_ratio'
                    ).value
                ),
                0.0,
            ),
            1.0,
        )
        self.image_timeout = max(
            float(self.get_parameter('image_timeout').value),
            0.05,
        )
        self.maximum_rgb_depth_skew = max(
            float(
                self.get_parameter('maximum_rgb_depth_skew').value
            ),
            0.0,
        )
        self.minimum_rgb_distance = max(
            float(self.get_parameter('minimum_rgb_distance').value),
            0.0,
        )
        self.maximum_rgb_distance = max(
            float(self.get_parameter('maximum_rgb_distance').value),
            self.minimum_rgb_distance,
        )
        self.minimum_depth = max(
            float(self.get_parameter('minimum_depth').value),
            0.0,
        )
        self.maximum_depth = max(
            float(self.get_parameter('maximum_depth').value),
            self.minimum_depth,
        )
        self.evaluation_window_seconds = max(
            float(
                self.get_parameter(
                    'evaluation_window_seconds'
                ).value
            ),
            1.0,
        )
        self.camera_pitch_down = float(
            self.get_parameter('camera_pitch_down').value
        )
        self.target_visual_height_offset = max(
            float(
                self.get_parameter(
                    'target_visual_height_offset'
                ).value
            ),
            0.0,
        )
        analysis_rate_hz = max(
            float(self.get_parameter('analysis_rate_hz').value),
            1.0,
        )
        self.analysis_period = 1.0 / analysis_rate_hz

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            self.color_ros_topic.rsplit('/', 1)[0] + '/camera_info',
            sensor_qos,
        )
        self.visible_pub = self.create_publisher(
            Bool,
            self.diagnostic_prefix + '/usv_visible',
            status_qos,
        )
        self.tof_valid_pub = self.create_publisher(
            Bool,
            self.diagnostic_prefix + '/usv_tof_valid',
            status_qos,
        )
        self.active_camera_pub = self.create_publisher(
            String,
            self.diagnostic_prefix + '/active_camera',
            status_qos,
        )
        self.range_pub = self.create_publisher(
            Float32,
            self.diagnostic_prefix + '/usv_range',
            status_qos,
        )
        self.depth_ratio_pub = self.create_publisher(
            Float32,
            self.diagnostic_prefix + '/usv_depth_valid_ratio',
            status_qos,
        )
        self.visibility_rate_pub = self.create_publisher(
            Float32,
            self.diagnostic_prefix + '/usv_visibility_rate',
            status_qos,
        )
        self.tof_valid_rate_pub = self.create_publisher(
            Float32,
            self.diagnostic_prefix + '/usv_tof_valid_rate',
            status_qos,
        )
        self.frame_change_pub = self.create_publisher(
            Float32,
            self.diagnostic_prefix + '/camera_frame_change',
            status_qos,
        )
        self.stream_alive_pub = self.create_publisher(
            Bool,
            self.diagnostic_prefix + '/camera_stream_alive',
            status_qos,
        )
        self.frame_count_pub = self.create_publisher(
            UInt64,
            self.diagnostic_prefix + '/camera_frame_count',
            status_qos,
        )
        self.red_pixel_count_pub = self.create_publisher(
            UInt64,
            self.diagnostic_prefix + '/usv_red_pixel_count',
            status_qos,
        )
        self.horizontal_angle_pub = self.create_publisher(
            Float32,
            self.diagnostic_prefix + '/target_horizontal_angle',
            status_qos,
        )
        self.vertical_angle_pub = self.create_publisher(
            Float32,
            self.diagnostic_prefix + '/target_vertical_angle',
            status_qos,
        )
        self.truth_in_fov_pub = self.create_publisher(
            Bool,
            self.diagnostic_prefix + '/target_truth_in_fov',
            status_qos,
        )
        self.truth_renderable_pub = self.create_publisher(
            Bool,
            self.diagnostic_prefix + '/target_truth_renderable',
            status_qos,
        )
        performance_prefix = f'/diagnostics/{self.camera_name}'
        self.rgb_hz_pub = self.create_publisher(
            Float32, performance_prefix + '/rgb_frame_hz', 10
        )
        self.depth_hz_pub = self.create_publisher(
            Float32, performance_prefix + '/depth_frame_hz', 10
        )
        self.compute_time_pub = self.create_publisher(
            Float32, performance_prefix + '/monitor_compute_time', 10
        )
        self.gazebo_rtf_pub = self.create_publisher(
            Float32, '/simulation/gazebo/real_time_factor', 10
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            sensor_qos,
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            sensor_qos,
        )

        self.lock = threading.Lock()
        self.last_color_time = -math.inf
        self.last_depth_time = -math.inf
        self.last_color_analysis_time = -math.inf
        self.last_depth_analysis_time = -math.inf
        self.image_width = 640
        self.image_height = 480
        self.previous_thumbnail = None
        self.frame_change = 0.0
        self.color_frame_count = 0
        self.red_mask = None
        self.red_pixels = 0
        self.depth = None
        self.target_position = None
        self.uav_position = None
        self.uav_attitude = None
        self.samples = deque()
        self.last_status = None
        self.rgb_rate_meter = RateMeter(window_seconds=1.0)
        self.depth_rate_meter = RateMeter(window_seconds=1.0)
        self.last_monitor_compute_time = 0.0
        self.gazebo_real_time_factor = 0.0
        self.shutting_down = False

        from gz.msgs10.image_pb2 import Image as GazeboImage
        from gz.transport13 import Node as GazeboTransportNode

        self.gazebo_node = GazeboTransportNode()
        color_subscribed = self.gazebo_node.subscribe(
            GazeboImage,
            self.color_gazebo_topic,
            self.color_callback,
        )
        self.depth_input_ros = bool(self.get_parameter('depth_input_ros').value)
        if self.depth_input_ros:
            self.depth_ros_sub = self.create_subscription(
                Image, self.depth_ros_topic, self.ros_depth_callback, sensor_qos,
            )
            depth_subscribed = True
        else:
            depth_subscribed = self.gazebo_node.subscribe(
                GazeboImage, self.depth_gazebo_topic, self.depth_callback,
            )
        if not color_subscribed or not depth_subscribed:
            raise RuntimeError(
                f'Could not subscribe to {self.camera_name} ToF streams.'
            )
        self.world_stats_topic = None
        if bool(self.get_parameter('publish_gazebo_rtf').value):
            from gz.msgs10.world_stats_pb2 import WorldStatistics
            world_name = str(
                self.get_parameter('gazebo_world_name').value
            ).strip().strip('/')
            self.world_stats_topic = f'/world/{world_name}/stats'
            self.gazebo_node.subscribe(
                WorldStatistics,
                self.world_stats_topic,
                self.world_stats_callback,
            )

        self.status_timer = self.create_timer(0.1, self.publish_status)
        self.get_logger().info(
            f'{self.camera_name.upper()} TOF READY | '
            f'RGB={self.color_gazebo_topic} | '
            f'depth={self.depth_ros_topic if self.depth_input_ros else self.depth_gazebo_topic} | '
            f'ROS RGB={self.color_ros_topic} | '
            f'RGB clip={self.minimum_rgb_distance:.1f}-'
            f'{self.maximum_rgb_distance:.1f} m | '
            f'ToF range={self.minimum_depth:.1f}-'
            f'{self.maximum_depth:.1f} m'
        )
        self.get_logger().info(
            'Image diagnostics only; target truth is not subscribed; '
            'ToF output is diagnostic only.'
        )

    def target_callback(self, message):
        self.target_position = (
            float(message.x),
            float(message.y),
            float(message.z) - self.target_visual_height_offset,
        )

    def position_callback(self, message):
        if all(
            math.isfinite(value)
            for value in (message.x, message.y, message.z)
        ):
            self.uav_position = (
                float(message.x),
                float(message.y),
                float(message.z),
            )

    def attitude_callback(self, message):
        quaternion = tuple(float(value) for value in message.q)
        if all(math.isfinite(value) for value in quaternion):
            self.uav_attitude = quaternion

    def color_callback(self, message):
        if self.shutting_down:
            return
        now = time.monotonic()
        self.rgb_rate_meter.observe(now)
        with self.lock:
            if now - self.last_color_analysis_time < self.analysis_period:
                return
            self.last_color_analysis_time = now
        format_info = COLOR_PIXEL_FORMATS.get(
            int(message.pixel_format_type)
        )
        if format_info is None:
            return
        encoding, channel_count, _, _, _ = format_info
        width = int(message.width)
        height = int(message.height)
        step = int(message.step)
        data = message.data
        stamp = self.get_clock().now().to_msg()
        mask = red_pixel_mask(
            data,
            width,
            height,
            step,
            encoding,
        )
        red_pixels = (
            0 if mask is None else int(np.count_nonzero(mask))
        )
        thumbnail = None
        minimum_step = width * channel_count
        raw = np.frombuffer(data, dtype=np.uint8)
        if (
            width > 0
            and height > 0
            and step >= minimum_step
            and raw.size >= height * step
        ):
            pixels = raw[:height * step].reshape(height, step)[
                :, :minimum_step
            ].reshape(height, width, channel_count)
            thumbnail = pixels[::24, ::24, :3].astype(np.int16)
        with self.lock:
            self.last_color_time = now
            self.color_frame_count += 1
            self.image_width = width
            self.image_height = height
            self.red_mask = mask
            self.red_pixels = red_pixels
            if (
                thumbnail is not None
                and self.previous_thumbnail is not None
                and thumbnail.shape == self.previous_thumbnail.shape
            ):
                self.frame_change = float(np.mean(np.abs(
                    thumbnail - self.previous_thumbnail
                ))) / 255.0
            if thumbnail is not None:
                self.previous_thumbnail = thumbnail
        self.camera_info_pub.publish(
            self.make_camera_info(
                width,
                height,
                stamp,
            )
        )
        self.last_monitor_compute_time = time.monotonic() - now

    def depth_callback(self, message):
        if self.shutting_down:
            return
        now = time.monotonic()
        self.depth_rate_meter.observe(now)
        with self.lock:
            if now - self.last_depth_analysis_time < self.analysis_period:
                return
            self.last_depth_analysis_time = now
        if int(message.pixel_format_type) != DEPTH_FLOAT32_PIXEL_FORMAT:
            return
        depth = decode_float32_depth(
            message.data,
            message.width,
            message.height,
            message.step,
        )
        if depth is None:
            return
        with self.lock:
            self.last_depth_time = now
            self.depth = depth
        self.last_monitor_compute_time = max(
            self.last_monitor_compute_time,
            time.monotonic() - now,
        )

    def ros_depth_callback(self, message):
        """Diagnose modeled returns, retaining NaNs instead of ideal fallback."""
        if self.shutting_down:
            return
        now = time.monotonic()
        self.depth_rate_meter.observe(now)
        try:
            depth = decode_depth_image(message)
        except ValueError:
            return
        with self.lock:
            if now - self.last_depth_analysis_time < self.analysis_period:
                return
            self.last_depth_analysis_time = now
            self.last_depth_time = now
            self.depth = depth
        self.last_monitor_compute_time = max(
            self.last_monitor_compute_time, time.monotonic() - now,
        )

    def world_stats_callback(self, message):
        value = float(message.real_time_factor)
        if math.isfinite(value) and value >= 0.0:
            self.gazebo_real_time_factor = value

    def make_camera_info(self, width, height, stamp):
        fx, fy, cx, cy = camera_intrinsics(
            width,
            height,
            self.horizontal_fov,
        )
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = self.camera_frame_id
        message.height = height
        message.width = width
        message.distortion_model = 'plumb_bob'
        message.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        message.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        message.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return message

    def truth_distance(self):
        if self.target_position is None or self.uav_position is None:
            return math.inf
        return math.sqrt(sum(
            (target - uav) ** 2
            for target, uav in zip(
                self.target_position,
                self.uav_position,
            )
        ))

    def truth_camera_geometry(self):
        if (
            self.target_position is None
            or self.uav_position is None
            or self.uav_attitude is None
        ):
            return math.nan, math.nan, False
        try:
            return target_camera_angles(
                self.uav_position,
                self.target_position,
                self.uav_attitude,
                self.camera_pitch_down,
                self.horizontal_fov,
                self.image_width,
                self.image_height,
            )
        except ValueError:
            return math.nan, math.nan, False

    def publish_status(self):
        now = time.monotonic()
        with self.lock:
            color_time = self.last_color_time
            depth_time = self.last_depth_time
            mask = self.red_mask
            red_pixels = self.red_pixels
            depth = self.depth
            frame_change = self.frame_change
            color_frame_count = self.color_frame_count
        color_fresh = now - color_time <= self.image_timeout
        depth_fresh = now - depth_time <= self.image_timeout
        synchronized = (
            abs(color_time - depth_time) <= self.maximum_rgb_depth_skew
        )
        visible = color_fresh and red_pixels >= self.minimum_red_pixels
        target_range, depth_ratio = target_depth_statistics(
            depth if depth_fresh and synchronized else None,
            mask if visible else None,
            self.minimum_depth,
            self.maximum_depth,
        )
        tof_valid = (
            visible
            and depth_fresh
            and synchronized
            and math.isfinite(target_range)
            and depth_ratio >= self.minimum_target_depth_ratio
        )
        horizontal_angle, vertical_angle, truth_in_fov = (
            self.truth_camera_geometry()
        )
        truth_distance = self.truth_distance()
        truth_renderable = target_renderable_in_rgb(
            truth_in_fov,
            truth_distance,
            self.minimum_rgb_distance,
            self.maximum_rgb_distance,
        )

        if color_fresh:
            self.samples.append((now, visible, tof_valid))
        cutoff = now - self.evaluation_window_seconds
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        if self.samples:
            visibility_rate = sum(
                sample[1] for sample in self.samples
            ) / len(self.samples)
            tof_valid_rate = sum(
                sample[2] for sample in self.samples
            ) / len(self.samples)
        else:
            visibility_rate = 0.0
            tof_valid_rate = 0.0

        visible_message = Bool()
        visible_message.data = visible
        self.visible_pub.publish(visible_message)
        valid_message = Bool()
        valid_message.data = tof_valid
        self.tof_valid_pub.publish(valid_message)
        camera_message = String()
        if tof_valid:
            camera_message.data = f'{self.camera_name}_tof'
        elif visible:
            camera_message.data = f'{self.camera_name}_rgb_only'
        else:
            camera_message.data = 'none'
        self.active_camera_pub.publish(camera_message)
        range_message = Float32()
        range_message.data = target_range
        self.range_pub.publish(range_message)
        ratio_message = Float32()
        ratio_message.data = depth_ratio
        self.depth_ratio_pub.publish(ratio_message)
        visibility_rate_message = Float32()
        visibility_rate_message.data = visibility_rate
        self.visibility_rate_pub.publish(visibility_rate_message)
        tof_valid_rate_message = Float32()
        tof_valid_rate_message.data = tof_valid_rate
        self.tof_valid_rate_pub.publish(tof_valid_rate_message)
        frame_change_message = Float32()
        frame_change_message.data = frame_change
        self.frame_change_pub.publish(frame_change_message)
        stream_alive_message = Bool()
        stream_alive_message.data = color_fresh
        self.stream_alive_pub.publish(stream_alive_message)
        frame_count_message = UInt64()
        frame_count_message.data = color_frame_count
        self.frame_count_pub.publish(frame_count_message)
        red_pixel_count_message = UInt64()
        red_pixel_count_message.data = red_pixels
        self.red_pixel_count_pub.publish(red_pixel_count_message)
        horizontal_angle_message = Float32()
        horizontal_angle_message.data = horizontal_angle
        self.horizontal_angle_pub.publish(horizontal_angle_message)
        vertical_angle_message = Float32()
        vertical_angle_message.data = vertical_angle
        self.vertical_angle_pub.publish(vertical_angle_message)
        truth_in_fov_message = Bool()
        truth_in_fov_message.data = truth_in_fov
        self.truth_in_fov_pub.publish(truth_in_fov_message)
        truth_renderable_message = Bool()
        truth_renderable_message.data = truth_renderable
        self.truth_renderable_pub.publish(truth_renderable_message)
        rgb_hz_message = Float32()
        rgb_hz_message.data = float(self.rgb_rate_meter.rate(now))
        self.rgb_hz_pub.publish(rgb_hz_message)
        depth_hz_message = Float32()
        depth_hz_message.data = float(self.depth_rate_meter.rate(now))
        self.depth_hz_pub.publish(depth_hz_message)
        compute_message = Float32()
        compute_message.data = float(self.last_monitor_compute_time)
        self.compute_time_pub.publish(compute_message)
        if self.world_stats_topic is not None:
            rtf_message = Float32()
            rtf_message.data = float(self.gazebo_real_time_factor)
            self.gazebo_rtf_pub.publish(rtf_message)

        status = (
            visible,
            tof_valid,
            camera_message.data,
            truth_in_fov,
            truth_renderable,
        )
        if status != self.last_status:
            range_text = (
                f'{target_range:.2f}'
                if math.isfinite(target_range)
                else 'nan'
            )
            status_text = (
                f'USV RGB={visible} | TOF={tof_valid} | '
                f'active={camera_message.data} | '
                f'range={range_text} m | '
                f'depth ratio={depth_ratio:.2f} | '
                f'window rates RGB/ToF='
                f'{visibility_rate:.2f}/{tof_valid_rate:.2f} | '
                f'frame change={frame_change:.4f} | '
                f'truth angle FOV={truth_in_fov} | '
                f'RGB render volume={truth_renderable} | '
                f'H/V angles='
                f'{math.degrees(horizontal_angle):.1f}/'
                f'{math.degrees(vertical_angle):.1f} deg | '
                f'truth distance={truth_distance:.2f} m'
            )
            if tof_valid:
                self.get_logger().info(status_text)
            else:
                self.get_logger().warn(status_text)
        self.last_status = status

    def destroy_node(self):
        self.shutting_down = True
        if hasattr(self, 'gazebo_node'):
            self.gazebo_node.unsubscribe(self.color_gazebo_topic)
            if not getattr(self, 'depth_input_ros', False):
                self.gazebo_node.unsubscribe(self.depth_gazebo_topic)
            if self.world_stats_topic is not None:
                self.gazebo_node.unsubscribe(self.world_stats_topic)
            time.sleep(0.1)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontTofMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
