"""
Acquisition-timed RGB bearing for visual yaw control and reacquisition.

Only the red validation detector is implemented here. There is no depth,
vehicle pose or target truth input; the Gazebo server clock supplies time
correspondence only. Positive image-right bearing means positive PX4 yaw.
"""

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import Image

from .front_tof_monitor import camera_intrinsics, red_pixel_mask
from .rgbd_target_localizer import GazeboImageClockMapper, aligned_camera_qos


@dataclass(frozen=True)
class BearingReading:
    stamp: float = 0.0
    raw_stamp: float = 0.0
    valid: bool = False
    bearing: float = 0.0
    body_bearing: float = 0.0
    confidence: float = 0.0
    reason: str = ''


def image_target_bearing(message, horizontal_fov, minimum_red_pixels,
                         camera_pitch_down=0.0):
    """Detect centroid and bearing in the image, without any 3D inputs."""
    try:
        mask = red_pixel_mask(
            message.data, message.width, message.height,
            message.step, message.encoding,
        )
        if mask is None:
            return BearingReading(reason='INVALID_RGB_IMAGE')
        rows, columns = np.nonzero(mask)
        minimum = max(int(minimum_red_pixels), 1)
        if len(columns) < minimum:
            return BearingReading(reason='TARGET_NOT_DETECTED')
        fx, fy, cx, cy = camera_intrinsics(
            message.width, message.height, horizontal_fov
        )
        pitch = float(camera_pitch_down)
        if not math.isfinite(pitch):
            return BearingReading(reason='INVALID_CAMERA_MOUNT')
        image_right = (float(np.mean(columns)) - cx) / fx
        image_down = (float(np.mean(rows)) - cy) / fy
        bearing = math.atan(image_right)
        # Rotate the unit camera FLU ray into body FRD. No range or pose is
        # needed; image centering and body direction remain distinct outputs.
        body_forward = math.cos(pitch) - math.sin(pitch) * image_down
        return BearingReading(
            valid=True, bearing=bearing,
            body_bearing=math.atan2(image_right, body_forward),
            confidence=min(float(len(columns)) / (4.0 * minimum), 1.0),
        )
    except (ValueError, TypeError, BufferError, AttributeError):
        return BearingReading(reason='INVALID_RGB_IMAGE')


def _sensor_stamp(message):
    try:
        sec = float(message.header.stamp.sec)
        nanosec = float(message.header.stamp.nanosec)
        if not (math.isfinite(sec) and math.isfinite(nanosec)):
            return math.nan
        if nanosec < 0 or nanosec >= 1e9 or sec < 0:
            return math.nan
        return sec + nanosec * 1e-9
    except (AttributeError, TypeError, ValueError):
        return math.nan


class RgbBearingProcessor:
    """
    Bounded RGB queue with causal mapping and one output per unique frame.

    The ROS node serializes calls because Gazebo clock callbacks use a native
    transport thread. Monotonic time controls waiting only, never acquisition.
    """

    def __init__(
        self, horizontal_fov=1.74, minimum_red_pixels=3,
        image_clock_reference_timeout=0.5, image_clock_history_duration=2.0,
        image_clock_wait_timeout=0.15, maximum_system_clock_step=0.25,
        maximum_frame_age=0.15, data_timeout=0.5,
        camera_pitch_down=0.0,
    ):
        self.horizontal_fov = float(horizontal_fov)
        self.camera_pitch_down = float(camera_pitch_down)
        self.minimum_red_pixels = max(int(minimum_red_pixels), 1)
        self.image_clock_wait_timeout = max(float(image_clock_wait_timeout), 0)
        self.maximum_frame_age = max(float(maximum_frame_age), 0)
        self.data_timeout = max(float(data_timeout), 1e-3)
        self.clock = GazeboImageClockMapper(
            maximum_reference_age=image_clock_reference_timeout,
            history_duration=image_clock_history_duration,
            maximum_system_clock_step=maximum_system_clock_step,
        )
        self._pending = deque(maxlen=8)
        self._latest_raw_stamp = -math.inf
        self._last_receipt_monotonic = None
        self._last_timeout_monotonic = -math.inf
        self._last_stamp = 0.0
        self._last_raw_stamp = 0.0
        self._reset_pending = False

    def add_clock(self, sim_time, system_time, receipt_ros, receipt_monotonic):
        reset_count = self.clock.reset_count
        self.clock.add_anchor(
            sim_time, system_time, receipt_ros, receipt_monotonic
        )
        if self.clock.reset_count != reset_count:
            self._pending.clear()
            self._latest_raw_stamp = -math.inf
            self._last_receipt_monotonic = None
            self._last_stamp = self._last_raw_stamp = 0.0
            self._last_timeout_monotonic = -math.inf
            self._reset_pending = True

    def submit(self, message, receipt_ros, receipt_monotonic):
        raw = _sensor_stamp(message)
        if math.isfinite(raw) and raw > 0:
            if raw <= self._latest_raw_stamp + 1e-9:
                return
            self._latest_raw_stamp = raw
        self._last_receipt_monotonic = float(receipt_monotonic)
        self._pending.append((message, raw, float(receipt_monotonic)))

    def poll(self, now_ros, now_monotonic):
        if self._reset_pending:
            self._reset_pending = False
            return BearingReading(reason='CLOCK_RESET')
        if not self._pending:
            if (
                self._last_receipt_monotonic is not None
                and now_monotonic - self._last_receipt_monotonic
                > self.data_timeout
                and now_monotonic - self._last_timeout_monotonic
                > self.data_timeout
            ):
                self._last_timeout_monotonic = now_monotonic
                return BearingReading(
                    stamp=self._last_stamp, raw_stamp=self._last_raw_stamp,
                    reason='RGB_TIMEOUT',
                )
            return None
        message, raw, received_monotonic = self._pending[0]
        stamp, reason = self.clock.map_time(raw, now_ros)
        waiting = reason in (
            'CLOCK_REFERENCE_UNAVAILABLE', 'CLOCK_REFERENCE_WARMING_UP',
            'IMAGE_AFTER_CLOCK_REFERENCE',
        )
        if (
            stamp is None and waiting
            and now_monotonic - received_monotonic
            <= self.image_clock_wait_timeout + 1e-9
        ):
            return None
        self._pending.popleft()
        raw_output = raw if math.isfinite(raw) and raw > 0 else 0.0
        self._last_raw_stamp = raw_output
        self._last_stamp = stamp if stamp is not None else 0.0
        if stamp is None:
            return BearingReading(raw_stamp=raw_output, reason=reason)
        age = now_ros - stamp
        if age < -1e-9 or age > self.maximum_frame_age + 1e-9:
            return BearingReading(
                stamp=stamp, raw_stamp=raw_output,
                reason=('IMAGE_TIMESTAMP_IN_FUTURE' if age < 0
                        else 'IMAGE_TIMESTAMP_STALE'),
            )
        detection = image_target_bearing(
            message, self.horizontal_fov, self.minimum_red_pixels,
            camera_pitch_down=self.camera_pitch_down,
        )
        return BearingReading(
            stamp=stamp, raw_stamp=raw_output, valid=detection.valid,
            bearing=detection.bearing, confidence=detection.confidence,
            body_bearing=detection.body_bearing,
            reason=detection.reason,
        )


def _time_message(seconds):
    nanoseconds = max(round(float(seconds) * 1e9), 0)
    return Time(sec=nanoseconds // 1000000000,
                nanosec=nanoseconds % 1000000000)


class TargetBearingNode(Node):
    def __init__(self):
        super().__init__('target_bearing_node')
        defaults = {
            'color_topic': '/camera/front/image_raw',
            'bearing_topic': '/perception/front/target_bearing',
            'horizontal_fov': 1.74,
            'camera_pitch_down': 0.4886921905584123,
            'minimum_red_pixels': 3,
            'gazebo_world_name': 'default',
            'image_clock_reference_timeout': 0.5,
            'image_clock_history_duration': 2.0,
            'image_clock_wait_timeout': 0.15,
            'maximum_system_clock_step': 0.25,
            'maximum_frame_age': 0.15,
            'data_timeout': 0.5,
            'processing_rate_hz': 50.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.processor = RgbBearingProcessor(**{
            name: values[name] for name in (
                'horizontal_fov', 'minimum_red_pixels', 'camera_pitch_down',
                'image_clock_reference_timeout', 'image_clock_history_duration',
                'image_clock_wait_timeout', 'maximum_system_clock_step',
                'maximum_frame_age', 'data_timeout',
            )
        })
        self._processor_lock = threading.Lock()
        # Lazy custom-message import allows helper tests before interface build.
        from uav_usv_interfaces.msg import TargetBearing
        self._message_type = TargetBearing
        self.publisher = self.create_publisher(
            TargetBearing, str(values['bearing_topic']), 10
        )
        self.color_subscription = self.create_subscription(
            Image, str(values['color_topic']), self.color_callback,
            aligned_camera_qos(),
        )
        from gz.msgs10.clock_pb2 import Clock as GazeboClock
        from gz.transport13 import Node as GazeboTransportNode
        world = str(values['gazebo_world_name']).strip().strip('/')
        self.gazebo_clock_topic = f'/world/{world}/clock'
        self.gazebo_clock_node = GazeboTransportNode()
        if not self.gazebo_clock_node.subscribe(
            GazeboClock, self.gazebo_clock_topic, self.gazebo_clock_callback
        ):
            raise RuntimeError('Could not subscribe to trusted Gazebo clock '
                               f'{self.gazebo_clock_topic}.')
        self.timer = self.create_timer(
            1.0 / max(float(values['processing_rate_hz']), 1.0), self.process
        )
        self.get_logger().info(
            'RGB TARGET BEARING READY | '
            f"RGB={values['color_topic']} | output={values['bearing_topic']} | "
            'image-right=positive PX4 yaw | detector=red validation sphere'
        )

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def gazebo_clock_callback(self, message):
        with self._processor_lock:
            self.processor.add_clock(
                message.sim.sec + message.sim.nsec * 1e-9,
                message.system.sec + message.system.nsec * 1e-9,
                self._ros_seconds(), time.monotonic(),
            )

    def color_callback(self, message):
        with self._processor_lock:
            self.processor.submit(message, self._ros_seconds(), time.monotonic())

    def process(self):
        with self._processor_lock:
            reading = self.processor.poll(self._ros_seconds(), time.monotonic())
            if reading is None:
                return
            message = self._message_type()
            message.stamp = _time_message(reading.stamp)
            message.raw_stamp = _time_message(reading.raw_stamp)
            message.valid = reading.valid
            message.bearing = reading.bearing
            message.body_bearing = reading.body_bearing
            message.confidence = reading.confidence
            # A clock reset must not interleave between polling and publishing.
            self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TargetBearingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
