"""
Localize the simulated red USV target from aligned RGB and depth images.

The detector is intentionally limited to the red validation sphere.  The
camera geometry and filtering interface remain reusable when a non-cooperative
USV detector replaces the color mask.
"""

import math
import threading
import time
from dataclasses import dataclass
from collections import deque

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from px4_msgs.msg import (
    TimesyncStatus,
    VehicleAttitude,
    VehicleLocalPosition,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from uav_usv_interfaces.msg import TargetObservation, UavState

from .front_tof_monitor import (
    camera_intrinsics,
    decode_float32_depth,
    red_pixel_mask,
)


# The PX4 local position is referenced to the Gazebo model origin.  The
# merged x500_base puts base_link at model z=+0.24 m; front_camera_link is
# z=+0.15 m relative to base_link, so its model-relative z is +0.39 m.
DEFAULT_CAMERA_TRANSLATION_FLU = (0.18, 0.0, 0.39)


def aligned_camera_qos():
    """Request retransmission from the reliable image bridge with bounded history."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=8,
    )


def validated_sensor_stamp(measurement_stamp, receipt_stamp, maximum_age):
    """Accept only acquisition stamps demonstrably in the ROS clock domain."""
    measurement_stamp = float(measurement_stamp)
    receipt_stamp = float(receipt_stamp)
    maximum_age = max(float(maximum_age), 0.0)
    if (
        not math.isfinite(measurement_stamp)
        or not math.isfinite(receipt_stamp)
        or measurement_stamp <= 0.0
    ):
        return None
    age = receipt_stamp - measurement_stamp
    if age < -1e-9 or age > maximum_age + 1e-9:
        return None
    return measurement_stamp


def synchronized_measurement_time(color_stamp, depth_stamp, maximum_skew):
    """Use RGB acquisition time for a valid pair and preserve the real skew."""
    color_stamp = float(color_stamp)
    depth_stamp = float(depth_stamp)
    if (
        not math.isfinite(color_stamp)
        or not math.isfinite(depth_stamp)
        or abs(color_stamp - depth_stamp) > float(maximum_skew) + 1e-9
    ):
        return None
    return color_stamp


def due_data_timeout(now, latest_receipt, last_report, timeout):
    """Report a missing image only after timeout and at a bounded rate."""
    timeout = max(float(timeout), 1e-3)
    return (
        float(now) - float(latest_receipt) > timeout
        and float(now) - float(last_report) >= timeout
    )


def quaternion_slerp(left, right, fraction):
    """Interpolate equivalent unit quaternions along the shortest arc."""
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    if left.shape != (4,) or right.shape != (4,):
        raise ValueError('quaternions must contain four values')
    left_norm = float(np.linalg.norm(left))
    right_norm = float(np.linalg.norm(right))
    if left_norm < 1e-9 or right_norm < 1e-9:
        raise ValueError('quaternions must be nonzero')
    left /= left_norm
    right /= right_norm
    dot = float(np.dot(left, right))
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = min(max(dot, -1.0), 1.0)
    fraction = min(max(float(fraction), 0.0), 1.0)
    if dot > 0.9995:
        result = left + fraction * (right - left)
    else:
        angle = math.acos(dot)
        sine = math.sin(angle)
        result = (
            math.sin((1.0 - fraction) * angle) / sine * left
            + math.sin(fraction * angle) / sine * right
        )
    result /= np.linalg.norm(result)
    return tuple(float(value) for value in result)


def pose_history_rejection_reason(
    measurement_stamp,
    position_range,
    attitude_range,
):
    """Explain why the acquisition time lacks a complete UAV pose."""
    if position_range is None:
        return 'POSITION_HISTORY_EMPTY'
    if attitude_range is None:
        return 'ATTITUDE_HISTORY_EMPTY'
    stamp = float(measurement_stamp)
    if stamp < position_range[0] - 1e-9:
        return 'POSITION_TIMESTAMP_BEFORE_HISTORY'
    if stamp < attitude_range[0] - 1e-9:
        return 'ATTITUDE_TIMESTAMP_BEFORE_HISTORY'
    if stamp > position_range[1] + 1e-9:
        return 'POSITION_TIMESTAMP_AFTER_HISTORY'
    if stamp > attitude_range[1] + 1e-9:
        return 'ATTITUDE_TIMESTAMP_AFTER_HISTORY'
    return ''


class TimestampedVectorHistory:
    """Interpolate a bounded monotonic history without extrapolation."""

    def __init__(self, maximum_age=1.0, interpolator=None):
        self.maximum_age = max(float(maximum_age), 1e-3)
        self.interpolator = interpolator
        self._samples = deque()

    @property
    def latest_value(self):
        return self._samples[-1][1] if self._samples else None

    @property
    def time_range(self):
        if not self._samples:
            return None
        return self._samples[0][0], self._samples[-1][0]

    def add(self, stamp, value):
        stamp = float(stamp)
        value = tuple(float(component) for component in value)
        if not math.isfinite(stamp) or not all(map(math.isfinite, value)):
            return False
        if self._samples and stamp < self._samples[-1][0] - 1e-9:
            return False
        if self._samples and abs(stamp - self._samples[-1][0]) <= 1e-9:
            self._samples[-1] = stamp, value
        else:
            self._samples.append((stamp, value))
        cutoff = stamp - self.maximum_age
        while len(self._samples) > 2 and self._samples[1][0] < cutoff:
            self._samples.popleft()
        return True

    def clear(self):
        self._samples.clear()

    def value_at(self, stamp):
        stamp = float(stamp)
        if not self._samples:
            return None
        if (
            stamp < self._samples[0][0] - 1e-9
            or stamp > self._samples[-1][0] + 1e-9
        ):
            return None
        for index, (sample_stamp, value) in enumerate(self._samples):
            if abs(stamp - sample_stamp) <= 1e-9:
                return value
            if sample_stamp > stamp and index > 0:
                previous_stamp, previous = self._samples[index - 1]
                fraction = (
                    (stamp - previous_stamp)
                    / max(sample_stamp - previous_stamp, 1e-9)
                )
                if self.interpolator is not None:
                    return self.interpolator(previous, value, fraction)
                return tuple(
                    left + fraction * (right - left)
                    for left, right in zip(previous, value)
                )
        return self._samples[-1][1]


class Px4RosClockMapper:
    """Map one clock shared by independently ordered source streams."""

    def __init__(
        self,
        maximum_offset_jump=0.05,
        calibration_samples=4,
        reset_regression_threshold=0.5,
        reset_confirmation_window=0.5,
        offset_recovery_samples=3,
        source_is_ros_time=False,
    ):
        self.maximum_offset_jump = max(float(maximum_offset_jump), 1e-6)
        self.calibration_samples = max(int(calibration_samples), 1)
        self.reset_regression_threshold = max(
            float(reset_regression_threshold), 1e-3
        )
        self.reset_confirmation_window = max(
            float(reset_confirmation_window), 1e-3
        )
        self.offset_recovery_samples = max(
            int(offset_recovery_samples), 2
        )
        self.source_is_ros_time = bool(source_is_ros_time)
        self.offset = 0.0 if self.source_is_ros_time else None
        self._offset_samples = deque(maxlen=32)
        self._offset_outlier_samples = {}
        self._last_source_times = {}
        self._last_receipt_time = None
        self._reset_candidates = {}
        self.reset_count = 0
        self.calibration_count = 1 if self.source_is_ros_time else 0
        self.offset_recalibration_count = 0
        self.last_status = 'UNCALIBRATED'

    def reset(self):
        self.offset = 0.0 if self.source_is_ros_time else None
        self._offset_samples.clear()
        self._offset_outlier_samples.clear()
        self._last_source_times.clear()
        self._last_receipt_time = None
        self._reset_candidates.clear()
        self.reset_count += 1
        self.last_status = 'CLOCK_RESET'

    def _seed_after_reset(
        self,
        stream_name,
        source_time,
        receipt_ros_time,
        observed_offset,
    ):
        self._last_source_times[stream_name] = source_time
        self._last_receipt_time = receipt_ros_time
        if not self.source_is_ros_time:
            self._offset_samples.append(observed_offset)

    def _confirmed_source_reset(
        self,
        stream_name,
        source_time,
        receipt_ros_time,
    ):
        cutoff = receipt_ros_time - self.reset_confirmation_window
        self._reset_candidates = {
            name: candidate
            for name, candidate in self._reset_candidates.items()
            if candidate[1] >= cutoff
        }
        self._reset_candidates[stream_name] = (
            source_time,
            receipt_ros_time,
        )
        return any(
            name != stream_name
            and abs(candidate[0] - source_time)
            <= self.reset_regression_threshold
            for name, candidate in self._reset_candidates.items()
        )

    def _confirmed_offset_change(
        self,
        stream_name,
        observed_offset,
        receipt_ros_time,
    ):
        samples = self._offset_outlier_samples.setdefault(
            stream_name,
            deque(maxlen=self.offset_recovery_samples),
        )
        if (
            samples
            and (
                receipt_ros_time - samples[-1][1]
                > self.reset_confirmation_window
                or abs(observed_offset - samples[-1][0])
                > self.maximum_offset_jump
            )
        ):
            samples.clear()
        samples.append((observed_offset, receipt_ros_time))
        if len(samples) < self.offset_recovery_samples:
            return False
        candidate = sum(value for value, _stamp in samples) / len(samples)
        for other_stream, other_samples in (
            self._offset_outlier_samples.items()
        ):
            if (
                other_stream == stream_name
                or len(other_samples) < self.offset_recovery_samples
            ):
                continue
            other_candidate = sum(
                value for value, _stamp in other_samples
            ) / len(other_samples)
            if abs(candidate - other_candidate) <= self.maximum_offset_jump:
                return True
        return False

    def to_ros_time(
        self,
        source_time,
        receipt_ros_time,
        stream_name=None,
    ):
        source_time = float(source_time)
        receipt_ros_time = float(receipt_ros_time)
        if (
            not math.isfinite(source_time)
            or source_time <= 0.0
            or not math.isfinite(receipt_ros_time)
        ):
            self.last_status = 'INVALID_TIMESTAMP'
            return None
        stream = (
            '__single_stream__'
            if stream_name is None else str(stream_name)
        )
        observed_offset = receipt_ros_time - source_time
        if (
            self._last_receipt_time is not None
            and receipt_ros_time < self._last_receipt_time - 1e-6
        ):
            self.reset()
            self._seed_after_reset(
                stream, source_time, receipt_ros_time, observed_offset
            )
            self.last_status = 'ROS_CLOCK_RESET'
            return None
        self._last_receipt_time = receipt_ros_time

        previous_source = self._last_source_times.get(stream)
        if previous_source is not None:
            regression = previous_source - source_time
            if abs(regression) <= 1e-9:
                self.last_status = 'DUPLICATE_SOURCE_TIMESTAMP'
                return None
            if regression > 0.0:
                if (
                    stream_name is not None
                    and regression <= self.reset_regression_threshold
                ):
                    self.last_status = 'OUT_OF_ORDER_SOURCE_TIMESTAMP'
                    return None
                confirmed = (
                    stream_name is None
                    or self._confirmed_source_reset(
                        stream, source_time, receipt_ros_time
                    )
                )
                if not confirmed:
                    self.last_status = 'SOURCE_RESET_CANDIDATE'
                    return None
                self.reset()
                self._seed_after_reset(
                    stream, source_time, receipt_ros_time, observed_offset
                )
                self.last_status = 'PX4_CLOCK_RESET'
                return None
        if self.source_is_ros_time:
            self._last_source_times[stream] = source_time
            self._reset_candidates.pop(stream, None)
            self.last_status = 'DIRECT_TIMESTAMP'
            return source_time
        if (
            stream_name is not None
            and self.offset is not None
            and observed_offset
            < self.offset - self.maximum_offset_jump
        ):
            if self._confirmed_offset_change(
                stream,
                observed_offset,
                receipt_ros_time,
            ):
                self.reset()
                self.offset_recalibration_count += 1
                self._seed_after_reset(
                    stream,
                    source_time,
                    receipt_ros_time,
                    observed_offset,
                )
                self.last_status = 'OFFSET_RECALIBRATING'
                return None
            self.last_status = 'OFFSET_OUTLIER'
            return None
        self._last_source_times[stream] = source_time
        self._reset_candidates.pop(stream, None)
        self._offset_outlier_samples.pop(stream, None)
        self._offset_samples.append(observed_offset)
        if len(self._offset_samples) < self.calibration_samples:
            self.last_status = 'UNCALIBRATED'
            return None
        if (
            max(self._offset_samples) - min(self._offset_samples)
            > self.maximum_offset_jump
            and self.offset is None
        ):
            self.last_status = 'OFFSET_UNSTABLE'
            return None
        candidate = min(self._offset_samples)
        if self.offset is not None and abs(
            candidate - self.offset
        ) > self.maximum_offset_jump:
            self.reset()
            self._seed_after_reset(
                stream, source_time, receipt_ros_time, observed_offset
            )
            self.last_status = 'OFFSET_JUMP_RESET'
            return None
        if self.offset is None:
            self.offset = candidate
            self.calibration_count += 1
        else:
            self.offset = min(self.offset, candidate)
        self.last_status = 'MAPPED'
        return source_time + self.offset


@dataclass(frozen=True)
class GazeboClockAnchor:
    """Gazebo simulation and system times sampled in one Clock message."""

    sim_time: float
    system_time: float
    receipt_ros_time: float
    receipt_monotonic: float


class GazeboImageClockMapper:
    """
    Map Gazebo sensor time using server-side sim/system clock pairs.

    Image receipt time is deliberately excluded from the mapping.  It is only
    used later for freshness checks.  Mapping is interpolation-only so an
    image newer than the latest clock anchor waits instead of being
    extrapolated from an uncertain real-time factor.
    """

    mapping_mode = 'GAZEBO_CLOCK_SYSTEM_INTERPOLATION'

    def __init__(
        self,
        maximum_reference_age=0.5,
        history_duration=2.0,
        maximum_system_clock_step=0.25,
        capacity=4096,
    ):
        self.maximum_reference_age = max(
            float(maximum_reference_age), 1e-3
        )
        self.history_duration = max(float(history_duration), 0.1)
        self.maximum_system_clock_step = max(
            float(maximum_system_clock_step), 1e-3
        )
        self._anchors = deque(maxlen=max(int(capacity), 4))
        self._lock = threading.Lock()
        self.reset_count = 0
        self.last_status = 'CLOCK_REFERENCE_UNAVAILABLE'
        self.offset = None
        self.last_quality = math.nan
        self.last_anchor_sim_time = math.nan
        self.last_anchor_system_time = math.nan
        self.last_reference_age = math.nan
        self._pending_reset_status = ''

    def reset(self, status='CLOCK_REFERENCE_RESET'):
        with self._lock:
            self._reset_locked(status)

    def _reset_locked(self, status):
        self._anchors.clear()
        self.reset_count += 1
        self.last_status = str(status)
        self.offset = None
        self.last_quality = math.nan
        self.last_anchor_sim_time = math.nan
        self.last_anchor_system_time = math.nan
        self.last_reference_age = math.nan
        self._pending_reset_status = self.last_status

    @property
    def anchor_range(self):
        with self._lock:
            if not self._anchors:
                return None
            return (
                self._anchors[0].sim_time,
                self._anchors[-1].sim_time,
            )

    def add_anchor(
        self,
        sim_time,
        system_time,
        receipt_ros_time,
        receipt_monotonic=None,
    ):
        """Add one trusted pair from the same Gazebo Clock message."""
        sim_time = float(sim_time)
        system_time = float(system_time)
        receipt_ros_time = float(receipt_ros_time)
        receipt_monotonic = (
            time.monotonic()
            if receipt_monotonic is None else float(receipt_monotonic)
        )
        if (
            not all(math.isfinite(value) for value in (
                sim_time,
                system_time,
                receipt_ros_time,
                receipt_monotonic,
            ))
            or sim_time < 0.0
            or system_time <= 0.0
        ):
            with self._lock:
                self.last_status = 'INVALID_CLOCK_REFERENCE'
            return False
        reference_age = receipt_ros_time - system_time
        if (
            reference_age < -1e-3
            or reference_age > self.maximum_reference_age
        ):
            with self._lock:
                self.last_status = 'CLOCK_REFERENCE_DOMAIN_MISMATCH'
                self.last_reference_age = reference_age
            return False
        anchor = GazeboClockAnchor(
            sim_time,
            system_time,
            receipt_ros_time,
            receipt_monotonic,
        )
        with self._lock:
            if self._anchors:
                previous = self._anchors[-1]
                sim_delta = sim_time - previous.sim_time
                system_delta = system_time - previous.system_time
                monotonic_delta = (
                    receipt_monotonic - previous.receipt_monotonic
                )
                if sim_delta < -1e-9:
                    self._reset_locked('SIM_TIME_RESET')
                    self._anchors.append(anchor)
                    self._record_anchor_locked(anchor, reference_age)
                    return False
                if abs(sim_delta) <= 1e-9:
                    self.last_status = 'SIM_TIME_PAUSED'
                    self.last_reference_age = reference_age
                    return False
                if (
                    system_delta <= 0.0
                    or monotonic_delta <= 0.0
                    or abs(system_delta - monotonic_delta)
                    > self.maximum_system_clock_step
                ):
                    self._reset_locked('SYSTEM_CLOCK_RESET')
                    self._anchors.append(anchor)
                    self._record_anchor_locked(anchor, reference_age)
                    return False
            self._anchors.append(anchor)
            cutoff = sim_time - self.history_duration
            while (
                len(self._anchors) > 2
                and self._anchors[1].sim_time < cutoff
            ):
                self._anchors.popleft()
            self._record_anchor_locked(anchor, reference_age)
            self.last_status = (
                'CLOCK_REFERENCE_READY'
                if len(self._anchors) >= 2
                else 'CLOCK_REFERENCE_WARMING_UP'
            )
            if len(self._anchors) >= 2:
                self._pending_reset_status = ''
            return True

    def _record_anchor_locked(self, anchor, reference_age):
        self.last_anchor_sim_time = anchor.sim_time
        self.last_anchor_system_time = anchor.system_time
        self.last_reference_age = reference_age

    def map_time(self, sim_time, now_ros_time):
        """Return a mapped time and status from one atomic clock snapshot."""
        sim_time = float(sim_time)
        now_ros_time = float(now_ros_time)
        if (
            not math.isfinite(sim_time)
            or sim_time <= 0.0
            or not math.isfinite(now_ros_time)
        ):
            with self._lock:
                self.last_status = 'INVALID_IMAGE_TIMESTAMP'
                return None, self.last_status
        with self._lock:
            if len(self._anchors) < 2:
                if self._pending_reset_status:
                    self.last_status = self._pending_reset_status
                elif self._anchors:
                    self.last_status = 'CLOCK_REFERENCE_WARMING_UP'
                else:
                    self.last_status = 'CLOCK_REFERENCE_UNAVAILABLE'
                return None, self.last_status
            latest = self._anchors[-1]
            reference_age = now_ros_time - latest.system_time
            self.last_reference_age = reference_age
            if reference_age > self.maximum_reference_age + 1e-9:
                self.last_status = 'CLOCK_REFERENCE_STALE'
                return None, self.last_status
            if sim_time < self._anchors[0].sim_time - 1e-9:
                self.last_status = 'IMAGE_BEFORE_CLOCK_REFERENCE'
                return None, self.last_status
            if sim_time > latest.sim_time + 1e-9:
                self.last_status = 'IMAGE_AFTER_CLOCK_REFERENCE'
                return None, self.last_status
            left = self._anchors[0]
            right = self._anchors[-1]
            for index, candidate in enumerate(self._anchors):
                if abs(sim_time - candidate.sim_time) <= 1e-9:
                    left = right = candidate
                    break
                if candidate.sim_time > sim_time:
                    left = self._anchors[index - 1]
                    right = candidate
                    break
            if left is right:
                mapped = left.system_time
                self.last_quality = 0.0
                self.last_status = 'MAPPED_EXACT'
            else:
                sim_delta = right.sim_time - left.sim_time
                fraction = (sim_time - left.sim_time) / sim_delta
                mapped = left.system_time + fraction * (
                    right.system_time - left.system_time
                )
                # The interpolation bracket width is the timing quality
                # metric: smaller is better, while zero is an exact anchor.
                self.last_quality = sim_delta
                self.last_status = 'MAPPED_INTERPOLATED'
            self.offset = mapped - sim_time
            self.last_anchor_sim_time = right.sim_time
            self.last_anchor_system_time = right.system_time
            return mapped, self.last_status

    def to_ros_time(self, sim_time, now_ros_time):
        """Interpolate a bracketed Gazebo acquisition time into ROS time."""
        mapped, _status = self.map_time(sim_time, now_ros_time)
        return mapped


@dataclass(frozen=True)
class TimestampedImageFrame:
    message: object
    raw_stamp: float
    receipt_stamp: float


@dataclass(frozen=True)
class RgbdTargetGeometry:
    """Bounded intermediate geometry for one RGB-D localization event."""

    red_pixel_count: int
    valid_depth_count: int
    mask_center: tuple
    projection_center: tuple
    mask_bbox: tuple
    depth_min: float
    depth_median: float
    depth_mad: float
    intrinsics: tuple
    surface_camera: tuple
    center_camera: tuple
    fit_position_std: float = 0.0


class RgbDepthPairBuffer:
    """Pair each frame at most once using bounded acquisition-time queues."""

    def __init__(self, maximum_skew, capacity=8):
        self.maximum_skew = max(float(maximum_skew), 0.0)
        self.capacity = max(int(capacity), 2)
        self.color = deque(maxlen=self.capacity)
        self.depth = deque(maxlen=self.capacity)
        self.last_stamp = {'color': None, 'depth': None}
        self.last_rejected_pair = None

    def reset(self):
        self.color.clear()
        self.depth.clear()
        self.last_stamp = {'color': None, 'depth': None}
        self.last_rejected_pair = None

    def add(self, stream, message, raw_stamp, receipt_stamp):
        raw_stamp = float(raw_stamp)
        receipt_stamp = float(receipt_stamp)
        if stream not in self.last_stamp:
            raise ValueError('stream must be color or depth')
        if not math.isfinite(raw_stamp) or raw_stamp <= 0.0:
            return 'RAW_TIMESTAMP_INVALID'
        previous = self.last_stamp[stream]
        if previous is not None and raw_stamp <= previous + 1e-9:
            if raw_stamp < previous - 1.0:
                self.reset()
                self.last_stamp[stream] = raw_stamp
                getattr(self, stream).append(TimestampedImageFrame(
                    message, raw_stamp, receipt_stamp
                ))
                return 'TIME_RESET'
            return (
                'DUPLICATE_FRAME' if abs(raw_stamp - previous) <= 1e-9
                else 'OUT_OF_ORDER_FRAME'
            )
        self.last_stamp[stream] = raw_stamp
        getattr(self, stream).append(TimestampedImageFrame(
            message, raw_stamp, receipt_stamp
        ))
        return ''

    def pop_pair(self):
        while self.color and self.depth:
            color_index, depth_index, minimum_skew = min(
                (
                    (color_index, depth_index, abs(
                        color.raw_stamp - depth.raw_stamp
                    ))
                    for color_index, color in enumerate(self.color)
                    for depth_index, depth in enumerate(self.depth)
                ),
                key=lambda candidate: candidate[2],
            )
            if minimum_skew <= self.maximum_skew + 1e-9:
                color = self.color[color_index]
                depth = self.depth[depth_index]
                del self.color[color_index]
                del self.depth[depth_index]
                return color, depth, ''
            if self.color[0].raw_stamp < self.depth[0].raw_stamp:
                self.last_rejected_pair = (self.color[0], self.depth[0])
                self.color.popleft()
                return None, None, 'RGB_FRAME_UNMATCHED'
            self.last_rejected_pair = (self.color[0], self.depth[0])
            self.depth.popleft()
            return None, None, 'DEPTH_FRAME_UNMATCHED'
        return None, None, ''


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _gazebo_time_seconds(stamp):
    return float(stamp.sec) + float(stamp.nsec) * 1e-9


def _seconds_to_time(value):
    if value is None or not math.isfinite(float(value)) or value <= 0.0:
        return Time()
    seconds = int(value)
    nanoseconds = int(round((float(value) - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def body_frd_to_ned_rotation(quaternion):
    """Return the body-FRD to local-NED rotation matrix used by PX4."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,):
        raise ValueError('attitude quaternion must contain four values')
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError('attitude quaternion must be finite and nonzero')
    w, x, y, z = q / norm
    return np.array([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])


def target_vector_from_rgbd(
    target_mask,
    depth,
    horizontal_fov,
    minimum_depth,
    maximum_depth,
    target_radius=0.0,
):
    """Recover the target-center vector in camera FLU coordinates."""
    geometry = target_geometry_from_rgbd(
        target_mask,
        depth,
        horizontal_fov,
        minimum_depth,
        maximum_depth,
        target_radius,
    )
    if geometry is None:
        return None
    return np.asarray(geometry.center_camera, dtype=float)


def validation_sphere_center(points, radius):
    """Recover the validation sphere center from its visible depth surface."""
    radius = float(radius)
    if (not math.isfinite(radius) or radius <= 0.0 or len(points) < 12
            or not np.isfinite(points).all()):
        return None
    # Keep the cost bounded even when the sphere fills the image. The fit
    # does not need a complete silhouette, unlike a median surface ray.
    points = points[::max(1, math.ceil(len(points) / 512))]
    matrix = np.column_stack((2.0 * points, np.ones(len(points))))
    solution, _, rank, singular = np.linalg.lstsq(
        matrix, np.sum(points * points, axis=1), rcond=None,
    )
    # A tiny nearly planar cap can have exact rank and negligible residual,
    # while amplifying depth/model errors by more than 100000 times.
    if rank != 4 or singular[-1] <= 1e-5 * singular[0]:
        return None
    center = solution[:3]
    radius_squared = solution[3] + np.dot(center, center)
    if not np.isfinite(center).all() or center[0] <= 0 or radius_squared <= 0:
        return None
    if abs(math.sqrt(radius_squared) - radius) > .1 * radius:
        return None
    residual = np.abs(np.linalg.norm(points - center, axis=1) - radius)
    if np.percentile(residual, 95) > max(.05 * radius, .001):
        return None
    return tuple(float(value) for value in center)


def target_geometry_from_rgbd(
    target_mask,
    depth,
    horizontal_fov,
    minimum_depth,
    maximum_depth,
    target_radius=0.0,
    require_valid_sphere=False,
    sphere_fit_mode='ideal',
    sphere_noise_std=.01,
    sphere_range_noise_scale=.001,
):
    """
    Return the current estimator inputs and intermediate camera vectors.

    Gazebo Rendering's RGB-D depth image contains the camera-forward X
    component. Recover the validation sphere center from the visible 3D
    surface, including partial close views. Sparse or degenerate surfaces
    retain the representative surface-ray approximation for diagnostics.
    The online node requires a valid sphere and rejects that approximation.
    Intermediate values keep the depth measurement independently auditable.
    """
    if target_mask is None or depth is None:
        return None
    if target_mask.shape != depth.shape:
        return None
    mask_rows, mask_columns = np.nonzero(target_mask)
    if mask_columns.size == 0:
        return None
    valid = (
        target_mask
        & np.isfinite(depth)
        & (depth >= float(minimum_depth))
        & (depth <= float(maximum_depth))
    )
    rows, columns = np.nonzero(valid)
    if columns.size == 0:
        return None
    height, width = depth.shape
    fx, fy, cx, cy = camera_intrinsics(width, height, horizontal_fov)
    valid_depths = np.asarray(depth[valid], dtype=float)
    surface_forward = float(np.median(valid_depths))
    mask_center_x = float(np.median(mask_columns))
    mask_center_y = float(np.median(mask_rows))
    image_x = float(np.median(columns))
    image_y = float(np.median(rows))
    # Depth is the camera-X component, so the surface point is
    # surface_forward * ray.  A Euclidean radius adds R / |ray| in X.
    ray_length = math.sqrt(
        1.0 + ((image_x - cx) / fx) ** 2
        + ((image_y - cy) / fy) ** 2
    )
    forward = surface_forward + max(float(target_radius), 0.0) / ray_length

    # Gazebo's camera optical axis is +X. Image right is camera -Y and
    # image down is camera -Z for the FLU camera-link convention.
    surface_left = -(image_x - cx) * surface_forward / fx
    surface_up = -(image_y - cy) * surface_forward / fy
    left = -(image_x - cx) * forward / fx
    up = -(image_y - cy) * forward / fy
    points = np.column_stack((
        valid_depths,
        -(columns - cx) * valid_depths / fx,
        -(rows - cy) * valid_depths / fy,
    ))
    fit_position_std = 0.0
    if sphere_fit_mode == 'tof' and target_radius > 0:
        from .tof_sphere_localization import fit_tof_sphere
        fit = fit_tof_sphere(points, target_radius, noise_std=sphere_noise_std,
                             range_noise_scale=sphere_range_noise_scale)
        fitted = None if fit is None else fit.center
        fit_position_std = 0.0 if fit is None else fit.position_std
    elif sphere_fit_mode == 'ideal':
        fitted = validation_sphere_center(points, target_radius)
    else:
        raise ValueError('sphere_fit_mode must be ideal or tof with positive radius')
    if require_valid_sphere and target_radius > 0.0 and fitted is None:
        return None
    if fitted is not None:
        forward, left, up = fitted
    depth_mad = float(np.median(np.abs(
        valid_depths - surface_forward
    )))
    return RgbdTargetGeometry(
        red_pixel_count=int(mask_columns.size),
        valid_depth_count=int(columns.size),
        mask_center=(mask_center_x, mask_center_y),
        projection_center=(image_x, image_y),
        mask_bbox=(
            int(mask_columns.min()),
            int(mask_rows.min()),
            int(mask_columns.max()),
            int(mask_rows.max()),
        ),
        depth_min=float(valid_depths.min()),
        depth_median=surface_forward,
        depth_mad=depth_mad,
        intrinsics=(float(fx), float(fy), float(cx), float(cy)),
        surface_camera=(surface_forward, surface_left, surface_up),
        center_camera=(forward, left, up),
        fit_position_std=fit_position_std,
    )


def camera_target_to_local_ned(
    camera_vector_flu,
    uav_position_ned,
    attitude_quaternion,
    camera_translation_flu,
    camera_pitch_down,
    target_reference_z_offset=0.0,
):
    """Transform a camera-FLU target vector to PX4 local NED position."""
    body_flu = camera_target_to_body_flu(
        camera_vector_flu,
        camera_translation_flu,
        camera_pitch_down,
    )
    uav_position = np.asarray(uav_position_ned, dtype=float)
    if uav_position.shape != (3,) or not np.all(np.isfinite(uav_position)):
        raise ValueError('vehicle position must be a finite 3-vector')
    body_frd = np.array((body_flu[0], -body_flu[1], -body_flu[2]))
    position_ned = (
        uav_position
        + body_frd_to_ned_rotation(attitude_quaternion) @ body_frd
    )
    position_ned[2] += float(target_reference_z_offset)
    return position_ned


def camera_target_to_body_flu(
    camera_vector_flu,
    camera_translation_flu,
    camera_pitch_down,
):
    """Apply the SDF camera mount transform without changing conventions."""
    camera_vector = np.asarray(camera_vector_flu, dtype=float)
    translation = np.asarray(camera_translation_flu, dtype=float)
    if (
        camera_vector.shape != (3,)
        or translation.shape != (3,)
        or not np.all(np.isfinite(camera_vector))
        or not np.all(np.isfinite(translation))
    ):
        raise ValueError('camera and translation must be finite 3-vectors')

    pitch = float(camera_pitch_down)
    cosine = math.cos(pitch)
    sine = math.sin(pitch)
    camera_x, camera_y, camera_z = camera_vector
    body_flu = translation + np.array((
        cosine * camera_x + sine * camera_z,
        camera_y,
        -sine * camera_x + cosine * camera_z,
    ))
    return body_flu


def local_ned_target_to_camera_flu(
    target_position_ned,
    uav_position_ned,
    attitude_quaternion,
    camera_translation_flu,
    camera_pitch_down,
):
    """Invert the configured camera mount and vehicle pose for diagnostics."""
    target_position = np.asarray(target_position_ned, dtype=float)
    uav_position = np.asarray(uav_position_ned, dtype=float)
    translation = np.asarray(camera_translation_flu, dtype=float)
    if (
        target_position.shape != (3,)
        or uav_position.shape != (3,)
        or translation.shape != (3,)
        or not np.all(np.isfinite(target_position))
        or not np.all(np.isfinite(uav_position))
        or not np.all(np.isfinite(translation))
    ):
        raise ValueError('target, vehicle and translation must be finite')
    relative_ned = target_position - uav_position
    body_frd = (
        body_frd_to_ned_rotation(attitude_quaternion).T @ relative_ned
    )
    body_flu = np.array((body_frd[0], -body_frd[1], -body_frd[2]))
    relative_body = body_flu - translation
    pitch = float(camera_pitch_down)
    cosine = math.cos(pitch)
    sine = math.sin(pitch)
    return np.array((
        cosine * relative_body[0] - sine * relative_body[2],
        relative_body[1],
        sine * relative_body[0] + cosine * relative_body[2],
    ))


class RgbdTargetLocalizer(Node):
    """Publish camera-derived USV position in the PX4 local NED frame."""

    def __init__(self):
        """Configure image, vehicle-state, and observation interfaces."""
        super().__init__('rgbd_target_localizer')
        self.declare_parameter('color_topic', '/camera/front/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/camera/front/depth/image_raw',
        )
        self.declare_parameter(
            'observation_topic',
            '/perception/front/target_observation',
        )
        self.declare_parameter(
            'position_topic',
            '/perception/front/target_position',
        )
        self.declare_parameter('frame_id', 'local_ned')
        self.declare_parameter('horizontal_fov', 1.74)
        self.declare_parameter('minimum_red_pixels', 3)
        self.declare_parameter('minimum_depth', 0.05)
        self.declare_parameter('maximum_depth', 25.0)
        self.declare_parameter('minimum_depth_ratio', 0.5)
        self.declare_parameter('maximum_depth_mad', 0.25)
        self.declare_parameter('maximum_rgb_depth_skew', 0.1)
        self.declare_parameter('data_timeout', 0.5)
        self.declare_parameter('state_history_duration', 1.0)
        self.declare_parameter('maximum_clock_offset_jump', 0.05)
        self.declare_parameter('px4_timestamp_is_ros_time', True)
        self.declare_parameter('gazebo_world_name', 'default')
        self.declare_parameter('image_clock_reference_timeout', 0.5)
        self.declare_parameter('image_clock_history_duration', 2.0)
        self.declare_parameter('image_clock_wait_timeout', 0.15)
        self.declare_parameter('maximum_system_clock_step', 0.25)
        self.declare_parameter('pose_wait_timeout', 0.15)
        self.declare_parameter('maximum_pose_wait_gap', 0.15)
        self.declare_parameter('image_pair_buffer_size', 8)
        self.declare_parameter('time_pair_diagnostics_enabled', False)
        self.declare_parameter('localization_rate_hz', 20.0)
        self.declare_parameter('camera_pitch_down', 0.4886921905584123)
        self.declare_parameter(
            'camera_translation_x', DEFAULT_CAMERA_TRANSLATION_FLU[0],
        )
        self.declare_parameter(
            'camera_translation_y', DEFAULT_CAMERA_TRANSLATION_FLU[1],
        )
        self.declare_parameter(
            'camera_translation_z', DEFAULT_CAMERA_TRANSLATION_FLU[2],
        )
        self.declare_parameter('target_radius', 0.25)
        self.declare_parameter('sphere_fit_mode', 'ideal')
        self.declare_parameter('sphere_noise_std', .01)
        self.declare_parameter('sphere_range_noise_scale', .001)
        self.declare_parameter('target_reference_z_offset', 0.42)
        self.declare_parameter('geometry_diagnostics_enabled', False)
        self.declare_parameter('base_position_std', 0.08)
        self.declare_parameter('range_position_std_scale', 0.01)

        self.horizontal_fov = float(
            self.get_parameter('horizontal_fov').value
        )
        self.minimum_red_pixels = max(
            int(self.get_parameter('minimum_red_pixels').value),
            1,
        )
        self.minimum_depth = max(
            float(self.get_parameter('minimum_depth').value),
            0.0,
        )
        self.maximum_depth = max(
            float(self.get_parameter('maximum_depth').value),
            self.minimum_depth,
        )
        self.minimum_depth_ratio = min(max(
            float(self.get_parameter('minimum_depth_ratio').value),
            0.0,
        ), 1.0)
        self.maximum_depth_mad = max(
            float(self.get_parameter('maximum_depth_mad').value),
            0.0,
        )
        self.maximum_rgb_depth_skew = max(
            float(self.get_parameter('maximum_rgb_depth_skew').value),
            0.0,
        )
        self.data_timeout = max(
            float(self.get_parameter('data_timeout').value),
            0.05,
        )
        self.state_history_duration = max(
            float(self.get_parameter('state_history_duration').value),
            self.data_timeout,
        )
        self.maximum_clock_offset_jump = max(
            float(self.get_parameter('maximum_clock_offset_jump').value),
            1e-3,
        )
        self.px4_timestamp_is_ros_time = bool(
            self.get_parameter('px4_timestamp_is_ros_time').value
        )
        self.image_clock_reference_timeout = max(
            float(
                self.get_parameter('image_clock_reference_timeout').value
            ),
            0.05,
        )
        self.image_clock_history_duration = max(
            float(
                self.get_parameter('image_clock_history_duration').value
            ),
            self.image_clock_reference_timeout,
        )
        self.image_clock_wait_timeout = max(
            float(self.get_parameter('image_clock_wait_timeout').value),
            0.0,
        )
        self.maximum_system_clock_step = max(
            float(self.get_parameter('maximum_system_clock_step').value),
            1e-3,
        )
        self.pose_wait_timeout = max(
            float(self.get_parameter('pose_wait_timeout').value),
            0.0,
        )
        self.maximum_pose_wait_gap = max(
            float(self.get_parameter('maximum_pose_wait_gap').value),
            0.0,
        )
        self.camera_pitch_down = float(
            self.get_parameter('camera_pitch_down').value
        )
        self.camera_translation_flu = tuple(
            float(self.get_parameter(name).value)
            for name in (
                'camera_translation_x',
                'camera_translation_y',
                'camera_translation_z',
            )
        )
        self.target_radius = max(
            float(self.get_parameter('target_radius').value),
            0.0,
        )
        self.sphere_fit_mode = str(self.get_parameter('sphere_fit_mode').value)
        self.sphere_noise_std = float(self.get_parameter('sphere_noise_std').value)
        self.sphere_range_noise_scale = float(
            self.get_parameter('sphere_range_noise_scale').value,
        )
        if (self.sphere_fit_mode not in ('ideal', 'tof')
                or (self.sphere_fit_mode == 'tof' and self.target_radius <= 0)
                or not math.isfinite(self.sphere_noise_std) or self.sphere_noise_std <= 0
                or not math.isfinite(self.sphere_range_noise_scale)
                or self.sphere_range_noise_scale < 0):
            raise ValueError('invalid sphere fitting mode/noise parameters')
        self.target_reference_z_offset = float(
            self.get_parameter('target_reference_z_offset').value
        )
        self.geometry_diagnostics_enabled = bool(
            self.get_parameter('geometry_diagnostics_enabled').value
        )
        self.base_position_std = max(
            float(self.get_parameter('base_position_std').value),
            1.0e-3,
        )
        self.range_position_std_scale = max(
            float(
                self.get_parameter('range_position_std_scale').value
            ),
            0.0,
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.time_pair_diagnostics_enabled = bool(
            self.get_parameter('time_pair_diagnostics_enabled').value
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        color_topic = str(self.get_parameter('color_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        image_qos = aligned_camera_qos()
        self.color_sub = self.create_subscription(
            Image,
            color_topic,
            self.color_callback,
            image_qos,
        )
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            image_qos,
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
        self.timesync_sub = self.create_subscription(
            TimesyncStatus,
            '/fmu/out/timesync_status',
            self.timesync_callback,
            sensor_qos,
        )
        self.observation_pub = self.create_publisher(
            TargetObservation,
            str(self.get_parameter('observation_topic').value),
            10,
        )
        self.target_position_pub = self.create_publisher(
            Point,
            str(self.get_parameter('position_topic').value),
            10,
        )
        self.compute_time_pub = self.create_publisher(
            Float32,
            '/diagnostics/rgbd_localizer/compute_time',
            10,
        )
        self.navigation_state_pub = self.create_publisher(
            UavState, '/navigation/uav_state', 10,
        )
        self._navigation_lock = threading.RLock()
        self._navigation_samples = deque(maxlen=32)
        self._navigation_last_stamp = -math.inf

        self.latest_color_message = None
        self.latest_depth_message = None
        self.color_mask = None
        self.color_time = -math.inf
        self.color_measurement_time = None
        self.depth = None
        self.depth_time = -math.inf
        self.depth_measurement_time = None
        self.uav_position = None
        self.uav_position_time = -math.inf
        self.uav_attitude = None
        self.uav_attitude_time = -math.inf
        self.position_source_stamp = math.nan
        self.position_mapped_stamp = math.nan
        self.attitude_source_stamp = math.nan
        self.attitude_mapped_stamp = math.nan
        self.processed_color_time = -math.inf
        self.last_image_timeout_report = -math.inf
        self.last_pair_rejection = ''
        self.pending_image_pairs = deque(maxlen=max(
            int(self.get_parameter('image_pair_buffer_size').value), 2
        ))
        self.waiting_image_pair = None
        self.image_pair_buffer = RgbDepthPairBuffer(
            self.maximum_rgb_depth_skew,
            self.pending_image_pairs.maxlen,
        )
        self.last_time_diagnostic = {}
        self.last_time_diagnostic_log = -math.inf
        self._pending_pose_samples = deque(maxlen=8)

        # P8.1: keep a causal history of the uXRCE-DDS PX4->Agent
        # time offset.  PX4 outgoing timestamps have already had this
        # conversion applied by uXRCE.  Undo it first, then map the recovered
        # PX4/Gazebo simulation time through the same Gazebo clock mapper used
        # by camera acquisition timestamps.
        self.timesync_history = deque(maxlen=16)
        self._pending_raw_pose_samples = deque(maxlen=32)
        self.latest_timesync_stamp = math.nan
        self.latest_timesync_offset = math.nan
        self.position_raw_sim_stamp = math.nan
        self.attitude_raw_sim_stamp = math.nan

        self.position_history = TimestampedVectorHistory(
            self.state_history_duration
        )
        self.attitude_history = TimestampedVectorHistory(
            self.state_history_duration,
            interpolator=quaternion_slerp,
        )
        self.image_clock_mapper = GazeboImageClockMapper(
            maximum_reference_age=self.image_clock_reference_timeout,
            history_duration=self.image_clock_history_duration,
            maximum_system_clock_step=self.maximum_system_clock_step,
        )
        self.px4_clock_mapper = Px4RosClockMapper(
            self.maximum_clock_offset_jump,
            source_is_ros_time=self.px4_timestamp_is_ros_time,
        )
        self._image_clock_reset_pending = False
        # Wake the ROS executor when data becomes ready. Keep decoding out of
        # DDS and Gazebo transport callbacks; the timer remains a timeout retry.
        self.localization_guard = self.create_guard_condition(self.timed_localize)
        from gz.msgs10.clock_pb2 import Clock as GazeboClock
        from gz.transport13 import Node as GazeboTransportNode

        world_name = str(
            self.get_parameter('gazebo_world_name').value
        ).strip().strip('/')
        self.gazebo_clock_topic = f'/world/{world_name}/clock'
        self.gazebo_clock_node = GazeboTransportNode()
        if not self.gazebo_clock_node.subscribe(
            GazeboClock,
            self.gazebo_clock_topic,
            self.gazebo_clock_callback,
        ):
            raise RuntimeError(
                'Could not subscribe to trusted Gazebo clock topic '
                f'{self.gazebo_clock_topic}.'
            )
        localization_rate_hz = max(
            float(self.get_parameter('localization_rate_hz').value),
            1.0,
        )
        self.timer = self.create_timer(
            1.0 / localization_rate_hz,
            self.timed_localize,
        )
        self.get_logger().info(
            'RGB-D TARGET LOCALIZER READY | '
            f'RGB={color_topic} | depth={depth_topic} | '
            f'clock={self.gazebo_clock_topic} | '
            'output frame=local_ned | detector=red validation sphere'
        )

    def gazebo_clock_callback(self, message):
        """Store a server-side Gazebo sim/system time correspondence."""
        reset_count = self.image_clock_mapper.reset_count
        self.image_clock_mapper.add_anchor(
            _gazebo_time_seconds(message.sim),
            _gazebo_time_seconds(message.system),
            self._ros_seconds(),
            time.monotonic(),
        )
        if self.image_clock_mapper.reset_count != reset_count:
            self._image_clock_reset_pending = True
            self._reset_pose_time_state()

        self._flush_pending_raw_pose_samples()
        self._flush_navigation_states()
        self._request_localization()

    def _request_localization(self):
        """Schedule ready or causally waiting image work in the ROS executor."""
        guard = getattr(self, 'localization_guard', None)
        if guard is not None and (
            self.pending_image_pairs or getattr(self, 'waiting_image_pair', None)
        ):
            guard.trigger()

    def _apply_pending_image_clock_reset(self):
        if not getattr(self, '_image_clock_reset_pending', False):
            return
        self._image_clock_reset_pending = False
        self.image_pair_buffer.reset()
        self.pending_image_pairs.clear()
        self.waiting_image_pair = None

    def color_callback(self, message):
        """Cache RGB by acquisition stamp without image work in DDS."""
        receipt = self._ros_seconds()
        source = _stamp_seconds(message.header.stamp)
        self.latest_color_message = message
        self.color_time = receipt
        self.last_pair_rejection = self.image_pair_buffer.add(
            'color', message, source, receipt
        )
        if not hasattr(self, 'last_time_diagnostic'):
            self.last_time_diagnostic = {}
        self.last_time_diagnostic.update({
            'rgb_raw_stamp': source,
            'rgb_receipt_stamp': receipt,
        })
        if self.last_pair_rejection == 'TIME_RESET':
            self.pending_image_pairs.clear()
            self.waiting_image_pair = None
            self.image_clock_mapper.reset()
        self._collect_image_pair()
        self._request_localization()

    def depth_callback(self, message):
        """Cache depth by acquisition stamp without decoding it in DDS."""
        receipt = self._ros_seconds()
        source = _stamp_seconds(message.header.stamp)
        self.latest_depth_message = message
        self.depth_time = receipt
        self.last_pair_rejection = self.image_pair_buffer.add(
            'depth', message, source, receipt
        )
        if not hasattr(self, 'last_time_diagnostic'):
            self.last_time_diagnostic = {}
        self.last_time_diagnostic.update({
            'depth_raw_stamp': source,
            'depth_receipt_stamp': receipt,
        })
        if self.last_pair_rejection == 'TIME_RESET':
            self.pending_image_pairs.clear()
            self.waiting_image_pair = None
            self.image_clock_mapper.reset()
        self._collect_image_pair()
        self._request_localization()

    def _collect_image_pair(self):
        while True:
            color, depth, rejection = self.image_pair_buffer.pop_pair()
            if rejection:
                self.last_pair_rejection = rejection
                rejected = self.image_pair_buffer.last_rejected_pair
                if rejected is not None:
                    rejected_color, rejected_depth = rejected
                    self.last_time_diagnostic.update({
                        'rgb_raw_stamp': rejected_color.raw_stamp,
                        'depth_raw_stamp': rejected_depth.raw_stamp,
                        'rgb_receipt_stamp': (
                            rejected_color.receipt_stamp
                        ),
                        'depth_receipt_stamp': (
                            rejected_depth.receipt_stamp
                        ),
                        'rgb_depth_acquisition_skew': abs(
                            rejected_color.raw_stamp
                            - rejected_depth.raw_stamp
                        ),
                    })
                continue
            if color is None:
                return
            self.pending_image_pairs.append((color, depth))
            self.last_pair_rejection = ''

    def timesync_callback(self, message):
        """Cache causal uXRCE PX4->Agent time offsets."""
        stamp_us = int(getattr(message, 'timestamp', 0))
        offset_us = int(getattr(message, 'estimated_offset', 0))

        if stamp_us <= 0:
            return

        stamp = stamp_us * 1e-6
        offset = offset_us * 1e-6

        if not math.isfinite(stamp) or not math.isfinite(offset):
            return

        if (
            self.timesync_history
            and stamp < self.timesync_history[-1][0] - 1e-6
        ):
            # New DDS/PX4 time epoch.  Never interpolate pose across it.
            self.timesync_history.clear()
            self._pending_raw_pose_samples.clear()
            self._reset_pose_time_state()

        if (
            self.timesync_history
            and abs(stamp - self.timesync_history[-1][0]) <= 1e-9
        ):
            self.timesync_history[-1] = (stamp, offset)
        else:
            self.timesync_history.append((stamp, offset))

        self.latest_timesync_stamp = stamp
        self.latest_timesync_offset = offset

    def _causal_timesync_offset(self, source_stamp):
        """Return the newest timesync sample not later than the pose stamp."""
        source_stamp = float(source_stamp)

        for stamp, offset in reversed(self.timesync_history):
            if stamp <= source_stamp + 1e-9:
                return offset, stamp

        return None, None

    def _map_px4_pose_stamp(self, source_stamp, receipt):
        """
        Recover PX4/Gazebo simulation sample time and map it to ROS time.

        uXRCE publishes:
            ros_stamp = raw_px4_stamp - estimated_offset

        therefore:
            raw_px4_stamp = ros_stamp + estimated_offset
        """
        offset, timesync_stamp = self._causal_timesync_offset(source_stamp)

        if offset is None:
            return None, None, None

        raw_sim_stamp = float(source_stamp) + float(offset)

        if (
            not math.isfinite(raw_sim_stamp)
            or raw_sim_stamp <= 0.0
        ):
            return None, None, timesync_stamp

        mapped_stamp = self.image_clock_mapper.to_ros_time(
            raw_sim_stamp,
            receipt,
        )

        return mapped_stamp, raw_sim_stamp, timesync_stamp

    def _queue_raw_pose_sample(
        self,
        kind,
        raw_sim_stamp,
        values,
    ):
        self._pending_raw_pose_samples.append((
            str(kind),
            float(raw_sim_stamp),
            tuple(float(value) for value in values),
        ))

    def _flush_pending_raw_pose_samples(self):
        """Retry poses once Gazebo clock interpolation brackets them."""
        if not self._pending_raw_pose_samples:
            return

        now = self._ros_seconds()
        remaining = deque(maxlen=self._pending_raw_pose_samples.maxlen)

        while self._pending_raw_pose_samples:
            kind, raw_sim_stamp, values = (
                self._pending_raw_pose_samples.popleft()
            )

            stamp = self.image_clock_mapper.to_ros_time(
                raw_sim_stamp,
                now,
            )

            if stamp is None:
                anchor_range = self.image_clock_mapper.anchor_range

                # Keep only samples that may still receive a future right
                # bracket. Samples older than the clock history are obsolete.
                if (
                    anchor_range is None
                    or raw_sim_stamp > anchor_range[1] + 1e-9
                ):
                    remaining.append((
                        kind,
                        raw_sim_stamp,
                        values,
                    ))
                continue

            if kind == 'position':
                if self.position_history.add(stamp, values):
                    self.uav_position = values
                    self.uav_position_time = stamp
                    self.position_mapped_stamp = stamp
                    self.position_raw_sim_stamp = raw_sim_stamp
            else:
                self._add_attitude_sample(stamp, values)
                self.attitude_mapped_stamp = stamp
                self.attitude_raw_sim_stamp = raw_sim_stamp

        self._pending_raw_pose_samples = remaining

    def position_callback(self, message):
        """Store UAV position using recovered physical PX4 sample time."""
        values = (message.x, message.y, message.z)

        if not all(math.isfinite(value) for value in values):
            return

        receipt = self._ros_seconds()
        source_timestamp = (
            getattr(message, 'timestamp_sample', 0)
            or getattr(message, 'timestamp', 0)
        )
        source_stamp = float(source_timestamp) * 1e-6
        self.position_source_stamp = source_stamp

        stamp, raw_sim_stamp, _timesync_stamp = (
            self._map_px4_pose_stamp(source_stamp, receipt)
        )

        if raw_sim_stamp is None:
            return

        self.position_raw_sim_stamp = raw_sim_stamp
        if getattr(self, 'navigation_state_pub', None) is not None:
            with self._navigation_lock:
                self._navigation_samples.append((
                    message, raw_sim_stamp, time.monotonic(),
                ))
            self._flush_navigation_states()
        position = tuple(float(value) for value in values)

        if stamp is None:
            self._queue_raw_pose_sample(
                'position',
                raw_sim_stamp,
                position,
            )
            return

        self.position_mapped_stamp = stamp

        if self.position_history.add(stamp, position):
            self.uav_position = position
            self.uav_position_time = stamp
        self._request_localization()

    def _flush_navigation_states(self):
        """Publish physical state only after a causal clock bracket exists."""
        if getattr(self, 'navigation_state_pub', None) is None:
            return
        with self._navigation_lock:
            remaining = deque(maxlen=self._navigation_samples.maxlen)
            now = self._ros_seconds()
            while self._navigation_samples:
                source, raw_sim_stamp, queued_at = (
                    self._navigation_samples.popleft()
                )
                stamp = self.image_clock_mapper.to_ros_time(raw_sim_stamp, now)
                if stamp is None:
                    if time.monotonic() - queued_at <= self.pose_wait_timeout:
                        remaining.append((source, raw_sim_stamp, queued_at))
                    continue
                if stamp <= self._navigation_last_stamp:
                    continue
                message = UavState()
                message.stamp = _seconds_to_time(stamp)
                message.frame_id = 'local_ned'
                for target, fields in (
                    (message.position, ('x', 'y', 'z')),
                    (message.velocity, ('vx', 'vy', 'vz')),
                    (message.acceleration, ('ax', 'ay', 'az')),
                ):
                    for axis, field in zip('xyz', fields):
                        setattr(target, axis, float(getattr(source, field)))
                message.heading = float(source.heading)
                message.valid = all(bool(getattr(source, field)) for field in (
                    'xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid',
                )) and all(math.isfinite(getattr(vector, axis))
                           for vector in (message.position, message.velocity,
                                          message.acceleration)
                           for axis in 'xyz')
                message.px4_timestamp_sample = int(source.timestamp_sample)
                message.native_timestamp_sample = round(raw_sim_stamp * 1e6)
                self.navigation_state_pub.publish(message)
                self._navigation_last_stamp = stamp
            self._navigation_samples = remaining

    def attitude_callback(self, message):
        """Store attitude using recovered physical PX4 sample time."""
        quaternion = tuple(float(value) for value in message.q)

        if not all(math.isfinite(value) for value in quaternion):
            return

        receipt = self._ros_seconds()
        source_timestamp = (
            getattr(message, 'timestamp_sample', 0)
            or getattr(message, 'timestamp', 0)
        )
        source_stamp = float(source_timestamp) * 1e-6
        self.attitude_source_stamp = source_stamp

        stamp, raw_sim_stamp, _timesync_stamp = (
            self._map_px4_pose_stamp(source_stamp, receipt)
        )

        if raw_sim_stamp is None:
            return

        self.attitude_raw_sim_stamp = raw_sim_stamp

        if stamp is None:
            self._queue_raw_pose_sample(
                'attitude',
                raw_sim_stamp,
                quaternion,
            )
            return

        self.attitude_mapped_stamp = stamp
        self._add_attitude_sample(stamp, quaternion)
        self._request_localization()

    def _pending_pose_samples_for_node(self):
        if not hasattr(self, '_pending_pose_samples'):
            self._pending_pose_samples = deque(maxlen=8)
        return self._pending_pose_samples

    def _add_attitude_sample(self, stamp, quaternion):
        previous = self.attitude_history.latest_value
        if (
            previous is not None
            and sum(left * right for left, right in zip(
                previous,
                quaternion,
            )) < 0.0
        ):
            quaternion = tuple(-value for value in quaternion)
        if self.attitude_history.add(stamp, quaternion):
            self.uav_attitude = quaternion
            self.uav_attitude_time = stamp

    def _reset_pose_time_state(self):
        if getattr(self, 'navigation_state_pub', None) is not None:
            with self._navigation_lock:
                self._navigation_samples.clear()
                self._navigation_last_stamp = -math.inf
                self.navigation_state_pub.publish(UavState())
        self._pending_pose_samples_for_node().clear()
        if hasattr(self, '_pending_raw_pose_samples'):
            self._pending_raw_pose_samples.clear()
        self.waiting_image_pair = None
        self.position_history.clear()
        self.attitude_history.clear()
        self.uav_position = None
        self.uav_attitude = None
        self.uav_position_time = -math.inf
        self.uav_attitude_time = -math.inf
        self.position_mapped_stamp = math.nan
        self.attitude_mapped_stamp = math.nan

    def _flush_pending_pose_samples(self):
        if self.px4_clock_mapper.offset is None:
            return
        pending = self._pending_pose_samples_for_node()
        while pending:
            kind, source_stamp, values = pending.popleft()
            stamp = source_stamp + self.px4_clock_mapper.offset
            if kind == 'position':
                if self.position_history.add(stamp, values):
                    self.uav_position = values
                    self.uav_position_time = stamp
                    self.position_mapped_stamp = stamp
            else:
                self._add_attitude_sample(stamp, values)
                self.attitude_mapped_stamp = stamp

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_invalid_observation(
        self,
        reason='INVALID_OBSERVATION',
        measurement_stamp=None,
        received_stamp=None,
        geometry=None,
    ):
        """Publish an explicit invalid observation without updating the KF."""
        message = TargetObservation()
        published = self._ros_seconds()
        message.stamp = _seconds_to_time(measurement_stamp)
        message.received_stamp = _seconds_to_time(received_stamp)
        message.processed_stamp = _seconds_to_time(published)
        message.published_stamp = _seconds_to_time(published)
        message.frame_id = self.frame_id
        message.position.x = math.nan
        message.position.y = math.nan
        message.position.z = math.nan
        message.covariance = [math.nan] * 9
        message.confidence = 0.0
        message.source = 'front_rgbd_red_sphere'
        message.rejection_reason = str(reason)
        self._fill_time_diagnostic(message)
        message.red_pixel_count = 0
        message.valid_depth_ratio = 0.0
        if geometry is not None:
            message.red_pixel_count = geometry.red_pixel_count
            message.valid_depth_count = geometry.valid_depth_count
            message.valid_depth_ratio = (
                geometry.valid_depth_count / geometry.red_pixel_count
            )
            message.depth_min = geometry.depth_min
            message.depth_median = geometry.depth_median
            message.depth_mad = geometry.depth_mad
            message.geometry_diagnostics_enabled = True
            message.mask_centroid_u, message.mask_centroid_v = (
                geometry.mask_center
            )
            (message.projection_centroid_u,
             message.projection_centroid_v) = geometry.projection_center
            (message.mask_bbox_left, message.mask_bbox_top,
             message.mask_bbox_right,
             message.mask_bbox_bottom) = geometry.mask_bbox
            (message.camera_fx, message.camera_fy,
             message.camera_cx, message.camera_cy) = geometry.intrinsics
            (message.surface_camera_x, message.surface_camera_y,
             message.surface_camera_z) = geometry.surface_camera
            (message.center_camera_x, message.center_camera_y,
             message.center_camera_z) = geometry.center_camera
            (message.camera_translation_x, message.camera_translation_y,
             message.camera_translation_z) = self.camera_translation_flu
            message.camera_pitch_down = self.camera_pitch_down
            message.target_reference_z_offset = (
                self.target_reference_z_offset
            )
        message.target_range = math.nan
        message.view_angle = math.nan
        message.valid = False
        self.observation_pub.publish(message)
        if (
            getattr(self, 'time_pair_diagnostics_enabled', False)
            and published - getattr(
                self, 'last_time_diagnostic_log', -math.inf
            )
            >= 1.0
        ):
            values = getattr(self, 'last_time_diagnostic', {})
            self.get_logger().warning(
                'RGB-D time reject | reason=%s | rgb_raw=%.6f | '
                'depth_raw=%.6f | rgb_receipt=%.6f | depth_receipt=%.6f'
                % (
                    reason,
                    values.get('rgb_raw_stamp', math.nan),
                    values.get('depth_raw_stamp', math.nan),
                    values.get('rgb_receipt_stamp', math.nan),
                    values.get('depth_receipt_stamp', math.nan),
                )
            )
            self.last_time_diagnostic_log = published

    def _fill_time_diagnostic(self, message):
        values = getattr(self, 'last_time_diagnostic', {})
        for name in (
            'rgb_raw_stamp', 'depth_raw_stamp',
            'rgb_receipt_stamp', 'depth_receipt_stamp',
            'rgb_mapped_stamp', 'depth_mapped_stamp',
            'image_measurement_stamp',
            'rgb_depth_acquisition_skew',
            'pose_history_start_stamp', 'pose_history_end_stamp',
            'position_source_stamp', 'position_mapped_stamp',
            'attitude_source_stamp', 'attitude_mapped_stamp',
            'position_history_start_stamp',
            'position_history_end_stamp',
            'attitude_history_start_stamp',
            'attitude_history_end_stamp',
            'image_clock_offset',
            'image_clock_anchor_sim_stamp',
            'image_clock_anchor_system_stamp',
            'image_clock_reference_age',
            'image_clock_sync_quality',
            'px4_clock_offset',
        ):
            setattr(message, name, float(values.get(name, math.nan)))
        message.image_clock_mapping_mode = str(values.get(
            'image_clock_mapping_mode', 'UNKNOWN'
        ))
        message.image_clock_status = str(values.get(
            'image_clock_status', 'UNKNOWN'
        ))
        message.image_clock_reset_count = int(values.get(
            'image_clock_reset_count', 0
        ))
        message.image_measurement_time_source = str(values.get(
            'image_measurement_time_source', 'RGB'
        ))
        message.px4_clock_reset_count = int(values.get(
            'px4_clock_reset_count', 0
        ))
        message.px4_clock_calibration_count = int(values.get(
            'px4_clock_calibration_count', 0
        ))
        message.px4_clock_recalibration_count = int(values.get(
            'px4_clock_recalibration_count', 0
        ))
        message.px4_clock_status = str(values.get(
            'px4_clock_status', 'UNKNOWN'
        ))

    def timed_localize(self):
        """Measure one rate-limited localization pass, including failures."""
        started = time.perf_counter()
        try:
            self.localize()
        finally:
            message = Float32()
            message.data = float(time.perf_counter() - started)
            self.compute_time_pub.publish(message)

    @staticmethod
    def _image_clock_rejection_reason(status):
        reasons = {
            'INVALID_IMAGE_TIMESTAMP': 'RAW_IMAGE_TIMESTAMP_INVALID',
            'INVALID_CLOCK_REFERENCE': 'IMAGE_CLOCK_REFERENCE_INVALID',
            'CLOCK_REFERENCE_DOMAIN_MISMATCH': (
                'IMAGE_CLOCK_REFERENCE_INVALID'
            ),
            'CLOCK_REFERENCE_UNAVAILABLE': (
                'IMAGE_CLOCK_REFERENCE_UNAVAILABLE'
            ),
            'CLOCK_REFERENCE_WARMING_UP': (
                'IMAGE_CLOCK_REFERENCE_UNAVAILABLE'
            ),
            'CLOCK_REFERENCE_STALE': 'IMAGE_CLOCK_REFERENCE_STALE',
            'SIM_TIME_RESET': 'IMAGE_CLOCK_RESET',
            'SYSTEM_CLOCK_RESET': 'IMAGE_CLOCK_RESET',
            'CLOCK_REFERENCE_RESET': 'IMAGE_CLOCK_RESET',
            'IMAGE_BEFORE_CLOCK_REFERENCE': (
                'IMAGE_CLOCK_REFERENCE_EXPIRED'
            ),
            'IMAGE_AFTER_CLOCK_REFERENCE': (
                'IMAGE_CLOCK_REFERENCE_NOT_YET_COVERING_IMAGE'
            ),
        }
        return reasons.get(status, 'IMAGE_CLOCK_MAPPING_FAILED')

    def localize(self):
        """Publish one synchronized RGB-D target observation when valid."""
        self._apply_pending_image_clock_reset()
        now = self._ros_seconds()
        waiting_pair = getattr(self, 'waiting_image_pair', None)
        if waiting_pair is None and not self.pending_image_pairs:
            if self.last_pair_rejection:
                self.publish_invalid_observation(
                    self.last_pair_rejection,
                    received_stamp=max(self.color_time, self.depth_time),
                )
                self.last_pair_rejection = ''
                return
            if due_data_timeout(
                now,
                max(self.color_time, self.depth_time),
                self.last_image_timeout_report,
                self.data_timeout,
            ):
                self.publish_invalid_observation(
                    'IMAGE_TIMEOUT',
                    received_stamp=now,
                )
                self.last_image_timeout_report = now
            return
        if waiting_pair is not None:
            color_frame, depth_frame = waiting_pair
        else:
            color_frame, depth_frame = self.pending_image_pairs.pop()
            self.pending_image_pairs.clear()
        color_message = color_frame.message
        depth_message = depth_frame.message
        raw_measurement_stamp = synchronized_measurement_time(
            color_frame.raw_stamp,
            depth_frame.raw_stamp,
            self.maximum_rgb_depth_skew,
        )
        received_stamp = max(
            color_frame.receipt_stamp, depth_frame.receipt_stamp
        )
        if raw_measurement_stamp is not None:
            rgb_mapped_stamp, rgb_clock_status = (
                self.image_clock_mapper.map_time(
                    color_frame.raw_stamp, now
                )
            )
            depth_mapped_stamp, depth_clock_status = (
                self.image_clock_mapper.map_time(
                    depth_frame.raw_stamp, now
                )
            )
        else:
            rgb_mapped_stamp = depth_mapped_stamp = None
            rgb_clock_status = depth_clock_status = (
                'INVALID_IMAGE_TIMESTAMP'
            )
        measurement_stamp = rgb_mapped_stamp
        image_offset = (
            measurement_stamp - color_frame.raw_stamp
            if measurement_stamp is not None else math.nan
        )
        position_range = self.position_history.time_range
        attitude_range = self.attitude_history.time_range
        pose_start = max(
            position_range[0] if position_range else -math.inf,
            attitude_range[0] if attitude_range else -math.inf,
        )
        pose_end = min(
            position_range[1] if position_range else math.inf,
            attitude_range[1] if attitude_range else math.inf,
        )
        self.last_time_diagnostic = {
            'rgb_raw_stamp': color_frame.raw_stamp,
            'depth_raw_stamp': depth_frame.raw_stamp,
            'rgb_receipt_stamp': color_frame.receipt_stamp,
            'depth_receipt_stamp': depth_frame.receipt_stamp,
            'rgb_mapped_stamp': (
                rgb_mapped_stamp
                if rgb_mapped_stamp is not None else math.nan
            ),
            'depth_mapped_stamp': (
                depth_mapped_stamp
                if depth_mapped_stamp is not None else math.nan
            ),
            'image_measurement_stamp': (
                measurement_stamp
                if measurement_stamp is not None else math.nan
            ),
            'rgb_depth_acquisition_skew': abs(
                color_frame.raw_stamp - depth_frame.raw_stamp
            ),
            'pose_history_start_stamp': pose_start,
            'pose_history_end_stamp': pose_end,
            'position_source_stamp': getattr(
                self, 'position_source_stamp', math.nan
            ),
            'position_mapped_stamp': getattr(
                self, 'position_mapped_stamp', math.nan
            ),
            'attitude_source_stamp': getattr(
                self, 'attitude_source_stamp', math.nan
            ),
            'attitude_mapped_stamp': getattr(
                self, 'attitude_mapped_stamp', math.nan
            ),
            'position_history_start_stamp': (
                position_range[0] if position_range else math.nan
            ),
            'position_history_end_stamp': (
                position_range[1] if position_range else math.nan
            ),
            'attitude_history_start_stamp': (
                attitude_range[0] if attitude_range else math.nan
            ),
            'attitude_history_end_stamp': (
                attitude_range[1] if attitude_range else math.nan
            ),
            'image_clock_offset': image_offset,
            'image_clock_mapping_mode': getattr(
                self.image_clock_mapper,
                'mapping_mode',
                'UNKNOWN',
            ),
            'image_clock_status': (
                rgb_clock_status
                if rgb_mapped_stamp is None else depth_clock_status
            ),
            'image_clock_reset_count': getattr(
                self.image_clock_mapper, 'reset_count', 0
            ),
            'image_clock_anchor_sim_stamp': getattr(
                self.image_clock_mapper,
                'last_anchor_sim_time',
                math.nan,
            ),
            'image_clock_anchor_system_stamp': getattr(
                self.image_clock_mapper,
                'last_anchor_system_time',
                math.nan,
            ),
            'image_clock_reference_age': getattr(
                self.image_clock_mapper,
                'last_reference_age',
                math.nan,
            ),
            'image_clock_sync_quality': getattr(
                self.image_clock_mapper,
                'last_quality',
                math.nan,
            ),
            'image_measurement_time_source': 'RGB',
            'px4_clock_offset': (
                self.px4_clock_mapper.offset
                if self.px4_clock_mapper.offset is not None else math.nan
            ),
            'px4_clock_reset_count': self.px4_clock_mapper.reset_count,
            'px4_clock_calibration_count': (
                self.px4_clock_mapper.calibration_count
            ),
            'px4_clock_recalibration_count': (
                self.px4_clock_mapper.offset_recalibration_count
            ),
            'px4_clock_status': self.px4_clock_mapper.last_status,
        }
        if raw_measurement_stamp is None:
            self.waiting_image_pair = None
            self.publish_invalid_observation(
                'RGB_DEPTH_TIME_MISMATCH',
                received_stamp=received_stamp,
            )
            return
        if rgb_mapped_stamp is None or depth_mapped_stamp is None:
            status = (
                rgb_clock_status
                if rgb_mapped_stamp is None else depth_clock_status
            )
            waiting_for_reference = status in (
                'CLOCK_REFERENCE_UNAVAILABLE',
                'CLOCK_REFERENCE_WARMING_UP',
                'IMAGE_AFTER_CLOCK_REFERENCE',
            )
            if (
                waiting_for_reference
                and now - received_stamp
                <= self.image_clock_wait_timeout + 1e-9
            ):
                self.waiting_image_pair = (color_frame, depth_frame)
                return
            self.waiting_image_pair = None
            self.publish_invalid_observation(
                self._image_clock_rejection_reason(status),
                received_stamp=received_stamp,
            )
            return
        measurement_age = now - measurement_stamp
        if measurement_age < -1e-9:
            self.waiting_image_pair = None
            self.publish_invalid_observation(
                'IMAGE_TIMESTAMP_IN_FUTURE',
                measurement_stamp,
                received_stamp,
            )
            return
        if (
            measurement_age > self.data_timeout + 1e-9
            or now - received_stamp > self.data_timeout + 1e-9
        ):
            self.waiting_image_pair = None
            self.publish_invalid_observation(
                'IMAGE_TIMESTAMP_STALE',
                measurement_stamp,
                received_stamp,
            )
            return
        pose_rejection = pose_history_rejection_reason(
            measurement_stamp,
            position_range,
            attitude_range,
        )
        if pose_rejection:
            missing_future_gap = max(
                measurement_stamp - position_range[1]
                if position_range else math.inf,
                measurement_stamp - attitude_range[1]
                if attitude_range else math.inf,
            )
            if (
                pose_rejection in (
                    'POSITION_TIMESTAMP_AFTER_HISTORY',
                    'ATTITUDE_TIMESTAMP_AFTER_HISTORY',
                )
                and now - received_stamp
                <= getattr(self, 'pose_wait_timeout', 0.0) + 1e-9
                and missing_future_gap
                <= getattr(self, 'maximum_pose_wait_gap', 0.0) + 1e-9
            ):
                self.waiting_image_pair = (color_frame, depth_frame)
                return
            self.waiting_image_pair = None
            self.publish_invalid_observation(
                pose_rejection,
                measurement_stamp,
                received_stamp,
            )
            return
        self.waiting_image_pair = None
        self.color_mask = red_pixel_mask(
            color_message.data,
            color_message.width,
            color_message.height,
            color_message.step,
            color_message.encoding.lower(),
        )
        if depth_message.encoding.lower() not in ('32fc1', '32fc'):
            self.depth = None
        else:
            self.depth = decode_float32_depth(
                depth_message.data,
                depth_message.width,
                depth_message.height,
                depth_message.step,
            )
        uav_position = self.position_history.value_at(measurement_stamp)
        uav_attitude = self.attitude_history.value_at(measurement_stamp)
        if uav_attitude is not None:
            norm = math.sqrt(sum(value * value for value in uav_attitude))
            uav_attitude = (
                tuple(value / norm for value in uav_attitude)
                if norm > 1e-9 else None
            )
        state_fresh = uav_position is not None and uav_attitude is not None
        images_fresh = (
            self.color_mask is not None
            and self.depth is not None
            and now - color_frame.receipt_stamp <= self.data_timeout
            and now - depth_frame.receipt_stamp <= self.data_timeout
            and now - measurement_stamp <= self.data_timeout
        )
        red_pixels = (
            0
            if self.color_mask is None
            else int(np.count_nonzero(self.color_mask))
        )
        if not state_fresh or not images_fresh or (
            red_pixels < self.minimum_red_pixels
        ):
            self.publish_invalid_observation(
                'POSE_TIME_GAP_TOO_LARGE'
                if not state_fresh else 'IMAGE_INVALID',
                measurement_stamp,
                received_stamp,
            )
            return

        valid_depth = (
            self.color_mask
            & np.isfinite(self.depth)
            & (self.depth >= self.minimum_depth)
            & (self.depth <= self.maximum_depth)
        )
        depth_ratio = float(np.count_nonzero(valid_depth)) / red_pixels
        if depth_ratio < self.minimum_depth_ratio:
            self.publish_invalid_observation(
                'DEPTH_RATIO_LOW', measurement_stamp, received_stamp
            )
            return
        geometry = target_geometry_from_rgbd(
            self.color_mask,
            self.depth,
            self.horizontal_fov,
            self.minimum_depth,
            self.maximum_depth,
            self.target_radius,
            require_valid_sphere=True,
            sphere_fit_mode=getattr(self, 'sphere_fit_mode', 'ideal'),
            sphere_noise_std=getattr(self, 'sphere_noise_std', .01),
            sphere_range_noise_scale=getattr(self, 'sphere_range_noise_scale', .001),
        )
        if geometry is None:
            self.publish_invalid_observation(
                'TARGET_VECTOR_INVALID', measurement_stamp, received_stamp
            )
            return
        # For one sphere, camera-X depths lie within center X +/- radius,
        # so their median absolute deviation cannot exceed the radius.
        # Reject broad depth mixtures before publishing a pose.
        if geometry.depth_mad > self.maximum_depth_mad:
            self.publish_invalid_observation(
                'DEPTH_MAD_HIGH', measurement_stamp, received_stamp,
                geometry=geometry,
            )
            return
        camera_vector = np.asarray(geometry.center_camera, dtype=float)
        try:
            body_flu = camera_target_to_body_flu(
                camera_vector,
                self.camera_translation_flu,
                self.camera_pitch_down,
            )
            target_position = camera_target_to_local_ned(
                camera_vector,
                uav_position,
                uav_attitude,
                self.camera_translation_flu,
                self.camera_pitch_down,
                self.target_reference_z_offset,
            )
        except ValueError:
            self.publish_invalid_observation(
                'TRANSFORM_INVALID', measurement_stamp, received_stamp
            )
            return

        target_range = float(np.linalg.norm(camera_vector))
        position_std = (
            self.base_position_std
            + self.range_position_std_scale * target_range
        )
        position_std = max(position_std, geometry.fit_position_std)
        variance = position_std * position_std
        covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        confidence = min(
            depth_ratio * red_pixels / max(self.minimum_red_pixels, 20),
            1.0,
        )
        observation = TargetObservation()
        processed_stamp = self._ros_seconds()
        observation.stamp = _seconds_to_time(measurement_stamp)
        observation.received_stamp = _seconds_to_time(received_stamp)
        observation.processed_stamp = _seconds_to_time(processed_stamp)
        observation.published_stamp = _seconds_to_time(
            self._ros_seconds()
        )
        observation.frame_id = self.frame_id
        observation.position.x = float(target_position[0])
        observation.position.y = float(target_position[1])
        observation.position.z = float(target_position[2])
        observation.covariance = covariance
        observation.confidence = float(confidence)
        observation.source = 'front_rgbd_red_sphere'
        observation.rejection_reason = ''
        self._fill_time_diagnostic(observation)
        observation.red_pixel_count = red_pixels
        observation.valid_depth_ratio = float(depth_ratio)
        observation.target_range = float(target_range)
        observation.view_angle = float(math.atan2(
            math.hypot(camera_vector[1], camera_vector[2]),
            max(camera_vector[0], 1e-9),
        ))
        # The validated camera ray is a perception output used by visual yaw,
        # independent of optional verbose pose/geometry audit fields.
        (observation.center_camera_x, observation.center_camera_y,
         observation.center_camera_z) = geometry.center_camera
        if getattr(self, 'geometry_diagnostics_enabled', False):
            observation.geometry_diagnostics_enabled = True
            observation.mask_centroid_u = geometry.mask_center[0]
            observation.mask_centroid_v = geometry.mask_center[1]
            observation.projection_centroid_u = geometry.projection_center[0]
            observation.projection_centroid_v = geometry.projection_center[1]
            (
                observation.mask_bbox_left,
                observation.mask_bbox_top,
                observation.mask_bbox_right,
                observation.mask_bbox_bottom,
            ) = geometry.mask_bbox
            observation.valid_depth_count = geometry.valid_depth_count
            observation.depth_min = geometry.depth_min
            observation.depth_median = geometry.depth_median
            observation.depth_mad = geometry.depth_mad
            (
                observation.camera_fx,
                observation.camera_fy,
                observation.camera_cx,
                observation.camera_cy,
            ) = geometry.intrinsics
            (
                observation.camera_translation_x,
                observation.camera_translation_y,
                observation.camera_translation_z,
            ) = self.camera_translation_flu
            observation.camera_pitch_down = self.camera_pitch_down
            (
                observation.surface_camera_x,
                observation.surface_camera_y,
                observation.surface_camera_z,
            ) = geometry.surface_camera
            (
                observation.target_body_flu_x,
                observation.target_body_flu_y,
                observation.target_body_flu_z,
            ) = body_flu
            (
                observation.interpolated_uav_x,
                observation.interpolated_uav_y,
                observation.interpolated_uav_z,
            ) = uav_position
            (
                observation.interpolated_attitude_w,
                observation.interpolated_attitude_x,
                observation.interpolated_attitude_y,
                observation.interpolated_attitude_z,
            ) = uav_attitude
            observation.target_reference_z_offset = (
                self.target_reference_z_offset
            )
        observation.valid = True
        self.observation_pub.publish(observation)

        point = Point()
        point.x = observation.position.x
        point.y = observation.position.y
        point.z = observation.position.z
        self.target_position_pub.publish(point)


def main(args=None):
    """Run the RGB-D target localizer node."""
    rclpy.init(args=args)
    node = RgbdTargetLocalizer()
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
