"""
Pair navigation and complete PX4 attitude for shadow MPC diagnostics.

This adapter owns no ROS node or publisher. Navigation already carries the
physical ROS/system acquisition epoch from the localizer. Attitude recovers
native sample time with a causal TimesyncStatus sample, then uses the same
trusted Gazebo dual-clock interpolation as that localizer. Receipt time only
limits freshness and waiting; it never supplies an acquisition timestamp.
"""

import math
import threading
from uav_control.guidance.follow_profile import profiled

from collections import deque
from dataclasses import dataclass
from typing import Optional

from uav_control.perception.rgbd_target_localizer import (
    GazeboImageClockMapper,
    TimestampedVectorHistory,
    quaternion_slerp,
)


@dataclass(frozen=True)
class ShadowPoseInput:
    """PVA and full FRD-to-NED attitude at one navigation sample epoch."""

    stamp: float
    position: tuple
    velocity: tuple
    acceleration: tuple
    quaternion_wxyz: tuple
    native_timestamp_sample: int
    px4_timestamp_sample: int
    navigation_receipt_stamp: float
    attitude_left_stamp: float
    attitude_right_stamp: float
    attitude_left_native_timestamp_sample: int
    attitude_right_native_timestamp_sample: int
    attitude_left_px4_timestamp_sample: int
    attitude_right_px4_timestamp_sample: int
    attitude_left_timesync_stamp: float
    attitude_right_timesync_stamp: float
    attitude_quat_reset_counter: int
    clock_generation: int
    observation_stamp: Optional[float] = None


@dataclass(frozen=True)
class PairResult:
    """Explicit pairing or rejection, without an inferred fallback epoch."""

    status: str
    snapshot: Optional[ShadowPoseInput] = None


@dataclass
class _Navigation:
    stamp: float
    position: tuple
    velocity: tuple
    acceleration: tuple
    native_timestamp_sample: int
    px4_timestamp_sample: int
    receipt_stamp: float
    queued_at: float
    ready: bool = False
    timed_out: bool = False


@dataclass
class _Attitude:
    px4_timestamp_sample: int
    quaternion: tuple
    reset_counter: int
    receipt_stamp: float
    queued_at: float
    raw_stamp: Optional[float] = None
    timesync_stamp: Optional[float] = None
    stamp: Optional[float] = None


class ShadowInputAdapter:
    """
    Bounded causal input history; call methods from subscription callbacks.

    Pass ROS/system seconds and monotonic seconds explicitly. All four input
    methods and pair_latest are serialized, including Gazebo transport clock
    callbacks. A reset invalidates every cached pair and increments
    clock_generation, which consumers must include in warm-start identity.
    """

    _WAITING = frozenset((
        'CLOCK_REFERENCE_UNAVAILABLE', 'CLOCK_REFERENCE_WARMING_UP',
        'CLOCK_BRACKET_AFTER', 'TIMESYNC_UNAVAILABLE',
        'ATTITUDE_UNAVAILABLE', 'ATTITUDE_AFTER_HISTORY',
        'ATTITUDE_WAIT_TIMEOUT',
    ))

    def __init__(
        self,
        maximum_sample_age=.125,
        maximum_receipt_age=.125,
        maximum_observation_age=.125,
        maximum_attitude_bracket_gap=.125,
        pose_wait_timeout=.08,
        history_duration=1.,
        maximum_clock_reference_age=.5,
        maximum_system_clock_step=.25,
        timestamp_tolerance=2e-6,
        capacity=512,
    ):
        """Configure bounded histories, freshness and causal wait deadlines."""
        limits = (maximum_sample_age, maximum_receipt_age,
                  maximum_observation_age, maximum_attitude_bracket_gap,
                  pose_wait_timeout, history_duration,
                  maximum_clock_reference_age, maximum_system_clock_step,
                  timestamp_tolerance)
        if not all(math.isfinite(float(v)) and float(v) > 0 for v in limits):
            raise ValueError(
                'history, freshness and wait limits must be positive')
        self.maximum_sample_age = float(maximum_sample_age)
        self.maximum_receipt_age = float(maximum_receipt_age)
        self.maximum_observation_age = float(maximum_observation_age)
        self.maximum_attitude_bracket_gap = float(maximum_attitude_bracket_gap)
        self.pose_wait_timeout = float(pose_wait_timeout)
        self.history_duration = float(history_duration)
        self.timestamp_tolerance = float(timestamp_tolerance)
        self._capacity = max(int(capacity), 4)
        self.clock_mapper = GazeboImageClockMapper(
            maximum_reference_age=maximum_clock_reference_age,
            history_duration=history_duration,
            maximum_system_clock_step=maximum_system_clock_step,
        )
        self.attitude_history = TimestampedVectorHistory(
            maximum_age=history_duration, interpolator=quaternion_slerp,
        )
        self._navigation = deque(maxlen=self._capacity)
        self._attitudes = deque(maxlen=self._capacity)
        self._pending_attitudes = deque(maxlen=self._capacity)
        self._timesync = deque(maxlen=self._capacity)
        self._lock = threading.RLock()
        self.clock_generation = 0
        self.last_status = 'NAVIGATION_UNAVAILABLE'
        self._navigation_empty_status = 'NAVIGATION_UNAVAILABLE'
        self._attitude_empty_status = 'ATTITUDE_UNAVAILABLE'
        self._reset_status = ''
        self._reset_counter = None
        self._last_now = None

    @staticmethod
    def _valid_receipt(receipt_stamp, monotonic_stamp):
        return (math.isfinite(float(receipt_stamp)) and receipt_stamp > 0
                and math.isfinite(float(monotonic_stamp)))

    def _clear_pose(self, status):
        self._navigation.clear()
        self._attitudes.clear()
        self._pending_attitudes.clear()
        self.attitude_history.clear()
        self._navigation_empty_status = status
        self._attitude_empty_status = status

    def _reset(self, status, clear_timesync=False):
        self.clock_generation += 1
        self._reset_status = str(status)
        self._clear_pose(self._reset_status)
        if clear_timesync:
            self._timesync.clear()
        self.last_status = self._reset_status

    def add_clock_anchor(
        self, sim_time, system_time, receipt_ros_time, receipt_monotonic,
    ):
        """Accept only a trusted same-message Gazebo sim/system clock pair."""
        with self._lock:
            previous_reset = self.clock_mapper.reset_count
            accepted = self.clock_mapper.add_anchor(
                sim_time, system_time, receipt_ros_time, receipt_monotonic,
            )
            if self.clock_mapper.reset_count != previous_reset:
                self._reset(self.clock_mapper.last_status, clear_timesync=True)
            self._refresh(float(receipt_ros_time), float(receipt_monotonic))
            self.last_status = self.clock_mapper.last_status
            return accepted

    def add_timesync(self, message):
        """Keep source-epoch offset history, never a receipt-derived offset."""
        with self._lock:
            try:
                stamp = int(message.timestamp) * 1e-6
                offset = int(message.estimated_offset) * 1e-6
            except (AttributeError, ValueError, TypeError, OverflowError):
                self.last_status = 'INVALID_TIMESYNC'
                return False
            if stamp <= 0 or not all(map(math.isfinite, (stamp, offset))):
                self.last_status = 'INVALID_TIMESYNC'
                return False
            if self._timesync and stamp < self._timesync[-1][0] - 1e-6:
                self._reset('TIMESYNC_TIME_RESET', clear_timesync=True)
            if self._timesync and abs(stamp - self._timesync[-1][0]) <= 1e-9:
                self._timesync[-1] = stamp, offset
            else:
                self._timesync.append((stamp, offset))
            self.last_status = 'TIMESYNC_READY'
            return True

    def _causal_timesync(self, source_stamp):
        for stamp, offset in reversed(self._timesync):
            if stamp <= source_stamp + 1e-9:
                return stamp, offset
        return None, None

    def add_navigation(self, message, receipt_ros_time, receipt_monotonic):
        """Copy physical UavState PVA and both original sample timestamps."""
        with self._lock:
            status = 'INVALID_NAVIGATION'
            try:
                stamp = message.stamp.sec + message.stamp.nanosec * 1e-9
                native = int(message.native_timestamp_sample)
                source = int(message.px4_timestamp_sample)
                vectors = tuple(tuple(float(getattr(vector, axis))
                                      for axis in 'xyz') for vector in (
                                          message.position, message.velocity,
                                          message.acceleration))
                valid = bool(message.valid) and message.frame_id == 'local_ned'
                valid = valid and all(math.isfinite(v)
                                      for vector in vectors for v in vector)
                valid = valid and self._valid_receipt(
                    receipt_ros_time, receipt_monotonic)
                if valid:
                    if (not math.isfinite(stamp)
                            or min(stamp, native, source) <= 0):
                        status = 'INVALID_NAVIGATION_TIMESTAMP'
                    elif stamp > receipt_ros_time + 1e-9:
                        status = 'NAVIGATION_FUTURE'
                    else:
                        status = ''
            except (AttributeError, ValueError, TypeError, OverflowError):
                pass
            if status:
                self._navigation.clear()
                self._navigation_empty_status = status
                self.last_status = status
                return False
            if self._navigation and native < (
                self._navigation[-1].native_timestamp_sample
            ):
                self.last_status = 'NAVIGATION_OUT_OF_ORDER'
                return False
            record = _Navigation(
                float(stamp), *vectors, native, source,
                float(receipt_ros_time), float(receipt_monotonic),
            )
            if self._navigation and native == (
                self._navigation[-1].native_timestamp_sample
            ):
                self._navigation[-1] = record
            else:
                self._navigation.append(record)
            cutoff = stamp - self.history_duration
            while (len(self._navigation) > 1
                   and self._navigation[0].stamp < cutoff):
                self._navigation.popleft()
            self._navigation_empty_status = 'NAVIGATION_UNAVAILABLE'
            self._refresh(float(receipt_ros_time), float(receipt_monotonic))
            self.last_status = 'NAVIGATION_ACCEPTED'
            return True

    def add_attitude(self, message, receipt_ros_time, receipt_monotonic):
        """Queue complete wxyz attitude until a causal clock bracket exists."""
        with self._lock:
            status = 'INVALID_ATTITUDE'
            try:
                source = int(message.timestamp_sample)
                quaternion = tuple(float(v) for v in message.q)
                counter = int(message.quat_reset_counter)
                valid = (len(quaternion) == 4
                         and all(map(math.isfinite, quaternion))
                         and self._valid_receipt(receipt_ros_time,
                                                 receipt_monotonic))
                norm = math.sqrt(sum(v * v for v in quaternion))
                valid = valid and math.isfinite(norm) and norm > 1e-9
                if valid:
                    status = '' if source > 0 else 'INVALID_ATTITUDE_TIMESTAMP'
            except (AttributeError, ValueError, TypeError, OverflowError):
                pass
            if status:
                self._attitudes.clear()
                self._pending_attitudes.clear()
                self.attitude_history.clear()
                self._attitude_empty_status = status
                for record in self._navigation:
                    record.ready = False
                self.last_status = status
                return False
            if (self._reset_counter is not None
                    and counter != self._reset_counter):
                self._reset('ATTITUDE_RESET')
            self._reset_counter = counter
            record = _Attitude(
                source, tuple(v / norm for v in quaternion), counter,
                float(receipt_ros_time), float(receipt_monotonic),
            )
            latest_source = None
            if self._pending_attitudes:
                latest_source = (
                    self._pending_attitudes[-1].px4_timestamp_sample)
            elif self._attitudes:
                latest_source = self._attitudes[-1].px4_timestamp_sample
            if latest_source is not None and source < latest_source:
                self.last_status = 'ATTITUDE_OUT_OF_ORDER'
                return False
            self._pending_attitudes.append(record)
            self._refresh(float(receipt_ros_time), float(receipt_monotonic))
            self.last_status = (
                self._attitude_empty_status if not self._attitudes
                else 'ATTITUDE_ACCEPTED')
            return self.last_status not in (
                'INVALID_ATTITUDE_NATIVE_TIMESTAMP', 'ATTITUDE_FUTURE',
            )

    @staticmethod
    def _clock_status(status):
        return {
            'IMAGE_BEFORE_CLOCK_REFERENCE': 'CLOCK_BRACKET_BEFORE',
            'IMAGE_AFTER_CLOCK_REFERENCE': 'CLOCK_BRACKET_AFTER',
            'INVALID_IMAGE_TIMESTAMP': 'INVALID_NATIVE_TIMESTAMP',
        }.get(status, status)

    def _flush_attitudes(self, now, monotonic_now):
        remaining = deque(maxlen=self._capacity)
        while self._pending_attitudes:
            record = self._pending_attitudes.popleft()
            if (monotonic_now - record.queued_at
                    > self.pose_wait_timeout + 1e-9):
                self._attitude_empty_status = 'ATTITUDE_WAIT_TIMEOUT'
                continue
            if record.raw_stamp is None:
                sync_stamp, offset = self._causal_timesync(
                    record.px4_timestamp_sample * 1e-6)
                if offset is None:
                    self._attitude_empty_status = 'TIMESYNC_UNAVAILABLE'
                    remaining.append(record)
                    continue
                record.raw_stamp = record.px4_timestamp_sample * 1e-6 + offset
                record.timesync_stamp = sync_stamp
            if not math.isfinite(record.raw_stamp) or record.raw_stamp <= 0:
                self._attitude_empty_status = (
                    'INVALID_ATTITUDE_NATIVE_TIMESTAMP')
                continue
            stamp, clock_status = self.clock_mapper.map_time(
                record.raw_stamp, now)
            if stamp is None:
                self._attitude_empty_status = self._clock_status(clock_status)
                if clock_status != 'IMAGE_BEFORE_CLOCK_REFERENCE':
                    remaining.append(record)
                continue
            if stamp > now + 1e-9:
                self._attitude_empty_status = 'ATTITUDE_FUTURE'
                continue
            record.stamp = stamp
            previous = self.attitude_history.latest_value
            if previous is not None and sum(a * b for a, b in zip(
                previous, record.quaternion,
            )) < 0:
                record.quaternion = tuple(-v for v in record.quaternion)
            if not self.attitude_history.add(stamp, record.quaternion):
                self._attitude_empty_status = 'ATTITUDE_OUT_OF_ORDER'
                continue
            if (self._attitudes
                    and abs(stamp - self._attitudes[-1].stamp) <= 1e-9):
                self._attitudes[-1] = record
            else:
                self._attitudes.append(record)
            cutoff = stamp - self.history_duration
            while (len(self._attitudes) > 2
                   and self._attitudes[1].stamp < cutoff):
                self._attitudes.popleft()
            self._attitude_empty_status = 'ATTITUDE_UNAVAILABLE'
        self._pending_attitudes = remaining

    def _attitude_bracket(self, stamp):
        if not self._attitudes:
            return None, None, self._attitude_empty_status
        if stamp < self._attitudes[0].stamp - 1e-9:
            return None, None, 'ATTITUDE_BEFORE_HISTORY'
        if stamp > self._attitudes[-1].stamp + 1e-9:
            return None, None, 'ATTITUDE_AFTER_HISTORY'
        left = self._attitudes[0]
        for right in self._attitudes:
            if abs(stamp - right.stamp) <= 1e-9:
                return right, right, ''
            if right.stamp > stamp:
                if (right.stamp - left.stamp
                        > self.maximum_attitude_bracket_gap):
                    return None, None, 'ATTITUDE_BRACKET_GAP'
                return left, right, ''
            left = right
        return None, None, 'ATTITUDE_AFTER_HISTORY'

    def _pair_navigation(self, navigation, now, observation_stamp):
        if navigation.timed_out:
            return PairResult('POSE_WAIT_TIMEOUT')
        sample_age = now - navigation.stamp
        receipt_age = now - navigation.receipt_stamp
        if sample_age < -1e-9 or receipt_age < -1e-9:
            return PairResult('NAVIGATION_FUTURE')
        if sample_age > self.maximum_sample_age + 1e-9:
            return PairResult('NAVIGATION_SAMPLE_STALE')
        if receipt_age > self.maximum_receipt_age + 1e-9:
            return PairResult('NAVIGATION_RECEIPT_STALE')
        mapped, status = self.clock_mapper.map_time(
            navigation.native_timestamp_sample * 1e-6, now,
        )
        if mapped is None:
            return PairResult(self._clock_status(status))
        if abs(mapped - navigation.stamp) > self.timestamp_tolerance:
            return PairResult('NAVIGATION_CLOCK_MISMATCH')
        left, right, status = self._attitude_bracket(navigation.stamp)
        if status:
            return PairResult(status)
        if right.stamp > now + 1e-9:
            return PairResult('ATTITUDE_FUTURE')
        quaternion = self.attitude_history.value_at(navigation.stamp)
        if quaternion is None:
            return PairResult('ATTITUDE_BRACKET_UNAVAILABLE')
        return PairResult('PAIRED', ShadowPoseInput(
            stamp=navigation.stamp, position=navigation.position,
            velocity=navigation.velocity, acceleration=navigation.acceleration,
            quaternion_wxyz=quaternion,
            native_timestamp_sample=navigation.native_timestamp_sample,
            px4_timestamp_sample=navigation.px4_timestamp_sample,
            navigation_receipt_stamp=navigation.receipt_stamp,
            attitude_left_stamp=left.stamp, attitude_right_stamp=right.stamp,
            attitude_left_native_timestamp_sample=round(left.raw_stamp * 1e6),
            attitude_right_native_timestamp_sample=round(
                right.raw_stamp * 1e6),
            attitude_left_px4_timestamp_sample=left.px4_timestamp_sample,
            attitude_right_px4_timestamp_sample=right.px4_timestamp_sample,
            attitude_left_timesync_stamp=left.timesync_stamp,
            attitude_right_timesync_stamp=right.timesync_stamp,
            attitude_quat_reset_counter=left.reset_counter,
            clock_generation=self.clock_generation,
            observation_stamp=observation_stamp,
        ))

    def _refresh(self, now, monotonic_now):
        if not self._valid_receipt(now, monotonic_now):
            return
        self._flush_attitudes(now, monotonic_now)
        for navigation in self._navigation:
            result = self._pair_navigation(navigation, now, None)
            deadline_expired = (monotonic_now - navigation.queued_at
                                > self.pose_wait_timeout + 1e-9)
            if (not navigation.ready and deadline_expired
                    and (result.snapshot is not None
                         or result.status in self._WAITING)):
                navigation.timed_out = True
            elif result.snapshot is not None:
                navigation.ready = True

    @profiled('navigation_sync')
    def pair_latest(self, now_ros_time, now_monotonic, observation_stamp=None):
        """
        Select the newest complete historical navigation acquisition epoch.

        A newer unbracketed navigation sample may wait while an older complete
        fresh sample is returned. Each waiting sample has its own bounded
        deadline. Observations retain their original epoch and are checked
        independently; they do not move the navigation/attitude pairing epoch.
        """
        with self._lock:
            now, monotonic_now = float(now_ros_time), float(now_monotonic)
            if not self._valid_receipt(now, monotonic_now):
                result = PairResult('INVALID_RECEIPT_TIMESTAMP')
            else:
                if self._last_now is not None and (
                    now < self._last_now[0] - 1e-9
                    or monotonic_now < self._last_now[1] - 1e-9
                ):
                    self.clock_mapper.reset('ROS_TIME_RESET')
                    self._reset('ROS_TIME_RESET', clear_timesync=True)
                self._last_now = now, monotonic_now
                result = self._pair_latest_locked(now, monotonic_now,
                                                  observation_stamp)
            self.last_status = result.status
            return result

    def _pair_latest_locked(self, now, monotonic_now, observation_stamp):
        if observation_stamp is not None:
            observation_stamp = float(observation_stamp)
            if not math.isfinite(observation_stamp) or observation_stamp <= 0:
                return PairResult('INVALID_OBSERVATION_TIMESTAMP')
            age = now - observation_stamp
            if age < -1e-9:
                return PairResult('OBSERVATION_FUTURE')
            if age > self.maximum_observation_age + 1e-9:
                return PairResult('OBSERVATION_STALE')
        self._refresh(now, monotonic_now)
        if not self._navigation:
            return PairResult(self._navigation_empty_status)
        latest_failure = None
        for navigation in reversed(self._navigation):
            result = self._pair_navigation(navigation, now, observation_stamp)
            if result.snapshot is not None:
                self._reset_status = ''
                return result
            if latest_failure is None:
                latest_failure = result
        return latest_failure
