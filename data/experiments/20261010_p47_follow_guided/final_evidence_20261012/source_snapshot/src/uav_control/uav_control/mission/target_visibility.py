# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""Truth-independent image bearing lock and finite yaw search core."""

from collections import deque
from dataclasses import dataclass
from dataclasses import fields
import math
from typing import Optional


@dataclass(frozen=True)
class VisibilityConfig:
    """Acquisition-time gates and conservative yaw search limits."""

    target_search_enable_height: float = 1.5
    initial_search_yaw_rate: float = 0.25
    reacquire_yaw_rate: float = 0.35
    maximum_search_yaw_rate: float = 0.6
    target_lock_min_frames: int = 3
    target_loss_frames: int = 3
    target_lock_max_age: float = 0.15
    target_lock_max_bearing: float = 0.15
    target_center_deadband_rad: float = 0.03
    vision_yaw_gain: float = 1.0
    target_reacquire_timeout: float = 2.0
    search_initial_arc: float = 0.52
    search_arc_increment: float = 0.52
    terminal_loss_recovery_timeout: float = 0.3

    def validate(self):
        """Reject nonfinite, nonpositive or inconsistent search parameters."""
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{field.name} must be finite and positive')
        for name in ('target_lock_min_frames', 'target_loss_frames'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f'{name} must be an integer')
        if self.target_center_deadband_rad > self.target_lock_max_bearing:
            raise ValueError('deadband must not exceed lock bearing limit')


@dataclass(frozen=True)
class VisibilityDecision:
    """One yaw owner's output; XYZ safety and hold remain with the caller."""

    state: str
    locked: bool
    visible: bool
    yaw_rate: float
    search_direction: int


class TargetVisibilityState:
    """
    Count acquisition frames, servo image bearing, and scan without truth.

    Positive image bearing denotes image right and positive NED yaw. The
    caller must independently preempt trajectory descent on stale KF or
    prediction sources. ``locked`` includes only brief visual dropouts while
    KF remains fresh; it is not permission to continue an expired trajectory.
    """

    def __init__(self, config=VisibilityConfig()):
        self.config = config
        config.validate()
        self.reset()

    def reset(self):
        """Forget locks, acquisition history and scan progress for a new mission."""
        self.last_valid_observation_stamp: Optional[float] = None
        self.last_valid_bearing_stamp: Optional[float] = None
        self.last_valid_image_bearing = 0.0
        self.last_valid_bearing_sign = 0
        self.last_lock_stamp: Optional[float] = None
        self.consecutive_valid_frames = 0
        self.consecutive_lost_frames = 0
        self.has_ever_seen_target = False
        self.has_ever_locked_target = False
        self._last_now = None
        self._last_bearing_stamp = None
        self._last_position_stamp = None
        self._latest_bearing_valid = False
        self._centered_stamps = deque(maxlen=self.config.target_lock_min_frames)
        self._direction_stamps = deque(maxlen=self.config.target_lock_min_frames)
        self._locked = False
        self._loss_started_at = None
        self._recovery_latched = False
        self._search_started_at = None
        self._clear_scan()

    def _clear_scan(self):
        self._scan_last_yaw = None
        self._scan_offset = 0.0
        self._scan_elapsed = 0.0
        self._scan_endpoints = []
        self._scan_index = 0
        self._scan_budget = 0.0
        self._scan_finished = False

    def _clock(self, now):
        if not math.isfinite(now):
            return False
        if self._last_now is not None and now < self._last_now:
            self.reset()
        self._last_now = now
        return True

    def _fresh(self, stamp, now):
        return stamp is not None and 0.0 <= now - stamp <= self.config.target_lock_max_age

    def mark_observation(self, stamp, valid, now=None):
        """
        Record a separate 3D acquisition stamp without adding bearing frames.

        Provide ``now`` on delayed callbacks. If omitted, the latest core
        clock is used (or stamp before any other input). Return whether the
        monotonic position callback was accepted, not whether it was valid.
        """
        if now is None:
            now = stamp if self._last_now is None else self._last_now
        if not self._clock(now) or not math.isfinite(stamp) or stamp <= 0.0 or stamp > now:
            return False
        if self._last_position_stamp is not None and stamp <= self._last_position_stamp:
            return False
        self._last_position_stamp = stamp
        if valid and self._fresh(stamp, now):
            self.last_valid_observation_stamp = stamp
        return True

    def observe(self, stamp, bearing, valid, now, position_valid=False):
        """
        Accept one distinct image stamp; reject duplicate or reordered frames.

        RGB validity is independent of 3D depth validity. Centered fresh
        bearing frames count toward lock; invalid frames count toward loss.
        """
        if not self._clock(now) or not math.isfinite(stamp) or stamp <= 0.0 or stamp > now:
            return False
        if position_valid:
            self.mark_observation(stamp, True, now=now)
        if self._last_bearing_stamp is not None and stamp <= self._last_bearing_stamp:
            return False
        self._last_bearing_stamp = stamp
        valid = bool(valid and math.isfinite(bearing) and self._fresh(stamp, now))
        self._latest_bearing_valid = valid
        if not valid:
            self.consecutive_valid_frames = 0
            self._centered_stamps.clear()
            self._direction_stamps.clear()
            self.consecutive_lost_frames += 1
            if self._loss_started_at is None:
                self._loss_started_at = stamp
            return True
        self.has_ever_seen_target = True
        self.last_valid_bearing_stamp = stamp
        self.last_valid_image_bearing = bearing
        if bearing != 0.0:
            self.last_valid_bearing_sign = 1 if bearing > 0 else -1
        self.consecutive_lost_frames = 0
        self._loss_started_at = None
        self._search_started_at = None
        self._clear_scan()
        if abs(bearing) < math.pi / 2.0:
            if self._direction_stamps and not self._fresh(self._direction_stamps[-1], stamp):
                self._direction_stamps.clear()
            self._direction_stamps.append(stamp)
        else:
            self._direction_stamps.clear()
        if abs(bearing) <= self.config.target_lock_max_bearing:
            if self._centered_stamps and not self._fresh(self._centered_stamps[-1], stamp):
                self._centered_stamps.clear()
            self._centered_stamps.append(stamp)
            self.consecutive_valid_frames = len(self._centered_stamps)
        else:
            self._centered_stamps.clear()
            self.consecutive_valid_frames = 0
        return True

    def _loss_time(self, now):
        if self._loss_started_at is not None:
            return self._loss_started_at
        if (self.last_valid_bearing_stamp is not None
                and not self._fresh(self.last_valid_bearing_stamp, now)):
            return self.last_valid_bearing_stamp + self.config.target_lock_max_age
        return None

    def _lock_ready(self, now, visible, kf_fresh):
        return (visible and kf_fresh
                and self._fresh(self.last_valid_observation_stamp, now)
                and len(self._centered_stamps) >= self.config.target_lock_min_frames)

    def bearing_approach_ready(self, now, maximum_age):
        """Grant direction motion from fresh RGB frames independently of centered lock."""
        stamp = self.last_valid_bearing_stamp
        return bool(
            self._latest_bearing_valid and stamp is not None
            and 0.0 <= now-stamp <= min(maximum_age, self.config.target_lock_max_age)
            and abs(self.last_valid_image_bearing) < math.pi / 2.0
            and len(self._direction_stamps) >= self.config.target_lock_min_frames
        )

    def _servo(self):
        if abs(self.last_valid_image_bearing) < self.config.target_center_deadband_rad:
            return 0.0
        limit = self.config.maximum_search_yaw_rate
        return max(-limit, min(limit, self.config.vision_yaw_gain * self.last_valid_image_bearing))

    def _start_scan(self, now, yaw):
        self._search_started_at = now
        self._scan_last_yaw = yaw
        if not self.has_ever_seen_target:
            self._scan_endpoints = [2.0 * math.pi]
            rate = min(self.config.initial_search_yaw_rate, self.config.maximum_search_yaw_rate)
        else:
            sign = self.last_valid_bearing_sign or 1
            magnitude = self.config.search_initial_arc
            endpoints = []
            # Bound stage count even for an infinitesimal positive increment.
            for _ in range(12):
                if magnitude >= math.pi:
                    break
                endpoints.append(sign * magnitude)
                magnitude += self.config.search_arc_increment
                sign *= -1
            endpoints.extend([sign * math.pi, -sign * math.pi])
            self._scan_endpoints = endpoints
            rate = min(self.config.reacquire_yaw_rate, self.config.maximum_search_yaw_rate)
        distance = abs(self._scan_endpoints[0])
        distance += sum(abs(b - a) for a, b in zip(
            self._scan_endpoints, self._scan_endpoints[1:]))
        # Finite command budget also stops a scan if yaw feedback never moves.
        self._scan_budget = distance / rate + 1.0

    def _search_rate(self, now, yaw, dt):
        if self._scan_finished:
            return 0.0, 0
        if self._scan_last_yaw is None:
            self._start_scan(now, yaw)
        delta = (yaw - self._scan_last_yaw + math.pi) % (2.0 * math.pi) - math.pi
        self._scan_offset += delta
        self._scan_last_yaw = yaw
        self._scan_elapsed += dt
        if self._scan_elapsed >= self._scan_budget:
            self._scan_finished = True
            return 0.0, 0
        remaining = self._scan_endpoints[self._scan_index] - self._scan_offset
        previous_endpoint = (self._scan_endpoints[self._scan_index - 1]
                             if self._scan_index else 0.0)
        segment = self._scan_endpoints[self._scan_index] - previous_endpoint
        direction = 1 if segment > 0.0 else -1
        # Advance on reaching or crossing the endpoint in the intended
        # direction; real yaw feedback need not equal a mathematical angle.
        if remaining * direction <= 1e-6:
            self._scan_index += 1
            if self._scan_index == len(self._scan_endpoints):
                self._scan_finished = True
                return 0.0, 0
            remaining = self._scan_endpoints[self._scan_index] - self._scan_offset
        sign = 1 if remaining > 0.0 else -1
        rate = (self.config.reacquire_yaw_rate if self.has_ever_seen_target
                else self.config.initial_search_yaw_rate)
        rate = min(rate, self.config.maximum_search_yaw_rate, abs(remaining) / dt)
        return sign * rate, sign

    def update(self, now, yaw, height, dt, kf_fresh, terminal=False):
        """
        Arbitrate visual yaw, debounced loss, finite search and recovery.

        The caller releases terminal recovery by passing ``terminal=False``
        only after climbing to its own maximum of search height and sea
        recovery clearance. No XYZ command is generated by this module.
        """
        if (not self._clock(now) or not all(math.isfinite(value) for value in (yaw, height, dt))
                or dt <= 0.0):
            self._locked = False
            return VisibilityDecision('SAFE_WAIT', False, False, 0.0, 0)
        visible = self._latest_bearing_valid and self._fresh(self.last_valid_bearing_stamp, now)
        loss_time = self._loss_time(now)
        silence_limit = self.config.target_lock_max_age * self.config.target_loss_frames
        silent = (self.last_valid_bearing_stamp is not None
                  and now - self.last_valid_bearing_stamp > silence_limit)
        lost = self.consecutive_lost_frames >= self.config.target_loss_frames or silent
        if not terminal:
            self._recovery_latched = False
        elif (loss_time is not None
              and now - loss_time >= self.config.terminal_loss_recovery_timeout):
            self._recovery_latched = True
        if self._recovery_latched:
            self._locked = False
            return VisibilityDecision('SAFE_RECOVERY', False, visible, 0.0, 0)
        if lost or not kf_fresh:
            self._locked = False
        ready = self._lock_ready(now, visible, kf_fresh)
        newly_locked = ready and not self._locked
        if newly_locked:
            self._locked = True
            self.has_ever_locked_target = True
            self.last_lock_stamp = self.last_valid_bearing_stamp
            self._search_started_at = None
            self._clear_scan()
        if self._locked:
            state = 'TARGET_LOCK' if newly_locked else 'TRACKING'
            rate = self._servo() if visible else 0.0
            return VisibilityDecision(state, True, visible, rate, 0)
        if terminal:
            return VisibilityDecision('SAFE_WAIT', False, visible, 0.0, 0)
        if visible:
            state = 'REACQUIRE' if self.has_ever_locked_target else 'TARGET_ACQUIRE'
            return VisibilityDecision(state, False, True, self._servo(), 0)
        state = 'REACQUIRE' if self.has_ever_seen_target else 'TARGET_ACQUIRE'
        if height < self.config.target_search_enable_height:
            return VisibilityDecision(state, False, False, 0.0, 0)
        rate, direction = self._search_rate(now, yaw, dt)
        timed_out = (self.has_ever_seen_target
                     and now - self._search_started_at >= self.config.target_reacquire_timeout)
        if self._scan_finished or timed_out:
            self._scan_finished = True
            return VisibilityDecision('SAFE_WAIT', False, False, 0.0, 0)
        return VisibilityDecision(state, False, False, rate, direction)
