"""Run-scoped visibility and phase metrics, independent of interception."""

import math


def _statistics(values):
    return {
        'count': len(values),
        'mean': sum(values) / len(values) if values else None,
        'minimum': min(values) if values else None,
        'maximum': max(values) if values else None,
    }


class RunMetricAccumulator:
    """Count lock events and actual search drift without controlling flight."""

    SEARCH_PHASES = {'TARGET_ACQUIRE', 'REACQUIRE', 'SAFE_WAIT'}

    def __init__(self, started_at):
        self.started_at = float(started_at)
        self.phase = None
        self.phase_started_at = self.started_at
        self.last_stamp = self.started_at
        self.durations = {}
        self.first_visible_at = None
        self.first_lock_at = None
        self.locked = False
        self.lock_loss_count = 0
        self.reacquire_count = 0
        self.reacquire_started_at = None
        self.reacquire_durations = []
        self.follow_estimated_distances = []
        self.follow_evaluation_distances = []
        self.search_anchor = None
        self.search_xy_drift_max = None
        self.control_status = ''
        self.bearing_approach_duration = 0.
        self.visual_braking_duration = 0.

    def observe(
        self, now, phase, *, visible=None, locked=None, position=None,
        estimated_distance=None, evaluation_distance=None,
        control_status=None,
    ):
        """Record monotonic phase events or fresh controller/sample metrics."""
        now = float(now)
        if not math.isfinite(now) or now < self.last_stamp:
            return False
        if self.control_status == 'BEARING_APPROACH':
            self.bearing_approach_duration += now - self.last_stamp
        if self.control_status == 'VISUAL_BRAKING':
            self.visual_braking_duration += now - self.last_stamp
        self.last_stamp = now
        phase = str(phase)
        if control_status is not None:
            self.control_status = str(control_status)
        elif phase != self.phase:
            self.control_status = ''
        if phase != self.phase:
            if self.phase is not None:
                self.durations[self.phase] = (
                    self.durations.get(self.phase, 0.)
                    + now - self.phase_started_at
                )
            self.phase = phase
            self.phase_started_at = now
        if visible and self.first_visible_at is None:
            self.first_visible_at = now
        if locked is not None:
            locked = bool(locked)
            if locked and self.first_lock_at is None:
                self.first_lock_at = now
            if self.locked and not locked:
                self.lock_loss_count += 1
            self.locked = locked
        if (phase in ('REACQUIRE', 'SAFE_WAIT')
                and self.first_lock_at is not None
                and self.reacquire_started_at is None and not self.locked):
            self.reacquire_started_at = now
            self.reacquire_count += 1
        if self.locked and self.reacquire_started_at is not None:
            self.reacquire_durations.append(now - self.reacquire_started_at)
            self.reacquire_started_at = None
        if phase == 'FOLLOW':
            for value, target in (
                (estimated_distance, self.follow_estimated_distances),
                (evaluation_distance, self.follow_evaluation_distances),
            ):
                if value is not None and math.isfinite(value) and value >= 0.:
                    target.append(float(value))
        if phase not in self.SEARCH_PHASES or self.control_status in (
            'BEARING_APPROACH', 'VISUAL_BRAKING',
        ):
            self.search_anchor = None
        elif position is not None and all(math.isfinite(v) for v in position):
            if self.search_anchor is None:
                self.search_anchor = tuple(position[:2])
            drift = math.hypot(
                position[0] - self.search_anchor[0],
                position[1] - self.search_anchor[1],
            )
            self.search_xy_drift_max = max(
                self.search_xy_drift_max or 0., drift,
            )
        return True

    def summary(self, now):
        """Return a non-mutating run summary; missing samples stay unknown."""
        now = max(float(now), self.last_stamp)
        durations = dict(self.durations)
        if self.phase is not None:
            durations[self.phase] = (
                durations.get(self.phase, 0.) + now - self.phase_started_at
            )
        return {
            'phase_durations': durations,
            'takeoff_duration': durations.get('TAKEOFF', 0.),
            'follow_duration': durations.get('FOLLOW', 0.),
            'bearing_approach_duration': self.bearing_approach_duration + (
                now - self.last_stamp if self.control_status == 'BEARING_APPROACH' else 0.),
            'visual_braking_duration': self.visual_braking_duration + (
                now - self.last_stamp if self.control_status == 'VISUAL_BRAKING' else 0.),
            'time_to_first_visible': (
                self.first_visible_at - self.started_at
                if self.first_visible_at is not None else None
            ),
            'time_to_first_lock': (
                self.first_lock_at - self.started_at
                if self.first_lock_at is not None else None
            ),
            'lock_loss_count': self.lock_loss_count,
            'reacquire_count': self.reacquire_count,
            'reacquire_duration': _statistics(self.reacquire_durations),
            'open_reacquire_duration': (
                now - self.reacquire_started_at
                if self.reacquire_started_at is not None else None
            ),
            'follow_estimated_distance': _statistics(
                self.follow_estimated_distances,
            ),
            'follow_evaluation_distance': _statistics(
                self.follow_evaluation_distances,
            ),
            'search_xy_drift_max': self.search_xy_drift_max,
            'event_time_basis': 'evaluator ROS callback/sample timestamps',
        }
