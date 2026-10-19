"""Evaluator-only native model geometry; never an online navigation correction."""

from dataclasses import dataclass
from collections import Counter
import math

from .gazebo_entity_pose import gazebo_enu_to_ned
from .intercept_evaluator import KinematicState


@dataclass(frozen=True)
class NativeCaptureSample:
    stamp: float
    native_stamp: float
    uav: KinematicState
    target: KinematicState
    maximum_bracket: float
    velocity_valid: bool = True
    segment_valid: bool = False
    uav_clock_bracket: tuple = ()
    target_clock_bracket: tuple = ()


def _clock_width(bracket, raw, mapped):
    if (bracket is None or len(bracket) != 4 or
            not all(math.isfinite(x) for x in bracket)):
        return None
    a, b, c, d = bracket
    if (not 0. <= b-a <= .05+1e-9 or not 0. <= d-c <= .05+1e-9 or
            not a-1e-9 <= raw <= b+1e-9 or not c-1e-6 <= mapped <= d+1e-6):
        return None
    expected = c if b == a else c+(raw-a)/(b-a)*(d-c)
    return max(b-a, d-c) if abs(expected-mapped) <= 1e-6 else None


class NativeCaptureStream:
    """
    Consume every causal exact-model pair, never just the newest pose.

    Position-only radius entry is valid without a velocity history. Target
    set_pose jumps cannot create swept-sphere hits between their endpoints.
    """

    def __init__(self, started_at):
        self.started_at = float(started_at)
        self.cursor = -math.inf
        self.generation = None
        self.previous = None
        self.counters = Counter()
        self.last_diagnostics = {}

    def drain(self, uav_tracker, target_tracker, now, visual_height_offset):
        """Read the existing bounded histories; preserve raw and mapped epochs."""
        self.last_diagnostics = dict(decision_stamp=now, rejected=[])
        if (uav_tracker is None or target_tracker is None or
                not all(math.isfinite(t) for t in (now, visual_height_offset))):
            self.counters['INVALID_SOURCE_OR_TIME'] += 1
            return []
        generation = (uav_tracker.reset_count, target_tracker.reset_count)
        if generation[0] != generation[1]:
            self.counters['CLOCK_GENERATION_MISMATCH'] += 1
            return []
        if generation != self.generation:
            if self.generation is not None:
                self.counters['CLOCK_RESET'] += 1
            self.generation, self.cursor, self.previous = generation, -math.inf, None
        uav = {raw: (mapped, value, bracket)
               for raw, mapped, value, bracket in uav_tracker.history.snapshot()}
        target = {raw: (mapped, value, bracket)
                  for raw, mapped, value, bracket in target_tracker.history.snapshot()}
        if (uav_tracker.reset_count, target_tracker.reset_count) != generation:
            self.previous = None
            self.counters['CLOCK_RESET_DURING_SNAPSHOT'] += 1
            return []
        result = []
        for raw in sorted(uav.keys() & target.keys()):
            if raw <= self.cursor:
                continue
            stamp, up, uav_bracket = uav[raw]
            target_stamp, tp, target_bracket = target[raw]
            if (math.isfinite(stamp) and math.isfinite(target_stamp) and
                    max(stamp, target_stamp) > now):
                # The transport thread may advance after the ROS timer read
                # `now`. Keep this native sample for the next causal tick.
                self.counters['DEFERRED_FUTURE_POSE'] += 1
                self.last_diagnostics['deferred_native_stamp'] = raw
                break
            self.cursor = raw
            widths = (_clock_width(uav_bracket, raw, stamp),
                      _clock_width(target_bracket, raw, target_stamp))
            reason = ''
            if not all(math.isfinite(t) for t in (raw, stamp, target_stamp, *up[:3], *tp[:3])):
                reason = 'NONFINITE_NATIVE_POSITION'
            elif abs(stamp-target_stamp) > 1e-6:
                reason = 'MAPPED_EPOCH_MISMATCH'
            elif any(width is None for width in widths):
                reason = 'CLOCK_MAPPING_BRACKET'
            elif stamp < self.started_at:
                reason = 'BEFORE_CAPTURE_START'
            elif not 0. <= now-stamp <= .125:
                reason = 'NATIVE_POSITION_AGE'
            if reason:
                self.counters[reason] += 1
                self.last_diagnostics['rejected'].append(dict(
                    native_stamp=raw, stamp=stamp, reason=reason,
                    uav_clock_bracket=uav_bracket, target_clock_bracket=target_bracket))
                self.previous = None
                continue
            up = gazebo_enu_to_ned(up[:3])
            marker = gazebo_enu_to_ned(tp[:3])
            tp = (*marker[:2], marker[2]+visual_height_offset)
            previous = self.previous
            velocity_valid = bool(previous and
                                  0. < raw-previous.native_stamp <= .05+1e-9 and
                                  0. < stamp-previous.stamp <= .05+1e-9)
            if velocity_valid:
                dt = raw-previous.native_stamp
                uv, tv = [tuple((a-b)/dt for a, b in zip(current, old)) for current, old in
                          ((up, previous.uav.position), (tp, previous.target.position))]
            else:
                uv = tv = (math.nan,)*3
            # Only a held target pose proves absence of a kinematic set_pose
            # jump in this scenario. A moving target endpoint is not a sweep.
            segment_valid = bool(velocity_valid and
                                 all(abs(a-b) <= 1e-9 for a, b in
                                     zip(tp, previous.target.position)))
            sample = NativeCaptureSample(stamp, raw, KinematicState(up, uv),
                                         KinematicState(tp, tv), max(widths),
                                         velocity_valid, segment_valid,
                                         uav_bracket, target_bracket)
            self.previous = sample
            self.counters['ACCEPTED_EXACT_PAIRS'] += 1
            if not velocity_valid:
                self.counters['POSITION_ONLY'] += 1
            result.append(sample)
        self.last_diagnostics.update(
            accepted=len(result),
            last_native_cursor=self.cursor if math.isfinite(self.cursor) else None,
            uav_status=uav_tracker.last_status, target_status=target_tracker.last_status,
            unmatched_uav=sum(raw > self.cursor and raw not in target for raw in uav),
            unmatched_target=sum(raw > self.cursor and raw not in uav for raw in target))
        return result


def _raw_stamp(query):
    return query.left_sim_stamp + query.fraction*(query.right_sim_stamp-query.left_sim_stamp)


def native_capture_sample(uav_tracker, target_tracker, now, visual_height_offset):
    """Use a common causal epoch and <=50 ms brackets, without extrapolation."""
    if uav_tracker is None or target_tracker is None:
        return None
    stamps = (uav_tracker.last_mapped_stamp, target_tracker.last_mapped_stamp)
    if not all(math.isfinite(t) for t in (*stamps, now, visual_height_offset)):
        return None
    stamp = min(stamps)
    if not 0. <= now-stamp <= .125:
        return None
    queries = [tracker.query_at(t) for t in (stamp, stamp-.02)
               for tracker in (uav_tracker, target_tracker)]
    for query in queries:
        if (query.status not in ('EXACT', 'INTERPOLATED') or query.value is None
                or not 0. <= query.interval <= .05+1e-9
                or not 0. <= query.right_sim_stamp-query.left_sim_stamp <= .05+1e-9
                or not all(math.isfinite(v) for v in query.value[:3])):
            return None
    current_uav, current_target, previous_uav, previous_target = queries
    current_raw, previous_raw = _raw_stamp(current_uav), _raw_stamp(previous_uav)
    if (abs(current_raw-_raw_stamp(current_target)) > 1e-6
            or abs(previous_raw-_raw_stamp(previous_target)) > 1e-6
            or not 0. < current_raw-previous_raw <= .05):
        return None
    positions = [gazebo_enu_to_ned(query.value[:3]) for query in queries]
    dt = current_raw-previous_raw
    velocity = [tuple((a-b)/dt for a, b in zip(positions[i], positions[i+2]))
                for i in (0, 1)]
    target = (*positions[1][:2], positions[1][2]+visual_height_offset)
    return NativeCaptureSample(stamp, current_raw,
                               KinematicState(positions[0], velocity[0]),
                               KinematicState(target, velocity[1]),
                               max(query.interval for query in queries))
