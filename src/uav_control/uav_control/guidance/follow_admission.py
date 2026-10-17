"""Receiver-owned research eligibility; neither this policy nor probes grant execution."""
import bisect
from dataclasses import dataclass
import math


NATIVE_BRACKET_SECONDS = .05


@dataclass(frozen=True)
class AdmissionConfig:
    horizontal_enter: float = 1.
    relative_enter: float = .5
    height_enter: float = .1
    stable_seconds: float = 5.
    cooldown_seconds: float = 2.
    horizontal_exit: float = 1.5
    relative_exit: float = .75
    performance_seconds: float = .5
    maximum_sample_gap: float = .2
    probe_maximum_gap: float = .45
    required_probes: int = 3
    maximum_entries: int = 3
    target_observation_age: float = .125

    def __post_init__(self):
        if (any(not math.isfinite(v) or v <= 0 for v in vars(self).values())
                or self.horizontal_exit <= self.horizontal_enter
                or self.relative_exit <= self.relative_enter
                or self.target_observation_age > .2
                or any(type(v) is not int for v in (self.required_probes, self.maximum_entries))):
            raise ValueError('INVALID_FOLLOW_ADMISSION_CONFIG')


def _tracking_turn_rate(tracking, target):
    """Fit causal velocity headings from distinct image updates, never future KF states."""
    frames = {}
    for row in sorted(tracking, key=lambda r: r['stamp']):
        if (row.get('valid') and 0 < row.get('source', 0.) <= row['stamp']
                and row['stamp'] <= target['stamp']
                and 0 <= target['source']-row['source'] <= .25
                and len(row.get('v', ())) == 3
                and all(math.isfinite(v) for v in row['v'])
                and math.hypot(*row['v'][:2]) >= .2):
            frames[row['source']] = row
    rows = sorted(frames.values(), key=lambda r: r['stamp'])[-5:]
    if len(rows) < 3 or any(a['source'] >= b['source'] for a, b in zip(rows, rows[1:])):
        return None
    # CV KF propagation changes its state epoch without changing velocity.
    # Different image acquisitions, not callback jitter, identify its updates.
    times = [row['source']-target['source'] for row in rows]
    angles = []
    for row in rows:
        angle = math.atan2(row['v'][1], row['v'][0])
        if angles:
            angle = angles[-1]+math.atan2(math.sin(angle-angles[-1]),
                                          math.cos(angle-angles[-1]))
        angles.append(angle)
    mean_t, mean_a = sum(times)/len(times), sum(angles)/len(angles)
    denominator = sum((t-mean_t)**2 for t in times)
    if denominator <= 1e-12:
        return None
    return sum((t-mean_t)*(a-mean_a) for t, a in zip(times, angles))/denominator


def follow_settling(navigation, tracking, now, distance=5., altitude=-5.,
                    admission_config=AdmissionConfig(), reference_mode='target'):
    """Compare KF P/V at its epoch with bracketed native nav; never extrapolate nav."""
    if reference_mode not in ('target', 'rear_point'):
        raise ValueError('INVALID_FOLLOW_REFERENCE_MODE')

    def valid(row):
        return row.get('valid') and all(len(row.get(k, ())) == 3 for k in ('p', 'v')) and all(
            math.isfinite(v) for k in ('p', 'v') for v in row[k])

    # An explicit invalid latest sample is a loss, not permission to pick an older valid one.
    if (not navigation or not tracking or not valid(navigation[-1]) or not valid(tracking[-1])
            or not 0 < tracking[-1].get('source', 0.) <= tracking[-1].get('stamp', 0.) <= now):
        return dict(settled=False, reason='LATEST_INPUT_INVALID')

    nav = sorted((row for row in navigation if valid(row) and 0 < row['stamp'] <= now),
                 key=lambda row: row['stamp'])
    stamps = [row['stamp'] for row in nav]
    for target in sorted(tracking, key=lambda row: row['stamp'], reverse=True):
        if (not valid(target) or not 0 <= now-target.get('stamp', 0.) <= .125
                or not 0 <= now-target.get('source', 0.)
                <= admission_config.target_observation_age):
            continue
        i = bisect.bisect_left(stamps, target['stamp'])
        if i < len(nav) and stamps[i] == target['stamp']:
            a = b = nav[i]
            p, v = a['p'], a['v']
        else:
            if not 0 < i < len(nav):
                continue
            a, b = nav[i-1], nav[i]
            dt = b['stamp']-a['stamp']
            if not 0 < dt <= NATIVE_BRACKET_SECONDS:
                continue
            f = (target['stamp']-a['stamp'])/dt
            p = [x+f*(y-x) for x, y in zip(a['p'], b['p'])]
            v = [x+f*(y-x) for x, y in zip(a['v'], b['v'])]
        speed = math.hypot(*target['v'][:2])
        direction = ([x/speed for x in target['v'][:2]] if speed > 1e-6 else [1., 0.])
        desired = [target['p'][i]-distance*direction[i] for i in range(2)]+[altitude]
        error = math.hypot(*(desired[i]-p[i] for i in range(2)))
        target_relative = math.hypot(*(target['v'][i]-v[i] for i in range(2)))
        reference_v = list(target['v'])
        turn_rate = 0.
        if reference_mode == 'rear_point' and speed >= .2:
            turn_rate = _tracking_turn_rate(tracking, target)
            if turn_rate is None:
                return dict(settled=False, reason='TURN_RATE_UNAVAILABLE')
            # d(target - distance*heading)/dt includes rotation of the rear offset.
            reference_v[0] += distance*turn_rate*direction[1]
            reference_v[1] -= distance*turn_rate*direction[0]
        relative = math.hypot(*(reference_v[i]-v[i] for i in range(2)))
        rx, ry = target['p'][0]-p[0], target['p'][1]-p[1]
        los_rate = ((rx*(target['v'][1]-v[1])-ry*(target['v'][0]-v[0]))
                    / (rx*rx+ry*ry)) if rx*rx+ry*ry > 1e-6 else 0.
        height = abs(p[2]-altitude)
        c = admission_config
        return dict(settled=(error <= c.horizontal_enter and relative <= c.relative_enter
                             and height <= c.height_enter),
                    epoch=target['stamp'], source_stamp=target['source'],
                    navigation_bracket=[a['stamp'], b['stamp']], horizontal_error=error,
                    relative_speed=relative, target_relative_speed=target_relative,
                    reference_velocity=reference_v, reference_mode=reference_mode,
                    turn_rate=turn_rate, los_yaw_rate=los_rate,
                    height_error=height, desired_position=desired,
                    method='native bracket interpolation; no extrapolation')
    latest = tracking[-1]
    # A fresh KF epoch just ahead of delivered native navigation is a causal wait.
    # No extrapolation or entry grant: the right bracket must actually arrive.
    if (nav and 0 < latest['stamp']-nav[-1]['stamp'] <= NATIVE_BRACKET_SECONDS
            and 0 <= now-latest['stamp'] <= .125
            and 0 <= now-latest['source'] <= admission_config.target_observation_age
            and 0 <= now-nav[-1]['stamp'] <= .125):
        return dict(settled=False, reason='AWAITING_NATIVE_BRACKET',
                    awaiting_native_bracket=True, epoch=latest['stamp'],
                    source_stamp=latest['source'], navigation_latest=nav[-1]['stamp'])
    return dict(settled=False, reason='NO_FRESH_COMMON_EPOCH')


class FollowAdmission:
    """Only the actual receiver calls this policy with its local context and owner."""

    def __init__(self, config=AdmissionConfig()):
        self.config = config
        self.state = 'FOLLOW_APPROACH'
        self.stable_since = self.last_now = self.bad_since = None
        self.recover_until = -math.inf
        self.probes, self.last_probe, self.entries = [], None, 0
        self.loss = None
        self.awaiting_epoch = False
        self.awaiting_since = None

    def update(self, now, metrics, legal, active=False):
        reversed_clock = self.last_now is not None and now < self.last_now
        gap = self.last_now is not None and now-self.last_now > self.config.maximum_sample_gap
        self.last_now = now
        self.awaiting_epoch = False
        c = self.config
        values = [metrics.get(k, math.inf) for k in
                  ('horizontal_error', 'relative_speed', 'height_error')]
        valid = legal and math.isfinite(now) and all(math.isfinite(v) for v in values)
        if reversed_clock:
            self.stable_since, self.probes, self.last_probe = None, [], None
            self.awaiting_since = None
            self.recover_until = now+c.cooldown_seconds
            return 'CLOCK_REVERSED'
        if (self.awaiting_since is not None and math.isfinite(now)
                and now-self.awaiting_since > NATIVE_BRACKET_SECONDS
                and (valid or (legal and metrics.get('awaiting_native_bracket')))):
            self.stable_since = self.bad_since = self.last_probe = None
            self.probes = []
            self.state = 'FOLLOW_RECOVERY' if self.loss else 'FOLLOW_APPROACH'
            self.awaiting_epoch = bool(metrics.get('awaiting_native_bracket'))
            if not self.awaiting_epoch:
                self.awaiting_since = None
            # A late successful bracket cannot erase a gap in ACTIVE observation.
            if active or self.awaiting_epoch:
                return 'NATIVE_BRACKET_TIMEOUT'
        if legal and math.isfinite(now) and not gap and metrics.get('awaiting_native_bracket'):
            self.awaiting_epoch = True
            # New KF epochs cannot renew an unresolved wait. No entry is granted
            # while geometry is missing, and sustained absence revokes execution.
            if self.awaiting_since is None:
                self.awaiting_since = now
            self.state = ('MINCO_ACTIVE' if active else
                          'FOLLOW_RECOVERY' if self.loss else 'FOLLOW_APPROACH')
            return ''
        self.awaiting_since = None
        if active:
            self.state, self.stable_since = 'MINCO_ACTIVE', None
            if not valid:
                return 'EXECUTION_CONTEXT_INVALID'
            bad = values[0] > c.horizontal_exit or values[1] > c.relative_exit
            self.bad_since = ((self.bad_since if self.bad_since is not None else now)
                              if bad else None)
            if self.bad_since is not None and now-self.bad_since >= c.performance_seconds:
                return 'PERFORMANCE_DEVIATION'
            return ''
        self.bad_since = None
        stable = (valid and values[0] <= c.horizontal_enter and values[1] <= c.relative_enter
                  and values[2] <= c.height_enter and now >= self.recover_until)
        if gap or not stable:
            self.stable_since, self.probes, self.last_probe = None, [], None
        if stable:
            self.stable_since = now if self.stable_since is None else self.stable_since
        if self.loss is None:
            self.state = 'FOLLOW_STABLE' if self.converged(now) else 'FOLLOW_APPROACH'
        else:
            self.state = 'FOLLOW_RECOVERY'
        return ''

    def converged(self, now):
        return (not self.awaiting_epoch and self.stable_since is not None
                and now-self.stable_since >= self.config.stable_seconds-1e-8)

    def probe(self, now, identity, valid):
        if (not valid or self.last_probe is None
                or not 0 <= now-self.last_probe <= self.config.probe_maximum_gap):
            self.probes = []
        if valid and identity not in self.probes:
            self.probes.append(identity)
            self.probes = self.probes[-self.config.required_probes:]
        self.last_probe = now if valid else None

    def allowed(self, now):
        return bool(self.converged(now) and now >= self.recover_until
                    and self.entries < self.config.maximum_entries
                    and len(self.probes) >= self.config.required_probes
                    and self.last_probe is not None
                    and 0 <= now-self.last_probe <= self.config.probe_maximum_gap)

    def consume(self):
        self.entries += 1

    def inhibition_reason(self, now):
        if self.entries >= self.config.maximum_entries:
            return 'FOLLOW_ENTRY_LIMIT_REACHED'
        return '' if self.allowed(now) else 'FOLLOW_ENTRY_INHIBITED'

    def lost(self, now, reason, curve):
        self.loss = dict(reason=reason, stamp=now, plan_id=curve.plan_id,
                         parent_id=curve.parent_id, start=curve.start,
                         lease_deadline=min(curve.end, curve.start+.45),
                         last_legal_execution=min(now, curve.end, curve.start+.45))
        self.state = 'FOLLOW_RECOVERY'
        self.recover_until = now+self.config.cooldown_seconds
        self.stable_since = self.bad_since = self.last_probe = None
        self.probes = []
