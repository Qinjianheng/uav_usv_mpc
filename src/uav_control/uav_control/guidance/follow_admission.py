"""Receiver-owned research eligibility; neither this policy nor probes grant execution."""
import bisect
from dataclasses import dataclass
import math


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

    def __post_init__(self):
        if (any(not math.isfinite(v) or v <= 0 for v in vars(self).values())
                or self.horizontal_exit <= self.horizontal_enter
                or self.relative_exit <= self.relative_enter
                or any(type(v) is not int for v in (self.required_probes, self.maximum_entries))):
            raise ValueError('INVALID_FOLLOW_ADMISSION_CONFIG')


def follow_settling(navigation, tracking, now, distance=5., altitude=-5.):
    """Compare KF P/V at its epoch with bracketed native nav; never extrapolate nav."""
    def valid(row):
        return row.get('valid') and all(len(row.get(k, ())) == 3 for k in ('p', 'v')) and all(
            math.isfinite(v) for k in ('p', 'v') for v in row[k])

    nav = sorted((row for row in navigation if valid(row) and 0 < row['stamp'] <= now),
                 key=lambda row: row['stamp'])
    stamps = [row['stamp'] for row in nav]
    for target in sorted(tracking, key=lambda row: row['stamp'], reverse=True):
        if not valid(target) or not all(
                0 <= now-target.get(k, 0.) <= .125 for k in ('stamp', 'source')):
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
            if not 0 < dt <= .05:
                continue
            f = (target['stamp']-a['stamp'])/dt
            p = [x+f*(y-x) for x, y in zip(a['p'], b['p'])]
            v = [x+f*(y-x) for x, y in zip(a['v'], b['v'])]
        speed = math.hypot(*target['v'][:2])
        direction = ([x/speed for x in target['v'][:2]] if speed > 1e-6 else [1., 0.])
        desired = [target['p'][i]-distance*direction[i] for i in range(2)]+[altitude]
        error = math.hypot(*(desired[i]-p[i] for i in range(2)))
        relative = math.hypot(*(target['v'][i]-v[i] for i in range(2)))
        height = abs(p[2]-altitude)
        return dict(settled=error <= 1. and relative <= .5 and height <= .1,
                    epoch=target['stamp'], source_stamp=target['source'],
                    navigation_bracket=[a['stamp'], b['stamp']], horizontal_error=error,
                    relative_speed=relative, height_error=height, desired_position=desired,
                    method='native bracket interpolation; no extrapolation')
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

    def update(self, now, metrics, legal, active=False):
        reversed_clock = self.last_now is not None and now < self.last_now
        gap = self.last_now is not None and now-self.last_now > self.config.maximum_sample_gap
        self.last_now = now
        c = self.config
        values = [metrics.get(k, math.inf) for k in
                  ('horizontal_error', 'relative_speed', 'height_error')]
        valid = legal and math.isfinite(now) and all(math.isfinite(v) for v in values)
        if reversed_clock:
            self.stable_since, self.probes, self.last_probe = None, [], None
            self.recover_until = now+c.cooldown_seconds
            return 'CLOCK_REVERSED'
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
        return (self.stable_since is not None
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

    def lost(self, now, reason, curve):
        self.loss = dict(reason=reason, stamp=now, plan_id=curve.plan_id,
                         parent_id=curve.parent_id, start=curve.start,
                         lease_deadline=min(curve.end, curve.start+.45),
                         last_legal_execution=min(now, curve.end, curve.start+.45))
        self.state = 'FOLLOW_RECOVERY'
        self.recover_until = now+self.config.cooldown_seconds
        self.stable_since = self.bad_since = self.last_probe = None
        self.probes = []
