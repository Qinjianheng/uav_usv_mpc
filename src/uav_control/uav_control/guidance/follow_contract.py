"""FOLLOW receiver contract. Nominal feasibility and publisher claims grant no authority."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math

import numpy as np

from uav_control.guidance.polynomial_extrema import derivative_peak


@dataclass(frozen=True)
class FollowCurve:
    """XYZ ascending coefficients (piece,6,3), yaw ascending (piece,4), all epochs explicit."""

    plan_id: int
    mission_id: int
    generation: int
    prediction_id: int
    parent_id: int
    navigation_stamp: float
    attitude_stamp: float
    observation_stamp: float
    source_stamp: float
    input_until: float
    coverage_until: float
    start: float
    end: float
    holding_until: float
    durations: tuple
    xyz: tuple
    yaw: tuple
    frame: str = 'local_ned'
    constraint_version: str = 'constraints-p4-v1'
    camera_version: str = 'camera-p1-v1'
    holding_model: str = ''
    receiver_boot_id: str = ''
    planner_boot_id: str = ''
    constraint_snapshot: str = ''
    constraint_fingerprint: str = ''

    def fingerprint(self):
        """Bind independent approval to every coefficient, epoch and identity."""
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True,
                                         allow_nan=False).encode()).hexdigest()

    def sample(self, epoch):
        """P/V/A/yaw/rate from a complete reference, without extrapolation or endpoint alias."""
        if not math.isfinite(epoch) or not self.start-1e-9 <= epoch <= self.end+1e-9:
            raise ValueError('REFERENCE_OUTSIDE')
        elapsed = max(0., epoch-self.start)
        index = min(int(np.searchsorted(np.cumsum(self.durations), elapsed, side='right')),
                    len(self.durations)-1)
        local = min(self.durations[index], elapsed-sum(self.durations[:index]))
        c = np.asarray(self.xyz).reshape(-1, 6, 3)[index]
        y = np.asarray(self.yaw)[index]
        values = [np.polynomial.polynomial.polyval(local,
                  np.polynomial.polynomial.polyder(c, m=d, axis=0)) for d in range(3)]
        return (*np.concatenate(values), float(np.polynomial.polynomial.polyval(local, y)),
                float(np.polynomial.polynomial.polyval(local,
                      np.polynomial.polynomial.polyder(y))))


@dataclass(frozen=True)
class ReceiverContext:
    """Only actual Tracker builds this; boundary is not an accepted future reference."""

    now: float
    mission_id: int
    generation: int
    prediction_id: int
    following: bool
    offboard: bool
    armed: bool
    visible: bool
    boundary: tuple = ()
    receiver_boot_id: str = ''


@dataclass(frozen=True)
class SafetyApproval:
    """Receiver-injected independent validator output, never parsed from candidate ROS data."""

    fingerprint: str
    valid_until: float
    evidence_id: str


@dataclass(frozen=True)
class FollowAck:
    """Actual receiving endpoint result; acceptance differs from replacement/activation."""

    plan_id: int
    mission_id: int
    generation: int
    stamp: float
    state: str
    reasons: tuple = ()
    receiver: str = 'trajectory_tracker_node'
    active_plan_id: int = 0
    pending_plan_id: int = 0
    receiver_boot_id: str = ''
    planner_boot_id: str = ''


def curve_errors(c, limits=None):
    """Fail closed on finite inputs whose coefficient arithmetic cannot be validated."""
    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            return _curve_errors(c, limits)
    except (ValueError, TypeError, OverflowError, FloatingPointError, np.linalg.LinAlgError):
        return ['INVALID_COEFFICIENTS']


def _curve_errors(c, limits=None):
    """Independent coefficient/time/analytic dynamics checks; no sampled-FOV safety claim."""
    reasons = []
    stamps = (c.navigation_stamp, c.attitude_stamp, c.observation_stamp, c.source_stamp,
              c.input_until, c.coverage_until, c.start, c.end)
    n = len(c.durations)
    a = np.asarray(c.xyz, dtype=float)
    y = np.asarray(c.yaw, dtype=float)
    ids = (c.plan_id, c.mission_id, c.generation, c.prediction_id, c.parent_id)
    if (n < 1 or n > 16 or a.shape != (n*6, 3) or y.shape != (n, 4)
            or not np.all(np.isfinite(a)) or not np.all(np.isfinite(y))
            or not all(math.isfinite(v) and v > 0 for v in (*stamps, *c.durations))
            or not math.isfinite(c.holding_until) or c.holding_until < 0
            or min(ids[:2]) < 1 or min(ids[2:]) < 0):
        return ['INVALID_COEFFICIENTS']
    if (not math.isclose(sum(c.durations), c.end-c.start, abs_tol=1e-7)
            or c.source_stamp > c.start or c.observation_stamp > c.source_stamp+1e-7
            or abs(c.navigation_stamp-c.attitude_stamp) > 1e-7):
        reasons.append('INVALID_INTERVAL')
    raw_until = min(c.navigation_stamp+.125, c.attitude_stamp+.125,
                    c.observation_stamp+.125, c.source_stamp+.125)
    if c.input_until > raw_until+1e-7:
        reasons.append('TTL_EXTENSION')
    if c.holding_until < c.end or not c.holding_model:
        reasons.append('HOLDING_DOES_NOT_COVER')
    if c.coverage_until < c.end or c.holding_until > c.coverage_until:
        reasons.append('PREDICTION_COVERAGE')
    if (c.frame != 'local_ned' or c.constraint_version != 'constraints-p4-v1'
            or c.camera_version != 'camera-p1-v1'):
        reasons.append('FRAME_OR_VERSION')
    if limits is not None and (c.constraint_fingerprint != limits.fingerprint
                               or c.constraint_snapshot != limits.payload):
        reasons.append('CONSTRAINT_MISMATCH')
    policy = limits.values['mpc'] if limits is not None else {}
    coefficients = a.reshape(n, 6, 3)
    for derivative, axes, limit in (
            (1, (0, 1), policy.get('maximum_horizontal_speed', 6.2)),
            (1, (2,), policy.get('maximum_vertical_speed', 4.)),
            (2, (0, 1), policy.get('maximum_horizontal_acceleration', 3.)),
            (2, (2,), policy.get('maximum_vertical_acceleration', 3.)),
            (3, (0, 1), policy.get('maximum_horizontal_jerk', 6.)),
            (3, (2,), policy.get('maximum_vertical_jerk', 4.))):
        if derivative_peak(coefficients, c.durations, derivative, axes)['value'] > limit+1e-6:
            reasons.append('DYNAMIC_LIMIT')
    for i, duration in enumerate(c.durations):
        yc = y[i]
        queries = [0., duration]
        if abs(yc[3]) > 1e-12 and 0 < -yc[2]/(3*yc[3]) < duration:
            queries.append(-yc[2]/(3*yc[3]))
        if np.max(np.abs(np.polynomial.polynomial.polyval(
                queries, np.polynomial.polynomial.polyder(yc)))) > (
                    policy.get('maximum_yaw_rate', 1.)+1e-7):
            reasons.append('YAW_RATE_LIMIT')
        if i < n-1:
            for d in range(3):
                polynomial = np.polynomial.polynomial.polyder(coefficients[i], m=d, axis=0)
                left = np.polynomial.polynomial.polyval(duration, polynomial)
                right = np.polynomial.polynomial.polyder(coefficients[i+1], m=d, axis=0)[0]
                if np.max(np.abs(left-right)) > 1e-6:
                    reasons.append('INNER_PVA_DISCONTINUITY')
            for d in range(2):
                polynomial = np.polynomial.polynomial.polyder(yc, m=d)
                left = np.polynomial.polynomial.polyval(duration, polynomial)
                right = np.polynomial.polynomial.polyder(y[i+1], m=d)[0]
                delta = math.atan2(math.sin(left-right), math.cos(left-right)) if d == 0 else (
                    left-right)
                if abs(delta) > 1e-6:
                    reasons.append('INNER_YAW_DISCONTINUITY')
    return list(dict.fromkeys(reasons))


class FollowReceiver:
    """Latest pending, old active retained on rejection, bounded nonrenewable authorization."""

    def __init__(self, safety_validator=None, bridge=None, limits=None):
        """Require independent holding and first bridge proofs; default to fail closed."""
        self.safety_validator, self.bridge = safety_validator, bridge
        self.limits = limits
        self.pending, self.active, self.seen, self.last_now = None, None, set(), None
        self.planner_boot, self.retired_planners = '', set()

    def set_limits(self, limits):
        """Only a receiver-local configuration change can revoke/update the envelope."""
        if self.limits != limits:
            self.active, self.pending = None, None
            self.limits = limits

    def _ack(self, c, ctx, state, reasons=()):
        return FollowAck(c.plan_id, c.mission_id, c.generation, ctx.now, state, tuple(reasons),
                         active_plan_id=self.active.plan_id if self.active else 0,
                         pending_plan_id=self.pending.plan_id if self.pending else 0,
                         receiver_boot_id=ctx.receiver_boot_id, planner_boot_id=c.planner_boot_id)

    @staticmethod
    def context_errors(c, ctx):
        """Mission, mode, target and latest version revoke even previously checked curves."""
        return [reason for condition, reason in (
            (not math.isfinite(ctx.now) or ctx.now <= 0, 'INVALID_CLOCK'),
            (not ctx.following, 'NOT_FOLLOW'), (not ctx.offboard, 'OFFBOARD_UNAVAILABLE'),
            (not ctx.armed, 'NOT_ARMED'), (not ctx.visible, 'TARGET_NOT_LOCKED'),
            (c.mission_id != ctx.mission_id, 'MISSION_CHANGED'),
            (c.generation != ctx.generation, 'CLOCK_GENERATION_CHANGED'),
            (c.receiver_boot_id != ctx.receiver_boot_id, 'RECEIVER_RESTARTED'),
            (c.prediction_id != ctx.prediction_id, 'PREDICTION_CHANGED')) if condition]

    def propose(self, c, ctx):
        """Receiver validation cannot trust planner validation flags, TTLs or model strings."""
        errors = curve_errors(c, self.limits)+self.context_errors(c, ctx)
        if c.planner_boot_id in self.retired_planners:
            errors.append('PLANNER_BOOT_REPLAY')
        elif (c.planner_boot_id and c.planner_boot_id != self.planner_boot
              and not self.context_errors(c, ctx)
              and not any(e != 'HOLDING_DOES_NOT_COVER' for e in errors)
              and max(c.navigation_stamp, c.attitude_stamp, c.observation_stamp, c.source_stamp)
              <= ctx.now < min(c.input_until, c.start)):
            if self.planner_boot:
                self.retired_planners.add(self.planner_boot)
                self.active, self.pending = None, None
                errors.append('PLANNER_RESTARTED')
            self.planner_boot = c.planner_boot_id
        key = (c.receiver_boot_id, c.planner_boot_id, c.mission_id, c.generation, c.plan_id)
        if key in self.seen:
            errors.append('PLAN_ID_REPLAY')
        self.seen.add(key)
        latest_acquisition = max(c.navigation_stamp, c.attitude_stamp,
                                 c.observation_stamp, c.source_stamp)
        if latest_acquisition > ctx.now:
            errors.append('FUTURE_INPUT')
        if ctx.now >= c.input_until:
            errors.append('INPUT_EXPIRED')
        if ctx.now >= c.start:
            errors.append('START_MISSED')
        if self.last_now is not None and ctx.now < self.last_now:
            errors.append('CLOCK_REVERSED')
        if self.safety_validator is None:
            errors.append('NO_QUALIFIED_HOLDING_MODEL')
        elif not errors:
            try:
                approval = self.safety_validator(c, ctx)
                if (not isinstance(approval, SafetyApproval) or not approval.evidence_id
                        or not math.isfinite(approval.valid_until)
                        or approval.fingerprint != c.fingerprint()
                        or approval.valid_until < c.holding_until):
                    errors.append('SAFETY_APPROVAL_MISMATCH')
            except (ValueError, TypeError, OverflowError):
                errors.append('SAFETY_VALIDATION_FAILED')
        old = self.active
        if old is None and self.bridge is None:
            errors.append('INITIAL_BRIDGE_UNAVAILABLE')
        elif not errors:
            try:
                if old is not None:
                    if c.parent_id != old.plan_id or c.start >= old.holding_until:
                        raise ValueError('PARENT_REFERENCE_INVALID')
                    boundary = old.sample(c.start)
                else:
                    if c.parent_id != 0:
                        raise ValueError('PARENT_REFERENCE_INVALID')
                    boundary = self.bridge(ctx, c.start)
                delta = np.asarray(c.sample(c.start))-np.asarray(boundary)
                if delta.shape != (11,) or not np.all(np.isfinite(delta)):
                    raise ValueError('INVALID_BOUNDARY')
                delta[9] = math.atan2(math.sin(delta[9]), math.cos(delta[9]))
                if np.max(np.abs(delta)) > 1e-6:
                    errors.append('PVA_YAW_DISCONTINUITY')
            except (ValueError, TypeError):
                errors.append('PARENT_REFERENCE_INVALID')
        if errors:
            return self._ack(c, ctx, 'REJECTED', errors)
        if self.pending and c.start < self.pending.start:
            return self._ack(c, ctx, 'REJECTED', ('PENDING_ORDER_CONFLICT',))
        self.pending = c
        return self._ack(c, ctx, 'ACCEPTED')

    def tick(self, ctx):
        """Activation rechecks latest version; loss/reset revoke; no failure extends expiry."""
        result = None
        reversed_clock = self.last_now is not None and ctx.now < self.last_now
        self.last_now = ctx.now
        for slot in ('pending', 'active'):
            c = getattr(self, slot)
            if c is None:
                continue
            errors = self.context_errors(c, ctx)
            if self.limits is not None and c.constraint_fingerprint != self.limits.fingerprint:
                errors.append('CONSTRAINT_MISMATCH')
            if reversed_clock:
                errors.append('CLOCK_REVERSED')
            if errors or ctx.now >= min(c.holding_until, c.end):
                setattr(self, slot, None)
                result = self._ack(c, ctx, 'REVOKED' if errors else 'EXPIRED', errors)
        if self.pending and ctx.now >= self.pending.start:
            c = self.pending
            # A missed exact switch is rejected; no reference clamping fabricates handover.
            if ctx.now-c.start > 1e-6:
                self.pending = None
                return self._ack(c, ctx, 'REJECTED', ('START_MISSED',))
            self.active, self.pending = c, None
            result = self._ack(c, ctx, 'ACTIVE')
        return result
