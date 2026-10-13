"""
Execute nominal rolling MINCO without claiming holding qualification or SafetyApproval.

The short execution lease is an experimental watchdog, not a physical error bound.
Qualified physical holding remains separate from this measured-feedback controller.
"""
from dataclasses import dataclass
import math

import numpy as np

from uav_control.controllers.follow_mpc_seed import PlanningContext
from uav_control.guidance.camera_visibility_batch import attitude_batch, visibility_batch
from uav_control.guidance.follow_attitude_bounds import attitude_extrema
from uav_control.guidance.follow_contract import FollowCurve, FollowReceiver, curve_errors
from uav_control.guidance.follow_fast_validation import fields
from uav_control.guidance.follow_problem import FutureRequest


@dataclass(frozen=True)
class AcceptedFollowRequest(FutureRequest):
    """Tracker-ACKed reference at handover, separate from measured P/V/A and ideal rollouts."""

    prior_curve: object = None
    reference_yaw_rate: float = 0.
    boundary_policy: str = 'tracker_accepted_reference'


def accepted_request(request, curve):
    """Keep input epochs, replacing only the execution boundary with an ACKed reference."""
    sample = curve.sample(request.context.execution_start_stamp)
    payload = dict(request.__dict__)
    payload.update(state=tuple(sample[:10]), prior_curve=curve,
                   reference_yaw_rate=sample[10],
                   boundary_policy='tracker_accepted_reference')
    return AcceptedFollowRequest(**payload)


def request_from_payload(payload):
    """Restore reference provenance after JSON logging and new-prediction revalidation."""
    values = dict(payload, context=PlanningContext(**payload['context']))
    if values.get('boundary_policy') == 'tracker_accepted_reference':
        values['prior_curve'] = FollowCurve(**values['prior_curve'])
        return AcceptedFollowRequest(**values)
    return FutureRequest(**values)


def reference_velocity(curve, now, navigation_stamp, position, gain):
    """Compare position at the same physical sample epoch; feedforward uses publication time."""
    reference_epoch = max(navigation_stamp, curve.start)
    if reference_epoch > now:
        raise ValueError('FUTURE_NAVIGATION')
    reference = curve.sample(reference_epoch)
    current = curve.sample(now)
    return tuple(np.asarray(current[3:6])+gain*(np.asarray(reference[:3])-position))


def reference_acceleration(curve, now, navigation_stamp, velocity, gain, horizontal_limit):
    """Differentiate nominal velocity feedback; vertical sea-guard authority stays separate."""
    epoch = max(navigation_stamp, curve.start)
    measured = np.asarray(velocity, dtype=float)
    if (not curve.start <= epoch <= now <= curve.end or measured.shape != (3,)
            or not np.all(np.isfinite(measured)) or not math.isfinite(gain) or gain < 0
            or not math.isfinite(horizontal_limit) or horizontal_limit <= 0):
        raise ValueError('INVALID_ACCELERATION_FEEDFORWARD_INPUT')
    reference, current = curve.sample(epoch), curve.sample(now)
    horizontal = np.asarray(current[6:8])+gain*(np.asarray(reference[3:5])-measured[:2])
    if not np.all(np.isfinite(horizontal)):
        raise ValueError('INVALID_ACCELERATION_FEEDFORWARD')
    norm = float(np.linalg.norm(horizontal))
    if norm > horizontal_limit:
        horizontal *= horizontal_limit/norm
    return float(horizontal[0]), float(horizontal[1]), math.nan


def shaped_acceleration(raw, desired, emitted, previous, dt, horizontal_limit, speed_limit):
    """Differentiate the emitted velocity when limited; suppress outward FF at the speed cap."""
    acceleration = np.asarray(raw[:2], dtype=float)
    desired_xy, emitted_xy = np.asarray(desired[:2]), np.asarray(emitted[:2])
    if (not all(np.all(np.isfinite(v)) for v in (acceleration, desired_xy, emitted_xy))
            or not math.isfinite(dt) or dt <= 0
            or not math.isfinite(horizontal_limit) or horizontal_limit <= 0
            or not math.isfinite(speed_limit) or speed_limit <= 0):
        raise ValueError('INVALID_SHAPED_ACCELERATION')
    if not np.allclose(desired_xy, emitted_xy, rtol=0., atol=1e-9):
        acceleration = (np.zeros(2) if previous is None else
                        (emitted_xy-np.asarray(previous[:2]))/dt)
    if not np.all(np.isfinite(acceleration)):
        raise ValueError('INVALID_SHAPED_ACCELERATION')
    speed = float(np.linalg.norm(emitted_xy))
    if speed >= speed_limit-1e-9:
        direction = emitted_xy/speed
        acceleration -= max(0., float(acceleration@direction))*direction
    norm = float(np.linalg.norm(acceleration))
    if norm > horizontal_limit:
        acceleration *= horizontal_limit/norm
    return float(acceleration[0]), float(acceleration[1]), math.nan


def nominal_curve(curve, prediction, model, start, end, diagnostics=None):
    """Receiver-local full sphere/range, ideal attitude and sea checks, never a robust proof."""
    detail = diagnostics if diagnostics is not None else {}
    detail['reason'] = 'INVALID_INTERVAL_OR_INPUT'
    try:
        if not curve.start <= start < end <= curve.end:
            return False
        query = np.linspace(start, end, int(math.ceil((end-start)/.025))+1)
        target_times = query-prediction.source_stamp
        ts = np.asarray(prediction.prediction_times)
        targets = np.asarray(prediction.target_positions)
        if (target_times.min() < ts[0]-1e-8 or target_times.max() > ts[-1]+1e-8
                or not np.all(np.isfinite(targets))):
            return False
        target = np.column_stack([np.interp(target_times, ts, targets[:, i]) for i in range(3)])
        xyz = np.asarray(curve.xyz).reshape(-1, 6, 3)
        # Large absolute ROS epochs lose ~1e-7 s when subtracting the encoded end.
        # Stay inside declared support and remove only representational endpoint error.
        if abs((curve.end-curve.start)-sum(curve.durations)) > 2*math.ulp(curve.end)+1e-7:
            return False
        local = np.clip(query-curve.start, 0., sum(curve.durations))
        samples = fields(xyz, curve.durations, np.asarray(curve.yaw), local)
        p, v, a, j, yaw, rate = samples
        attitude = attitude_batch(a, yaw, model.attitude_config)
        if not all(value.valid for value in attitude):
            return False
        rotations = np.asarray([value.rotation_frd_to_ned for value in attitude])
        views = visibility_batch(p, rotations, target, model.intrinsics, model.extrinsics,
                                 model.target, model.visibility)
        c = model.config
        descent = np.maximum(v[:, 2], 0.)
        clearance = (c.sea_surface_z-c.reserve_clearance-p[:, 2]-descent*c.response_delay
                     - descent**2/(2*c.braking_acceleration))
        detail.update(horizontal_margin=min(v.horizontal_margin_rad for v in views),
                      vertical_margin=min(v.vertical_margin_rad for v in views),
                      sea_margin=float(clearance.min()),
                      reason=('VALID' if all(v.whole_target_safe for v in views)
                              and clearance.min() >= 0 else 'FOV_OR_SEA'))
        return detail['reason'] == 'VALID'
    except (ValueError, TypeError, IndexError, OverflowError, FloatingPointError):
        return False


class NominalFollowReceiver(FollowReceiver):
    """Separate nominal lease lifecycle; never invokes or fabricates independent approval."""

    lease_seconds = .45
    activation_window = .05

    def __init__(self, limits=None, admission=None):
        """Start without an executable curve or an independently qualified holding window."""
        super().__init__(limits=limits)
        self.deadline = 0.
        self.admission, self.events = admission, []
        self.entry_evidence = {}

    def drain_events(self):
        """Return every tick ACK, including simultaneous pending and active failures."""
        result, self.events = self.events, []
        return result

    def propose(self, curve, ctx, validated_prediction_id=None, probe=False):
        """Admit a locally revalidated nominal curve; retain the old lease on rejection."""
        errors = [e for e in curve_errors(curve, self.limits) if e != 'HOLDING_DOES_NOT_COVER']
        errors += [e for e in self.context_errors(curve, ctx)
                   if not (e == 'PREDICTION_CHANGED'
                           and validated_prediction_id == ctx.prediction_id)]
        if curve.holding_until != 0. or curve.holding_model:
            errors.append('UNEXPECTED_HOLDING_CLAIM')
        if not curve.planner_boot_id or not curve.receiver_boot_id:
            errors.append('MISSING_BOOT_IDENTITY')
        key = (curve.receiver_boot_id, curve.planner_boot_id, curve.mission_id,
               curve.generation, curve.plan_id)
        if key in self.seen:
            errors.append('PLAN_ID_REPLAY')
        self.seen.add(key)
        if max(curve.navigation_stamp, curve.attitude_stamp, curve.observation_stamp,
               curve.source_stamp) > ctx.now:
            errors.append('FUTURE_INPUT')
        if ctx.now >= curve.input_until:
            errors.append('INPUT_EXPIRED')
        if ctx.now >= curve.start:
            errors.append('START_MISSED')
        if self.last_now is not None and ctx.now < self.last_now:
            errors.append('CLOCK_REVERSED')
        if self.active:
            old = self.active
            if (curve.parent_id != old.plan_id or curve.start >= self.deadline
                    or curve.planner_boot_id != old.planner_boot_id):
                errors.append('PARENT_REFERENCE_INVALID')
            elif not errors:
                delta = np.asarray(curve.sample(curve.start))-old.sample(curve.start)
                delta[9] = math.atan2(math.sin(delta[9]), math.cos(delta[9]))
                if np.max(np.abs(delta)) > 1e-6:
                    errors.append('PVA_YAW_DISCONTINUITY')
        elif curve.parent_id:
            errors.append('PARENT_REFERENCE_INVALID')
        if self.pending and curve.start < self.pending.start:
            errors.append('PENDING_ORDER_CONFLICT')
        if probe:
            if self.admission is not None:
                self.admission.probe(ctx.now, key, not errors)
            return self._ack(curve, ctx, 'REJECTED', errors or ('SHADOW_PROBE_VALID',))
        if self.active is None and self.admission is not None:
            if not self.admission.allowed(ctx.now):
                errors.append(self.admission.inhibition_reason(ctx.now))
            if self.pending:
                errors.append('INITIAL_PENDING_EXISTS')
        if errors:
            return self._ack(curve, ctx, 'REJECTED', errors)
        if self.active is None and self.admission is not None:
            gate = self.admission
            self.entry_evidence = dict(
                plan_id=curve.plan_id, stamp=ctx.now, stable_since=gate.stable_since,
                recovery_until=max(0., gate.recover_until), valid_probe_count=len(gate.probes),
                last_probe=gate.last_probe, entry_attempt=gate.entries+1, loss=gate.loss)
            self.admission.consume()
        self.pending = curve
        return self._ack(curve, ctx, 'ACCEPTED', ('NOMINAL_EXECUTION',))

    def tick(self, ctx, nominal_valid=False):
        """Require a local prefix check for new predictions, without deadline renewal."""
        reversed_clock = self.last_now is not None and ctx.now < self.last_now
        self.last_now = ctx.now
        result = None
        self.events = []
        previous_active, previous_pending = self.active, self.pending

        def emit(curve, state, reasons=()):
            ack = self._ack(curve, ctx, state, reasons)
            self.events.append(ack)
            return ack

        for slot in ('pending', 'active'):
            curve = getattr(self, slot)
            if curve is None:
                continue
            errors = [e for e in self.context_errors(curve, ctx) if e != 'PREDICTION_CHANGED']
            if reversed_clock:
                errors.append('CLOCK_REVERSED')
            checked = (nominal_valid.get(curve.plan_id, False)
                       if isinstance(nominal_valid, dict) else nominal_valid)
            until = min(curve.end, curve.start+self.lease_seconds)
            if not checked and ctx.now < until:
                errors.append('LATEST_PREFIX_INVALID')
            if errors or ctx.now >= until:
                setattr(self, slot, None)
                result = emit(curve, 'REVOKED' if errors else 'EXPIRED', errors)
        if previous_active and self.active is None and self.pending and self.pending.parent_id:
            child, self.pending = self.pending, None
            reason = ('PARENT_EXPIRED_AT_ACTIVATION' if ctx.now >= child.start
                      else 'PARENT_REFERENCE_LOST')
            result = emit(child, 'REJECTED', (reason,))
        if self.pending and ctx.now >= self.pending.start:
            curve = self.pending
            reason = ''
            if curve.parent_id and (self.active is None
                                    or curve.parent_id != self.active.plan_id):
                reason = 'PARENT_EXPIRED_AT_ACTIVATION'
            elif ctx.now-curve.start > self.activation_window+1e-7:
                reason = 'START_MISSED'
            elif (self.active is None and self.admission is not None
                  and not self.admission.converged(ctx.now)):
                reason = 'FOLLOW_ENTRY_INHIBITED'
            if reason:
                self.pending = None
                result = emit(curve, 'REJECTED', (reason,))
            else:
                self.active, self.pending = curve, None
                self.deadline = min(curve.end, curve.start+self.lease_seconds)
                result = emit(curve, 'ACTIVE', ('NOMINAL_EXECUTION',))
        lost = previous_active or previous_pending
        if lost is not None and self.active is None and self.pending is None and self.admission:
            event = next((a for a in self.events if a.plan_id == lost.plan_id), result)
            reason = ('LEASE_EXPIRED_NO_SUCCESSOR' if event.state == 'EXPIRED'
                      else ','.join(event.reasons))
            self.admission.lost(ctx.now, reason, lost)
        return result


def nominal_attitude(curve, model):
    """Check continuous ideal thrust and tilt from coefficients, independently of the planner."""
    return attitude_extrema(np.asarray(curve.xyz).reshape(-1, 6, 3), curve.durations,
                            model.attitude_config)['valid']
