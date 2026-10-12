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

    def __init__(self, limits=None):
        """Start without an executable curve or an independently qualified holding window."""
        super().__init__(limits=limits)
        self.deadline = 0.

    def propose(self, curve, ctx, validated_prediction_id=None):
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
        if errors:
            return self._ack(curve, ctx, 'REJECTED', errors)
        self.pending = curve
        return self._ack(curve, ctx, 'ACCEPTED', ('NOMINAL_EXECUTION',))

    def tick(self, ctx, nominal_valid=False):
        """Require a local prefix check for new predictions, without deadline renewal."""
        reversed_clock = self.last_now is not None and ctx.now < self.last_now
        self.last_now = ctx.now
        result = None
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
                result = self._ack(curve, ctx, 'REVOKED' if errors else 'EXPIRED', errors)
        if self.pending and ctx.now >= self.pending.start:
            curve = self.pending
            if ctx.now-curve.start > self.activation_window+1e-7:
                self.pending = None
                return self._ack(curve, ctx, 'REJECTED', ('START_MISSED',))
            self.active, self.pending = curve, None
            self.deadline = min(curve.end, curve.start+self.lease_seconds)
            result = self._ack(curve, ctx, 'ACTIVE', ('NOMINAL_EXECUTION',))
        return result


def nominal_attitude(curve, model):
    """Check continuous ideal thrust and tilt from coefficients, independently of the planner."""
    return attitude_extrema(np.asarray(curve.xyz).reshape(-1, 6, 3), curve.durations,
                            model.attitude_config)['valid']
