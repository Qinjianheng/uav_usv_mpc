"""P4.3 independent full nominal admission, with target-independent invariant reuse."""
from collections import OrderedDict
import hashlib
import time

import numpy as np

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.guidance.follow_limits import constraint_snapshot
from uav_control.guidance.follow_attitude_bounds import attitude_extrema
from uav_control.guidance.follow_minco_optimizer import _trajectory, _junction_residual
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_profile import profiled, count
from uav_control.guidance.polynomial_bounds import derivative_upper_bound
from uav_control.guidance.polynomial_extrema import derivative_peak
from uav_control.guidance.yaw_trajectory import YawTrajectory

DERIVATIVES = (('horizontal_speed', 1, (0, 1)), ('vertical_speed', 1, (2,)),
               ('horizontal_acceleration', 2, (0, 1)), ('vertical_acceleration', 2, (2,)),
               ('horizontal_jerk', 3, (0, 1)), ('vertical_jerk', 3, (2,)))


@profiled('trajectory_samples')
def fields(xyz, durations, yaw, query):
    """Vectorized Horner P/V/A/J and yaw/rate; no extrapolation."""
    ts, query = np.asarray(durations), np.asarray(query)
    if np.any(query < -1e-9) or np.any(query > sum(ts)+1e-9):
        raise ValueError('REFERENCE_OUTSIDE')
    knots = np.r_[0., np.cumsum(ts)]
    index = np.minimum(np.searchsorted(knots[1:], query, side='right'), len(ts)-1)
    local = query-knots[index]
    output = []
    for derivative in range(4):
        coefficients = np.polynomial.polynomial.polyder(xyz, derivative, axis=1)[index]
        value = np.zeros((len(query), 3))
        for k in range(coefficients.shape[1]-1, -1, -1):
            value = value*local[:, None]+coefficients[:, k]
        output.append(value)
    for derivative in range(2):
        coefficients = np.polynomial.polynomial.polyder(yaw, derivative, axis=1)[index]
        value = np.zeros(len(query))
        for k in range(coefficients.shape[1]-1, -1, -1):
            value = value*local+coefficients[:, k]
        output.append(value)
    return tuple(output)


class InvariantCache:
    """Eight immutable mathematical results; no target/FOV/TTL/approval is stored."""

    def __init__(self, capacity=8):
        self.capacity, self.entries = capacity, OrderedDict()

    attitude_check = staticmethod(attitude_extrema)

    def key(self, request, metrics, model):
        """Bind full coefficients/time/yaw, calibration, policy and local mission/epoch."""
        h = hashlib.sha256()
        for name in ('xyz_coefficients', 'durations', 'yaw_coefficients'):
            array = np.asarray(metrics[name], dtype='<f8')
            h.update(str(array.shape).encode())
            h.update(array.tobytes())
        h.update(constraint_snapshot(model).payload.encode())
        h.update(str((request.context.mission_id, request.context.clock_generation)).encode())
        return h.hexdigest()

    def obtain(self, request, metrics, model):
        """Recalculate peaks on any key change; prediction-independent does not mean approved."""
        key = self.key(request, metrics, model)
        if key in self.entries:
            self.entries.move_to_end(key)
            count('invariant_hits')
            return self.entries[key], True
        ts, xyz = np.array(metrics['durations']), np.array(metrics['xyz_coefficients'])
        yaw = np.array(metrics['yaw_coefficients'])[::-1].T
        if (ts.ndim != 1 or not 0 < len(ts) <= 16 or xyz.shape != (len(ts), 6, 3)
                or yaw.shape != (len(ts), 4) or np.any(ts <= 0)
                or not np.all(np.isfinite(np.r_[ts, xyz.ravel(), yaw.ravel()]))):
            raise ValueError('INVALID_COEFFICIENTS')
        peaks = {}
        for name, derivative, axes in DERIVATIVES:
            upper = derivative_upper_bound(xyz, ts, derivative, axes)
            limit = getattr(model.config, 'maximum_'+name)
            peaks[name] = (upper if upper <= limit else
                           derivative_peak(xyz, ts, derivative, axes)['value'])
            if peaks[name] > limit+1e-6:
                raise ValueError('DYNAMIC_INFEASIBLE')
        maximum_rate = 0.
        for yc, duration in zip(yaw, ts):
            times = [0., duration]
            if abs(yc[3]) > 1e-12 and 0 < -yc[2]/(3*yc[3]) < duration:
                times.append(-yc[2]/(3*yc[3]))
            maximum_rate = max(maximum_rate, float(np.max(np.abs(
                np.polynomial.polynomial.polyval(times, np.polynomial.polynomial.polyder(yc))))))
        if maximum_rate > model.config.maximum_yaw_rate+1e-6:
            raise ValueError('YAW_RATE_LIMIT')
        attitude = self.attitude_check(xyz, ts, model.attitude_config)
        if not attitude['valid']:
            raise ValueError('ATTITUDE_LIMIT')
        # Arrays are private immutable copies; no caller output dictionary can mutate them.
        for value in (ts, xyz, yaw):
            value.setflags(write=False)
        entry = dict(durations=ts, xyz=xyz, yaw=yaw, peaks=peaks,
                     maximum_rate=maximum_rate, attitude=attitude)
        self.entries[key] = entry
        while len(self.entries) > self.capacity:
            self.entries.popitem(last=False)
        return entry, False


@profiled('full_validation')
def assess_curve(request, metrics, model, cache=None, budget=.05, clock=time.perf_counter):
    """Re-evaluate current target, planned attitude, whole sphere, range and sea."""
    started = clock()
    cache = cache if cache is not None else InvariantCache()
    invariant, reused = cache.obtain(request, metrics, model)
    ts, xyz, yaw = (invariant[k] for k in ('durations', 'xyz', 'yaw'))
    problem = FollowProblem(request, model, float(sum(ts)))
    knots = np.r_[0., np.cumsum(ts)]
    query = np.unique(np.r_[knots, np.linspace(0., sum(ts), int(np.ceil(sum(ts)/.05))+1)])
    for iteration in range(3):
        if clock()-started >= budget:
            raise ValueError('REVALIDATION_DEADLINE')
        samples = fields(xyz, ts, yaw, query)
        margins, views, geometry, tilts = problem.assess(query, *samples, batch=True)
        count('full_validation_samples', len(query))
        if iteration == 2:
            break
        close = ((geometry.min(axis=1) < .08)
                 | (np.vstack(tuple(margins.values())).min(axis=0) < .1))
        intervals = close[:-1] | close[1:]
        if not intervals.any():
            break
        query = np.unique(np.r_[query, ((query[:-1]+query[1:])/2)[intervals]])
    violations = {k: max(0., float(-np.min(v))) for k, v in margins.items()}
    violations['visibility'] = max(0., float(-geometry.min()))
    valid = (all(view.whole_target_safe for view in views)
             and all(v <= 1e-6 for v in violations.values()))
    if clock()-started >= budget:
        raise ValueError('REVALIDATION_DEADLINE')
    report = dict(valid=valid, holding_qualified=False, samples=len(query),
                  reason='' if valid else 'PREDICTION_REVALIDATION_FAILED',
                  invariants_reused=reused, analytic_peaks=invariant['peaks'],
                  minimum_horizontal_margin=float(geometry[:, 0].min()),
                  minimum_vertical_margin=float(geometry[:, 1].min()),
                  elapsed=clock()-started)
    return report, (query, samples, margins, views, geometry, tilts, violations, invariant)


def validate_seed(request, seed, model, cache=None, budget=.05, clock=time.perf_counter):
    """Construct MINCO once then run independent full validation, never proxy admission."""
    started = clock()
    try:
        if (not seed.valid_input or seed.context != request.context
                or len(seed.start) != 10 or len(seed.end) != 9
                or np.asarray(seed.q).shape != (2, 3) or len(seed.durations) != 3
                or len(seed.yaw) != 4
                or not np.allclose(seed.start, request.state, rtol=0., atol=1e-9)):
            raise ValueError('INVALID_MINCO_SEED')
        curve = _trajectory(seed, seed.q, seed.durations)
        yaw = YawTrajectory(np.r_[0., np.cumsum(seed.durations)], seed.yaw,
                            start_rate=getattr(request, 'reference_yaw_rate', 0.))
        metrics = dict(q=seed.q, durations=seed.durations, yaw_knots=tuple(yaw.values),
                       xyz_coefficients=curve.coefficients.tolist(),
                       yaw_coefficients=yaw.spline.c.tolist(), end=seed.end)
        report, assessed = assess_curve(request, metrics, model, cache,
                                        budget-(clock()-started), clock)
        query, samples, margins, views, geometry, tilts, violations, invariant = assessed
        p, v, a, j, angles, rates = samples
        residual = max(float(np.max(np.abs(np.r_[p[0], v[0], a[0]]-seed.start[:9]))),
                       float(np.max(np.abs(np.r_[p[-1], v[-1], a[-1]]-seed.end))))
        junction = _junction_residual(curve)
        valid = report['valid'] and residual < 1e-7 and junction < 1e-5
        metrics.update(full_projection_validated=True, sampled_feasible=valid,
                       continuous_time_guarantee=False, boundary_residual=residual,
                       junction_residual=junction, validation_samples=len(query),
                       jerk_integral=curve.control_effort(), **{
                           'maximum_'+k: value for k, value in invariant['peaks'].items()},
                       minimum_horizontal_margin=report['minimum_horizontal_margin'],
                       minimum_vertical_margin=report['minimum_vertical_margin'],
                       maximum_yaw_rate=invariant['maximum_rate'],
                       continuous_attitude_extrema=invariant['attitude'],
                       maximum_tilt_rad=float(np.max(tilts)),
                       maximum_specific_thrust=float(np.max(np.linalg.norm(
                           np.array((0., 0., model.attitude_config.gravity))-a, axis=1))),
                       visible_fraction=float(np.mean([x.whole_target_safe for x in views])))
        status = 'SAMPLED_FEASIBLE' if valid else (
            'DYNAMIC_INFEASIBLE' if any(
                v > 1e-6 for k, v in violations.items() if k != 'visibility')
            else 'VISIBILITY_INFEASIBLE')
        return MpcSeedResult(request.context, valid, status, 'nominal full sampled check',
                             tuple(query), tuple(map(tuple, p)), tuple(map(tuple, v)),
                             tuple(map(tuple, a)), tuple(map(tuple, j)),
                             tuple(angles), tuple(rates),
                             tuple(map(tuple, geometry[:, :2])), clock()-started,
                             dict(validation=clock()-started), violations, metrics,
                             initial_visibility=views[0].whole_target_safe,
                             candidate_kind='p43_follow_minco')
    except (ValueError, TypeError, OverflowError, FloatingPointError, np.linalg.LinAlgError) as e:
        return MpcSeedResult(request.context, solver_status=str(e), reason=str(e),
                             solve_time=clock()-started)
