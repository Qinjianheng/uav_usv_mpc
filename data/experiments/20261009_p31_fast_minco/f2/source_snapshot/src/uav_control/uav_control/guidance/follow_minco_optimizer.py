"""
Visibility-aware research Q/T/yaw optimization using the existing MINCO map.

Optimizer convergence, sampled feasibility, and live research admission are
separate. Dense/adaptive sampling is not a continuous-time safety proof.
"""

from dataclasses import dataclass, replace
import math
import time

import numpy as np
from scipy.optimize import minimize

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.minco_trajectory import MincoS3Trajectory
from uav_control.guidance.yaw_trajectory import YawTrajectory


@dataclass(frozen=True)
class MincoConfig:
    """Small research optimization, with fixed total horizon and variable piece times."""

    mode: str = 'qt'
    maximum_iterations: int = 2
    budget: float = .5
    visibility_weight: float = 20.
    jerk_weight: float = .003
    follow_weight: float = 1.
    dynamic_weight: float = 100.
    safety_weight: float = 1000.
    yaw_weight: float = .05
    spatial_radius: float = .75
    validation_step: float = .05
    feasible_priority: bool = False
    fast_feasible_seed: bool = False

    def validate(self):
        """Reject invalid dimensions, modes, budgets and negative penalty weights."""
        if (self.mode not in ('fixed', 'q', 'qt') or isinstance(self.maximum_iterations, bool)
                or not isinstance(self.maximum_iterations, int) or self.maximum_iterations < 1
                or not all(math.isfinite(v) and v > 0 for v in (
                    self.budget, self.spatial_radius, self.validation_step))
                or self.validation_step > .1
                or not all(math.isfinite(v) and v >= 0 for v in (
                    self.visibility_weight, self.jerk_weight, self.follow_weight,
                    self.dynamic_weight, self.safety_weight, self.yaw_weight))):
            raise ValueError('INVALID_MINCO_CONFIG')


def _trajectory(seed, q, durations):
    return MincoS3Trajectory(seed.start[:3], seed.start[3:6], seed.start[6:9],
                             seed.end[:3], seed.end[3:6], seed.end[6:9], q, durations)


def _samples(trajectory, times):
    samples = [trajectory.sample(t) for t in times]
    return tuple(np.asarray([getattr(s, name) for s in samples])
                 for name in ('position', 'velocity', 'acceleration', 'jerk'))


def _junction_residual(trajectory):
    worst = 0.
    for i in range(trajectory.piece_count - 1):
        for derivative in range(5):
            left = np.polynomial.polynomial.polyder(trajectory.coefficients[i],
                                                    m=derivative, axis=0)
            right = np.polynomial.polynomial.polyder(trajectory.coefficients[i + 1],
                                                     m=derivative, axis=0)
            value = np.polynomial.polynomial.polyval(trajectory.durations[i], left)
            worst = max(worst, float(np.max(np.abs(value - right[0]))))
    return worst


class _Deadline(Exception):
    pass


class FollowMincoOptimizer:
    """Optimize Q first, then optionally piece-time distribution, with shared yaw variables."""

    def __init__(self, config=MincoConfig(), model=None, clock=time.perf_counter):
        """Keep mathematical dependencies explicit and a bounded per-call wall clock."""
        config.validate()
        self.config, self.model, self.clock = config, model, clock

    def solve(self, request, seed):
        """Optimize real coefficients and reject any unverified or over-budget output."""
        start = self.clock()
        timing = {}
        c = self.config

        def deadline():
            if self.clock() - start >= c.budget:
                raise _Deadline()

        try:
            deadline()
            if not seed.valid_input or seed.context != request.context:
                raise ValueError(seed.reason or 'SEED_CONTEXT_MISMATCH')
            q0 = np.asarray(seed.q, dtype=float)
            durations0 = np.asarray(seed.durations, dtype=float)
            yaw0 = np.asarray(seed.yaw, dtype=float)
            if (q0.shape != (2, 3) or durations0.shape != (3,) or yaw0.shape != (4,)
                    or not np.all(np.isfinite(np.r_[q0.ravel(), durations0, yaw0]))
                    or np.min(durations0) <= 0 or len(seed.start) != 10 or len(seed.end) != 9
                    or not np.allclose(seed.start, request.state, rtol=0., atol=1e-9)):
                raise ValueError('INVALID_MINCO_SEED')
            total = float(sum(durations0))
            problem = FollowProblem(request, self.model, total)
            logits0 = np.log(durations0[:2] / durations0[-1])
            x0 = np.r_[q0.ravel(), logits0 if c.mode == 'qt' else [], yaw0[1:]]
            timing['preparation'] = self.clock() - start
            yaw_time = 0.

            def construct(flat):
                nonlocal yaw_time
                q = flat[:6].reshape(2, 3)
                durations = (MincoS3Trajectory.durations_from_logits(total, flat[6:8])
                             if c.mode == 'qt' else tuple(durations0))
                trajectory = _trajectory(seed, q, durations)
                stamp = self.clock()
                yaw_values = np.r_[yaw0[0], flat[-3:]]
                yaw = YawTrajectory(np.r_[0., np.cumsum(durations)], yaw_values)
                yaw_time += self.clock() - stamp
                return trajectory, yaw, q, durations

            cache = {}

            def evaluate(flat):
                deadline()
                key = flat.tobytes()
                if key in cache:
                    return cache[key]
                trajectory, yaw, _, durations = construct(flat)
                knots = np.r_[0., np.cumsum(durations)]
                times = np.sort(np.r_[knots, (knots[:-1] + knots[1:]) / 2])
                p, v, a, j = _samples(trajectory, times)
                angle, rate = yaw.sample(times)
                margin, _, geometry, _ = problem.assess(times, p, v, a, j, angle, rate)
                reference, _, _ = problem.reference(times)
                dynamic = sum(float(np.mean(np.minimum(m, 0.)**2))
                              for name, m in margin.items() if name != 'sea_clearance')
                safety = float(np.mean(np.minimum(margin['sea_clearance'], 0.)**2))
                visibility = float(np.mean(np.logaddexp(0., -12 * geometry)**2) / 144)
                cost = (c.jerk_weight * trajectory.control_effort()
                        + c.visibility_weight * visibility
                        + c.follow_weight * float(np.mean(np.sum((p - reference)**2, axis=1)))
                        + c.dynamic_weight * dynamic + c.safety_weight * safety
                        + c.yaw_weight * float(np.mean(rate**2)))
                if not math.isfinite(cost):
                    raise ValueError('NONFINITE_OBJECTIVE')
                if len(cache) >= 256:
                    cache.clear()
                cache[key] = cost
                return cost

            initial_validated = None
            if c.mode != 'fixed' and (c.feasible_priority or c.fast_feasible_seed):
                initial_validated = FollowMincoOptimizer(replace(
                    c, mode='fixed', feasible_priority=False, fast_feasible_seed=False,
                    budget=max(1e-6, c.budget-(self.clock()-start))),
                    self.model, self.clock).solve(request, seed)
                deadline()
                if c.fast_feasible_seed and initial_validated.valid:
                    elapsed = self.clock()-start
                    return replace(initial_validated, solve_time=elapsed,
                                   timing=dict(initial_validated.timing, total=elapsed),
                                   metrics=dict(initial_validated.metrics,
                                                optimization_skipped_reason=(
                                                    'VALIDATED_FIXED_SEED')))
            initial_cost = evaluate(x0)
            optimize_start = self.clock()
            solved = None
            flat = x0
            if c.mode != 'fixed':
                bounds = [(float(x - c.spatial_radius), float(x + c.spatial_radius))
                          for x in q0.ravel()]
                if c.mode == 'qt':
                    bounds += [(-1.5, 1.5)] * 2
                bounds += [(float(y - .8), float(y + .8)) for y in yaw0[1:]]
                solved = minimize(evaluate, x0, method='L-BFGS-B', bounds=bounds,
                                  options={'maxiter': c.maximum_iterations, 'ftol': 1e-5,
                                           'maxls': 5})
                if solved.fun < initial_cost:
                    flat = solved.x
            deadline()
            timing['minco_optimization'] = self.clock() - optimize_start
            validation_start = self.clock()
            trajectory, yaw, q, durations = construct(flat)
            knots = np.r_[0., np.cumsum(durations)]
            times = np.unique(np.r_[np.linspace(0., total,
                                                int(math.ceil(total / c.validation_step)) + 1),
                                    knots])

            def assessment(query):
                deadline()
                p, v, a, j = _samples(trajectory, query)
                angle, rate = yaw.sample(query)
                return (p, v, a, j, angle, rate,
                        *problem.assess(query, p, v, a, j, angle, rate))

            assessed = assessment(times)
            # Refine near any tight margin, and where adjacent feasibility differs.
            for _ in range(2):
                margin, views, geometry = assessed[6:9]
                close = np.min(geometry, axis=1) < .08
                close |= np.min(np.vstack(tuple(margin.values())), axis=0) < .1
                intervals = close[:-1] | close[1:]
                if not np.any(intervals):
                    break
                midpoints = (times[:-1] + times[1:])[intervals] / 2
                times = np.unique(np.r_[times, midpoints])
                assessed = assessment(times)
            p, v, a, j, angles, rates, margin, views, geometry, tilts = assessed
            boundary_residual = max(float(np.max(np.abs(np.r_[p[0], v[0], a[0]]
                                                        - np.asarray(seed.start[:9])))),
                                    float(np.max(np.abs(np.r_[p[-1], v[-1], a[-1]]
                                                        - np.asarray(seed.end)))))
            junction_residual = _junction_residual(trajectory)
            violations = {name: max(0., float(-np.min(m))) for name, m in margin.items()}
            violations['yaw_rate'] = max(violations['yaw_rate'],
                                         yaw.maximum_rate() - problem.limits.maximum_yaw_rate)
            violations['visibility'] = max(0., float(-np.min(geometry)))
            dynamic_valid = (all(v <= 1e-6 for k, v in violations.items() if k != 'visibility')
                             and boundary_residual < 1e-7 and junction_residual < 1e-5)
            visible = [view.whole_target_safe for view in views]
            valid = dynamic_valid and all(visible)
            status = 'SAMPLED_FEASIBLE' if valid else 'DYNAMIC_INFEASIBLE'
            if dynamic_valid and not valid:
                status = ('RECOVERY_CANDIDATE' if not visible[0] and all(visible[-3:])
                          else 'VISIBILITY_INFEASIBLE')
            reference, _, _ = problem.reference(times)
            metrics = {
                'q': tuple(map(tuple, q)), 'durations': tuple(durations),
                'yaw_knots': tuple(yaw.values),
                'xyz_coefficients': trajectory.coefficients.tolist(),
                'yaw_coefficients': yaw.spline.c.tolist(),
                'initial_objective': initial_cost, 'final_objective': evaluate(flat),
                'optimizer_success': bool(solved.success) if solved is not None else False,
                'optimizer_status': int(solved.status) if solved is not None else None,
                'optimizer_iterations': int(solved.nit) if solved is not None else 0,
                'sampled_feasible': valid, 'continuous_time_guarantee': False,
                'boundary_residual': boundary_residual, 'junction_residual': junction_residual,
                'validation_samples': len(times), 'jerk_integral': trajectory.control_effort(),
                'visible_fraction': float(np.mean(visible)),
                'minimum_horizontal_margin': float(np.min(geometry[:, 0])),
                'minimum_vertical_margin': float(np.min(geometry[:, 1])),
                'follow_rmse': float(np.sqrt(np.mean(np.sum((p - reference)**2, axis=1)))),
                'maximum_horizontal_speed': float(np.max(np.linalg.norm(v[:, :2], axis=1))),
                'maximum_vertical_speed': float(np.max(np.abs(v[:, 2]))),
                'maximum_horizontal_acceleration': float(np.max(np.linalg.norm(a[:, :2], axis=1))),
                'maximum_vertical_acceleration': float(np.max(np.abs(a[:, 2]))),
                'maximum_horizontal_jerk': float(np.max(np.linalg.norm(j[:, :2], axis=1))),
                'maximum_vertical_jerk': float(np.max(np.abs(j[:, 2]))),
                'maximum_yaw_rate': yaw.maximum_rate(), 'maximum_tilt_rad': float(np.max(tilts)),
            }
            timing['validation'] = self.clock() - validation_start
            timing['yaw_construction_within_optimization'] = yaw_time
            elapsed = self.clock() - start
            if elapsed >= c.budget:
                raise _Deadline()
            timing['total'] = elapsed
            if (c.feasible_priority and initial_validated is not None
                    and initial_validated.valid and not valid):
                return replace(initial_validated, solve_time=elapsed, timing=timing,
                               metrics=dict(initial_validated.metrics,
                                            unsafe_candidate_discarded=True,
                                            discarded_objective=metrics['final_objective']))
            return MpcSeedResult(
                request.context, valid, status, 'sampled/adaptive admission only', tuple(times),
                tuple(map(tuple, p)), tuple(map(tuple, v)), tuple(map(tuple, a)),
                tuple(map(tuple, j)), tuple(angles), tuple(rates),
                tuple(map(tuple, geometry[:, :2])), elapsed, timing, violations, metrics,
                initial_visibility=visible[0], candidate_kind='follow_minco',
                objective_value=metrics['final_objective'])
        except _Deadline:
            reason = 'DEADLINE_EXCEEDED'
        except (ValueError, TypeError, OverflowError, FloatingPointError) as error:
            reason = str(error)
        elapsed = self.clock() - start
        timing['total'] = elapsed
        return MpcSeedResult(request.context, solver_status=reason, reason=reason,
                             solve_time=elapsed, timing=timing)
