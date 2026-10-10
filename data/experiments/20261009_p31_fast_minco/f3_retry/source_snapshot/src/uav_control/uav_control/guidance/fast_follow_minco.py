"""Feasibility-first MINCO research engine; original P3 solver remains available."""
from dataclasses import dataclass, replace
import time

import numpy as np
from scipy.optimize import minimize

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.guidance.fast_minco_objective import FastMincoObjective
from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, _trajectory
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.polynomial_extrema import derivative_peak


@dataclass(frozen=True)
class FastConfig:
    """Independent switches enable measured ablations and conservative publication reserves."""

    feasible_priority: bool = True
    fast_feasible_seed: bool = True
    exact_precheck: bool = True
    freshness_budget: bool = False
    publication_reserve: float = .02
    validation_reserve: float = .025
    yaw_optimize: bool = True
    visibility_mode: str = 'full'

    def validate(self):
        """Reject unknown proxy modes and invalid reserves, retaining the 125ms hard gate."""
        if (self.visibility_mode not in ('full', 'horizontal') or not np.all(np.isfinite((
                self.publication_reserve, self.validation_reserve)))
                or min(self.publication_reserve, self.validation_reserve) < 0):
            raise ValueError('INVALID_FAST_CONFIG')


def freshness(request, now):
    """Compute remaining raw-input freshness without rewriting any acquisition epoch."""
    c = request.context
    until = min(c.navigation_stamp+.125, c.observation_stamp+.125,
                c.prediction_source_stamp+.125, c.prediction_valid_until)
    return until, until-now


class FastFollowMinco:
    """Validate the initial curve before optimizing; a lower unsafe cost cannot displace it."""

    def __init__(self, config, model, fast=FastConfig(), clock=time.perf_counter,
                 wall_clock=time.time):
        """Inject clocks for deterministic freshness and deadline tests."""
        config.validate()
        fast.validate()
        self.config, self.model, self.fast = config, model, fast
        self.clock, self.wall_clock = clock, wall_clock

    def solve(self, request, seed):
        """Return only independently validated samples, with explicit skipped optimization."""
        started = self.clock()
        c, f = self.config, self.fast
        until, remaining = freshness(request, self.wall_clock() if f.freshness_budget else
                                     request.now_stamp)
        budget = min(c.budget, remaining-f.publication_reserve) if f.freshness_budget else c.budget
        diagnostics = dict(fresh_until=until, remaining_freshness_at_solver_start=remaining,
                           effective_budget=budget, original_budget=c.budget,
                           publication_reserve=f.publication_reserve,
                           prediction_ttl_before_execution=(
                               request.context.prediction_valid_until <
                               request.context.execution_start_stamp), accepted_by_tracker=False,
                           seed_metrics=seed.metrics)
        if budget <= 0:
            return MpcSeedResult(request.context, solver_status='NO_FRESHNESS_BUDGET',
                                 reason='no time after publication reserve', metrics=diagnostics)
        end = started+budget
        objective = None
        validation_calls, optimization_calls = 0, 0

        def validate(candidate):
            nonlocal validation_calls
            validation_calls += 1
            left = end-self.clock()
            if left <= 0:
                raise TimeoutError()
            if f.exact_precheck and candidate.valid_input:
                trajectory = _trajectory(candidate, candidate.q, candidate.durations)
                limits = self.model.config
                peaks = {name: derivative_peak(trajectory.coefficients, trajectory.durations, d,
                                               axes) for name, d, axes in (
                    ('horizontal_speed', 1, (0, 1)), ('vertical_speed', 1, (2,)),
                    ('horizontal_acceleration', 2, (0, 1)), ('vertical_acceleration', 2, (2,)),
                    ('horizontal_jerk', 3, (0, 1)), ('vertical_jerk', 3, (2,)))}
                violations = {name: max(0., peak['value']-getattr(limits, 'maximum_'+name))
                              for name, peak in peaks.items()}
                if max(violations.values()) > 1e-6:
                    return MpcSeedResult(request.context, solver_status='DYNAMIC_INFEASIBLE',
                                         reason='analytic derivative peak precheck',
                                         constraint_violations=violations,
                                         metrics=dict(analytic_peaks=peaks,
                                                      xyz_coefficients=(
                                                          trajectory.coefficients.tolist()),
                                                      durations=candidate.durations,
                                                      q=candidate.q, yaw_knots=candidate.yaw,
                                                      end=candidate.end,
                                                      full_projection_validated=False,
                                                      jerk_integral=trajectory.control_effort()))
            left = end-self.clock()
            if left <= 0:
                raise TimeoutError()
            result = FollowMincoOptimizer(replace(c, mode='fixed', budget=left), self.model,
                                          self.clock).solve(request, candidate)
            return replace(result, metrics=dict(result.metrics, full_projection_validated=bool(
                result.metrics.get('validation_samples', 0))))

        best = MpcSeedResult(request.context, solver_status=seed.reason)
        try:
            if not seed.valid_input:
                raise ValueError(seed.reason)
            # Same strict seed validation as P3 runs even when a peak precheck would reject.
            if (seed.context != request.context or len(seed.start) != 10 or len(seed.end) != 9
                    or np.asarray(seed.q).shape != (2, 3) or len(seed.yaw) != 4
                    or len(seed.durations) != 3
                    or not np.allclose(seed.start, request.state, rtol=0., atol=1e-9)):
                raise ValueError('INVALID_MINCO_SEED')
            problem = FollowProblem(request, self.model, sum(seed.durations))
            best = validate(seed)
            diagnostics['initial_feasible'] = best.valid
            diagnostics['initial_status'] = best.solver_status
            if best.valid and f.fast_feasible_seed:
                diagnostics['optimization_skipped_reason'] = 'VALIDATED_FIXED_SEED'
            elif c.mode != 'fixed' and end-self.clock() > f.validation_reserve+.008:
                # Q before optional Q/T. Both are bounded by the same whole deadline.
                for mode in ('q', 'qt') if c.mode == 'qt' else ('q',):
                    if end-self.clock() <= f.validation_reserve+.008:
                        break
                    objective = FastMincoObjective(problem, seed, replace(c, mode=mode),
                                                   f.yaw_optimize, f.visibility_mode)
                    x0 = objective.initial()
                    bounds = [(float(x-c.spatial_radius), float(x+c.spatial_radius))
                              for x in x0[:6]]
                    if mode == 'qt':
                        bounds += [(-1.5, 1.5)]*2
                    if f.yaw_optimize:
                        bounds += [(float(x-.8), float(x+.8)) for x in x0[-3:]]

                    def evaluate(x):
                        if self.clock() >= end-f.validation_reserve:
                            raise TimeoutError()
                        return objective(x)

                    optimization_calls += 1
                    initial_cost = evaluate(x0)[0]
                    solved = minimize(evaluate, x0, jac=True, method='L-BFGS-B', bounds=bounds,
                                      options=dict(maxiter=c.maximum_iterations,
                                                   maxls=5, ftol=1e-5))
                    q, ts, ys = objective.unpack(solved.x)
                    candidate = replace(seed, q=tuple(map(tuple, q)), durations=tuple(ts),
                                        yaw=tuple(ys))
                    tested = validate(candidate)
                    diagnostics[mode] = dict(counts=dict(objective.counts),
                                             timing=dict(objective.timing),
                                             iterations=int(solved.nit),
                                             function_evaluations=int(solved.nfev),
                                             gradient_evaluations=int(solved.njev),
                                             initial_objective=initial_cost,
                                             final_objective=float(solved.fun),
                                             candidate_feasible=tested.valid)
                    if tested.valid or not (f.feasible_priority and best.valid):
                        best = tested
                    else:
                        diagnostics['unsafe_lower_cost_discarded'] = (
                            bool(solved.fun < initial_cost))
                    if tested.valid and f.fast_feasible_seed:
                        break
            else:
                diagnostics['optimization_skipped_reason'] = 'MODE_OR_REMAINING_BUDGET'
        except TimeoutError:
            diagnostics['optimization_timeout'] = True
            if not best.valid:
                best = replace(best, solver_status='DEADLINE_EXCEEDED',
                               reason='research time budget')
        except (ValueError, TypeError, OverflowError, FloatingPointError) as error:
            best = MpcSeedResult(request.context, solver_status=str(error), reason=str(error))
        diagnostics.update(validation_calls=validation_calls,
                           optimization_calls=optimization_calls,
                           retained_feasible=best.valid,
                           remaining_freshness_at_solver_finish=remaining-(self.clock()-started))
        if objective is not None:
            diagnostics['last_objective_counts'] = dict(objective.counts)
        elapsed = self.clock()-started
        if elapsed >= budget:
            best = replace(best, valid=False, solver_status='DEADLINE_EXCEEDED',
                           reason='whole effective budget expired')
        return replace(best, solve_time=elapsed, timing=dict(best.timing, total=elapsed),
                       metrics=dict(best.metrics, fast=diagnostics))
