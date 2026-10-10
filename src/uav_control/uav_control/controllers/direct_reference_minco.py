"""Light causal reference frontends and a single MINCO map, with independent strict admission."""
from dataclasses import replace

import numpy as np
from scipy.optimize import minimize

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.controllers.p44_follow_solver import P44FollowSolver
from uav_control.guidance.fast_follow_minco import freshness
from uav_control.guidance.follow_fast_validation import validate_seed
from uav_control.guidance.follow_limits import constraint_snapshot
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.follow_minco_optimizer import _trajectory
from uav_control.guidance.follow_fast_validation import fields
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_profile import stage, current as current_profile
from uav_control.guidance.follow_reference import forecast_viewpoint
from uav_control.guidance.follow_rollout import rollout_follow, FollowRolloutConfig
from uav_control.guidance.greedy_follow_initializer import FollowSeed
from uav_control.guidance.p43_minco_objective import TrackingMincoObjective


def follow_guided_seed(request, problem, reference):
    """Map the full virtual response into the existing three-piece MINCO boundary."""
    duration = problem.duration
    times = np.arange(1, 4)*duration/3
    samples = [reference.sample(float(t)) for t in times]
    headings = [forecast_viewpoint(problem, float(t))[3] for t in times]
    angles = np.unwrap([request.state[9], *headings])
    return FollowSeed(request.context, True, 'FOLLOW_GUIDED_SEED', tuple(request.state),
                      tuple(np.concatenate(samples[-1])),
                      tuple(tuple(sample[0]) for sample in samples[:2]), (duration/3,)*3,
                      tuple(angles), (0.,)*3,
                      dict(frontend='virtual_follow', future_uav_source='internal',
                           rollout=reference.metrics))


def reference_fit(seed, reference, curve=None):
    """Quantify the entire reference fit, rather than only interpolated knot residuals."""
    curve = curve if curve is not None else _trajectory(seed, seed.q, seed.durations)
    samples = fields(curve.coefficients, seed.durations, np.zeros((3, 4)),
                     np.asarray(reference.times))
    report = {}
    for name, actual, wanted in zip(('position', 'velocity', 'acceleration'), samples[:3],
                                    (reference.positions, reference.velocities,
                                     reference.accelerations)):
        errors = actual-np.asarray(wanted)
        report[name+'_rmse'] = float(np.sqrt(np.mean(np.sum(errors**2, axis=1))))
        report[name+'_maximum'] = float(np.max(np.linalg.norm(errors, axis=1)))
        report[name+'_vertical_rmse'] = float(np.sqrt(np.mean(errors[:, 2]**2)))
    report['height_rmse'] = float(np.sqrt(np.mean(
        (samples[0][:, 2]-reference.metrics['flight_altitude'])**2)))
    return report


def direct_seed(request, problem, duration, frontend='rear'):
    """Generate two Qs and a forecast PVA endpoint; keep the common execution-start PVA."""
    times = np.arange(1, 4)*duration/3
    references = [forecast_viewpoint(problem, float(t)) for t in times]
    points = [r[0] for r in references]
    endpoint = (*references[-1][0], *references[-1][1], *references[-1][2])
    if frontend == 'virtual_follow':
        return follow_guided_seed(request, problem, rollout_follow(problem))
    elif frontend != 'rear':
        raise ValueError('INVALID_DIRECT_FRONTEND')
    angles = np.unwrap([request.state[9], *(r[3] for r in references)])
    return FollowSeed(request.context, True, 'DIRECT_REFERENCE_SEED', tuple(request.state),
                      tuple(endpoint), tuple(map(tuple, points[:2])), (duration/3,)*3,
                      tuple(angles), (0.,)*3,
                      dict(frontend=frontend, future_uav_source='internal'))


class DirectReferenceMinco(P44FollowSolver):
    """Preserve P44/D as budgeted fallback; never grant control authority or holding."""

    def __init__(self, model, duration=1.2, rolling=True, refinement='none', frontend='rear',
                 **kwargs):
        super().__init__(model, duration=duration, rolling=rolling, refinement='none', **kwargs)
        if refinement not in ('none', 'q', 'qt') or frontend not in ('rear', 'virtual_follow'):
            raise ValueError('INVALID_DIRECT_CONFIG')
        self.direct_refinement, self.frontend = refinement, frontend
        self.fallback = P44FollowSolver(
            model, duration=duration, rolling=rolling, refinement='none',
            clock=self.clock, wall_clock=self.wall_clock)
        self.mode = 'direct_reference_minco'

    def clear_warm_start(self):
        """Reset both strategies without manufacturing a new boundary or expiry."""
        super().clear_warm_start()
        self.fallback.clear_warm_start()

    def _solve(self, request):
        started = self.clock()
        until, _ = freshness(request, self.wall_clock())
        end = started+min(self.config.solve_budget, until-self.wall_clock()-.02)
        best = MpcSeedResult(request.context, solver_status='NO_FRESHNESS_BUDGET')
        direct_status, fallback_used = best.solver_status, False
        refinement_status = 'NOT_ATTEMPTED'
        init_time = 0.
        try:
            if end <= started:
                raise ValueError('NO_FRESHNESS_BUDGET')
            with stage('direct_reference_initialization'):
                problem = FollowProblem(request, self.model, self.duration)
                seed = direct_seed(request, problem, self.duration, self.frontend)
            init_time = self.clock()-started
            with stage('direct_strict_validation'):
                best = validate_seed(request, seed, self.model, self.invariant_cache,
                                     end-self.clock(), self.clock)
            direct_status = best.solver_status
            if best.valid and self.direct_refinement != 'none' and end-self.clock() > .015:
                config = MincoConfig(mode=self.direct_refinement, maximum_iterations=1,
                                     jerk_weight=.03/36, follow_weight=0., yaw_weight=.005)
                objective = TrackingMincoObjective(problem, seed, config)
                x0 = objective.initial()
                refinement_end = min(end-.008, self.clock()+.015)

                def evaluate(x):
                    if self.clock() >= refinement_end:
                        raise TimeoutError()
                    return objective(x)

                try:
                    initial = evaluate(x0)[0]
                    bounds = [(float(x-.25), float(x+.25)) for x in x0[:6]]
                    if self.direct_refinement == 'qt':
                        bounds += [(-1.5, 1.5)]*2
                    bounds += [(float(x-.3), float(x+.3)) for x in x0[-3:]]
                    with stage('direct_refinement'):
                        solved = minimize(evaluate, x0, jac=True, method='L-BFGS-B', bounds=bounds,
                                          options=dict(maxiter=1, maxls=2, ftol=1e-5))
                    refinement_status = 'NO_IMPROVEMENT'
                    if solved.fun < initial and end-self.clock() > .003:
                        q, ts, yaw = objective.unpack(solved.x)
                        improved = replace(seed, q=tuple(map(tuple, q)), durations=tuple(ts),
                                           yaw=tuple(yaw))
                        tested = validate_seed(request, improved, self.model, self.invariant_cache,
                                               end-self.clock(), self.clock)
                        refinement_status = tested.solver_status
                        if tested.valid:
                            best = tested
                except TimeoutError:
                    refinement_status = 'BUDGET_STOP_RETAIN_VALID_FIXED'
            if not best.valid and end-self.clock() > .025:
                fallback_used = True
                self.fallback.config = replace(self.config, solve_budget=end-self.clock())
                best = self.fallback._solve(request)
        except (ValueError, TypeError, FloatingPointError, OverflowError,
                np.linalg.LinAlgError) as e:
            best = MpcSeedResult(request.context, solver_status=str(e), reason=str(e))
        elapsed = self.clock()-started
        if self.clock() >= end or self.wall_clock() >= until-.02:
            best = replace(best, valid=False, solver_status='DEADLINE_EXCEEDED')
        snapshot = constraint_snapshot(self.model)
        return replace(best, solve_time=elapsed,
                       candidate_kind=('p44_fallback' if fallback_used else self.mode),
                       timing=dict(best.timing, initialization=init_time, total=elapsed),
                       metrics=dict(best.metrics, constraint_snapshot=snapshot.payload,
                                    constraint_fingerprint=snapshot.fingerprint,
                                    direct=dict(frontend=self.frontend,
                                                direct_status=direct_status,
                                                fallback_used=fallback_used,
                                                refinement=self.direct_refinement,
                                                refinement_status=refinement_status,
                                                holding_qualified=False)))


class FollowGuidedMinco(DirectReferenceMinco):
    """One FOLLOW initialization; bounded Q or Q/T/yaw, then strict admission and P44/D."""

    def __init__(self, model, refinement='none', rollout_config=FollowRolloutConfig(),
                 fallback_enabled=True, optimization_budget=.025, maximum_iterations=2, **kwargs):
        super().__init__(model, refinement=refinement, frontend='virtual_follow', **kwargs)
        rollout_config.validate()
        if (not np.isfinite(optimization_budget) or not 0 < optimization_budget <= .05
                or isinstance(maximum_iterations, bool) or not isinstance(maximum_iterations, int)
                or not 1 <= maximum_iterations <= 8):
            raise ValueError('INVALID_FOLLOW_GUIDED_BUDGET')
        self.rollout_config, self.fallback_enabled = rollout_config, fallback_enabled
        self.optimization_budget, self.maximum_iterations = optimization_budget, maximum_iterations
        self.mode = 'follow_guided_minco'

    def _solve(self, request):
        started = self.clock()
        now = self.wall_clock()
        until, _ = freshness(request, now)
        end = started+min(self.config.solve_budget, until-now-self.fast.publication_reserve)
        timing = dict(initialization=0., coefficient_construction=0., optimization=0.,
                      validation=0., fallback=0., fit_analysis=0.)
        detail = dict(refinement=self.direct_refinement, independent_valid=False,
                      fallback_used=False, fallback_valid=False, initial_status='NOT_ATTEMPTED',
                      refinement_status='NOT_ATTEMPTED', holding_qualified=False)
        best = MpcSeedResult(request.context, solver_status='NO_FRESHNESS_BUDGET')

        def check_deadline():
            if self.clock() >= end:
                raise ValueError('DEADLINE_EXCEEDED')

        def validate(seed):
            check_deadline()
            stamp = self.clock()
            result = validate_seed(request, seed, self.model, self.invariant_cache,
                                   end-stamp, self.clock)
            timing['validation'] += self.clock()-stamp
            return result

        def fit(seed, reference):
            check_deadline()
            stamp = self.clock()
            curve = _trajectory(seed, seed.q, seed.durations)
            timing['coefficient_construction'] += self.clock()-stamp
            check_deadline()
            stamp = self.clock()
            result = reference_fit(seed, reference, curve)
            timing['fit_analysis'] += self.clock()-stamp
            return result

        try:
            if (not np.isfinite(now) or now < request.now_stamp-1e-9
                    or end-started < self.fast.validation_reserve):
                raise ValueError('NO_FRESHNESS_BUDGET')
            stamp = self.clock()
            try:
                problem = FollowProblem(request, self.model, self.duration)
                reference = rollout_follow(problem, self.rollout_config, deadline=end,
                                           clock=self.clock)
                check_deadline()
                seed = follow_guided_seed(request, problem, reference)
            finally:
                timing['initialization'] = self.clock()-stamp
            detail['rollout'] = reference.metrics
            measured = np.asarray(getattr(request, 'measurement_state', ()) or request.state)
            projected = measured.copy()
            dt = request.context.execution_start_stamp-request.context.navigation_stamp
            projected[:3] += measured[3:6]*dt+measured[6:9]*dt**2/2
            projected[3:6] += measured[6:9]*dt
            detail['boundary_policy'] = getattr(request, 'boundary_policy', 'measured')
            for name, part in (('position', slice(0, 3)), ('velocity', slice(3, 6)),
                               ('acceleration', slice(6, 9))):
                detail['boundary_measurement_'+name+'_error'] = float(np.linalg.norm(
                    np.asarray(request.state)[part]-measured[part]))
                detail['boundary_projected_measurement_'+name+'_error'] = float(np.linalg.norm(
                    np.asarray(request.state)[part]-projected[part]))
            previous_jerk = np.zeros(3)
            prior = getattr(request, 'prior_curve', None)
            if prior is not None:
                query = np.array([request.context.execution_start_stamp-prior.start])
                previous_jerk = fields(np.asarray(prior.xyz).reshape(-1, 6, 3), prior.durations,
                                       np.asarray(prior.yaw), query)[3][0]
            detail['previous_jerk_source'] = 'parent_curve' if prior is not None else 'zero_prior'
            detail['initial_fit'] = fit(seed, reference)
            best = validate(seed)
            detail['initial_status'] = best.solver_status
            selected = seed
            # A failed initial admission is deliberately allowed into bounded optimization.
            if self.direct_refinement != 'none' and end-self.clock() > .025:
                stamp = self.clock()
                config = MincoConfig(mode=self.direct_refinement,
                                     maximum_iterations=self.maximum_iterations,
                                     jerk_weight=.03/36, follow_weight=0., yaw_weight=.005)
                objective = TrackingMincoObjective(
                    problem, seed, config, previous_jerk=previous_jerk, reference=reference,
                    tracking_weights=(self.duration*4./25, self.duration*2./9, 0.),
                    yaw_optimize=self.direct_refinement == 'qt')
                x0 = objective.initial()
                optimize_end = min(end-.015, stamp+self.optimization_budget)
                incumbent = [float('inf'), x0.copy()]

                def evaluate(x):
                    if self.clock() >= optimize_end:
                        raise TimeoutError()
                    value = objective(x)
                    if value[0] < incumbent[0]:
                        incumbent[:] = [value[0], np.asarray(x).copy()]
                    return value

                bounds = [(float(x-.25), float(x+.25)) for x in x0[:6]]
                if self.direct_refinement == 'qt':
                    bounds += [(-1.5, 1.5)]*2
                    bounds += [(float(x-.3), float(x+.3)) for x in x0[-3:]]
                detail['optimization_attempted'] = True
                initial_cost = None
                try:
                    initial_cost = evaluate(x0)[0]
                    solved = minimize(evaluate, x0, jac=True, method='L-BFGS-B', bounds=bounds,
                                      options=dict(maxiter=self.maximum_iterations,
                                                   maxls=3, ftol=1e-5))
                    detail['refinement_status'] = str(solved.message)
                except TimeoutError:
                    detail['refinement_status'] = 'BOUNDED_STOP'
                timing['optimization'] = self.clock()-stamp
                detail.update(initial_objective=initial_cost,
                              best_objective=incumbent[0] if np.isfinite(incumbent[0]) else None,
                              objective_counts=objective.counts)
                if not np.array_equal(incumbent[1], x0) and end-self.clock() > .005:
                    q, ts, yaw = objective.unpack(incumbent[1])
                    candidate = replace(seed, q=tuple(map(tuple, q)), durations=tuple(ts),
                                        yaw=tuple(yaw))
                    tested = validate(candidate)
                    detail['optimized_status'] = tested.solver_status
                    if tested.valid:
                        best, selected = tested, candidate
            detail['independent_valid'] = best.valid
            detail['independent_status'] = best.solver_status
            detail['fit'] = (detail['initial_fit'] if selected is seed
                             else fit(selected, reference))
            if best.valid:
                detail['start_jerk_jump'] = float(np.linalg.norm(
                    np.asarray(best.jerks[0])-previous_jerk))
            if not best.valid and self.fallback_enabled and end-self.clock() > .025:
                detail['fallback_used'] = True
                stamp = self.clock()
                self.fallback.wall_clock = self.wall_clock
                self.fallback.config = replace(self.config, solve_budget=end-stamp)
                best = self.fallback._solve(request)
                timing['fallback'] = self.clock()-stamp
                detail['fallback_valid'] = best.valid
        except (ValueError, TypeError, FloatingPointError, OverflowError,
                np.linalg.LinAlgError) as e:
            best = MpcSeedResult(request.context, solver_status=str(e), reason=str(e))
            detail['independent_valid'] = False
        elapsed = self.clock()-started
        current = self.wall_clock()
        if (self.clock() >= end or not np.isfinite(current) or current < request.now_stamp-1e-9
                or current >= until-self.fast.publication_reserve):
            best = replace(best, valid=False, solver_status='DEADLINE_EXCEEDED')
            detail['independent_valid'] = False
            detail['fallback_valid'] = False
        snapshot = constraint_snapshot(self.model)
        recorder = current_profile()
        timing['initial_coefficient_construction'] = timing['coefficient_construction']
        timing['coefficient_construction'] = (recorder.seconds['minco_coefficients']
                                              if recorder is not None else None)
        return replace(best, solve_time=elapsed,
                       candidate_kind='p44_fallback' if detail['fallback_used'] else self.mode,
                       timing=dict(timing, total=elapsed), metrics=dict(
                           best.metrics, follow_guided=detail,
                           constraint_snapshot=snapshot.payload,
                           constraint_fingerprint=snapshot.fingerprint))
