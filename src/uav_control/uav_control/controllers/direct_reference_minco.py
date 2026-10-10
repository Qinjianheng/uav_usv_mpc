"""Light causal reference frontends and a single MINCO map, with independent strict admission."""
from dataclasses import replace
import math

import numpy as np
from scipy.optimize import minimize

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.controllers.p44_follow_solver import P44FollowSolver
from uav_control.guidance.fast_follow_minco import freshness
from uav_control.guidance.follow_fast_validation import validate_seed
from uav_control.guidance.follow_limits import constraint_snapshot
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_profile import stage
from uav_control.guidance.follow_reference import forecast_viewpoint
from uav_control.guidance.greedy_follow_initializer import FollowSeed
from uav_control.guidance.p43_minco_objective import TrackingMincoObjective


def direct_seed(request, problem, duration, frontend='rear'):
    """Generate two Qs and a forecast PVA endpoint; keep the common execution-start PVA."""
    times = np.arange(1, 4)*duration/3
    references = [forecast_viewpoint(problem, float(t)) for t in times]
    points = [r[0] for r in references]
    endpoint = (*references[-1][0], *references[-1][1], *references[-1][2])
    if frontend == 'virtual_follow':
        p, v, a = np.asarray(request.state[:9]).reshape(3, 3).copy()
        points = []
        elapsed = 0.
        # Virtual kinematics only: no future measured UAV or true target samples.
        for knot in times:
            while elapsed < knot-1e-10:
                dt = min(.025, knot-elapsed)
                rp, rv, _, _, _ = forecast_viewpoint(problem, elapsed)
                desired = rv+.8*(rp-p)
                wanted = 2.*(desired-v)
                delta = wanted-a
                delta *= min(1., problem.limits.maximum_horizontal_jerk*dt/max(
                    np.linalg.norm(delta), 1e-12))
                new_a = a+delta
                new_a[:2] *= min(1., problem.limits.maximum_horizontal_acceleration/max(
                    np.linalg.norm(new_a[:2]), 1e-12))
                new_a[2] = np.clip(new_a[2], -problem.limits.maximum_vertical_acceleration,
                                   problem.limits.maximum_vertical_acceleration)
                p += v*dt+.5*new_a*dt**2
                v += new_a*dt
                a = new_a
                elapsed += dt
            points.append(p.copy())
        endpoint = (*p, *v, *a)
    elif frontend != 'rear':
        raise ValueError('INVALID_DIRECT_FRONTEND')
    angles = np.unwrap([request.state[9], *(r[3] for r in references)])
    return FollowSeed(request.context, True, 'DIRECT_REFERENCE_SEED', tuple(request.state),
                      tuple(endpoint), tuple(map(tuple, points[:2])), (duration/3,)*3,
                      tuple(angles), (0.,)*3, dict(frontend=frontend, future_uav_source='internal'))


class DirectReferenceMinco(P44FollowSolver):
    """Preserve P44/D as budgeted fallback; never grant control authority or holding."""

    def __init__(self, model, duration=1.2, rolling=True, refinement='none', frontend='rear',
                 **kwargs):
        super().__init__(model, duration=duration, rolling=rolling, refinement='none', **kwargs)
        if refinement not in ('none', 'q', 'qt') or frontend not in ('rear', 'virtual_follow'):
            raise ValueError('INVALID_DIRECT_CONFIG')
        self.direct_refinement, self.frontend = refinement, frontend
        self.fallback = P44FollowSolver(model, duration=duration, rolling=rolling, refinement='none',
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
        direct_status, fallback_used, refinement_status = best.solver_status, False, 'NOT_ATTEMPTED'
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
        except (ValueError, TypeError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as e:
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
                                    direct=dict(frontend=self.frontend, direct_status=direct_status,
                                                fallback_used=fallback_used,
                                                refinement=self.direct_refinement,
                                                refinement_status=refinement_status,
                                                holding_qualified=False)))
