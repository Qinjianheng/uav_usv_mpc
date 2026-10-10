"""Opt-in cheap-ranking, bounded adjoint MINCO refinement, independent full final check."""
from dataclasses import replace
import math

import numpy as np
from scipy.optimize import minimize

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.controllers.progress_follow_solver import TrackingFollowSolver, optimal_local_jerk
from uav_control.controllers.short_follow_solver import local_seed
from uav_control.guidance.fast_follow_minco import freshness
from uav_control.guidance.fast_minco_objective import FastMincoObjective
from uav_control.guidance.follow_fast_validation import InvariantCache, validate_seed
from uav_control.guidance.follow_limits import constraint_snapshot
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_profile import profiled, stage, current, CycleProfile
from uav_control.guidance.follow_reference import forecast_viewpoint
from uav_control.guidance.p43_minco_objective import TrackingMincoObjective
from uav_control.guidance.yaw_trajectory import YawTrajectory


class P43FollowSolver(TrackingFollowSolver):
    """Preserve old strategies; no control authority, holding or accepted-reference fiction."""

    def __init__(self, model, duration=1.2, rolling=True, refinement='qt',
                 refinement_budget=.025, **kwargs):
        """Limit refinement to one main candidate and one iteration per Q/QT stage."""
        super().__init__(model, duration, rolling, refinement=False, **kwargs)
        if refinement not in ('none', 'q', 'qt') or not 0 < refinement_budget <= .05:
            raise ValueError('INVALID_P43_REFINEMENT')
        self.refinement_mode, self.refinement_budget = refinement, refinement_budget
        self.invariant_cache = InvariantCache()
        self.mode = 'p43_visibility_minco'

    def solve(self, request):
        """Attach per-worker exclusive timings; an outer matched-study recorder takes priority."""
        if current() is not None:
            return self._solve(request)
        with CycleProfile(request.context.cycle_id, self.clock) as profile:
            result = self._solve(request)
        return replace(result, metrics=dict(result.metrics, cycle_profile=profile.report()))

    @profiled('candidate_selection')
    def _solve(self, request):
        """Heuristic sorting grants no qualification; every retained curve gets strict checks."""
        started = self.clock()
        until, _ = freshness(request, self.wall_clock())
        end = started+min(self.config.solve_budget, until-self.wall_clock()-.02)
        best = MpcSeedResult(request.context, solver_status='NO_FRESHNESS_BUDGET')
        attempts, refinements, selected = [], [], None
        try:
            if end <= started:
                raise ValueError('NO_FRESHNESS_BUDGET')
            problem = FollowProblem(request, self.model, self.duration)
            p, v, a = np.asarray(request.state[:9]).reshape(3, 3)
            old = np.asarray(self.hint[1]) if self.usable_hint(request) else np.zeros(3)
            template = local_seed(request, self.duration, np.zeros(3), 0., self.maximum_horizon)
            proxy = FastMincoObjective(problem, template, MincoConfig(mode='q'))
            candidates = []
            query = np.array((0., self.duration/2, self.duration))
            targets, _ = problem.target_state(query)
            for beta in (0., -.6, .6):
                rp, rv, ra, heading, details = forecast_viewpoint(problem, self.duration, beta)
                ra[:2] *= min(1., self.config.maximum_horizontal_acceleration/max(
                    np.linalg.norm(ra[:2]), 1e-12))
                for scale in ((1., .5, 0.) if beta == 0. else (1.,)):
                    with stage('jerk_candidates'):
                        j = optimal_local_jerk(request.state[:9], rp, rv, self.duration,
                                               self.weights, old, ra)*scale
                        j[:2] *= min(1., self.config.maximum_horizontal_jerk/max(
                            np.linalg.norm(j[:2]), 1e-12))
                        j[2] = np.clip(j[2], -self.config.maximum_vertical_jerk,
                                       self.config.maximum_vertical_jerk)
                        end_a = a+j*self.duration
                        end_a[:2] *= min(1., self.config.maximum_horizontal_acceleration/max(
                            np.linalg.norm(end_a[:2]), 1e-12))
                        end_a[2] = np.clip(end_a[2], -self.config.maximum_vertical_acceleration,
                                           self.config.maximum_vertical_acceleration)
                        j = (end_a-a)/self.duration
                        direction = math.atan2(math.sin(heading-request.state[9]),
                                               math.cos(heading-request.state[9]))
                        rate = np.clip(direction/self.duration, -self.config.maximum_yaw_rate,
                                       self.config.maximum_yaw_rate)
                        seed = local_seed(request, self.duration, j, rate, self.maximum_horizon)
                    with stage('coarse_geometry'):
                        cp = p+v*query[:, None]+a*query[:, None]**2/2+j*query[:, None]**3/6
                        cv = v+a*query[:, None]+j*query[:, None]**2/2
                        ca = a+j*query[:, None]
                        ys = YawTrajectory(np.r_[0., np.cumsum(seed.durations)], seed.yaw,
                                           start_rate=getattr(request, 'reference_yaw_rate', 0.))
                        angles, _ = ys.sample(query)
                        _, margin, _, _ = proxy.geometry(cp, ca, angles, targets)
                        cost = (4*np.sum((cp[-1, :2]-rp[:2])**2)/25
                                + 2*np.sum((cv[-1, :2]-rv[:2])**2)/9
                                + np.sum((ca[-1]-ra)**2)/9
                                + .03*self.duration*np.dot(j, j)/36
                                + .01*(beta**2+np.sum((j-old)**2)/36)
                                + .05*np.mean(np.maximum(.1-margin, 0.)**2/.01))
                    candidates.append((int(beta != 0.), cost, seed, beta, j, details))
            for _, cost, seed, beta, j, details in sorted(candidates, key=lambda row: row[:2]):
                if end-self.clock() <= .003:
                    break
                tested = validate_seed(request, seed, self.model, self.invariant_cache,
                                       end-self.clock(), self.clock)
                attempts.append(dict(valid=tested.valid, status=tested.solver_status,
                                     beta=beta, cost=float(cost), jerk=j.tolist()))
                best = tested
                if best.valid:
                    selected = (seed, beta, j, details)
                    break
            if selected and self.refinement_mode != 'none':
                seed, beta, j, _ = selected
                refinement_end = min(end-.005, self.clock()+self.refinement_budget)
                for mode in ('q', 'qt') if self.refinement_mode == 'qt' else ('q',):
                    if refinement_end-self.clock() < .008:
                        break
                    config = MincoConfig(mode=mode, maximum_iterations=1, jerk_weight=.03/36,
                                         follow_weight=0., yaw_weight=.005)
                    objective = TrackingMincoObjective(problem, seed, config, beta, old)
                    x0 = objective.initial()
                    bounds = [(float(x-.25), float(x+.25)) for x in x0[:6]]
                    if mode == 'qt':
                        bounds += [(-1.5, 1.5)]*2
                    bounds += [(float(x-.3), float(x+.3)) for x in x0[-3:]]
                    initial = None

                    def evaluate(x):
                        if self.clock() >= refinement_end-.005:
                            raise TimeoutError()
                        return objective(x)

                    try:
                        with stage('qt_yaw_optimization'):
                            initial = evaluate(x0)[0]
                            solved = minimize(
                                evaluate, x0, jac=True, method='L-BFGS-B', bounds=bounds,
                                options=dict(maxiter=1, maxls=2, ftol=1e-5))
                        if solved.fun >= initial or end-self.clock() < .003:
                            refinements.append(dict(mode=mode, improved=False))
                            continue
                        q, ts, ys = objective.unpack(solved.x)
                        candidate = replace(seed, q=tuple(map(tuple, q)),
                                            durations=tuple(ts), yaw=tuple(ys))
                        tested = validate_seed(
                            request, candidate, self.model, self.invariant_cache,
                            end-self.clock(), self.clock)
                        refinements.append(dict(
                            mode=mode, initial=initial, final=float(solved.fun),
                            valid=tested.valid, iterations=int(solved.nit),
                            counts=objective.counts))
                        if tested.valid:
                            best, seed = tested, candidate
                    except TimeoutError:
                        refinements.append(dict(mode=mode, timeout=True, initial=initial))
                        break
                    except (ValueError, TypeError, FloatingPointError,
                            OverflowError, np.linalg.LinAlgError) as error:
                        refinements.append(dict(mode=mode, failure=str(error),
                                                retained_baseline=True))
                        break
            if selected:
                _, beta, j, details = selected
                self.hint = (request.context, tuple(j), until, 0.)
                best = replace(best, metrics=dict(best.metrics, progress=dict(
                    beta=beta, attempts=attempts, reference_derivatives=details)))
        except (ValueError, TypeError, OverflowError, FloatingPointError,
                np.linalg.LinAlgError) as e:
            if best.valid:
                refinements.append(dict(failure=str(e), retained_baseline=True))
            else:
                best = MpcSeedResult(request.context, solver_status=str(e), reason=str(e))
        elapsed = self.clock()-started
        if self.clock() >= end or self.wall_clock() >= until-.02:
            best = replace(best, valid=False, solver_status='DEADLINE_EXCEEDED')
        snapshot = constraint_snapshot(self.model)
        return replace(best, solve_time=elapsed, timing=dict(best.timing, total=elapsed),
                       metrics=dict(best.metrics, p43=dict(
                           refinements=refinements, refinement_mode=self.refinement_mode,
                           holding_qualified=False, cheap_proxy_grants_admission=False,
                           attempts=attempts), constraint_snapshot=snapshot.payload,
                           constraint_fingerprint=snapshot.fingerprint))
