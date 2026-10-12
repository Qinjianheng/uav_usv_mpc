"""Normalized local progress candidates mapped through existing MINCO, shadow only."""
from uav_control.guidance.follow_profile import profiled, current, CycleProfile

from dataclasses import dataclass, replace
import math

import numpy as np

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.controllers.short_follow_solver import ShortHorizonFollowSolver, local_seed
from uav_control.guidance.fast_follow_minco import FastFollowMinco, freshness
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_reference import forecast_viewpoint
from uav_control.guidance.yaw_trajectory import YawTrajectory


@dataclass(frozen=True)
class ProgressWeights:
    """Dimensionless costs after normalization by 5m, 3m/s, 3m/s² and 6m/s³."""

    position: float = 4.
    velocity: float = 2.
    acceleration: float = 1.
    jerk: float = .03
    switching: float = .01
    visibility: float = .05

    def validate(self):
        """Reject nonfinite/negative weights and a cost without any tracking objective."""
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(v) and v >= 0 for v in values) or (
                self.position+self.velocity <= 0):
            raise ValueError('INVALID_PROGRESS_WEIGHTS')


@profiled('jerk_candidates')
def optimal_local_jerk(state, reference_p, reference_v, duration, weights, previous,
                       reference_a=None):
    """Minimize terminal P/V/A + exact H||j||² integral + jerk switching quadratic."""
    weights.validate()
    x = np.asarray(state, dtype=float)
    rp, rv, old = (np.asarray(v, dtype=float) for v in (reference_p, reference_v, previous))
    if (x.shape != (9,) or any(v.shape != (3,) for v in (rp, rv, old))
            or not np.all(np.isfinite(np.r_[x, rp, rv, old, duration])) or duration <= 0):
        raise ValueError('INVALID_PROGRESS_STATE')
    ra = np.zeros(3) if reference_a is None else np.asarray(reference_a, dtype=float)
    if ra.shape != (3,) or not np.all(np.isfinite(ra)):
        raise ValueError('INVALID_REFERENCE_ACCELERATION')
    p, v, a = x.reshape(3, 3)
    h = duration
    kp, kv = h**3/6, h*h/2
    wp, wv, wa = weights.position/25, weights.velocity/9, weights.acceleration/9
    wj, ws = weights.jerk/36, weights.switching/36
    denominator = wp*kp*kp+wv*kv*kv+wa*h*h+wj*h+ws
    return -(wp*kp*(p+v*h+a*h*h/2-rp)+wv*kv*(v+a*h-rv)+wa*h*(a-ra)-ws*old)/denominator


class ProgressFollowSolver(ShortHorizonFollowSolver):
    """At most six full checks; math hints have no TTL or execution authority."""

    def __init__(self, model, duration=1.2, rolling=True, weights=ProgressWeights(),
                 acceleration_target='zero', rear_priority=False, refinement=False, **kwargs):
        """Keep original physical envelope, raw freshness and shared camera model."""
        super().__init__(model, duration, rolling, **kwargs)
        weights.validate()
        if acceleration_target not in ('zero', 'reference', 'bounded', 'none'):
            raise ValueError('INVALID_ACCELERATION_TARGET')
        self.acceleration_target, self.rear_priority = acceleration_target, rear_priority
        self.refinement = refinement
        self.weights, self.mode, self.stall_count = weights, 'progress_minco', 0
        self.fast = replace(self.fast, bernstein_precheck=True)

    def usable_hint(self, request):
        """Old math may seed a new fresh solve, but cannot cross resets or source reversal."""
        if not self.rolling or self.hint is None:
            return False
        old, _, _, _ = self.hint
        c = request.context
        return (old.mission_id == c.mission_id and old.clock_generation == c.clock_generation
                and old.frame_id == c.frame_id and old.prediction_source == c.prediction_source
                and old.prediction_source_stamp < c.prediction_source_stamp
                and old.prediction_sequence_id < c.prediction_sequence_id
                and old.observation_stamp <= c.observation_stamp
                and old.execution_start_stamp <= c.execution_start_stamp)

    def solve(self, request):
        """Profile only explicit live P4 research; historical/offline defaults stay disabled."""
        if not getattr(self, 'profile_enabled', False) or current() is not None:
            return self._solve(request)
        with CycleProfile(request.context.cycle_id, self.clock) as cycle:
            result = self._solve(request)
        return replace(result, metrics=dict(result.metrics, cycle_profile=cycle.report()))

    @profiled('candidate_selection')
    def _solve(self, request):
        """Rank progress seeds, then require independent full dynamic/FOV admission."""
        started = self.clock()
        until, _ = freshness(request, self.wall_clock())
        attempts, warm, selection = [], self.usable_hint(request), None
        best = MpcSeedResult(request.context, solver_status='NO_FRESHNESS_BUDGET')
        initialization = 0.
        try:
            problem = FollowProblem(request, self.model, self.duration)
            p, v, a = np.asarray(request.state[:9]).reshape(3, 3)
            old = np.asarray(self.hint[1]) if warm else np.zeros(3)
            candidates = []
            w = (replace(self.weights, acceleration=0.) if self.acceleration_target == 'none'
                 else self.weights)

            def bounded(j):
                j = np.asarray(j).copy()
                j[:2] *= min(1., self.config.maximum_horizontal_jerk/max(
                    np.linalg.norm(j[:2]), 1e-12))
                j[2] = np.clip(j[2], -self.config.maximum_vertical_jerk,
                               self.config.maximum_vertical_jerk)
                # Endpoint acceleration projection preserves a fixed measured start.
                end_a = a+j*self.duration
                end_a[:2] *= min(1., self.config.maximum_horizontal_acceleration/max(
                    np.linalg.norm(end_a[:2]), 1e-12))
                end_a[2] = np.clip(end_a[2], -self.config.maximum_vertical_acceleration,
                                   self.config.maximum_vertical_acceleration)
                return (end_a-a)/self.duration

            for beta in (0., -.6, .6):
                rp, rv, ra, heading, details = forecast_viewpoint(problem, self.duration, beta)
                if self.acceleration_target in ('zero', 'none'):
                    ra = np.zeros(3)
                if self.acceleration_target == 'bounded':
                    ra[:2] *= min(1., self.config.maximum_horizontal_acceleration/max(
                        np.linalg.norm(ra[:2]), 1e-12))
                    ra[2] = np.clip(ra[2], -self.config.maximum_vertical_acceleration,
                                    self.config.maximum_vertical_acceleration)
                details = dict(details, acceleration_target=ra.tolist())
                wanted = bounded(optimal_local_jerk(request.state[:9], rp, rv,
                                                    self.duration, w, old, reference_a=ra))
                for scale in ((1., .5, 0.) if beta == 0. else (1.,)):
                    j = bounded(wanted*scale)
                    end_p = p+v*self.duration+a*self.duration**2/2+j*self.duration**3/6
                    end_v = v+a*self.duration+j*self.duration**2/2
                    position_error = float(np.linalg.norm(end_p[:2]-rp[:2]))
                    velocity_error = float(np.linalg.norm(end_v[:2]-rv[:2]))
                    cost = (w.position*position_error**2/25+w.velocity*velocity_error**2/9
                            + w.acceleration*np.linalg.norm(a+j*self.duration-ra)**2/9
                            + w.jerk*self.duration*np.dot(j, j)/36
                            + w.switching*(beta*beta+np.sum((j-old)**2)/36))
                    direction = math.atan2(math.sin(heading-request.state[9]),
                                           math.cos(heading-request.state[9]))
                    rate = float(np.clip(direction/self.duration,
                                         -self.config.maximum_yaw_rate,
                                         self.config.maximum_yaw_rate))
                    query = np.array((0., self.duration/2, self.duration))
                    cp = p+v*query[:, None]+a*query[:, None]**2/2+j*query[:, None]**3/6
                    cv = v+a*query[:, None]+j*query[:, None]**2/2
                    ca = a+j*query[:, None]
                    ys = YawTrajectory((0., self.duration/3, 2*self.duration/3, self.duration),
                                       request.state[9]+rate*np.array((0., self.duration/3,
                                                                      2*self.duration/3,
                                                                      self.duration)),
                                       start_rate=getattr(request, 'reference_yaw_rate', 0.))
                    angles, rates = ys.sample(query)
                    _, _, geometry, _ = problem.assess(
                        query, cp, cv, ca, np.tile(j, (3, 1)), angles, rates, batch=True)
                    cost += w.visibility*float(np.mean(np.maximum(
                        .1-geometry[:, :2], 0.)**2/.1**2))
                    candidates.append((cost, j, rate, beta, rp, rv, details))
            if warm:
                rp, rv, _, heading, details = forecast_viewpoint(problem, self.duration)
                candidates.append((float('inf'), bounded(old), 0., 0., rp, rv, details))
            initialization = self.clock()-started
            for cost, j, rate, beta, rp, rv, details in sorted(
                    candidates, key=lambda z: (int(z[3] != 0.) if self.rear_priority else 0,
                                               z[0])):
                remaining = min(self.config.solve_budget-(self.clock()-started),
                                until-self.wall_clock()-self.fast.publication_reserve)
                if remaining < self.fast.validation_reserve:
                    break
                seed = local_seed(request, self.duration, j, rate, self.maximum_horizon)
                optimizer = MincoConfig(mode='fixed', budget=remaining, batch_validation=True)
                tested = FastFollowMinco(optimizer, self.model, self.fast, self.clock,
                                         self.wall_clock).solve(request, seed)
                if self.refinement and not tested.valid:
                    from uav_control.guidance.start_jerk import adjust_start
                    remaining = min(self.config.solve_budget-(self.clock()-started),
                                    until-self.wall_clock()-self.fast.publication_reserve)
                    if remaining > self.fast.validation_reserve+.008:
                        local_deadline = self.clock()+remaining-self.fast.validation_reserve
                        shaped = adjust_start(seed, 'combined', radius=.25,
                                              deadline=local_deadline,
                                              clock=self.clock)
                        remaining = min(self.config.solve_budget-(self.clock()-started),
                                        until-self.wall_clock()-self.fast.publication_reserve)
                        if remaining < self.fast.validation_reserve:
                            continue
                        improved = FastFollowMinco(replace(optimizer, mode='qt', budget=remaining,
                                                           maximum_iterations=1),
                                                   self.model, self.fast, self.clock,
                                                   self.wall_clock).solve(request, shaped)
                        attempts.append(dict(valid=improved.valid,
                                             status=improved.solver_status,
                                             refinement=True, metrics=improved.metrics))
                        if improved.valid:
                            tested = improved
                attempts.append(dict(valid=tested.valid, status=tested.solver_status,
                                     jerk=j.tolist(), beta=beta, cost=cost))
                best = tested
                if best.valid:
                    selection = (j, beta, rp, rv, details)
                    self.hint = (request.context, tuple(j), until, rate)
                    break
        except (ValueError, TypeError, OverflowError, FloatingPointError) as error:
            best = MpcSeedResult(request.context, solver_status=str(error), reason=str(error))
        elapsed = self.clock()-started
        if best.valid and (self.wall_clock() >= until-self.fast.publication_reserve
                           or elapsed >= self.config.solve_budget):
            best = replace(best, valid=False, solver_status='DEADLINE_EXCEEDED')
        progress = dict(warm_used=warm, hint_grants_execution=False, attempts=attempts,
                        duration=self.duration, acceleration_target=self.acceleration_target,
                        rear_priority=self.rear_priority, refinement=self.refinement,
                        stage_A_dynamic=bool(best.valid),
                        stage_B_FOV=bool(best.valid), stage_C_progress=False,
                        stage_D_closed_loop_proven=False, continuous_time_guarantee=False)
        if best.valid and selection:
            j, beta, rp, rv, details = selection
            initial_ref, _, _ = problem.reference((0.,), beta)
            start_error = float(np.linalg.norm(p[:2]-initial_ref[0, :2]))
            end_error = float(np.linalg.norm(np.asarray(best.positions[-1])[:2]-rp[:2]))
            speed_before = float(np.linalg.norm(v[:2]-rv[:2]))
            speed_after = float(np.linalg.norm(np.asarray(best.velocities[-1])[:2]-rv[:2]))
            improvement = start_error-end_error
            meaningful = improvement > .01 or speed_before-speed_after > .01
            equilibrium = start_error < .3 and speed_before < .2
            self.stall_count = 0 if meaningful or equilibrium else self.stall_count+1
            progress.update(position_improvement=improvement, start_error=start_error,
                            end_error=end_error, relative_velocity_error=speed_after,
                            velocity_improvement=speed_before-speed_after,
                            stage_C_progress=meaningful or equilibrium,
                            stall_count=self.stall_count, stall_detected=self.stall_count >= 5,
                            beta=beta, zero_jerk=bool(np.linalg.norm(j) < 1e-6),
                            reference_derivatives=details,
                            diagnostics=[name for name, condition in (
                                ('INSUFFICIENT_PROGRESS', self.stall_count >= 5),
                                ('VELOCITY_MISMATCH', speed_after > .5),
                                ('DISTANCE_GROWING', improvement < -.1),
                                ('VIEWPOINT_CHANGED', getattr(self, 'last_beta', beta) != beta))
                                if condition])
            self.last_beta = beta
        else:
            progress['diagnostics'] = [best.solver_status]
            self.clear_warm_start()
        return replace(best, solve_time=elapsed, timing=dict(best.timing,
                       initialization=initialization, total=elapsed),
                       metrics=dict(best.metrics, progress=progress))


class TrackingFollowSolver(ProgressFollowSolver):
    """P4.1 opt-in dynamic-acceleration and rear-first policy; P3.3 defaults stay unchanged."""

    maximum_horizon = 2.4

    def __init__(self, model, duration=1.2, rolling=True, acceleration_target='bounded',
                 rear_priority=True, **kwargs):
        """Use the same local free-terminal family over .8–2.4s without a forced far endpoint."""
        super().__init__(model, duration, rolling, acceleration_target=acceleration_target,
                         rear_priority=rear_priority, **kwargs)
