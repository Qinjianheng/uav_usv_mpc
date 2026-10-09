"""Opt-in short local progress and rolling jerk hints, with no execution authority."""
from dataclasses import replace
import math
import time

import numpy as np

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.guidance.fast_follow_minco import FastFollowMinco, FastConfig, freshness
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.greedy_follow_initializer import FollowSeed


def local_seed(request, duration, jerk, yaw_rate):
    """Sample one cubic at common knots; end PVA progresses rather than forcing a far goal."""
    jerk = np.asarray(jerk, dtype=float)
    if (not math.isfinite(duration) or not .8 <= duration <= 1.6 or jerk.shape != (3,)
            or not np.all(np.isfinite(jerk)) or not math.isfinite(yaw_rate)):
        raise ValueError('INVALID_LOCAL_SEED')
    p, v, a = np.asarray(request.state[:9], dtype=float).reshape(3, 3)
    times = np.arange(1, 4)*duration/3
    points = p+v*times[:, None]+a*times[:, None]**2/2+jerk*times[:, None]**3/6
    end = (*points[-1], *(v+a*duration+jerk*duration**2/2), *(a+jerk*duration))
    yaw = request.state[9]+yaw_rate*np.r_[0., times]
    return FollowSeed(request.context, True, 'LOCAL_PROGRESS_SEED', tuple(request.state),
                      tuple(end), tuple(map(tuple, points[:2])), (duration/3,)*3,
                      tuple(yaw), (0.,)*3, dict(local_jerk=jerk.tolist(), yaw_rate=yaw_rate))


class ShortHorizonFollowSolver:
    """Greedily try at most four short seeds; revalidate each under current predictions."""

    def __init__(self, model, duration=1.2, rolling=True, clock=time.perf_counter,
                 wall_clock=time.time, publication_reserve=.02, validation_reserve=.025):
        """Select a short research horizon while preserving all existing physical limits."""
        if not math.isfinite(duration) or not .8 <= duration <= 1.6:
            raise ValueError('INVALID_SHORT_HORIZON')
        self.model, self.config, self.duration = model, model.config, duration
        self.clock, self.wall_clock, self.rolling = clock, wall_clock, rolling
        self.fast = FastConfig(publication_reserve=publication_reserve,
                               validation_reserve=validation_reserve)
        self.fast.validate()
        self.mode, self.hint = 'greedy_minco', None
        for name in ('intrinsics', 'extrinsics', 'target', 'visibility', 'attitude_config'):
            setattr(self, name, getattr(model, name))

    def clear_warm_start(self):
        """Discard research hints on rejection and invalidate the underlying model cache."""
        self.hint = None
        self.model.clear_warm_start()

    def usable_hint(self, request):
        """Hints never extend expiry, substitute accepted PVA or cross source discontinuities."""
        if not self.rolling or self.hint is None:
            return False
        old, _, expiry, _ = self.hint
        c = request.context
        return (old.mission_id == c.mission_id and old.clock_generation == c.clock_generation
                and old.frame_id == c.frame_id and old.prediction_source == c.prediction_source
                and old.prediction_source_stamp < c.prediction_source_stamp
                and old.observation_stamp <= c.observation_stamp
                and old.execution_start_stamp <= c.execution_start_stamp
                and request.now_stamp < expiry)

    def solve(self, request):
        """Emit only sampled research feasibility; short horizon is no holding authorization."""
        started, now = self.clock(), self.wall_clock()
        until, _ = freshness(request, now)
        attempts, warm_used, initial_time = [], False, 0.
        best = MpcSeedResult(request.context, solver_status='NO_FRESHNESS_BUDGET')

        def remaining():
            current = self.wall_clock()
            if not math.isfinite(current) or current < request.now_stamp-1e-9:
                return -1.
            return min(self.config.solve_budget-(self.clock()-started),
                       until-current-self.fast.publication_reserve)

        try:
            if remaining() >= self.fast.validation_reserve:
                problem = FollowProblem(request, self.model, self.duration)
                ref, target_v, target_yaw = problem.reference((self.duration,))
                p, v, a = np.asarray(request.state[:9]).reshape(3, 3)
                wanted = 6*(ref[0]-p-v*self.duration-a*self.duration**2/2)/self.duration**3

                def bounded(j):
                    j = np.asarray(j, dtype=float).copy()
                    norm = np.linalg.norm(j[:2])
                    j[:2] *= min(1., self.config.maximum_horizontal_jerk/max(norm, 1e-12))
                    j[2] = np.clip(j[2], -self.config.maximum_vertical_jerk,
                                   self.config.maximum_vertical_jerk)
                    return j

                jerks = [bounded(wanted), np.zeros(3), bounded(-a/self.duration)]
                angle = math.atan2(math.sin(target_yaw[0]-request.state[9]),
                                   math.cos(target_yaw[0]-request.state[9]))
                rate = float(np.clip(angle/self.duration, -self.config.maximum_yaw_rate,
                                     self.config.maximum_yaw_rate))
                ranked = sorted(jerks, key=lambda j: np.linalg.norm(
                    p+v*self.duration+a*self.duration**2/2+j*self.duration**3/6-ref[0])**2
                    + .2*np.linalg.norm(v+a*self.duration+j*self.duration**2/2-target_v[0])**2)
                warm_used = self.usable_hint(request)
                if warm_used:
                    ranked.insert(0, bounded(self.hint[1]))
                else:
                    self.hint = None
                seeds = [local_seed(request, self.duration, j, rate) for j in ranked]
                initial_time = self.clock()-started
                for seed in seeds:
                    left = remaining()
                    if left < self.fast.validation_reserve:
                        break
                    optimizer = MincoConfig(mode='fixed', budget=left, batch_validation=True)
                    best = FastFollowMinco(optimizer, self.model, self.fast, self.clock,
                                           self.wall_clock).solve(request, seed)
                    attempts.append(dict(status=best.solver_status, seed_jerk=(
                        seed.metrics['local_jerk']), valid=best.valid, solve_time=best.solve_time))
                    if best.valid:
                        self.hint = (request.context, seed.metrics['local_jerk'], until, rate)
                        break
        except (ValueError, TypeError, OverflowError) as error:
            best = MpcSeedResult(request.context, solver_status=str(error), reason=str(error))
        elapsed = self.clock()-started
        if best.valid and remaining() <= 0:
            best = replace(best, valid=False, solver_status='DEADLINE_EXCEEDED')
        if not best.valid:
            self.clear_warm_start()
        diagnostics = dict(duration=self.duration, terminal_policy='local_progress',
                           attempts=attempts, warm_used=warm_used, fresh_until=until,
                           remaining_after_planning=until-self.wall_clock(),
                           accepted_by_tracker=False, continuous_time_guarantee=False)
        return replace(best, solve_time=elapsed,
                       timing=dict(best.timing, initialization=initial_time, total=elapsed),
                       metrics=dict(best.metrics, research_mode=self.mode,
                                    short_follow=diagnostics))
