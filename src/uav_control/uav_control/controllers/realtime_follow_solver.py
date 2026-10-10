"""Opt-in P3.2 tiers, preserving raw 125ms epochs and independent final admission."""
from dataclasses import replace
import cProfile
import math
import time

from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.guidance.adaptive_follow_initializer import (
    AdaptiveFollowInitializer, GreedyConfig,
)
from uav_control.guidance.fast_follow_minco import FastFollowMinco, FastConfig, freshness
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
from uav_control.guidance.start_jerk import adjust_start


class RealtimeFollowSolver:
    """L0 fixed rear, L1 three angles, L2 bounded adaptive, L3 gradient only with reserve."""

    def __init__(self, model, optimizer=None, clock=time.perf_counter, wall_clock=time.time,
                 publication_reserve=.02, validation_reserve=.025, detailed=False,
                 profile_every=0, initialization_cap=.012):
        """Use ROS epoch clock from the node; monotonic clock measures computation only."""
        self.model, self.config = model, model.config
        self.mode = 'greedy_minco'
        self.optimizer = optimizer or MincoConfig(budget=self.config.solve_budget)
        self.clock, self.wall_clock = clock, wall_clock
        self.fast = FastConfig(publication_reserve=publication_reserve,
                               validation_reserve=validation_reserve)
        self.fast.validate()
        self.detailed = detailed
        if not isinstance(profile_every, int) or profile_every < 0:
            raise ValueError('INVALID_PROFILE_INTERVAL')
        self.profile_every, self.cycles = profile_every, 0
        if not math.isfinite(initialization_cap) or initialization_cap <= 0:
            raise ValueError('INVALID_INITIALIZATION_CAP')
        self.initialization_cap = initialization_cap
        self.previous_direction = 0.
        for name in ('intrinsics', 'extrinsics', 'target', 'visibility', 'attitude_config'):
            setattr(self, name, getattr(model, name))

    def clear_warm_start(self):
        """No stale candidate or invalid task retains a direction."""
        self.previous_direction = 0.
        self.model.clear_warm_start()

    def solve(self, request):
        """Return only research results, with no acknowledgement from the actual Tracker."""
        started = self.clock()
        stages, seed, selected = [], None, None
        initial_now = self.wall_clock()
        until, initial_remaining = freshness(request, initial_now)
        best = MpcSeedResult(request.context, solver_status='NO_FRESHNESS_BUDGET')
        initialization = 0.
        self.cycles += 1
        profiler = cProfile.Profile() if self.profile_every and (
            self.cycles % self.profile_every == 0) else None

        def remaining():
            now = self.wall_clock()
            if not math.isfinite(now) or now < request.now_stamp-1e-9:
                return -1.
            return min(self.config.solve_budget-(self.clock()-started),
                       until-now-self.fast.publication_reserve)

        for level in ('L0', 'L1', 'L2', 'L3'):
            available = remaining()
            required = self.fast.validation_reserve + (.008 if level == 'L3' else 0.)
            if available < required:
                break
            before = self.clock()
            init_time = 0.
            if level != 'L3':
                init_start = self.clock()
                if profiler:
                    profiler.enable()
                deadline = self.clock()+min(available-self.fast.validation_reserve,
                                            self.initialization_cap)
                if level == 'L0':
                    seed = GreedyFollowInitializer(self.model).build(request, method='simple')
                else:
                    config = GreedyConfig(strategy='B' if level == 'L1' else 'D', batch=True,
                                          feasible_first=True, two_level=True, start_score=True,
                                          maximum_angles=3, maximum_distances=2,
                                          detailed=self.detailed)
                    seed = AdaptiveFollowInitializer(self.model, config).build(
                        request, self.previous_direction, deadline=deadline, clock=self.clock)
                if seed.valid_input and level != 'L0' and remaining() > required+.004:
                    seed = adjust_start(seed, mode='q' if level == 'L1' else 'combined',
                                        deadline=deadline, clock=self.clock)
                if profiler:
                    profiler.disable()
                init_time = self.clock()-init_start
                initialization += init_time
            if seed is None or not seed.valid_input:
                best = MpcSeedResult(request.context, solver_status=(
                    seed.reason if seed is not None else 'NO_SEED'))
            elif remaining() >= self.fast.validation_reserve:
                solver = FastFollowMinco(replace(self.optimizer, mode=(
                    self.optimizer.mode if level == 'L3' else 'fixed'), budget=remaining(),
                    batch_validation=True), self.model, self.fast, self.clock, self.wall_clock)
                best = solver.solve(request, seed)
            else:
                best = MpcSeedResult(request.context, solver_status='NO_VALIDATION_BUDGET')
            stages.append(dict(tier=level, initialization=init_time,
                               elapsed=self.clock()-before, remaining_before=available,
                               remaining_after=remaining(), status=best.solver_status,
                               valid=best.valid, seed_metrics=seed.metrics if seed else {},
                               q=seed.q if seed else (), durations=seed.durations if seed else (),
                               yaw=seed.yaw if seed else (), end=seed.end if seed else ()))
            if best.valid:
                selected = level
                self.previous_direction = seed.directions[-2]
                break
        elapsed = self.clock()-started
        if best.valid and (remaining() <= 0 or elapsed >= self.config.solve_budget):
            best = replace(best, valid=False, solver_status='DEADLINE_EXCEEDED')
        if not best.valid:
            self.clear_warm_start()
        diagnostics = dict(stages=stages, selected_tier=selected, fresh_until=until,
                           initialization_cap=self.initialization_cap,
                           remaining_before_initialization=initial_remaining,
                           remaining_after_planning=until-self.wall_clock(),
                           execution_ttl_gap=request.context.prediction_valid_until-(
                               request.context.execution_start_stamp),
                           accepted_by_tracker=False,
                           handover_dry_run='OLD_TRAJECTORY_UNAVAILABLE',
                           continuous_time_guarantee=False)
        if profiler:
            diagnostics['initialization_profile'] = [dict(
                function=entry.code.co_name, file=entry.code.co_filename,
                calls=entry.callcount, self_seconds=entry.inlinetime,
                cumulative_seconds=entry.totaltime) for entry in profiler.getstats()
                if not isinstance(entry.code, str) and 'uav_control' in entry.code.co_filename]
        return replace(best, solve_time=elapsed,
                       timing=dict(best.timing, initialization=initialization, total=elapsed),
                       metrics=dict(best.metrics, realtime=diagnostics, research_mode=self.mode))
