"""One research solver protocol for greedy seeds, visibility MINCO and original MPC."""

from dataclasses import asdict, replace
import time

from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig, MpcSeedResult
from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer


class FollowResearchSolver:
    """Keep the P2 research result/provenance protocol without copying its ROS runner."""

    def __init__(self, mode='greedy_minco', config=MpcConfig(), optimizer_config=None):
        """Only implemented modes are selectable; preserve original MPC optimization rules."""
        if mode not in ('greedy_seed', 'greedy_minco', 'mpc_seed'):
            raise ValueError('unsupported research mode')
        self.mode, self.config = mode, config
        self.model = FollowMpcSeed(config)
        for name in ('intrinsics', 'extrinsics', 'target', 'visibility', 'attitude_config'):
            setattr(self, name, getattr(self.model, name))
        self.optimizer_config = optimizer_config or MincoConfig(budget=config.solve_budget)
        self.initializer = GreedyFollowInitializer(self.model)
        self.previous_direction = 0.

    def clear_warm_start(self):
        """Clear retained direction and original MPC warm state on input/mission rejection."""
        self.previous_direction = 0.
        self.model.clear_warm_start()

    def solve(self, request):
        """Include initialization and strict MINCO validation in the entire research budget."""
        start = time.perf_counter()
        if self.mode == 'mpc_seed':
            result = self.model.solve(request)
        else:
            seed = self.initializer.build(request, self.previous_direction)
            seed_time = time.perf_counter() - start
            if not seed.valid_input:
                result = MpcSeedResult(request.context, solver_status=seed.reason,
                                       reason=seed.reason)
            elif self.mode == 'greedy_seed':
                result = MpcSeedResult(
                    request.context, solver_status='COARSE_SEED',
                    reason='final validation pending',
                    relative_times=(0., .8, 1.6, 2.4),
                    positions=(seed.start[:3], *seed.q, seed.end[:3]),
                    metrics={'seed': asdict(seed)})
            else:
                remaining = self.config.solve_budget - seed_time
                if remaining <= 0:
                    result = MpcSeedResult(request.context, solver_status='DEADLINE_EXCEEDED')
                else:
                    result = FollowMincoOptimizer(
                        replace(self.optimizer_config, budget=remaining), self.model).solve(
                            request, seed)
                    if result.valid:
                        self.previous_direction = seed.directions[-2]
            timing = dict(result.timing, initialization=seed_time)
            result = replace(result, timing=timing)
        elapsed = time.perf_counter() - start
        if elapsed >= self.config.solve_budget:
            result = MpcSeedResult(request.context, solver_status='DEADLINE_EXCEEDED',
                                   reason='whole research planning budget expired')
        return replace(result, solve_time=elapsed,
                       metrics=dict(result.metrics, research_mode=self.mode),
                       timing=dict(result.timing, total=elapsed))
