"""Extend the existing shadow node with one selectable research algorithm."""

from functools import partial

import rclpy

from uav_control.controllers.follow_mpc_shadow_node import (
    FollowMpcShadowNode, ShadowResearchRunner, make_shadow_request,
)
from uav_control.controllers.follow_research_solver import FollowResearchSolver
from uav_control.guidance.follow_minco_optimizer import MincoConfig


class FollowResearchShadowNode(FollowMpcShadowNode):
    """Reuse all synchronization, subscriptions, task handling and admission checks."""

    node_name = 'follow_research_shadow_node'

    def make_runner(self, config):
        """Inject a single solver and explicit execution boundary into the original runner."""
        for name, default in (('research_mode', 'greedy_minco'), ('execution_lead', .15),
                              ('minco_mode', 'qt'), ('minco_iterations', 2),
                              ('visibility_weight', 20.), ('minco_engine', 'legacy'),
                              ('initializer_strategy', 'legacy'), ('greedy_strategy', 'D'),
                              ('dynamic_endpoint', True), ('fast_feasible_seed', True),
                              ('greedy_minimum_distance', 3.), ('greedy_maximum_distance', 10.),
                              ('p32_profile_every', 0), ('p32_detailed_diagnostics', False),
                              ('p32_initialization_cap', .012), ('short_horizon', 1.2),
                              ('short_rolling_hint', True)):
            self.declare_parameter(name, default)
        mode = self.get_parameter('research_mode').value
        optimizer = MincoConfig(mode=self.get_parameter('minco_mode').value,
                                maximum_iterations=self.get_parameter('minco_iterations').value,
                                visibility_weight=self.get_parameter('visibility_weight').value,
                                budget=config.solve_budget)
        optimizer.validate()
        engine = self.get_parameter('minco_engine').value
        if engine not in ('legacy', 'p31', 'p32', 'p32_short', 'p33_progress'):
            raise ValueError('unsupported minco engine')
        fast, greedy = None, None
        initialization = self.get_parameter('initializer_strategy').value
        if engine in ('p31', 'p32', 'p32_short', 'p33_progress'):
            from uav_control.guidance.fast_follow_minco import FastConfig
            from uav_control.guidance.adaptive_follow_initializer import GreedyConfig
            fast = FastConfig(freshness_budget=True, fast_feasible_seed=bool(
                self.get_parameter('fast_feasible_seed').value))
            greedy = GreedyConfig(strategy=self.get_parameter('greedy_strategy').value,
                                  dynamic_endpoint=bool(
                                      self.get_parameter('dynamic_endpoint').value),
                                  minimum_distance=float(
                                      self.get_parameter('greedy_minimum_distance').value),
                                  maximum_distance=float(
                                      self.get_parameter('greedy_maximum_distance').value))
        elif initialization != 'legacy':
            raise ValueError('new initialization requires explicit p31 engine')
        solver = FollowResearchSolver(mode, config, optimizer, fast, greedy, initialization)
        if engine in ('p32', 'p32_short', 'p33_progress'):
            if mode != 'greedy_minco':
                raise ValueError('p32 requires greedy_minco shadow mode')
            if engine in ('p32_short', 'p33_progress'):
                from uav_control.controllers.short_follow_solver import ShortHorizonFollowSolver
                if engine == 'p33_progress':
                    from uav_control.controllers.progress_follow_solver import ProgressFollowSolver
                    selected_solver = ProgressFollowSolver
                else:
                    selected_solver = ShortHorizonFollowSolver
                solver = selected_solver(solver.model, duration=float(
                    self.get_parameter('short_horizon').value), rolling=bool(
                    self.get_parameter('short_rolling_hint').value), wall_clock=lambda: (
                    self.get_clock().now().nanoseconds/1e9))
            else:
                from uav_control.controllers.realtime_follow_solver import RealtimeFollowSolver
                solver = RealtimeFollowSolver(solver.model, optimizer, wall_clock=lambda: (
                    self.get_clock().now().nanoseconds/1e9), profile_every=int(
                        self.get_parameter('p32_profile_every').value), detailed=bool(
                        self.get_parameter('p32_detailed_diagnostics').value),
                    initialization_cap=float(
                            self.get_parameter('p32_initialization_cap').value))
        factory = make_shadow_request
        if mode != 'mpc_seed':
            lead = float(self.get_parameter('execution_lead').value)
            if not 0 <= lead <= .3:
                raise ValueError('execution lead must be in [0, .3]')
            factory = partial(make_shadow_request, execution_lead=lead)
        return ShadowResearchRunner(config, self.adapter, self.pool, self._publish_event,
                                    float(self.get_parameter('shadow_rate_hz').value),
                                    solver, factory)


def main(args=None):
    """Run a single research mode, keeping all flight control in the original tracker."""
    rclpy.init(args=args)
    node = None
    try:
        node = FollowResearchShadowNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
