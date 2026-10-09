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
                              ('visibility_weight', 20.)):
            self.declare_parameter(name, default)
        mode = self.get_parameter('research_mode').value
        optimizer = MincoConfig(mode=self.get_parameter('minco_mode').value,
                                maximum_iterations=self.get_parameter('minco_iterations').value,
                                visibility_weight=self.get_parameter('visibility_weight').value,
                                budget=config.solve_budget)
        optimizer.validate()
        solver = FollowResearchSolver(mode, config, optimizer)
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
