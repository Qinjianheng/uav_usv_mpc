"""Research factory integration with original ROS admission and immutable input epochs."""

from functools import partial
from importlib import import_module

import pytest

from test_follow_mpc_shadow_node import _pose, _prediction, _time
from test_follow_research_algorithms import request
from uav_control.controllers.follow_mpc_seed import MpcConfig


def test_future_factory_covers_prediction_when_navigation_precedes_source():
    module = import_module('uav_control.controllers.follow_mpc_shadow_node')
    prediction = module.prediction_from_message(_prediction(
        source_stamp=_time(100.03), observation_stamp=_time(100.01),
        generated_stamp=_time(100.031)))
    with pytest.raises(ValueError, match='PREDICTION_HORIZON'):
        module.make_shadow_request(_pose(), prediction, 7, 1, 100.04, MpcConfig())
    future = module.make_shadow_request(_pose(), prediction, 7, 1, 100.04, MpcConfig(),
                                        execution_lead=.15)
    assert future.context.execution_start_stamp == pytest.approx(100.19)
    assert future.context.navigation_stamp == 100.
    assert future.context.prediction_source_stamp == 100.03
    assert future.context.observation_stamp == 100.01
    assert module.completion_rejection(future, MpcConfig(), 100.16, 7, 2, .05, True)


@pytest.mark.parametrize('mode,valid', [
    ('greedy_seed', False), ('greedy_minco', True), ('mpc_seed', True)])
def test_one_factory_mode_uses_existing_input_and_common_result(mode, valid):
    from uav_control.controllers.follow_research_solver import FollowResearchSolver
    req = request()
    solver = FollowResearchSolver(mode, MpcConfig(solve_budget=2.))
    result = solver.solve(req)
    assert result.valid == valid, (result.reason, result.solver_status)
    assert result.context == req.context
    assert result.metrics['research_mode'] == mode
    solver.clear_warm_start()


def test_runner_solver_and_request_factory_are_injected_without_ros_node_copy():
    from test_follow_mpc_shadow_node import _Adapter, _Pool
    module = import_module('uav_control.controllers.follow_mpc_shadow_node')
    from uav_control.controllers.follow_research_solver import FollowResearchSolver
    solver = FollowResearchSolver('greedy_minco')
    factory = partial(module.make_shadow_request, execution_lead=.15)
    runner = module.ShadowResearchRunner(MpcConfig(), _Adapter(), _Pool(), lambda e: None,
                                         solver=solver, request_factory=factory)
    assert runner.solver is solver and runner.request_factory is factory


def test_original_mpc_rejects_future_epoch_instead_of_fabricating_matching_time():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_problem import future_request
    result = FollowMpcSeed().solve(future_request(request(), .15))
    assert not result.valid and result.solver_status == 'INVALID_INPUT'


def test_publication_rejects_missed_future_start_even_with_fresh_measurement():
    from uav_control.guidance.follow_problem import future_request
    module = import_module('uav_control.controllers.follow_mpc_shadow_node')
    future = future_request(request(), .02)
    assert module.completion_rejection(future, MpcConfig(), 100.03, 1, 0, .01, True) == (
        'FUTURE_EXECUTION_MISSED')
