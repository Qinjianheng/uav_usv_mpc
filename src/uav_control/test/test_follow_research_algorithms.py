"""Pure research planning contracts without ROS or simulation dependencies."""

from dataclasses import replace
import math

import numpy as np
import pytest

from uav_control.controllers.follow_mpc_seed import MpcRequest, PlanningContext


def request(speed=0.0):
    """Use known co-moving geometry with a measured level FRD orientation."""
    times = np.arange(0, 4.01, .1)
    context = PlanningContext(1, 1, 100., 100., 100., 100., 99.99, 1, 100.115)
    return MpcRequest(context, (-5, 0, -5, speed, 0, 0, 0, 0, 0, 0), tuple(times),
                      tuple((speed * t, 0, 0) for t in times),
                      tuple((speed, 0, 0) for t in times), tuple(map(tuple, np.eye(3))), 100.)


@pytest.mark.parametrize('rotation', [
    ((1, 0, 0), (0, 1, 0), (0, 0, -1)), ((math.nan, 0, 0), (0, 1, 0), (0, 0, 1))])
def test_illegal_actual_attitude_is_rejected(rotation):
    from uav_control.guidance.follow_problem import FollowProblem
    with pytest.raises(ValueError):
        FollowProblem(replace(request(), actual_rotation=rotation))


def test_future_boundary_cannot_be_tampered_or_reprojected():
    from uav_control.guidance.follow_problem import FollowProblem, future_request
    future = future_request(request(), .15)
    with pytest.raises(ValueError):
        FollowProblem(replace(future, state=(0,) * 10))
    with pytest.raises(ValueError):
        future_request(future, .15)


def test_target_heave_does_not_enter_constant_altitude_endpoint():
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    req = request(2.)
    req = replace(req, target_positions=tuple((2 * t, 0, .15 * math.sin(t))
                                              for t in req.prediction_times),
                  target_velocities=tuple((2., 0, .15 * math.cos(t))
                                          for t in req.prediction_times))
    seed = GreedyFollowInitializer().build(req)
    assert seed.valid_input
    assert seed.end[2] == -5.
    assert seed.end[5] == seed.end[8] == 0.


def test_final_dense_validation_rejects_unsafe_interior_with_safe_endpoints():
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    req = request()
    seed = replace(GreedyFollowInitializer().build(req), q=((-5, 3, -5), (-5, -3, -5)))
    result = FollowMincoOptimizer(MincoConfig(mode='fixed', budget=2.)).solve(req, seed)
    assert not result.valid and result.solver_status == 'DYNAMIC_INFEASIBLE'
    assert result.metrics['validation_samples'] > 50
    assert result.positions[0] == pytest.approx((-5, 0, -5))
    assert result.positions[-1] == pytest.approx((-5, 0, -5))


def test_disabling_visibility_cost_still_checks_whole_target_view():
    from scipy.spatial.transform import Rotation
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    req = replace(request(), state=(*request().state[:9], 1.1),
                  actual_rotation=tuple(map(tuple, Rotation.from_euler('z', 1.1).as_matrix())))
    seed = replace(GreedyFollowInitializer().build(req, method='simple'),
                   q=(req.state[:3], req.state[:3]), end=(*req.state[:3], *(0.,) * 6),
                   yaw=(1.1,) * 4)
    result = FollowMincoOptimizer(MincoConfig(
        mode='fixed', budget=2., visibility_weight=0.)).solve(req, seed)
    assert not result.valid and result.constraint_violations['visibility'] > 0
    assert result.solver_status == 'VISIBILITY_INFEASIBLE'
    assert all(v <= 1e-6 for k, v in result.constraint_violations.items() if k != 'visibility')


def test_exact_yaw_rate_bound_is_part_of_final_admission():
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    req = request()
    seed = replace(GreedyFollowInitializer().build(req), yaw=(0., .9, .9, 0.))
    result = FollowMincoOptimizer(MincoConfig(mode='fixed', budget=2.)).solve(req, seed)
    assert not result.valid
    assert result.constraint_violations['yaw_rate'] > 0


def test_future_execution_keeps_original_epochs_and_projects_pva():
    from uav_control.guidance.follow_problem import future_request
    original = replace(request(), state=(-5, 0, -5, 2, 0, 0, 1, 0, 0, .2))
    future = future_request(original, .2)
    assert future.context.navigation_stamp == 100.
    assert future.context.observation_stamp == 99.99
    assert future.context.execution_start_stamp == pytest.approx(100.2)
    assert future.state[:3] == pytest.approx((-4.58, 0, -5))
    assert future.state[3:6] == pytest.approx((2.2, 0, 0))
    assert future.measurement_state == original.state


@pytest.mark.parametrize('field,value', [
    ('observation_stamp', 99.), ('attitude_stamp', 99.9), ('frame_id', 'enu')])
def test_bad_original_input_is_not_repaired_by_future_execution(field, value):
    from uav_control.guidance.follow_problem import FollowProblem, future_request
    req = replace(request(), context=replace(request().context, **{field: value}))
    with pytest.raises(ValueError):
        FollowProblem(future_request(req, .2))


def test_future_horizon_rejects_missing_prediction_without_clamping():
    from uav_control.guidance.follow_problem import FollowProblem, future_request
    req = replace(request(), prediction_times=(0., .1),
                  target_positions=((0, 0, 0),) * 2, target_velocities=((0, 0, 0),) * 2)
    with pytest.raises(ValueError, match='PREDICTION_HORIZON'):
        FollowProblem(future_request(req, .2))


def test_yaw_wrap_continuity_clamped_rates_and_exact_extremum():
    from uav_control.guidance.yaw_trajectory import YawTrajectory
    yaw = YawTrajectory((0., 1.), (math.radians(170), math.radians(-170)))
    assert yaw.sample(.5)[0] == pytest.approx(math.pi)
    assert yaw.sample(1.)[0] == pytest.approx(math.radians(190))
    assert yaw.maximum_rate() == pytest.approx(math.radians(30))
    assert yaw.sample(0.)[1] == pytest.approx(0.)


@pytest.mark.parametrize('speed', [0., 2., 4.])
def test_greedy_outputs_two_waypoints_without_final_safety_claim(speed):
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    seed = GreedyFollowInitializer().build(request(speed))
    assert seed.valid_input, seed.reason
    assert len(seed.q) == 2 and len(seed.durations) == 3
    assert sum(seed.durations) == pytest.approx(2.4)
    assert seed.start == request(speed).state
    assert not seed.final_validated
    assert seed.end[:3] == pytest.approx((speed * 2.4 - 5, 0, -5))


def test_low_speed_heading_and_direction_hysteresis_do_not_chatter():
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    initializer = GreedyFollowInitializer()
    a = initializer.build(request(), previous_direction=.6)
    b = initializer.build(request(), previous_direction=a.directions[-1])
    assert a.valid_input and b.valid_input
    assert a.directions == b.directions


@pytest.mark.parametrize('mode', ['fixed', 'q', 'qt'])
def test_minco_modes_preserve_boundaries_and_hover_is_sampled_feasible(mode):
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    req = request()
    seed = GreedyFollowInitializer().build(req)
    optimizer = FollowMincoOptimizer(MincoConfig(
        mode=mode, maximum_iterations=2, budget=2., visibility_weight=0.))
    result = optimizer.solve(req, seed)
    assert result.valid, (result.reason, result.constraint_violations)
    assert result.positions[0] == pytest.approx(seed.start[:3])
    assert result.positions[-1] == pytest.approx(seed.end[:3])
    assert result.metrics['boundary_residual'] < 1e-7
    assert result.metrics['junction_residual'] < 1e-5
    assert result.metrics['jerk_integral'] == pytest.approx(0., abs=1e-9)


def test_q_optimization_changes_waypoints_and_reduces_real_jerk_objective():
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    req = request()
    seed = GreedyFollowInitializer().build(req)
    seed = replace(seed, q=((-5, .3, -5), (-5, -.3, -5)))
    result = FollowMincoOptimizer(MincoConfig(
        mode='q', maximum_iterations=8, budget=3., jerk_weight=.1)).solve(req, seed)
    assert result.metrics['final_objective'] < result.metrics['initial_objective'] - 1e-4
    assert not np.allclose(result.metrics['q'], seed.q)


def test_qt_optimization_retimes_unequal_segments_and_keeps_positive_total():
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    req = request(2.)
    seed = GreedyFollowInitializer().build(req)
    seed = replace(seed, durations=(.4, .8, 1.2))
    result = FollowMincoOptimizer(MincoConfig(
        mode='qt', maximum_iterations=4, budget=3.)).solve(req, seed)
    assert min(result.metrics['durations']) > 0.
    assert sum(result.metrics['durations']) == pytest.approx(2.4)
    assert not np.allclose(result.metrics['durations'], seed.durations)


def test_dense_validation_rejects_unsafe_initial_yaw_even_if_later_recovers():
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    req = replace(request(), state=(-5, 0, -5, 0, 0, 0, 0, 0, 0, 1.1))
    rotation = ((math.cos(1.1), -math.sin(1.1), 0),
                (math.sin(1.1), math.cos(1.1), 0), (0, 0, 1))
    req = replace(req, actual_rotation=rotation)
    seed = GreedyFollowInitializer().build(req, method='simple')
    result = FollowMincoOptimizer(MincoConfig(mode='fixed')).solve(req, seed)
    assert not result.valid
    assert result.solver_status in ('RECOVERY_CANDIDATE', 'VISIBILITY_INFEASIBLE',
                                    'DYNAMIC_INFEASIBLE')


def test_budget_timeout_does_not_return_a_valid_candidate():
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    result = FollowMincoOptimizer(MincoConfig(budget=1e-9)).solve(
        request(), GreedyFollowInitializer().build(request()))
    assert not result.valid and result.solver_status == 'DEADLINE_EXCEEDED'
