"""Dynamics, timestamps, safety admission and rolling seed tests."""

from dataclasses import replace
import math

import numpy as np
import pytest

from uav_control.controllers.follow_mpc_seed import (
    FollowMpcSeed, MpcConfig, MpcRequest, PlanningContext, integrate_state,
    rollout, to_minco_seed,
)


def request(state=None, velocity=(0, 0, 0), **context_changes):
    times = np.arange(41) * 0.1
    positions = np.outer(times, velocity)
    context = PlanningContext(1, 1, 100.0, 100.0, 100.0, 100.0, 99.99, 1, 100.125)
    context = replace(context, **context_changes)
    if state is None:
        state = (-5, 0, -5, *velocity, 0, 0, 0, 0)
    yaw = state[9]
    actual = ((math.cos(yaw), -math.sin(yaw), 0), (math.sin(yaw), math.cos(yaw), 0), (0, 0, 1))
    return MpcRequest(context, tuple(state), tuple(times), tuple(map(tuple, positions)),
                      tuple(map(tuple, np.tile(velocity, (41, 1)))), actual,
                      100.01)


def test_exact_third_order_integrator_and_unwrapped_yaw():
    x = np.array((1, 2, -3, 2, -1, .5, 3, -2, 1, math.pi - .1))
    result = integrate_state(x, (6, 3, -3, .5), .2)
    assert result[:3] == pytest.approx((1.468, 1.764, -2.884))
    assert result[3:6] == pytest.approx((2.72, -1.34, .64))
    assert result[6:9] == pytest.approx((4.2, -1.4, .4))
    assert result[9] == pytest.approx(math.pi)
    assert integrate_state(result, (0, 0, 0, .5), .2)[9] > math.pi


def test_rollout_has_exact_initial_state_and_known_constant_jerk():
    initial = np.zeros(10)
    controls = np.tile((3, 0, 0, .2), (5, 1))
    states = rollout(initial, controls, .2)
    assert states[0] == pytest.approx(initial)
    assert states[-1, :3] == pytest.approx((.5, 0, 0))
    assert states[-1, 3:9] == pytest.approx((1.5, 0, 0, 3, 0, 0))
    assert states[-1, 9] == pytest.approx(.2)


@pytest.mark.parametrize('speed', [0.0, 2.0, 4.0])
def test_stationary_or_co_moving_standoff_generates_strictly_safe_seed(speed):
    planner = FollowMpcSeed(MpcConfig(solve_budget=2.0, maximum_iterations=8))
    result = planner.solve(request(velocity=(speed, 0, 0)))
    assert result.valid, (result.solver_status, result.constraint_violations)
    assert len(result.relative_times) == 13
    assert result.context.execution_start_stamp == 100.0
    states = np.column_stack((result.positions, result.velocities, result.accelerations,
                              result.yaw_refs))
    for k, control in enumerate(np.column_stack((result.jerks, result.yaw_rates))):
        assert states[k + 1] == pytest.approx(integrate_state(states[k], control, .2), abs=1e-9)
    assert result.metrics['dynamics_residual'] < 1e-9
    assert result.metrics['visible_fraction'] == 1.0
    assert result.metrics['maximum_horizontal_speed'] <= 6.2 + 1e-6
    assert result.metrics['maximum_horizontal_acceleration'] <= 3.0 + 1e-6


def test_warm_start_shifts_controls_but_never_replaces_measured_start():
    planner = FollowMpcSeed(MpcConfig(solve_budget=2.0, maximum_iterations=4))
    first = planner.solve(request())
    assert first.valid
    second_request = request(cycle_id=2, execution_start_stamp=100.2, navigation_stamp=100.2,
                             attitude_stamp=100.2, prediction_source_stamp=100.2,
                             observation_stamp=100.19, prediction_valid_until=100.325)
    second_request = replace(second_request, now_stamp=100.21)
    second = planner.solve(second_request)
    assert second.warm_started
    assert second.positions[0] == second_request.state[:3]
    assert second.velocities[0] == second_request.state[3:6]
    other = replace(second_request, context=replace(second_request.context, mission_id=2))
    assert not planner.solve(other).warm_started


def test_expired_or_clock_reset_seed_cannot_warm_start():
    planner = FollowMpcSeed(MpcConfig(solve_budget=2.0, maximum_iterations=3))
    assert planner.solve(request()).valid
    newer = request(clock_generation=1)
    assert not planner.solve(newer).warm_started


def test_warm_shift_preserves_exact_control_knots_despite_float_roundoff():
    planner = FollowMpcSeed(MpcConfig(control_blocks=12))
    controls = np.zeros((12, 4))
    controls[:, 0] = np.arange(12) * .1
    previous = planner.evaluate_controls(request(), controls)
    assert previous.valid
    planner.last_valid = previous
    newer = request(execution_start_stamp=100.2, navigation_stamp=100.2,
                    attitude_stamp=100.2, prediction_source_stamp=100.2,
                    observation_stamp=100.19, prediction_valid_until=100.325)
    newer = replace(newer, now_stamp=100.21)
    shifted = planner._warm_controls(newer)
    assert shifted[:, 0] == pytest.approx([.1, .2, .3, .4, .5, .6, .7, .8, .9, 1, 1.1, 1.1])


@pytest.mark.parametrize('changes,status', [
    ({'observation_stamp': 99.0}, 'STALE_INPUT'),
    ({'navigation_stamp': 101.0}, 'INVALID_INPUT'),
    ({'attitude_stamp': 99.9}, 'INVALID_INPUT'),
    ({'prediction_source': 'truth'}, 'INVALID_INPUT'),
    ({'frame_id': 'world'}, 'INVALID_INPUT'),
    ({'prediction_valid_until': 100.0}, 'STALE_INPUT'),
])
def test_input_epoch_and_source_failures_are_specific(changes, status):
    result = FollowMpcSeed().solve(request(**changes))
    assert not result.valid and result.solver_status == status
    assert not result.positions


def test_prediction_horizon_is_never_extrapolated():
    req = request()
    req = replace(req, prediction_times=(0.0, 0.1),
                  target_positions=req.target_positions[:2],
                  target_velocities=req.target_velocities[:2])
    result = FollowMpcSeed().solve(req)
    assert result.solver_status == 'PREDICTION_HORIZON'


def test_invalid_prediction_samples_and_state_are_rejected():
    req = replace(request(), state=(math.nan,) * 10)
    assert FollowMpcSeed().solve(req).solver_status == 'INVALID_INPUT'


def test_missing_actual_attitude_cannot_be_replaced_by_planned_attitude():
    req = replace(request(), actual_rotation=None)
    planner = FollowMpcSeed()
    assert planner.solve(req).solver_status == 'INVALID_INPUT'
    assert planner.evaluate_controls(req, np.zeros((12, 4))).solver_status == 'INVALID_INPUT'
    req = replace(request(), prediction_times=tuple(reversed(request().prediction_times)))
    assert FollowMpcSeed().solve(req).solver_status == 'INVALID_INPUT'


def test_initial_speed_violation_is_dynamic_failure_not_optimizer_error():
    req = request(state=(-5, 0, -5, 10, 0, 0, 0, 0, 0, 0))
    result = FollowMpcSeed().solve(req)
    assert not result.valid and result.solver_status == 'DYNAMIC_INFEASIBLE'


def test_initial_loss_and_recoverable_yaw_candidate_are_not_safe():
    req = request(state=(-5, 0, -5, 0, 0, 0, 0, 0, 0, 1.5))
    planner = FollowMpcSeed(MpcConfig(solve_budget=2.0))
    controls = np.zeros((12, 4))
    controls[:, 3] = -.625
    result = planner.evaluate_controls(req, controls)
    assert not result.initial_visibility and not result.valid
    assert result.solver_status == 'RECOVERY_CANDIDATE'
    assert result.metrics['visible_fraction'] > 0
    with pytest.raises(ValueError):
        to_minco_seed(result)


def test_invalid_future_attitude_has_no_fabricated_horizontal_view():
    req = request(state=(-5, 0, -5, 0, 0, 0, 0, 0, 9.80665, 0))
    result = FollowMpcSeed().evaluate_controls(req, np.zeros((12, 4)))
    assert result.solver_status == 'DYNAMIC_INFEASIBLE'
    assert result.visibility_margins[1] == (None, None)


@pytest.mark.parametrize('north', [-.2, -40.0])
def test_range_or_view_failure_never_becomes_safe(north):
    req = request(state=(north, 0, -5, 0, 0, 0, 0, 0, 0, 0))
    controls = np.zeros((12, 4))
    result = FollowMpcSeed().evaluate_controls(req, controls)
    assert not result.valid and result.solver_status == 'VISIBILITY_INFEASIBLE'


def test_baseline_controls_obey_same_strict_jerk_and_sea_limits():
    controls = np.zeros((12, 4))
    controls[:, 0] = 20
    result = FollowMpcSeed().evaluate_controls(request(), controls)
    assert not result.valid and result.constraint_violations['horizontal_jerk'] > 0
    req = request(state=(-5, 0, -.2, 0, 0, 1, 0, 0, 0, 0))
    result = FollowMpcSeed().evaluate_controls(req, np.zeros((12, 4)))
    assert not result.valid and result.constraint_violations['sea_clearance'] > 0


def test_minco_seed_preserves_common_epoch_and_full_pva_boundaries():
    from uav_control.guidance.minco_trajectory import MincoS3Trajectory
    result = FollowMpcSeed().evaluate_controls(request(), np.zeros((12, 4)))
    assert result.valid
    seed = to_minco_seed(result, waypoint_stride=4)
    trajectory = MincoS3Trajectory(**{
        key: value for key, value in seed.items()
        if key not in ('context', 'yaw_refs', 'relative_times')})
    assert seed['context'] == result.context
    assert trajectory.duration == pytest.approx(result.relative_times[-1])
    for sample, index in ((trajectory.sample(0), 0), (trajectory.sample(trajectory.duration), -1)):
        assert sample.position == pytest.approx(result.positions[index])
        assert sample.velocity == pytest.approx(result.velocities[index])
        assert sample.acceleration == pytest.approx(result.accelerations[index])


def test_deadline_includes_preparation_and_never_reports_a_valid_late_seed():
    class TickingClock:
        def __init__(self):
            self.now = 0.0

        def __call__(self):
            self.now += .02
            return self.now

    planner = FollowMpcSeed(MpcConfig(solve_budget=.01), clock=TickingClock())
    result = planner.solve(request())
    assert result.solver_status == 'DEADLINE_EXCEEDED' and not result.valid


def test_final_elapsed_read_crossing_budget_is_rejected():
    class ClosingClock:
        def __init__(self, close_at=None):
            self.calls = 0
            self.close_at = close_at

        def __call__(self):
            self.calls += 1
            return .6 if self.close_at is not None and self.calls >= self.close_at else 0.0

    config = MpcConfig(maximum_iterations=1, solve_budget=.5)
    counter = ClosingClock()
    assert FollowMpcSeed(config, clock=counter).solve(request()).valid
    closing = ClosingClock(counter.calls)
    result = FollowMpcSeed(config, clock=closing).solve(request())
    assert not result.valid and result.solver_status == 'DEADLINE_EXCEEDED'


@pytest.mark.parametrize('config', [
    MpcConfig(dt=0), MpcConfig(control_blocks=13),
    MpcConfig(horizon_steps=2.5), MpcConfig(maximum_horizontal_speed=7)])
def test_configuration_rejects_invalid_or_loosened_baseline_limits(config):
    with pytest.raises(ValueError):
        FollowMpcSeed(config)
