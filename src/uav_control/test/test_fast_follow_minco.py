"""Independent polynomial and full-state research planning checks."""
import sys
from pathlib import Path

import numpy as np
import pytest

from uav_control.guidance.polynomial_extrema import derivative_peak

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))


def test_internal_jerk_peak():
    # p''' = 4 - (t - 1)^2; maximum 4 at t=1, endpoints 3.
    c = np.zeros((1, 6, 3))
    c[0, 3:6, 0] = (3 / 6, 2 / 24, -1 / 60)
    peak = derivative_peak(c, (2.,), 3, (0, 1))
    assert abs(peak['value'] - 4) < 1e-10
    assert abs(peak['time'] - 1) < 1e-10
    assert peak['location'] == 'interior'


def test_peak_at_junction_and_end():
    c = np.zeros((2, 6, 3))
    c[0, 4, 0] = 1 / 24
    c[1, 3, 0] = 2 / 6
    peak = derivative_peak(c, (1., 1.), 3, (0, 1))
    assert peak['value'] == 2
    assert peak['location'] == 'junction'


def test_rotating_reference_derivatives():
    from uav_control.guidance.follow_reference import viewpoint_state
    # Circle heading north, clockwise 0.2 rad/s, rear distance 5:
    # rear reference adds -1 eastward velocity and +.2 north acceleration.
    p, v, a = viewpoint_state((0, 0, 0), (2, 0, 0), (0, .4, 0),
                              0., .2, 0., distance=5.)
    assert np.allclose(p, (-5, 0, -5))
    assert np.allclose(v, (2, -1, 0))
    assert np.allclose(a, (.2, .4, 0))


def test_adaptive_distance_derivatives_independent_difference():
    from uav_control.guidance.follow_reference import viewpoint_state

    def position(t):
        d, theta = 5 + .3 * t + .1 * t*t, .4 + .2*t + .03*t*t
        return np.array((2*t - d*np.cos(theta), -d*np.sin(theta), -5))
    _, v, a = viewpoint_state((0, 0, 0), (2, 0, 0), (0, 0, 0), .4, .2, .06,
                              distance=5, distance_rate=.3, distance_acceleration=.2)
    h = 1e-4
    assert np.allclose(v, (position(h)-position(-h))/(2*h), atol=1e-7)
    assert np.allclose(a, (position(h)-2*position(0)+position(-h))/h**2, atol=1e-6)


@pytest.mark.parametrize('name', ['constant_velocity', 'right_turn', 'figure_eight',
                                  'fov_edge', 'wrong_yaw'])
def test_adjoint_gradient_matches_central_difference(name):
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.follow_minco_optimizer import MincoConfig
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.fast_minco_objective import FastMincoObjective
    from follow_minco_experiment import scenario
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    request = scenario(name)
    seed = GreedyFollowInitializer(model).build(request, method='simple')
    objective = FastMincoObjective(FollowProblem(request, model), seed, MincoConfig())
    x = objective.initial()
    x[6:8] = (.12, -.2)
    cost, gradient = objective(x)
    numeric = []
    for i in range(len(x)):
        step = 1e-5
        left, right = x.copy(), x.copy()
        left[i] -= step
        right[i] += step
        numeric.append((objective(right)[0]-objective(left)[0])/(2*step))
    assert np.allclose(gradient, numeric, rtol=2e-4, atol=2e-4), (gradient, numeric, cost)


def test_fast_fixed_seed_is_fully_validated_and_skips_optimizer():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.follow_minco_optimizer import MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.fast_follow_minco import FastFollowMinco
    from follow_minco_experiment import scenario
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    request = scenario('constant_velocity')
    seed = GreedyFollowInitializer(model).build(request, method='simple')
    result = FastFollowMinco(MincoConfig(), model).solve(request, seed)
    assert result.valid
    assert result.metrics['full_projection_validated']
    assert result.metrics['fast']['optimization_calls'] == 0
    assert result.metrics['fast']['optimization_skipped_reason'] == 'VALIDATED_FIXED_SEED'


def test_negative_fresh_budget_and_original_stamps():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.follow_minco_optimizer import MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.fast_follow_minco import FastFollowMinco, FastConfig
    from follow_minco_experiment import scenario
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    request = scenario('constant_velocity')
    seed = GreedyFollowInitializer(model).build(request)
    result = FastFollowMinco(MincoConfig(), model, FastConfig(freshness_budget=True),
                             wall_clock=lambda: request.now_stamp+.2).solve(request, seed)
    assert not result.valid
    assert result.solver_status == 'NO_FRESHNESS_BUDGET'
    assert result.context == request.context


def test_adaptive_candidates_bounded_and_common_boundary():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.adaptive_follow_initializer import AdaptiveFollowInitializer
    from follow_minco_experiment import scenario
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    request = scenario('right_turn')
    common = GreedyFollowInitializer(model).build(request, method='simple')
    seed = AdaptiveFollowInitializer(model).build(request, common_endpoint=common.end)
    assert seed.valid_input and seed.end == common.end
    assert len(seed.metrics['candidates']) <= 45
    assert all(3 <= c['distance'] <= 10 for c in seed.metrics['candidates'])
    assert not seed.final_validated


def test_proxy_optical_and_whole_sphere_sign_matches_p1():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.follow_minco_optimizer import MincoConfig
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.fast_minco_objective import FastMincoObjective
    from uav_control.guidance.camera_visibility import evaluate_visibility
    from uav_control.guidance.planned_attitude import planned_attitude
    from follow_minco_experiment import scenario
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    request = scenario('constant_velocity')
    seed = GreedyFollowInitializer(model).build(request, method='simple')
    objective = FastMincoObjective(FollowProblem(request, model), seed, MincoConfig())
    for position in ((-5, 0, -5), (-5, -8, -5), (-5, 0, -15), (5, 0, -5)):
        for acceleration, yaw in (((0, 0, 0), 0.), ((1, -2, .2), .4)):
            optical, margins, _, _ = objective.geometry(np.array([position]), np.array([
                acceleration]), np.array([yaw]), np.zeros((1, 3)))
            rotation = planned_attitude(acceleration, yaw).rotation_frd_to_ned
            measured = evaluate_visibility(position, rotation,
                                           (0, 0, 0), model.intrinsics, model.extrinsics,
                                           model.target, model.visibility)
            assert np.allclose(optical[0], measured.target_camera_optical)
            assert bool(np.all(margins >= 0)) == measured.whole_target_safe


def test_feasible_retention_after_unsafe_optimization(monkeypatch):
    from dataclasses import replace
    from types import SimpleNamespace
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.fast_follow_minco import FastFollowMinco, FastConfig
    import uav_control.guidance.fast_follow_minco as fast_module
    from follow_minco_experiment import scenario
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    request = scenario('constant_velocity')
    seed = GreedyFollowInitializer(model).build(request, method='simple')
    safe = FollowMincoOptimizer(MincoConfig(mode='fixed'), model).solve(request, seed)
    responses = iter((safe, replace(safe, valid=False, solver_status='DYNAMIC_INFEASIBLE')))
    monkeypatch.setattr(FollowMincoOptimizer, 'solve', lambda *a: next(responses))
    monkeypatch.setattr(fast_module, 'minimize', lambda fun, x, **kwargs: SimpleNamespace(
        x=x, fun=-1., nit=1, nfev=1, njev=1))
    result = FastFollowMinco(MincoConfig(mode='q'), model, FastConfig(
        fast_feasible_seed=False, exact_precheck=False)).solve(request, seed)
    assert result.valid and result.positions == safe.positions
    assert result.metrics['fast']['unsafe_lower_cost_discarded']


def test_timeout_is_not_labelled_full_projection_validated(monkeypatch):
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig, MpcSeedResult
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.fast_follow_minco import FastFollowMinco, FastConfig
    from follow_minco_experiment import scenario
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    request = scenario('constant_velocity')
    seed = GreedyFollowInitializer(model).build(request, method='simple')
    monkeypatch.setattr(FollowMincoOptimizer, 'solve', lambda *a: MpcSeedResult(
        request.context, solver_status='DEADLINE_EXCEEDED'))
    result = FastFollowMinco(MincoConfig(mode='fixed'), model, FastConfig(
        exact_precheck=False)).solve(request, seed)
    assert not result.valid and not result.metrics['full_projection_validated']


def test_adaptive_seed_diagnostics_use_native_json_scalars():
    import json
    from dataclasses import asdict
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.adaptive_follow_initializer import AdaptiveFollowInitializer
    from follow_minco_experiment import scenario
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    seed = AdaptiveFollowInitializer(model).build(scenario('right_turn'))
    assert seed.valid_input
    json.dumps(asdict(seed), allow_nan=False)


def test_gradient_near_duration_bounds_and_yaw_wrap():
    from dataclasses import replace
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.follow_minco_optimizer import MincoConfig
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.fast_minco_objective import FastMincoObjective
    from follow_minco_experiment import scenario
    request = scenario('constant_velocity')
    request = replace(request, state=(*request.state[:9], 3.3))
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    seed = GreedyFollowInitializer(model).build(request, method='simple')
    objective = FastMincoObjective(FollowProblem(request, model), seed, MincoConfig())
    x = objective.initial()
    x[6:8] = (1.499, -1.499)
    _, gradient = objective(x)
    numeric = []
    for i in range(len(x)):
        left, right = x.copy(), x.copy()
        left[i] -= 1e-5
        right[i] += 1e-5
        numeric.append((objective(right)[0]-objective(left)[0])/2e-5)
    assert np.allclose(gradient, numeric, rtol=2e-4, atol=2e-4)


def test_low_speed_reference_is_bounded():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.follow_reference import forecast_viewpoint
    from follow_minco_experiment import scenario
    request = scenario('stationary')
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    p, v, a, yaw, diagnostic = forecast_viewpoint(FollowProblem(request, model), 2.4)
    assert np.all(np.isfinite(np.r_[p, v, a, yaw]))
    assert np.allclose(v, 0) and np.allclose(a, 0)
    assert diagnostic['heading_rate'] == 0


def test_research_distance_parameter_bounds():
    from uav_control.guidance.adaptive_follow_initializer import GreedyConfig
    GreedyConfig(minimum_distance=4., maximum_distance=8.).validate()
    for lo, hi in ((0, 10), (3, 11), (6, 8), (3, float('nan'))):
        with pytest.raises(ValueError):
            GreedyConfig(minimum_distance=lo, maximum_distance=hi).validate()
