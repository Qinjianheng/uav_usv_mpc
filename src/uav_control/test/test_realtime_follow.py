"""P3.2 numerical equivalence, deadline and fail-closed handover tests."""
from dataclasses import replace, asdict
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from follow_minco_experiment import scenario  # noqa: E402
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig  # noqa: E402
from uav_control.guidance.adaptive_follow_initializer import (  # noqa: E402
    AdaptiveFollowInitializer, GreedyConfig,
)


def model():
    return FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))


@pytest.mark.parametrize('name', ['constant_velocity', 'right_turn', 'figure_eight',
                                  'wrong_yaw', 'stationary'])
def test_batched_candidates_preserve_every_scalar_field(name):
    m, request = model(), scenario(name)
    old = AdaptiveFollowInitializer(m).build(request)
    new = AdaptiveFollowInitializer(m, GreedyConfig(batch=True)).build(request)
    assert new.valid_input == old.valid_input
    assert np.allclose(new.q, old.q, atol=1e-12)
    assert np.allclose(new.end, old.end, atol=1e-12)
    for x, y in zip(old.metrics['candidates'], new.metrics['candidates']):
        for key in ('p', 'v', 'a', 'yaw', 'cost', 'fov'):
            assert np.allclose(x[key], y[key], atol=1e-10)
        assert x['feasible_coarse'] == y['feasible_coarse']
        assert x['selected'] == y['selected']
        assert x['violations'].keys() == y['violations'].keys()
        assert np.allclose(list(x['violations'].values()), list(y['violations'].values()))


def test_batch_geometry_is_same_p1_with_bad_rotations_and_back_targets():
    from uav_control.guidance.camera_visibility_batch import visibility_batch
    from uav_control.guidance.camera_visibility import evaluate_visibility
    m = model()
    p = np.array([[-5, 0, -5], [5, 0, -5], [-5, 8, -5], [-5, 0, -5.]])
    rotations = np.repeat(np.eye(3)[None], 4, axis=0)
    rotations[-1, 0, 0] = 2.
    batch = visibility_batch(p, rotations, np.zeros((4, 3)), m.intrinsics,
                             m.extrinsics, m.target, m.visibility)
    for i, result in enumerate(batch):
        scalar = evaluate_visibility(p[i], rotations[i], (0, 0, 0), m.intrinsics,
                                     m.extrinsics, m.target, m.visibility)
        assert asdict(result) == asdict(scalar)


def test_vectorized_viewpoints_match_scalar_kinematics():
    from uav_control.guidance.follow_reference import viewpoint_state, viewpoint_state_batch
    betas, distances = np.array((-.6, 0., .8)), np.array((3., 5., 10.))
    batch = viewpoint_state_batch((1, 2, 3), (2, 1, .2), (.1, -.3, .4),
                                  .8, .3, -.2, distances, betas, -5.)
    for k, (beta, distance) in enumerate(zip(betas, distances)):
        scalar = viewpoint_state((1, 2, 3), (2, 1, .2), (.1, -.3, .4), .8, .3, -.2,
                                 distance=distance, beta=beta)
        for computed, expected in zip(batch, scalar):
            assert np.allclose(computed[k], expected, atol=1e-12)


def test_feasible_priority_and_bounded_candidate_count():
    request = scenario('right_turn')
    seed = AdaptiveFollowInitializer(model(), GreedyConfig(
        batch=True, feasible_first=True, maximum_angles=3, maximum_distances=2)).build(request)
    assert seed.metrics['candidate_count'] <= 18
    for knot in (1, 2, 3):
        rows = [r for r in seed.metrics['candidates'] if r['knot'] == knot]
        if any(r['feasible_coarse'] for r in rows):
            assert next(r for r in rows if r['selected'])['feasible_coarse']


def test_initializer_can_stop_before_full_candidate_search():
    seed = AdaptiveFollowInitializer(model(), GreedyConfig(batch=True)).build(
        scenario('constant_velocity'), deadline=0., clock=lambda: 1.)
    assert not seed.valid_input and seed.reason == 'INITIALIZATION_DEADLINE'


def test_start_adjustment_preserves_pva_endpoint_total_and_reduces_start_jerk():
    from uav_control.guidance.start_jerk import adjust_start, start_jerk
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    request = scenario('right_turn')
    seed = GreedyFollowInitializer(model()).build(request, method='simple')
    adjusted = adjust_start(seed, mode='combined')
    assert adjusted.start == seed.start and adjusted.end == seed.end
    assert sum(adjusted.durations) == pytest.approx(sum(seed.durations))
    assert np.linalg.norm(start_jerk(adjusted)[:2]) <= np.linalg.norm(start_jerk(seed)[:2])+1e-9
    assert not adjusted.final_validated


def test_budget_rejects_before_initializer_and_keeps_source_stamps():
    from uav_control.controllers.realtime_follow_solver import RealtimeFollowSolver
    request = scenario('constant_velocity')
    solver = RealtimeFollowSolver(model(), wall_clock=lambda: request.now_stamp+.2)
    result = solver.solve(request)
    assert result.solver_status == 'NO_FRESHNESS_BUDGET'
    assert result.context == request.context
    assert result.metrics['realtime']['stages'] == []


def test_simple_tier_requires_full_geometry_validation():
    from uav_control.controllers.realtime_follow_solver import RealtimeFollowSolver
    request = scenario('constant_velocity')
    result = RealtimeFollowSolver(model(), wall_clock=lambda: request.now_stamp).solve(request)
    assert result.valid and result.metrics['full_projection_validated']
    assert result.metrics['realtime']['selected_tier'] == 'L0'
    assert not result.metrics['realtime']['accepted_by_tracker']


def test_handover_never_invents_an_accepted_old_trajectory():
    from uav_control.guidance.follow_handover import CandidateContract, dry_run
    candidate = CandidateContract('new', 1, 2, 100., 100.1, 102.5, 100.12,
                                  104., (0.,)*11, True)
    result = dry_run(candidate, None, 100.05, 1, 2, True)
    assert not result.eligible and result.reason == 'OLD_TRAJECTORY_UNAVAILABLE'
    assert not result.accepted_by_tracker


def test_handover_continuity_epoch_expiry_and_mission():
    from uav_control.guidance.follow_handover import CandidateContract, OldContract, dry_run
    old = OldContract('old', 1, 2, 99., 101., 'ACCEPTED', (0.,)*11, 100.1)
    c = CandidateContract('new', 1, 2, 100., 100.1, 102.5, 100.12, 104., (0.,)*11, True,
                          holding_valid_until=102.5, safety_model_id='offline_test_only')
    assert dry_run(c, old, 100.05, 1, 2, True).eligible
    for candidate, now, mission, generation, expected in (
            (c, 100.13, 1, 2, 'INPUT_EXPIRED'),
            (c, 100.05, 2, 2, 'MISSION_CHANGED'),
            (c, 100.05, 1, 3, 'CLOCK_GENERATION_CHANGED'),
            (replace(c, state='EXPIRED'), 100.05, 1, 2, 'CANDIDATE_EXPIRED'),
            (replace(c, handover_state=(1.,)+(0.,)*10), 100.05, 1, 2, 'HANDOVER_DISCONTINUITY'),
            (replace(c, execution_end=105.), 100.05, 1, 2, 'PREDICTION_COVERAGE')):
        assert dry_run(candidate, old, now, mission, generation, True).reason == expected


def test_coverage_is_not_a_holding_authorization():
    from uav_control.guidance.follow_handover import CandidateContract, OldContract, dry_run
    old = OldContract('old', 1, 2, 99., 101., 'ACCEPTED', (0.,)*11, 100.1)
    candidate = CandidateContract('new', 1, 2, 100., 100.1, 102.5, 100.12,
                                  104., (0.,)*11, True)
    assert dry_run(candidate, old, 100.05, 1, 2, True).reason == 'HOLDING_CONTRACT_UNESTABLISHED'


def test_expired_contract_cannot_resurrect_and_shadow_cannot_acknowledge():
    from uav_control.guidance.follow_handover import transition
    assert transition('PROPOSED', 'VALIDATED') == 'VALIDATED'
    assert transition('VALIDATED', 'ELIGIBLE') == 'ELIGIBLE'
    for state, target in (('ELIGIBLE', 'ACCEPTED'), ('EXPIRED', 'ACTIVE'),
                          ('EXPIRED', 'VALIDATED'), ('PROPOSED', 'ACTIVE')):
        with pytest.raises(ValueError):
            transition(state, target)


@pytest.mark.parametrize('name', ['constant_velocity', 'right_turn', 'figure_eight',
                                  'fov_edge', 'wrong_yaw'])
def test_batch_final_validation_retains_full_p1_decision(name):
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    request, m = scenario(name), model()
    seed = GreedyFollowInitializer(m).build(request, method='simple')
    a = FollowMincoOptimizer(MincoConfig(mode='fixed'), m).solve(request, seed)
    b = FollowMincoOptimizer(MincoConfig(mode='fixed', batch_validation=True), m).solve(
        request, seed)
    assert a.valid == b.valid and a.solver_status == b.solver_status
    assert np.allclose(a.visibility_margins, b.visibility_margins, atol=1e-12)
    assert a.metrics['validation_samples'] == b.metrics['validation_samples']


def test_start_jerk_sensitivity_includes_both_waypoints_and_initial_state():
    from uav_control.guidance.start_jerk import start_jerk
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    seed = GreedyFollowInitializer(model()).build(scenario('constant_velocity'), method='simple')
    baseline = start_jerk(seed)
    for index in (0, 1):
        q = np.array(seed.q)
        q[index, 0] += .1
        assert abs(start_jerk(replace(seed, q=tuple(map(tuple, q))))[0]-baseline[0]) > .01
    x = list(seed.start)
    x[3] += .1
    assert abs(start_jerk(replace(seed, start=tuple(x)))[0]-baseline[0]) > .01


def test_offline_monitor_encodes_px4_float32_and_invalid_position():
    import json
    from p32_shadow_monitor import encode_record
    record = json.loads(encode_record(dict(v=[np.float32(1.25)],
                                           p=[np.float32(np.nan)], valid=np.bool_(False))))
    assert record['v'] == [1.25] and not record['valid']
    assert np.isnan(record['p'][0])


def test_realtime_initializer_cap_is_explicit_and_finite():
    from uav_control.controllers.realtime_follow_solver import RealtimeFollowSolver
    for cap in (0., -.01, float('nan')):
        with pytest.raises(ValueError):
            RealtimeFollowSolver(model(), initialization_cap=cap)
    solver = RealtimeFollowSolver(model(), initialization_cap=.012)
    assert solver.initialization_cap == .012
    request = scenario('deceleration')
    result = RealtimeFollowSolver(model(), initialization_cap=1e-10,
                                  wall_clock=lambda: request.now_stamp).solve(request)
    assert not result.valid
    assert any(s['status'] == 'INITIALIZATION_DEADLINE'
               for s in result.metrics['realtime']['stages'])


def test_realtime_mode_and_invalid_clock_are_explicit():
    from uav_control.controllers.realtime_follow_solver import RealtimeFollowSolver
    request = scenario('constant_velocity')
    solver = RealtimeFollowSolver(model(), wall_clock=lambda: float('nan'))
    assert solver.mode == 'greedy_minco'
    result = solver.solve(request)
    assert not result.valid and result.metrics['realtime']['stages'] == []
