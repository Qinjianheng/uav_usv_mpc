"""Offline comparisons must retain failures, common samples and causal episode boundaries."""
from dataclasses import replace
from pathlib import Path
import sys

import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.guidance.follow_problem import future_request

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def comparison_row(line, epoch, error, age=None):
    """Build a hand-computed common-window fixture, independent of solver behavior."""
    return dict(line=line, source_sha256=str(line), epoch=epoch,
                continuous_age=epoch if age is None else age,
                sampled_state=[0.]*11 if error is not None else None,
                error=None if error is None else dict(
                    fixed=error, selected=error, distance=error,
                    relative_velocity=error, height=2.))


def test_common_window_excludes_uncovered_samples_and_requires_episode_age():
    from p46_follow_study import matched
    groups = {'A': [comparison_row(i, t, e) for i, t, e in
                    [(1, 0., 2.), (2, 10., 4.), (3, 20., 6.), (4, 30., 8.)]],
              'B': [comparison_row(i, t, e) for i, t, e in
                    [(1, 0., 1.), (2, 10., 1.), (3, 20., None), (4, 30., 1.)]]}
    report = matched(groups)
    assert report['count'] == 3 and report['steady_count'] == 1
    assert report['coverage_fraction'] == .75
    assert report['groups']['A']['after20']['fixed'] == 8.
    assert report['groups']['A']['after20']['height'] == 2.
    # A later restart must not inherit 30 seconds of convergence from the old episode.
    groups['B'][-1]['continuous_age'] = 5.
    report = matched(groups)
    assert report['steady_count'] == 0
    assert report['groups']['A']['after20']['fixed'] is None


def test_fallback_success_is_not_independent_success():
    from p46_follow_study import summarize
    row = comparison_row(1, 100., 0., 0.)
    row.update(seconds=.03, result=dict(
        valid=True, solver_status='VALID', timing={},
        metrics={'follow_guided': dict(independent_valid=False, fallback_used=True,
                                       fallback_valid=True)}), stop_reason=None)
    report = summarize([row], 'B', 100., 100.)
    assert report['feasible'] == 1
    assert report['independent_valid'] == 0
    assert report['fallback_used'] == 1 and report['fallback_valid'] == 1
    assert report['timing']['optimization'] is None
    assert report['continuous_survival_seconds'] is None


def shifted(raw, stamp):
    """Refresh only a synthetic fixture's epochs; production replay never does this."""
    context = replace(raw.context, navigation_stamp=stamp, attitude_stamp=stamp,
                      execution_start_stamp=stamp, prediction_source_stamp=stamp,
                      observation_stamp=stamp-.01, prediction_valid_until=stamp+.115)
    return future_request(replace(raw, now_stamp=stamp, context=context), .15)


def test_expired_rollout_stops_without_returning_to_logged_uav_state(tmp_path):
    from p46_follow_study import run_case
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    raw, original = request(2.), model()
    requests = [shifted(raw, t) for t in (100., 100.2, 101.6, 102.)]
    first = P44FollowSolver(
        original, duration=1.2, rolling=False, refinement='none',
        wall_clock=lambda: requests[0].now_stamp).solve(requests[0])
    assert first.valid

    class FailAfterFirst:
        def solve(self, req):
            return first if req.now_stamp == 100. else MpcSeedResult(
                req.context, solver_status='CONTROLLED_REJECTION')

    cases = [('fixture', i, str(i), r, original) for i, r in enumerate(requests, 1)]
    rows = run_case(cases, tmp_path/'rolling.jsonl', 'A', 1.2, rolling=True,
                    factory=lambda *args: FailAfterFirst())
    assert rows[1]['sampled_state'] is not None
    assert rows[1]['request']['boundary_policy'] == 'hypothetical_previous_curve'
    assert rows[2]['stop_reason'] == 'CURVE_EXPIRED'
    assert rows[2]['result'] is None and rows[3]['result'] is None
    assert rows[3]['sampled_state'] is None
    assert rows[3]['request'] is None
    assert rows[3]['raw_request']['state'] == list(requests[3].state)


def test_height_is_relative_to_flight_altitude_not_target_heave():
    from p46_follow_study import sample_errors
    req = shifted(request(), 100.)
    actual = (*req.state[:2], -4., *req.state[3:], 0.)
    errors = sample_errors(req, model(), actual, 0., 1.2)
    assert errors['height'] == 1.


def test_matched_rejects_same_source_with_different_execution_epoch():
    from p46_follow_study import matched
    with pytest.raises(ValueError, match='EPOCH_MISMATCH'):
        matched({'A': [comparison_row(1, 0., 1.)],
                 'B': [comparison_row(1, .01, 1.)]})


def test_final_curve_evaluation_is_available_for_baseline_and_uses_same_start():
    from p46_follow_study import evaluation
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    req = shifted(request(2.), 100.)
    result = P44FollowSolver(model(), duration=1.2, rolling=False, refinement='none',
                             wall_clock=lambda: req.now_stamp).solve(req)
    assert result.valid
    report = evaluation(req, model(), result, 1.2)
    assert report['final_curve_to_follow_fit']['position_rmse'] < 1e-7
    assert report['reference_assumptions']['command_source'] == 'execution_velocity_initialization'
    assert report['analysis_outside_solver_timer']


def test_valid_flag_controls_success_even_if_detail_accidentally_claims_success():
    from p46_follow_study import summarize
    row = comparison_row(1, 100., None, 0.)
    row.update(seconds=.03, stop_reason=None, result=dict(
        valid=False, solver_status='DEADLINE_EXCEEDED', timing={},
        metrics={'follow_guided': dict(independent_valid=True, fallback_used=True,
                                       fallback_valid=True)}))
    report = summarize([row], 'B', 100., 100.)
    assert report['independent_valid'] == 0 and report['fallback_valid'] == 0
    assert report['fallback_used'] == 1


def test_rejected_initialization_still_has_reproducible_forecast_and_seed():
    from p46_follow_study import evaluation
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    req = shifted(request(2.), 100.)
    rejected = MpcSeedResult(req.context, solver_status='DYNAMIC_INFEASIBLE')
    report = evaluation(req, model(), rejected, 1.2)
    assert report['final_curve_to_follow_fit'] is None
    assert report['initial_seed']['start'] == req.state
    assert report['reference']['positions'][0] == req.state[:3]
    assert report['reference']['velocities'][0] == req.state[3:6]
    assert report['reference']['accelerations'][0] == req.state[6:9]
    assert report['reference']['times'][-1] == pytest.approx(1.2)


def test_yaw_evaluation_uses_spline_orientation_and_polynomial_order():
    from p46_follow_study import evaluation
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    raw = request(2.)
    req = shifted(replace(raw, state=(*raw.state[:9], .2)), 100.)
    result = P44FollowSolver(model(), duration=1.2, rolling=False, refinement='none',
                             wall_clock=lambda: req.now_stamp).solve(req)
    assert result.valid
    report = evaluation(req, model(), result, 1.2)
    assert report['final_curve_yaw']['initial'] == pytest.approx(.2)
    assert report['final_curve_yaw']['initial_rate'] == pytest.approx(0., abs=1e-8)


def test_physical_thrust_uses_recorded_gravity_configuration():
    from p46_follow_study import physical
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    original = model()
    original.attitude_config = replace(original.attitude_config, gravity=8.)
    result = MpcSeedResult(request().context, valid=True, velocities=((0., 0., 0.),),
                           accelerations=((0., 0., 0.),), jerks=((0., 0., 0.),),
                           yaw_rates=(0.,))
    assert physical(result, original)['sampled_maxima']['specific_thrust'] == 8.
