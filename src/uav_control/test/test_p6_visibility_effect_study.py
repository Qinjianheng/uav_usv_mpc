"""Offline attribution must preserve failures, uncertainty and causal transitions."""
from pathlib import Path
from dataclasses import replace
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
import p6_visibility_effect_study as study  # noqa: E402
from uav_control.guidance.adaptive_viewpoint import profile_state  # noqa: E402


def test_summary_counts_rejected_and_fallback_windows_in_the_denominator():
    rows = [dict(group='D', pressure='normal', valid=True,
                 risk=dict(robust_ok=False), truth_follow_rmse=.4),
            dict(group='D', pressure='normal', valid=False, status='DEADLINE')]
    result = study.summarize_frozen(rows)['D/normal']
    assert result['attempts'] == 2
    assert result['nominal_valid'] == 1
    assert result['robust_pass'] == 0
    assert result['conditional_metric_denominator'] == 1
    assert result['statuses']['DEADLINE'] == 1


def test_score_decomposition_exposes_cost_needed_to_leave_fixed_rear_point():
    parts = study.score_components(((5., 0.), (4.5, 0.), (6., 0.)),
                                   (0., 0., 0.), (0., 0., 0.), (0., 0., 0.))
    assert parts[0]['total'] == 0.
    assert parts[1]['preferred'] == pytest.approx(.05)
    assert parts[1]['extra_error'] == pytest.approx(.05)
    assert parts[1]['smooth'] == pytest.approx(.05)
    assert parts[1]['switch'] == pytest.approx(.05)
    assert parts[1]['total'] == pytest.approx(.2)
    assert parts[2]['total'] == pytest.approx(.65)
    assert parts[1]['risk_gain_needed_with_hysteresis'] == pytest.approx(.3)


def test_transition_does_not_teleport_to_the_scored_candidate():
    profile = study.transition_profile((4.5, 0.))
    at_lease_end = profile_state(profile, .45)
    assert profile['duration'] == pytest.approx(6.333333333333333)
    assert 4.998 < at_lease_end.d < 5.
    assert at_lease_end.d != 4.5
    np.testing.assert_allclose(profile_state(profile, 0.).as_tuple(),
                               (5., 0., 0., 0., 0., 0.), atol=1e-12)


def test_reference_probe_uses_acceleration_instead_of_reference_yaw_as_attitude_input():
    from test_follow_research_algorithms import request
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    raw = replace(request(2.), model='BCTRA_BOUNDED_Z', target_covariances=tuple(
        np.tile(np.eye(3).reshape(9)*.01, (41, 1)).tolist()))
    model = FollowMpcSeed()
    model.p6_policy = study.P6Policy(
        visibility_mode='robust', calibration_id='test', calibration_sha256='test',
        calibration_family='bctra')
    result = study.evaluate_reference(raw, model, (4.5, 0.), True, np.linspace(0., 1.6, 33))
    assert result['risk']['valid_input']
    assert result['risk']['checked_seconds'][-1] == pytest.approx(.45)
    assert len(result['risk']['branches'][0]['samples']) == 10


def test_reconstructing_prepare_epoch_retains_all_acquired_times_and_measurement_boundary():
    from test_follow_research_algorithms import request
    from uav_control.guidance.follow_problem import future_request
    saved = future_request(request(2.), .15)
    raw = study.preparation_request(saved)
    assert raw.context.execution_start_stamp == 100.
    assert raw.state == saved.measurement_state
    assert raw.context.navigation_stamp == saved.context.navigation_stamp
    assert raw.context.prediction_source_stamp == saved.context.prediction_source_stamp
    assert raw.context.observation_stamp == saved.context.observation_stamp
    assert raw.now_stamp == saved.now_stamp
    assert raw.target_covariances == saved.target_covariances


def test_conditional_failure_is_not_hidden_by_fused_pass_or_low_probability():
    from uav_control.guidance.robust_visibility import (
        ModelVisibilityReport, RobustVisibilityReport, RobustVisibilitySample,
    )
    safe = RobustVisibilitySample(True, True, .1, .1, .2, .2, ((0., 0.), (0., 0.)))
    bad = RobustVisibilitySample(True, False, .1, .5, .2, -.2,
                                 ((0., 0.), (0., 0.)), ('ROBUST_VERTICAL_FOV',))
    report = RobustVisibilityReport(
        valid_input=True, nominal_ok=True, samples=(safe, safe), model_reports=(
            ModelVisibilityReport(0, (.001, .001), False, True, False, (safe, bad)),))
    result = study.risk_summary(report, {'checked_indices': (0, 1)}, (0., .45))
    assert not result['robust_ok']
    assert result['first_failure_seconds'] == .45
    assert result['branches'][1]['model_probability'] == (.001, .001)
    assert result['branches'][1]['minimum_vertical_margin'] == -.2


def test_horizon_specific_coverage_uses_fitted_scale_without_refitting_on_heldout():
    calibration = dict(empirical_scale=2., covariance_scale=9., per_horizon=[
        dict(horizon=.5, covariance_scale=1.)])
    rows = [dict(backend='BCTRA', comparison='complete_chain', valid=True,
                 horizon=.5, error=(1., 0., 0.), covariance=np.eye(3).tolist()),
            dict(backend='BCTRA', comparison='complete_chain', valid=True,
                 horizon=.5, error=(3., 0., 0.), covariance=np.eye(3).tolist())]
    result = study.coverage_by_horizon(rows, calibration, 'bctra')[0]
    assert result['horizon_scale'] == 1.
    assert result['heldout_coverage_horizon_scale'] == .5
    assert result['heldout_coverage_global_scale'] == 1.
