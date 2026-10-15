"""Matched offline study provenance, component removal and failure accounting."""

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
import p6_follow_ablation as study  # noqa: E402


def test_predeclared_groups_and_pressure_do_not_depend_on_results():
    assert study.GROUPS['A'] == ('bctra', 'nominal', False, 'full')
    assert study.GROUPS['B'] == ('imm', 'nominal', False, 'full')
    assert study.GROUPS['C'] == ('bctra', 'robust', True, 'full')
    assert study.GROUPS['D'] == ('imm', 'robust', True, 'full')
    assert set(study.PRESSURES) == {'normal', 'lateral_2m', 'yaw_0p25'}
    assert study.DEFAULT_SEED not in (127, 731)
    assert study.DEFAULT_EPOCHS == (8., 16., 24., 40., 56.)
    assert study.GROUPS['C_lease_prefix'] == ('bctra', 'robust', True, 'lease_prefix')
    assert study.GROUPS['D_lease_prefix'] == ('imm', 'robust', True, 'lease_prefix')
    assert study.GROUPS['D_fixed_profile'] == ('imm', 'robust', False, 'fixed_profile')


def test_fixed_profile_intervention_retains_the_forecast_and_analytic_reference():
    from test_follow_research_algorithms import request
    from test_progress_follow import model
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.follow_reference import forecast_viewpoint

    raw = request(2.)
    fixed = study.remove_component(raw, 'fixed_profile')
    assert fixed.viewpoint_profile == (0., 0., (5., 0., 0., 0., 0., 0.), (0.,)*6)
    assert fixed.target_positions == raw.target_positions
    assert fixed.target_velocities == raw.target_velocities
    assert fixed.target_covariances == raw.target_covariances
    assert study.robust_claim_eligible('fixed_profile')
    explicit = FollowProblem(fixed, model(), study.HORIZON)
    position, velocity, acceleration, _, _ = forecast_viewpoint(explicit, .4)
    np.testing.assert_allclose(position, (-4.2, 0., -5.))
    np.testing.assert_allclose(velocity, (2., 0., 0.))
    np.testing.assert_allclose(acceleration, (0., 0., 0.), atol=1e-12)
    turning = replace(raw, target_positions=tuple((10*np.sin(.2*t), 10*(1-np.cos(.2*t)), 0.)
                                                  for t in raw.prediction_times),
                      target_velocities=tuple((2*np.cos(.2*t), 2*np.sin(.2*t), 0.)
                                              for t in raw.prediction_times))
    explicit_turn = FollowProblem(study.remove_component(turning, 'fixed_profile'),
                                  model(), study.HORIZON)
    _, velocity, _, _, _ = forecast_viewpoint(explicit_turn, .4)
    normal = np.array((-np.sin(.08), np.cos(.08), 0.))
    target_velocity = np.array((2*np.cos(.08), 2*np.sin(.08), 0.))
    np.testing.assert_allclose(velocity, target_velocity-5.*.2*normal,
                               atol=1e-11)


def test_component_removal_cannot_carry_full_robust_qualification():
    from test_follow_research_algorithms import request
    raw = replace(request(2.), model='IMM_CV_CTRV_CTRA',
                  target_covariances=tuple(map(tuple, np.tile(np.eye(3).reshape(9), (41, 1)))),
                  model_positions=(((0., 0., 0.),)*41,)*3,
                  model_covariances=((tuple(map(tuple, np.eye(3))),)*41,)*3,
                  model_probabilities=(.3, .4, .3))
    removed = study.remove_component(raw, 'without_covariance')
    assert np.count_nonzero(removed.target_covariances) == 0
    assert np.count_nonzero(removed.model_covariances) == 0
    assert removed.model_positions == raw.model_positions
    missing = study.remove_component(raw, 'without_models')
    assert missing.model_positions == () and missing.model_covariances == ()
    assert missing.model == 'IMM_CV_CTRV_CTRA'
    assert not study.robust_claim_eligible('without_covariance')
    assert not study.robust_claim_eligible('without_models')
    assert not study.robust_claim_eligible('without_yaw')


def test_summary_preserves_failures_and_reports_conditional_metric_denominator():
    rows = [dict(group='A', mode='linear', pressure='normal', valid=True,
                 status='SAMPLED_FEASIBLE', total_ms=10.,
                 truth_follow_rmse=1., actual_target_visible_fraction=1.),
            dict(group='A', mode='linear', pressure='normal', valid=False,
                 status='DEADLINE_EXCEEDED', total_ms=130.)]
    result = study.summarize(rows)[0]
    assert result['attempts'] == 2 and result['valid_count'] == 1
    assert result['rejection_fraction'] == .5
    assert result['conditional_metrics_denominator'] == 1
    assert result['status_counts']['DEADLINE_EXCEEDED'] == 1
    assert result['truth_follow_rmse_mean'] == 1.


def test_synthetic_study_seed_must_be_independent_of_both_calibration_splits():
    evidence = dict(source_provenance=dict(seed=127), heldout_provenance=dict(seed=731))
    study.validate_evaluation_seed(1913, (evidence,))
    with pytest.raises(ValueError, match='EVALUATION_SEED_OVERLAPS_CALIBRATION'):
        study.validate_evaluation_seed(731, (evidence,))


def test_json_sanitizing_retains_nonfinite_evidence_as_explicit_text():
    value = study.json_safe(dict(margin=-np.inf, risk=np.float64(.2), reason='NO_CANDIDATE'))
    assert value['margin'] == '-Infinity'
    assert value['risk'] == .2 and value['reason'] == 'NO_CANDIDATE'
