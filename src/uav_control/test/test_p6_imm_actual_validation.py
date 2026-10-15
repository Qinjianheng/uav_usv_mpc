"""Independent residual arithmetic and evidence boundaries for offline IMM diagnostics."""

import copy
import importlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.stats import chi2

from test_p6_actual_analysis import actual_execution, fixture_rows, healthy_summary

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))


def api():
    assert importlib.util.find_spec('p6_imm_actual_validation'), 'IMM validation tool is missing'
    return importlib.import_module('p6_imm_actual_validation')


def analyze(rows=None):
    return api().analyze_rows(
        fixture_rows(count=3) if rows is None else rows, healthy_summary(), actual_execution(),
        dict(role='pilot', motion_mode='linear', prediction_model='bctra'))


def test_whitening_uses_full_covariance_and_dimension_specific_95_percent_regions():
    tool = api()
    for dimension in (2, 3):
        result = tool.covariance_score([2., 3., 4.], np.diag([4., 9., 16.]), dimension)
        assert result['scored'] and result['mahalanobis_squared'] == pytest.approx(dimension)
        assert result['whitened_error'] == pytest.approx([1.] * dimension)
        assert result['marginal_standardized_error'] == pytest.approx([1.] * dimension)
        assert result['threshold_95'] == pytest.approx(chi2.ppf(.95, dimension), rel=1e-12)
    correlated = tool.covariance_score([2., 3., 0.],
                                       [[4., 3., 0.], [3., 9., 0.], [0., 0., 1.]], 2)
    assert correlated['mahalanobis_squared'] == pytest.approx(4. / 3.)
    assert correlated['marginal_standardized_error'] == pytest.approx([1., 1.])


def test_horizontal_ellipse_does_not_hide_vertical_ellipsoid_failure():
    tool = api()
    assert tool.covariance_score([0., 0., 4.], np.eye(3), 2)['covered_95']
    assert not tool.covariance_score([0., 0., 4.], np.eye(3), 3)['covered_95']


@pytest.mark.parametrize('covariance,reason', [
    (np.diag([1., 0., 1.]), 'covariance_not_positive_definite'),
    (np.diag([1., -1., 1.]), 'covariance_not_positive_definite'),
    ([[1., .1, 0.], [0., 1., 0.], [0., 0., 1.]], 'covariance_not_symmetric'),
    (np.diag([1., np.nan, 1.]), 'covariance_nonfinite_or_shape'),
])
def test_covariance_is_never_regularized_or_pseudoinverted(covariance, reason):
    result = api().covariance_score([1., 2., 3.], covariance, 3)
    assert not result['scored'] and result['reason'] == reason
    assert result['covered_95'] is None


def test_exact_source_increment_and_initial_cv_vector_decomposition_is_saved():
    report, forecasts, sources = analyze()
    score = next(row for row in forecasts if row['topic'] == api().actual.SHADOW
                 and row['horizon'] == .5 and row['scored'])
    np.testing.assert_allclose(score['error_xyz'], [2.1, 0., 0.])
    np.testing.assert_allclose(score['source_error_xyz'], [2., 0., 0.])
    np.testing.assert_allclose(score['increment_error_xyz'], [.1, 0., 0.])
    np.testing.assert_allclose(np.array(score['source_error_xyz']) + score['increment_error_xyz'],
                               score['error_xyz'])
    np.testing.assert_allclose(score['initial_cv_effect_xyz'], [2.1, 0., 0.])
    assert report['qualification_generated'] is False
    assert len([row for row in sources if row['kind'] == 'imm_source']) == 3
    assert report['current_state']['imm_source']['position']['mean_xyz'] == [2., 0., 0.]


def test_kf_comparison_is_at_source_epoch_and_labels_retrospective_interpolation():
    rows = fixture_rows(count=3)
    kf = [row for row in rows if row['kind'] == 'kf']
    # Change all logged KF epochs; compare interpolated states, never closest receipt.
    for row in kf:
        row['stamp'] -= .01
        row['p'][0] -= .01
    for left, right in zip(kf[:-1], kf[1:]):
        midpoint = (left['stamp'] + right['stamp']) / 2.
        rows.append(dict(left, stamp=midpoint, receipt=midpoint + .001,
                         p=[midpoint + .5, 0., 0.]))
    report, _, sources = analyze(rows)
    paired = [row for row in sources if row['kind'] == 'same_epoch_imm_kf' and row['scored']]
    assert paired and paired[0]['position_delta_xyz'] == pytest.approx([1.5, 0., 0.])
    assert paired[0]['epoch'] == pytest.approx(kf[0]['stamp'] + .01)
    assert report['same_epoch_imm_kf']['scope'].startswith('Retrospective')


def test_invalid_and_censored_forecasts_remain_in_covariance_denominators():
    rows = fixture_rows(count=3)
    first = next(row for row in rows if row['kind'] == 'p6_prediction'
                 and row['topic'] == api().actual.SHADOW)
    rows.append(dict(first, sequence=99, valid=False, reason='OBSERVATION_STALE',
                     times=[], p=[], v=[], a=[], covariance=[]))
    late = copy.deepcopy(first)
    late.update(sequence=100, source_stamp=159.9, observation_stamp=159.88, receipt=159.91)
    rows.append(late)
    report, forecasts, _ = analyze(rows)
    horizon = report['prediction'][api().actual.SHADOW]['4.0']
    assert horizon['attempts'] == 5
    assert horizon['invalid_forecasts'] == 1
    assert horizon['unscored_reasons']['right_censored'] == 1
    assert horizon['covariance_2d']['attempts'] == 5
    assert horizon['covariance_2d']['scored_count'] == 3
    assert any(row.get('invalid_reason') == 'OBSERVATION_STALE' for row in forecasts)


def test_malformed_identity_blocks_covariance_scoring_without_deleting_physical_errors():
    rows = fixture_rows(count=3)
    diagnostic = next(row for row in rows if row['kind'] == 'p6_imm')
    diagnostic['clock_identity'] = [2, 0]
    report, forecasts, _ = analyze(rows)
    scores = [row for row in forecasts if row['topic'] == api().actual.SHADOW
              and row['sequence'] == diagnostic['sequence_id']]
    assert scores[0]['scored']
    assert scores[0]['covariance_2d']['reason'] == 'prediction_protocol_invalid'
    assert report['protocol_reasons']['prediction_protocol_invalid'] == 1


def test_current_error_keeps_signed_bias_and_model_probabilities_by_recorded_stage():
    report, _, _ = analyze()
    assert report['current_state']['observation']['position']['mean_xyz'] == pytest.approx(
        [1., 0., 0.])
    assert report['current_state']['kf']['position']['mean_xyz'] == pytest.approx([.5, 0., 0.])
    # Conversion of decimal motion epochs can place same-epoch tags slightly in the future.
    assert report['model_probabilities']['by_stage']['straight']['count'] == 1
    assert report['model_probabilities']['by_stage']['unknown']['count'] == 2
    assert report['model_probabilities']['overall']['count'] == 3
    assert report['independence']['fitted_calibration'] is False


def test_case_writer_preserves_inputs_and_rejects_overwrite_and_internal_output(tmp_path):
    tool = api()
    case, output = tmp_path / 'case', tmp_path / 'report'
    flight = case / 'flight'
    flight.mkdir(parents=True)
    evidence = flight / 'offline_evidence.jsonl'
    evidence.write_text(''.join(json.dumps(row) + '\n' for row in fixture_rows(count=3))
                        + 'malformed retained source line\n')
    (case / 'case.json').write_text(json.dumps(dict(
        role='pilot', motion_mode='linear', prediction_model='bctra')))
    (flight / 'p5_run_summary.json').write_text(json.dumps(healthy_summary()))
    (flight / 'execution_analysis.json').write_text(json.dumps(actual_execution()))
    before = tool.actual.sha(evidence)
    report = tool.analyze_case(case, output, data_role='diagnostic')
    assert report['malformed_lines'] == [len(fixture_rows(count=3)) + 1]
    assert before == tool.actual.sha(evidence)
    assert (output / 'forecast_validation.jsonl').exists()
    assert (output / 'source_validation.jsonl').exists()
    assert (output / 'SHA256SUMS').exists()
    assert (output / 'source_snapshot/p6_imm_actual_validation.py').exists()
    assert report['independence']['data_role'] == 'diagnostic'
    with pytest.raises(FileExistsError):
        tool.analyze_case(case, output)
    with pytest.raises(ValueError, match='EXTERNAL'):
        tool.analyze_case(case, case / 'analysis')
    with pytest.raises(ValueError, match='DATA_ROLE'):
        tool.analyze_case(case, tmp_path / 'other', data_role='invalid')


def test_missing_fixed_window_anchor_cannot_be_replaced_by_zero():
    with pytest.raises(ValueError, match='FIRST_MINCO'):
        api().analyze_rows([], {}, {}, {})


def test_nonfinite_invalid_input_epoch_is_preserved_as_rejection_and_serializes(tmp_path):
    tool = api()
    rows = fixture_rows(count=1)
    rows.append(dict(kind='p6_observation', receipt=101., stamp=float('nan'), valid=False,
                     reason='BAD_STAMP', p=[float('nan')] * 3))
    _, _, sources = analyze(rows)
    value = next(row for row in sources if row.get('invalid_reason') == 'BAD_STAMP')
    assert not value['scored']
    parsed = json.loads(tool.serialize(value))
    assert parsed['epoch'] is None and parsed['invalid_reason'] == 'BAD_STAMP'


def test_exact_wire_identity_rejects_submicrosecond_drift_accepted_by_offline_join():
    rows = fixture_rows(count=3)
    diagnostic = next(row for row in rows if row['kind'] == 'p6_imm')
    diagnostic['observation_stamp'] += 239e-9
    report, forecasts, _ = analyze(rows)
    assert report['existing_shadow_diagnostic']['checks']['matching_full_imm_diagnostics']
    assert report['wire_identity']['exact_matches'] == 2
    assert report['wire_identity']['mismatched_fields']['observation_stamp'] == 1
    first = next(row for row in forecasts if row['topic'] == api().actual.SHADOW)
    assert not first['protocol_valid'] and first['scored']
    assert first['covariance_2d']['reason'] == 'prediction_protocol_invalid'


@pytest.mark.parametrize('times', (None, [], [float('nan')]))
def test_malformed_diagnostic_time_array_is_rejected_without_losing_forecast(times):
    rows = fixture_rows(count=1)
    next(row for row in rows if row['kind'] == 'p6_imm')['relative_times'] = times
    report, forecasts, _ = analyze(rows)
    assert report['wire_identity']['mismatched_fields']['relative_times'] == 1
    assert next(row for row in forecasts if row['topic'] == api().actual.SHADOW)['scored']
