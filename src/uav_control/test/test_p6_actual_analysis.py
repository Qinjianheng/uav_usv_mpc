"""Synthetic fixtures verify an offline reader; they are never flight qualification."""

import copy
import importlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))


def api():
    assert importlib.util.find_spec('p6_actual_analysis'), 'actual P6 analyzer is missing'
    return importlib.import_module('p6_actual_analysis')


def healthy_summary():
    return dict(completed=True, healthy=True, observation_seconds=60.,
                diagnostic_coverage=1., first_minco_receipt=100., minco_fraction=1.,
                longest_minco_seconds=60., p51_audit=dict(passed=True, checks=dict(
                    reference_height=True, entry_recovery=True, execution_gates=True,
                    evidence_coverage=True, switching=True, handovers=True)))


def actual_execution():
    return dict(authority=dict(discovered_samples=61, all_discovered_single_tracker=True,
                               conflicts=[]), lifecycle=dict(accepted_unique=1, active_unique=1),
                native_command=dict(period_seconds=dict(count=1200)))


def fixture_rows(count=300, motion=True):
    rows = []
    times = np.arange(41) * .1
    covariance = np.tile((np.eye(3) * .1).ravel(), (41, 1)).tolist()
    for stamp in np.arange(99.9, 160.051, .05):
        rows.append(dict(kind='ideal', receipt=float(stamp + .001), stamp=float(stamp),
                         valid=True, p=[float(stamp), 0., 0.], v=[1., 0., 0.], a=[0.] * 3))
        rows.append(dict(kind='reference', receipt=float(stamp + .002), stamp=float(stamp),
                         v=[1., 0., 0.], a=[0.] * 3))
    for index in range(61):
        rows.append(dict(kind='graph', receipt=100. + index, publishers={
            '/planning/target_prediction': [dict(
                node='target_predictor_node', namespace='/', type='TargetPrediction', gid=[1])]}))
    for index in range(count):
        source = 100.02 + index * .2
        observation = source - .02
        rows.extend([
            dict(kind='p6_observation', receipt=source - .01, stamp=observation,
                 valid=True, reason='', p=[observation + 1., 0., 0.],
                 covariance=(np.eye(3) * .01).ravel().tolist(), source='front_rgbd',
                 frame='local_ned', image_measurement_stamp=observation,
                 image_clock_reset_count=0, px4_clock_reset_count=0,
                 received=source - .01, processed=source - .009, published=source - .008),
            dict(kind='kf', receipt=source + .001, stamp=source, source=observation,
                 valid=True, p=[source + .5, 0., 0.], v=[1., 0., 0.], a=[0.] * 3),
        ])
        position = [[source + 2. + 1.2 * t, 0., 0.] for t in times]
        velocity, acceleration = [[1.2, 0., 0.]] * 41, [[0., 0., 0.]] * 41
        prediction = dict(
            kind='p6_prediction', receipt=source + .005, mission=1, sequence=index + 1,
            source_stamp=source, observation_stamp=observation, generated_stamp=source,
            valid_until=observation + .125, model='IMM_CV_CTRV_CTRA', source='tracking',
            frame='local_ned', valid=True, reason='', compute=.005,
            times=times.tolist(), p=position, v=velocity, a=acceleration, covariance=covariance)
        rows.append(dict(prediction, topic='/planning/imm_target_prediction'))
        rows.append(dict(prediction, topic='/planning/target_prediction', model='BCTRA'))
        probability = .5 + .05 * np.sin(index * .1)
        probabilities = [float(probability), float(.8 - probability), .2]
        rows.append(dict(
            kind='p6_imm', receipt=source + .006, schema_version='p6_imm_v1',
            mission_id=1, sequence_id=index + 1, source_stamp=source,
            observation_stamp=observation, generated_stamp=source,
            valid_until=observation + .125, source='tracking', model='IMM_CV_CTRV_CTRA',
            frame_id='local_ned', valid=True, clock_identity=[0, 0],
            observation_source='front_rgbd',
            raw_input_stamp=observation, model_probabilities=probabilities,
            relative_times=times.tolist(), state=[source + 2., 0., 1.2, 0., 0., 0., 0.],
            state_covariance=(np.eye(7) * .1).tolist(), compute_time=.005,
            compute_time_scope='observation_update_plus_forecast_message_excludes_json_dds',
            model_predictions=[dict(model=model, probability=probabilities[i],
                                    positions=position, velocities=velocity,
                                    accelerations=acceleration, position_covariances=covariance)
                               for i, model in enumerate(('CV', 'CTRV', 'CTRA'))]))
        if motion:
            rows.append(dict(kind='p6_motion_truth', receipt=source + .001,
                             stamp_ns=round(source * 1e9), stage='straight', mode='linear',
                             position=[source, 0., 0.], velocity=[1., 0., 0.],
                             acceleration=[0.] * 3, turn_rate=0., angular_acceleration=0.,
                             longitudinal_acceleration=0., evaluation_only=True))
    return rows


def analyze(rows, summary=None, execution=None):
    return api().analyze_rows(
        rows, healthy_summary() if summary is None else summary,
        actual_execution() if execution is None else execution,
        dict(prediction_model='bctra', role='pilot', motion_mode='linear'))


def test_bounded_truth_interpolation_never_extrapolates_or_crosses_long_gap():
    entry = api()
    series = entry.BoundedTruth([
        dict(stamp=10., p=[1., 0., 0.], v=[2., 0., 0.], valid=True),
        dict(stamp=10.1, p=[1.2, 0., 0.], v=[2., 0., 0.], valid=True),
        dict(stamp=11., p=[3., 0., 0.], v=[2., 0., 0.], valid=True)])
    value, reason = series.at(10.05)
    assert reason == '' and value['p'][0] == pytest.approx(1.1)
    assert series.at(9.9)[1] == 'left_censored'
    assert series.at(11.1)[1] == 'right_censored'
    assert series.at(10.5)[1] == 'interpolation_gap'


def test_source_position_and_velocity_decomposition_is_exact_without_model_causality_claim():
    report, scores = analyze(fixture_rows())
    score = next(row for row in scores if row['topic'] == '/planning/imm_target_prediction'
                 and row['horizon'] == .5 and row['scored'])
    assert score['horizontal_error'] == pytest.approx(2.1)
    assert score['source_position_error'] == pytest.approx(2.)
    assert score['displacement_increment_error'] == pytest.approx(.1)
    assert score['initial_cv_effect_error'] == pytest.approx(2.1)
    assert score['beyond_initial_cv_error'] == pytest.approx(0., abs=1e-10)
    assert score['velocity_error'] == pytest.approx(.2)
    assert report['observation']['horizontal']['rmse'] == pytest.approx(1.)
    assert report['kf']['horizontal']['rmse'] == pytest.approx(.5)
    assert report['shadow_gate']['passed']
    assert report['qualification_generated'] is False


def test_unmatured_future_points_are_right_censored_and_do_not_use_post_stop_freeze():
    report, scores = analyze(fixture_rows())
    horizon = report['prediction']['/planning/imm_target_prediction']['horizons']['4.0']
    assert horizon['censor_reasons']['right_censored'] > 0
    assert horizon['scored_count'] < horizon['attempts']
    assert any(not row['scored'] and row['reason'] == 'right_censored' for row in scores)


def test_invalid_messages_and_severe_errors_remain_in_denominators():
    rows = fixture_rows()
    first = next(row for row in rows if row['kind'] == 'p6_prediction'
                 and row['topic'] == '/planning/imm_target_prediction')
    changed = copy.deepcopy(first)
    changed['sequence'] = 9001
    changed['p'][5][0] += 1000.
    rows.append(changed)
    rows.append(dict(first, sequence=9002, valid=False, reason='OBSERVATION_STALE',
                     times=[], p=[], v=[], a=[], covariance=[]))
    report, _ = analyze(rows)
    metric = report['prediction']['/planning/imm_target_prediction']['horizons']['0.5']
    assert metric['invalid_count'] == 1
    assert metric['horizontal']['max'] > 900.
    assert metric['horizontal']['rmse'] > 20.


def test_missing_motion_truth_is_unknown_in_stage_metrics():
    report, scores = analyze(fixture_rows(motion=False))
    assert {row['stage'] for row in scores} == {'unknown'}
    assert 'unknown' in report['prediction']['/planning/imm_target_prediction']['stages']


@pytest.mark.parametrize('fault,check', [
    ('lease', 'original_acquisition_lease'),
    ('identity', 'matching_full_imm_diagnostics'),
    ('psd', 'conditional_covariance_psd'),
    ('primary', 'unique_primary_publisher'),
    ('compute', 'compute_p95_under_20ms'),
    ('missing_raw', 'raw_recording_complete'),
    ('constant_probs', 'model_probabilities_vary'),
    ('conditional_mean', 'matching_full_imm_diagnostics'),
    ('invalid_truth', 'raw_recording_complete'),
    ('malformed_epoch', 'original_acquisition_lease'),
])
def test_shadow_gate_fails_on_contract_or_evidence_fault(fault, check):
    rows = fixture_rows()
    if fault == 'lease':
        for row in rows:
            if row['kind'] == 'p6_prediction' and row['model'].startswith('IMM'):
                row['valid_until'] = row['observation_stamp'] + .2
    elif fault == 'identity':
        for row in rows:
            if row['kind'] == 'p6_imm':
                row['mission_id'] = 2
    elif fault == 'psd':
        for row in rows:
            if row['kind'] == 'p6_imm':
                row['model_predictions'][0]['position_covariances'][0][0] = -1.
    elif fault == 'primary':
        next(row for row in rows if row['kind'] == 'graph')['publishers'][
            '/planning/target_prediction'].append(dict(node='imm_target_predictor_node'))
    elif fault == 'compute':
        for row in rows:
            if row['kind'] == 'p6_prediction':
                row['compute'] = .021
    elif fault == 'missing_raw':
        rows = [row for row in rows if row['kind'] != 'p6_observation']
    elif fault == 'constant_probs':
        for row in rows:
            if row['kind'] == 'p6_imm':
                row['model_probabilities'] = [.5, .3, .2]
    elif fault == 'conditional_mean':
        row = next(row for row in rows if row['kind'] == 'p6_imm')
        row['model_predictions'][0]['positions'] = [
            [value[0] + 1., value[1], value[2]]
            for value in row['model_predictions'][0]['positions']]
    elif fault == 'invalid_truth':
        for row in rows:
            if row['kind'] == 'ideal' and row['stamp'] > 110.:
                row['valid'] = False
    elif fault == 'malformed_epoch':
        next(row for row in rows if row['kind'] == 'p6_prediction')['source_stamp'] = None
    report, _ = analyze(rows)
    assert not report['shadow_gate']['passed']
    assert report['shadow_gate']['checks'][check] is False


def test_two_hundred_forecasts_or_unhealthy_short_pilot_cannot_pass():
    report, _ = analyze(fixture_rows(count=200))
    assert not report['shadow_gate']['checks']['more_than_200_full_shadow_forecasts']
    summary = dict(healthy_summary(), observation_seconds=25., healthy=False)
    report, _ = analyze(fixture_rows(), summary=summary)
    assert not report['shadow_gate']['checks']['actual_healthy_60s_pilot']


def test_truth_clock_rollback_is_reported_and_blocks_interpolation_and_gate():
    rows = fixture_rows()
    rows.append(dict(kind='ideal', receipt=159.9, stamp=100., valid=True,
                     p=[100., 0., 0.], v=[1., 0., 0.], a=[0.] * 3))
    report, _ = analyze(rows)
    assert report['truth']['clock_reversals'] > 0
    assert not report['shadow_gate']['passed']


def test_duplicate_motion_stamp_and_later_invalid_same_image_do_not_destroy_prior_evidence():
    rows = fixture_rows()
    duplicate = copy.deepcopy(next(row for row in rows if row['kind'] == 'p6_motion_truth'))
    duplicate['receipt'] += .0001
    rows.append(duplicate)
    first = next(row for row in rows if row['kind'] == 'p6_observation')
    rows.append(dict(first, valid=False, reason='DEPTH_RATIO_LOW', p=[0., 0., 0.]))
    report, _ = analyze(rows)
    assert report['shadow_gate']['passed']


@pytest.mark.parametrize('topic', [
    '/planning/target_prediction', '/planning/imm_target_prediction'])
def test_incomplete_main_or_shadow_raw_recording_blocks_gate(topic):
    rows = [row for row in fixture_rows() if not (
        row['kind'] == 'p6_prediction' and row['topic'] == topic and row['receipt'] > 145.)]
    report, _ = analyze(rows)
    assert not report['shadow_gate']['checks']['raw_recording_complete']


def test_auxiliary_join_preserves_acquisition_before_start_and_diagnostic_after_end():
    rows = fixture_rows()
    first = next(row for row in rows if row['kind'] == 'p6_observation')
    first['receipt'] = 99.999
    last = [row for row in rows if row['kind'] == 'p6_imm'][-1]
    last['receipt'] = 160.001
    report, _ = analyze(rows)
    assert report['shadow_gate']['passed']
    assert report['truth']['cutoff'] == 'receipt <= predefined 60s end; no post-stop freeze'


def test_large_epoch_wire_roundtrip_acquisition_can_cross_the_original_lookup_bucket():
    rows = fixture_rows()
    offset = 1791992166.966803 - 100.
    fields = ('receipt', 'stamp', 'source_stamp', 'observation_stamp', 'generated_stamp',
              'valid_until', 'received', 'processed', 'published', 'image_measurement_stamp',
              'raw_input_stamp')
    for row in rows:
        for name in fields:
            if name in row:
                row[name] += offset
    first = next(row for row in rows if row['kind'] == 'p6_prediction'
                 and row['topic'] == '/planning/imm_target_prediction')
    first['observation_stamp'] = float(np.nextafter(first['observation_stamp'], np.inf))
    raw = next(row for row in rows if row['kind'] == 'p6_observation')
    assert round(raw['stamp'] / 1e-6) != round(first['observation_stamp'] / 1e-6)
    report, _ = analyze(rows, summary=dict(healthy_summary(), first_minco_receipt=100. + offset))
    assert report['shadow_gate']['checks']['matching_full_imm_diagnostics']
    assert report['shadow_gate']['checks']['conditional_covariance_psd']


def raw_join_fixture():
    rows = fixture_rows(count=1)
    raw = next(row for row in rows if row['kind'] == 'p6_observation')
    prediction = next(row for row in rows if row['kind'] == 'p6_prediction')
    diagnostic = next(row for row in rows if row['kind'] == 'p6_imm')
    return raw, prediction, diagnostic


def test_raw_lookup_preserves_same_bucket_aliases_and_rejects_multiple_candidates():
    raw, prediction, diagnostic = raw_join_fixture()
    alias = dict(raw, stamp=raw['stamp'] + .25e-6,
                 image_measurement_stamp=raw['stamp'] + .25e-6)
    assert round(raw['stamp'] / 1e-6) == round(alias['stamp'] / 1e-6)
    result, reason = api()._ObservationIndex([raw, alias]).match(prediction, diagnostic)
    assert result is None and reason == 'RAW_OBSERVATION_AMBIGUOUS'


@pytest.mark.parametrize('fault', ['outside_tolerance', 'source', 'clock', 'frame'])
def test_raw_lookup_never_chooses_an_out_of_tolerance_or_other_identity_record(fault):
    raw, prediction, diagnostic = raw_join_fixture()
    if fault == 'outside_tolerance':
        raw = dict(raw, stamp=raw['stamp'] + 2e-6,
                   image_measurement_stamp=raw['stamp'] + 2e-6)
    elif fault == 'source':
        raw = dict(raw, source='another_camera')
    elif fault == 'clock':
        raw = dict(raw, px4_clock_reset_count=1)
    else:
        raw = dict(raw, frame='another_frame')
    result, reason = api()._ObservationIndex([raw]).match(prediction, diagnostic)
    assert result is None and reason == 'RAW_OBSERVATION_NOT_FOUND'


def test_raw_lookup_uses_declared_source_identity_without_nearest_candidate_selection():
    raw, prediction, diagnostic = raw_join_fixture()
    other_source = dict(raw, source='another_camera')
    result, reason = api()._ObservationIndex([raw, other_source]).match(prediction, diagnostic)
    assert result is raw and reason == 'MATCHED'


def test_raw_lookup_rejects_two_nearby_epochs_even_if_one_is_exact():
    raw, prediction, diagnostic = raw_join_fixture()
    nearby = dict(raw, stamp=raw['stamp'] + .75e-6,
                  image_measurement_stamp=raw['stamp'] + .75e-6)
    result, reason = api()._ObservationIndex([raw, nearby]).match(prediction, diagnostic)
    assert result is None and reason == 'RAW_OBSERVATION_AMBIGUOUS'


def test_analyzer_writes_only_new_external_report_and_preserves_source(tmp_path):
    entry = api()
    case, output = tmp_path / 'case', tmp_path / 'analysis'
    flight = case / 'flight'
    flight.mkdir(parents=True)
    (case / 'case.json').write_text(json.dumps(dict(
        prediction_model='bctra', role='pilot', motion_mode='linear')))
    evidence = flight / 'offline_evidence.jsonl'
    evidence.write_text(''.join(json.dumps(row) + '\n' for row in fixture_rows()))
    (flight / 'p5_run_summary.json').write_text(json.dumps(healthy_summary()))
    (flight / 'execution_analysis.json').write_text(json.dumps(actual_execution()))
    before = evidence.read_bytes()
    report = entry.analyze_case(case, output)
    assert evidence.read_bytes() == before
    assert (output / 'analysis.json').exists() and (output / 'forecast_scores.jsonl').exists()
    assert report['qualification_generated'] is False
    assert not (output / 'qualification.json').exists()
    with pytest.raises(FileExistsError):
        entry.analyze_case(case, output)
    with pytest.raises(ValueError, match='P6_NEW_EXTERNAL_ANALYSIS_DIRECTORY_REQUIRED'):
        entry.analyze_case(case, case / 'new_analysis')
