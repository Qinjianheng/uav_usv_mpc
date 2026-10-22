"""Admission recomputes raw evidence and never reuses the historical flight budget."""
import json
import gzip

import pytest

from test_p761_calibration import ROOT, script
from uav_control.common.p76_readiness import sha256, source_fingerprint


def test_saved_e1_pass_flags_cannot_replace_recomputable_raw_inputs(tmp_path):
    raw = tmp_path/'assertions.json'
    raw.write_text('{}')
    e1 = dict(evidence_kind='P764_REAL_SYNC_DKF_EXECUTOR',
              sources=source_fingerprint(ROOT), raw_evidence={str(raw): sha256(raw)},
              coverage=dict(evidence_class='REAL_SYNCHRONIZED_UNARMED_SESSION',
                            qualified=True, causal_native_pose_navigation_brackets=50,
                            posterior_and_covariance_preserved=True, unarmed=True),
              tracker_shadow=dict(evidence_class='ACTUAL_ROS_EXECUTOR_DISCONNECTED_SHADOW',
                                  qualified=True, missed_cycles=0, compute_deadline_misses=0),
              replay=dict(evidence_class='REAL_MATCHED_CALLBACK_REPLAY',
                          original_acquisition_epochs_preserved=True,
                          original_receipt_epochs_preserved=True,
                          parallel=dict(spatial_distinct_updates_after_y=1,
                                        illegal_authorizations=0)))
    with pytest.raises(ValueError, match='P765_E1_RAW'):
        script('p761_session').validate_e1_evidence(e1)


@pytest.mark.parametrize('fault,reason', [
    ('overflow', 'RECORDER_INCOMPLETE'), ('graph', 'RAW_GRAPH_REQUIRED')])
def test_e1_incomplete_recorder_or_graph_cannot_be_relabelled_as_a_pass(tmp_path, fault, reason):
    helper = script('p765_qualification')
    for name in ('synchronized_packets.jsonl.gz', 'actual_callback_inputs.jsonl.gz'):
        with gzip.open(tmp_path/name, 'wt') as stream:
            stream.write('')
    (tmp_path/'phase_labels.json').write_text('[]')
    raw = dict(schema_version=1, evidence_class='ACTUAL_ROS_EXECUTOR_DISCONNECTED_SHADOW',
               sources=source_fingerprint(ROOT),
               contract_sha256=sha256(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'),
               graphs=[{}], recorder_overflows=1 if fault == 'overflow' else 0)
    (tmp_path/'e1_executor_raw.json').write_text(json.dumps(raw))
    e1 = dict(raw_evidence={str(p): sha256(p) for p in tmp_path.iterdir()})
    with pytest.raises(ValueError, match=reason):
        helper.validate_e1_raw(e1)


def e1_stream_fixture(directory):
    directory.mkdir(parents=True)
    run_id = 'a'*32
    names = ('synchronized_packets.jsonl.gz', 'actual_callback_inputs.jsonl.gz',
             'phase_labels.json', 'e1_executor_raw.json')
    paths = {name: directory/name for name in names}
    for name in names[:2]:
        with gzip.open(paths[name], 'wt') as stream:
            stream.write(json.dumps(dict(run_id=run_id))+'\n')
    paths['phase_labels.json'].write_text(json.dumps([dict(run_id=run_id)]))
    raw = dict(run_id=run_id,
               raw_inputs={name: sha256(paths[name]) for name in names[:3]})
    paths['e1_executor_raw.json'].write_text(json.dumps(raw))
    return paths, raw


def test_e1_stream_binding_preserves_one_run_and_its_pinned_original_bytes(tmp_path):
    paths, raw = e1_stream_fixture(tmp_path/'same_run')
    records, events, phases = script('p765_qualification').validate_e1_streams(paths, raw)
    assert records == events == phases == [{'run_id': 'a'*32}]


@pytest.mark.parametrize('fault', ['mixed_parent', 'mixed_run', 'raw_hash', 'missing_run_id'])
def test_e1_rejects_mixed_run_streams_or_pins_for_different_raw_files(tmp_path, fault):
    paths, raw = e1_stream_fixture(tmp_path/'run_a')
    if fault == 'mixed_parent':
        other, _ = e1_stream_fixture(tmp_path/'run_b')
        paths['phase_labels.json'] = other['phase_labels.json']
    elif fault == 'mixed_run':
        path = paths['actual_callback_inputs.jsonl.gz']
        with gzip.open(path, 'wt') as stream:
            stream.write(json.dumps(dict(run_id='b'*32))+'\n')
        raw['raw_inputs'][path.name] = sha256(path)
    elif fault == 'raw_hash':
        raw['raw_inputs']['phase_labels.json'] = 'f'*64
    else:
        raw.pop('run_id')
    with pytest.raises(ValueError, match='P765_E1_STREAM_BINDING'):
        script('p765_qualification').validate_e1_streams(paths, raw)


def test_e1_admission_checks_stream_binding_before_recomputing_coverage(tmp_path):
    paths, raw = e1_stream_fixture(tmp_path/'run_a')
    helper = script('p765_qualification')
    path = paths['actual_callback_inputs.jsonl.gz']
    with gzip.open(path, 'wt') as stream:
        stream.write(json.dumps(dict(run_id='b'*32))+'\n')
    raw['raw_inputs'][path.name] = sha256(path)
    graph = {name: [] for name in helper.COMMAND_TOPICS}
    graph['/perception/target_detection'] = ['yolo_target_detector']
    raw.update(schema_version=1, evidence_class='ACTUAL_ROS_EXECUTOR_DISCONNECTED_SHADOW',
               sources=source_fingerprint(ROOT), recorder_overflows=0, graphs=[graph],
               contract_sha256=sha256(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'))
    paths['e1_executor_raw.json'].write_text(json.dumps(raw))
    e1 = dict(raw_evidence={str(path): sha256(path) for path in paths.values()})
    with pytest.raises(ValueError, match='P765_E1_STREAM_BINDING'):
        helper.validate_e1_raw(e1)


@pytest.mark.parametrize('stage', ['A1', 'A2', 'B', 'C', 'D'])
def test_every_future_stage_requires_new_independently_approved_budget(tmp_path, stage):
    with pytest.raises(ValueError, match='P765_NEW_APPROVED_EXPERIMENT_BUDGET_REQUIRED'):
        script('p761_session').run(tmp_path/'session_future', stage, authorized_stage=stage)


def test_new_budget_is_scoped_to_its_exact_independent_approval(tmp_path):
    helper = script('p765_qualification')
    task = ROOT/'data/experiments/p765_uncreated_budget_test'
    path = tmp_path/'approval.json'
    data = dict(schema_version=1, kind='P765_SEPARATELY_APPROVED_EXPERIMENT_BUDGET',
                budget_id='test-only', approval_source='EXPLICIT_USER_APPROVAL',
                approval_reference='unit-test fixture; no actual experiment authorization',
                task_root=str(task), session_limit=1, stages=['D'])
    path.write_text(json.dumps(data))
    accepted = helper.validate_experiment_budget(path, 'test-only', task/'session_D', 'D')
    assert accepted['sessions_used'] == 0 and accepted['session_limit'] == 1
    assert not task.exists()
    with pytest.raises(ValueError, match='P765_NEW_BUDGET_SCOPE_OR_APPROVAL_INVALID'):
        helper.validate_experiment_budget(path, 'different-approval', task/'session_D', 'D')
    data['task_root'] = str(ROOT/'data/experiments/20261021_p761_01')
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='P765_NEW_BUDGET_SCOPE_OR_APPROVAL_INVALID'):
        helper.validate_experiment_budget(path, 'test-only',
                                          ROOT/'data/experiments/20261021_p761_01/session_D', 'D')


@pytest.mark.parametrize('sample,missed,deadlines', [(.005, 0, 0), (.010, 200, 200)])
def test_executor_qualification_is_recomputed_from_each_raw_interval_and_cost(
        sample, missed, deadlines):
    raw = dict(timer_times=[10.+i*sample for i in range(201)], shadow_costs=[sample]*200+[.001],
               shadow_epochs=[10.+i*.005 for i in range(200)], qualified=True, missed_cycles=0,
               compute_deadline_misses=0)
    actual = script('p765_qualification').recompute_executor_timing(raw)
    assert actual['missed_cycles'] == missed
    assert actual['compute_deadline_misses'] == deadlines
    assert actual['qualified'] is (sample == .005)


def test_historical_session_count_and_unqualified_b_bounds_remain_explicit():
    report = script('p764_qualification_audit').audit(
        ROOT/'data/experiments/20261021_p763_01/frozen/models')
    assert report['budget']['primary_sessions_used'] == 3
    assert report['budget']['primary_session_limit'] == 5
    assert report['budget']['primary_sessions_remaining'] == 2
    assert report['budget']['old_budget_sufficient'] is False
    assert report['budget']['historical_remaining_slots_usable_for_future_stages'] is False
    assert report['b_physical_bounds']['qualification'] == 'UNQUALIFIED_ENGINEERING_CANDIDATE'
    assert not report['b_physical_bounds']['actual_measurement_available']


def capture_fixture(tmp_path):
    session = tmp_path/'session_C'
    session.mkdir()
    identity = dict(sources={'runtime.py': 'source'}, contract_sha256='contract',
                    physical_identity=dict(model_manifest_sha256='model', model_files={'m': 'h'},
                                           calibration_sha256='a', detector_sha256='yolo',
                                           px4=dict(binary_sha256='px4', dds_sha256='dds')))
    (session/'validation_identity.json').write_text(json.dumps(identity))
    (session/'run_result.json').write_text(json.dumps(dict(
        stage='C', success=True, reason='CAPTURE_RADIUS_REACHED', y_sent=20.)))
    terminal = dict(success=True, capture_radius=.25, reason='CAPTURE_RADIUS_REACHED')
    (session/'validation.jsonl').write_text(json.dumps(dict(
        kind='result', receipt=30.02, message=terminal))+'\n')
    summary = dict(outcome='SUCCESS', reason='CAPTURE_RADIUS_REACHED',
                   capture_geometry_source='gazebo_native', evaluation_capture_radius=.25,
                   body_contact_geometry='SDF_MODEL_ORIGIN_FULL_ATTITUDE',
                   gazebo_pause_succeeded=True)
    (session/'run_summary.json').write_text(json.dumps(summary))
    samples = [dict(kind='EVALUATED', stamp=30.+i*.01, native_stamp=10.+i*.01,
                    decision_stamp=30.+i*.01+.02,
                    uav_position=[0., 0., -.4], target_position=[distance, 0., -.4],
                    body_lowest_z=-.1, maximum_bracket=.01,
                    uav_clock_bracket=[9.995+i*.01, 10.005+i*.01,
                                       29.995+i*.01, 30.005+i*.01],
                    target_clock_bracket=[9.995+i*.01, 10.005+i*.01,
                                          29.995+i*.01, 30.005+i*.01],
                    segment_valid=True, velocity_valid=True, distance=distance,
                    terminal_reason='CAPTURE_RADIUS_REACHED' if distance <= .25 else '')
               for i, distance in enumerate([.5, .2])]
    (session/'run_capture.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in samples))
    return session, identity


def test_native_capture_rejects_invalid_original_clock_mapping_even_if_distance_passes(tmp_path):
    helper = script('p765_qualification')
    session, identity = capture_fixture(tmp_path)
    path = session/'run_capture.jsonl'
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[0]['uav_clock_bracket'][3] += .1
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError, match='P765_NATIVE_CAPTURE_INVALID_NATIVE_GEOMETRY'):
        helper.capture_evidence(session, identity)


def test_d_binds_successful_c_to_all_current_physical_identities_and_raw_hashes(tmp_path):
    helper = script('p765_qualification')
    session, identity = capture_fixture(tmp_path)
    evidence = helper.capture_evidence(session, identity)
    (session/'capture_qualification.json').write_text(json.dumps(evidence))
    accepted = helper.validate_previous_c(session, identity)
    assert accepted['native_capture']['qualified']
    assert accepted['raw_evidence'][str(session/'run_capture.jsonl')] == sha256(
        session/'run_capture.jsonl')


@pytest.mark.parametrize('part', [
    'model_manifest_sha256', 'model_files', 'calibration_sha256', 'px4', 'detector_sha256'])
def test_d_rejects_changed_physical_model_px4_dds_or_calibration(tmp_path, part):
    helper = script('p765_qualification')
    session, identity = capture_fixture(tmp_path)
    (session/'capture_qualification.json').write_text(json.dumps(
        helper.capture_evidence(session, identity)))
    current = json.loads(json.dumps(identity))
    current['physical_identity'][part] = 'changed'
    with pytest.raises(ValueError, match='P765_C_CURRENT_IDENTITY_MISMATCH'):
        helper.validate_previous_c(session, current)


def test_d_rejects_tampered_native_capture_bytes(tmp_path):
    helper = script('p765_qualification')
    session, identity = capture_fixture(tmp_path)
    (session/'capture_qualification.json').write_text(json.dumps(
        helper.capture_evidence(session, identity)))
    with (session/'run_capture.jsonl').open('a') as stream:
        stream.write('{}\n')
    with pytest.raises(ValueError, match='P765_C_RAW_EVIDENCE'):
        helper.validate_previous_c(session, identity)


@pytest.mark.parametrize('tamper', ['missed_capture', 'sea_contact', 'wrong_radius'])
def test_saved_success_cannot_override_native_capture_failure(tmp_path, tamper):
    helper = script('p765_qualification')
    session, identity = capture_fixture(tmp_path)
    if tamper == 'wrong_radius':
        path = session/'run_summary.json'
        summary = json.loads(path.read_text())
        summary['evaluation_capture_radius'] = .5
        path.write_text(json.dumps(summary))
    else:
        path = session/'run_capture.jsonl'
        rows = [json.loads(x) for x in path.read_text().splitlines()]
        if tamper == 'sea_contact':
            rows[0]['body_lowest_z'] = .01
        else:
            rows[1].update(target_position=[.4, 0., -.4], distance=.4)
        path.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    with pytest.raises(ValueError, match='P765_NATIVE_CAPTURE'):
        helper.capture_evidence(session, identity)


def test_saved_e2_pass_cannot_override_unchanged_500ms_stress_failure():
    e2 = json.loads((ROOT/'data/baselines/20261021_p764_validation_01/'
                     'e2_control_replay.json').read_text())
    e2['passed'] = e2['logic_passed'] = True
    for row in e2['response_cases']:
        row['passed'] = True
    # Re-pin present source bytes so this tests recomputation, not an old fingerprint.
    e2['source_inputs'] = {p: sha256(p) for p in e2['source_inputs']}
    with pytest.raises(ValueError, match='P765_E2_FIXED_STRESS_NOT_QUALIFIED'):
        script('p765_qualification').validate_e2_evidence(e2)
