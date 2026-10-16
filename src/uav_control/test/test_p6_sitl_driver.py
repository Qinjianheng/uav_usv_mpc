"""P6 SITL preparation/gates exercise files and boundaries, never start a flight."""

import copy
import importlib
import json
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))


def driver():
    assert importlib.util.find_spec('p6_minco_sitl'), 'P6 SITL driver is missing'
    return importlib.import_module('p6_minco_sitl')


@pytest.fixture
def bundle_inputs(tmp_path, monkeypatch):
    entry = driver()
    experiment_root = tmp_path / 'experiments'
    experiment_root.mkdir()
    monkeypatch.setattr(entry, 'EXPERIMENT_ROOT', experiment_root)
    flight = yaml.safe_load(entry.DEFAULT_FLIGHT.read_text())
    research = yaml.safe_load(entry.DEFAULT_RESEARCH.read_text())
    flight_path, research_path = tmp_path / 'flight.yaml', tmp_path / 'research.yaml'
    flight_path.write_text(yaml.safe_dump(flight))
    research_path.write_text(yaml.safe_dump(research))
    return entry, experiment_root, flight_path, research_path


def test_ten_imm_only_conditions_keep_pilot_baseline_order():
    entry = driver()
    cases = entry.matrix_conditions()
    assert len(cases) == 10
    assert len({(case['motion_mode'], case['group']) for case in cases}) == 10
    assert cases[0]['motion_mode'] == 'figure_eight' and cases[0]['group'] == 'B'
    assert set(entry.GROUPS) == {'B', 'D'}
    assert entry.GROUPS['B'] == ('imm', 'nominal', False)
    assert entry.GROUPS['D'] == ('imm', 'robust', True)


def test_default_linear_pilot_inherits_f2_motion_instead_of_figure_eight_velocity(
        bundle_inputs):
    entry, root, _, _ = bundle_inputs
    case = entry.prepare_case(root / 'default_pilot', 'linear', 'B', role='pilot')
    saved = yaml.safe_load((case / 'config/flight.yaml').read_text())
    motion = saved['moving_target']['ros__parameters']
    assert (motion['velocity_x'], motion['velocity_y'], motion['horizontal_speed']) == (2., 0., 2.)
    assert motion['trajectory_type'] == 'linear'
    sources = json.loads((case / 'case.json').read_text())['config_sources']
    assert any('f2_h16_01/config/flight.yaml' in path for path in sources)


def test_imm_only_preparation_refuses_obsolete_shadow_switch(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    with pytest.raises(ValueError, match='IMM_SHADOW_REMOVED'):
        entry.prepare_case(root / 'shadow', 'linear', 'B', flight, research, imm_shadow=True)
    assert not (root / 'shadow').exists()


@pytest.mark.parametrize('mode', [
    'linear', 'figure_eight', 'variable_speed_eight',
    'variable_speed_s', 'variable_speed_z',
])
def test_prepare_copies_full_inputs_and_changes_only_explicit_p6_choices(bundle_inputs, mode):
    entry, root, flight_path, research_path = bundle_inputs
    original = flight_path.read_bytes(), research_path.read_bytes()
    case = entry.prepare_case(root / mode, mode, 'B', flight_path, research_path)
    saved = yaml.safe_load((case / 'config/flight.yaml').read_text())
    research = yaml.safe_load((case / 'config/research.yaml').read_text())
    manifest = json.loads((case / 'case.json').read_text())
    assert saved['moving_target']['ros__parameters']['trajectory_type'] == mode
    original_nodes = set(yaml.safe_load(flight_path.read_text()))
    assert set(saved) == original_nodes | {'imm_target_predictor_node'}
    assert research['p4_follow_planner_node']['ros__parameters']['p6_visibility_mode'] == 'nominal'
    assert saved['trajectory_tracker_node']['ros__parameters']['p6_adaptive_viewpoint'] is False
    assert manifest['observation_seconds'] == 60.
    assert manifest['launch_environment']['UAV_USV_P6_PREDICTOR'] == 'imm'
    assert manifest['launch_environment']['UAV_USV_P6_IMM_SHADOW'] == 'false'
    assert manifest['launch_environment']['UAV_USV_P6_RECORD'] == '1'
    assert (flight_path.read_bytes(), research_path.read_bytes()) == original
    assert not (case / 'flight').exists()
    entry.verify_case(case)
    with pytest.raises(FileExistsError):
        entry.prepare_case(case, mode, 'B', flight_path, research_path)


def test_full_imm_and_motion_parameters_are_materialized_without_overwriting_user_values(
        bundle_inputs):
    entry, root, flight, research = bundle_inputs
    config = yaml.safe_load(flight.read_text())
    config['moving_target']['ros__parameters']['variable_speed_period'] = 30.
    config['moving_target']['ros__parameters']['initial_x'] = 11.
    config['imm_target_predictor_node'] = dict(ros__parameters=dict(imm_initial_velocity_std=2.))
    flight.write_text(yaml.safe_dump(config))
    case = entry.prepare_case(root / 'explicit', 'variable_speed_s', 'B', flight, research)
    saved = yaml.safe_load((case / 'config/flight.yaml').read_text())
    motion = saved['moving_target']['ros__parameters']
    assert motion['variable_speed_period'] == 30. and motion['initial_x'] == 11.
    assert motion['maximum_horizontal_acceleration'] == 3.2
    assert motion['maximum_angular_acceleration'] == .35
    assert motion['variable_startup_duration'] == 8.
    imm = saved['imm_target_predictor_node']['ros__parameters']
    assert imm['imm_initial_velocity_std'] == 2.
    assert imm['input_timeout'] == .125
    assert imm['observation_topic'] == '/perception/front/target_observation'
    assert imm['prediction_topic'] == '/planning/target_prediction'
    assert imm['imm_transition_matrix'] == [.96, .025, .015, .025, .95, .025, .015, .035, .95]
    from uav_control.tracking.imm_predictor import ImmConfig
    assert all('imm_' + name in imm for name in ImmConfig.__dataclass_fields__)


def test_robust_case_requires_calibration_and_embeds_identical_risk_for_both_sides(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    with pytest.raises(ValueError, match='P6_CALIBRATION_REQUIRED'):
        entry.prepare_case(root / 'missing', 'variable_speed_z', 'D', flight, research)
    assert not (root / 'missing').exists()
    calibration = root.parent / 'calibration.json'
    calibration.write_text('{"calibrated": true}\n')
    case = entry.prepare_case(root / 'd', '5', 'D', flight, research,
                              calibration_file=calibration, imm_shadow=False)
    saved_flight = yaml.safe_load((case / 'config/flight.yaml').read_text())
    saved_research = yaml.safe_load((case / 'config/research.yaml').read_text())
    receiver = saved_flight['trajectory_tracker_node']['ros__parameters']
    planner = saved_research['p4_follow_planner_node']['ros__parameters']
    for key in ('p6_visibility_mode', 'p6_adaptive_viewpoint', 'p6_calibration_file'):
        assert receiver[key] == planner[key]
    assert Path(receiver['p6_calibration_file']).read_bytes() == calibration.read_bytes()
    assert planner['p6_visibility_mode'] == 'robust'
    assert planner['p6_adaptive_viewpoint'] is True


def test_prepare_rejects_wrong_role_and_frozen_or_existing_namespace(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    for mode, group, role, shadow in (
            ('figure_eight', 'B', 'pilot', False),
            ('linear', 'D', 'pilot', False),
            ('linear', 'A', 'pilot', False),
            ('linear', 'B', 'baseline', False),
            ('figure_eight', 'D', 'baseline', False)):
        with pytest.raises(ValueError, match='P6_ROLE_MISMATCH'):
            entry.prepare_case(root / 'bad', mode, group, flight, research,
                               role=role, imm_shadow=shadow)
    with pytest.raises(ValueError, match='NEW_P6_EXPERIMENT_DIRECTORY_REQUIRED'):
        entry.prepare_case(root.parent / 'baselines/bad', 'linear', 'B', flight, research)


def test_config_mutation_invalidates_prepared_case_before_run(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    case = entry.prepare_case(root / 'a', 'linear', 'B', flight, research)
    (case / 'config/flight.yaml').write_text('changed')
    with pytest.raises(ValueError, match='P6_CASE_ARTIFACT_CHANGED'):
        entry.verify_case(case)


def test_p5_prepare_overlay_has_no_historical_flight_dependency(bundle_inputs, monkeypatch):
    entry, root, flight, research = bundle_inputs
    import p5_minco_sitl as harness
    monkeypatch.setattr(harness, 'legacy_prepare', lambda *args: pytest.fail('historical read'))
    condition = ('overlay', 'f2', False, 'p47_follow_guided', 3., 1., 1.6, 'none')
    harness.prepare(root / 'overlay', condition, overlay_config=flight)
    saved = yaml.safe_load((root / 'overlay/config/flight.yaml').read_text())
    assert saved['moving_target']['ros__parameters']['trajectory_type'] == 'figure_eight'
    assert (root / 'overlay/config/research.yaml').exists()


def test_p5_prepare_none_still_delegates_unchanged_legacy_boundary(tmp_path, monkeypatch):
    driver()
    import p5_minco_sitl as harness
    calls = []
    monkeypatch.setattr(harness, 'legacy_prepare', lambda *args: calls.append(args))
    condition = ('original', 'f2', False, 'p47_follow_guided', 3., 1., 1.6, 'none')
    harness.prepare(tmp_path / 'old', condition)
    assert calls == [(tmp_path / 'old', condition)]


def test_60_second_assessment_retains_late_owner_loss(tmp_path, monkeypatch):
    driver()
    import p4_follow_analysis
    import p45_execution_analysis
    import p5_minco_sitl as harness
    rows = [dict(kind='diagnostic', receipt=100. + i * .05, stamp=100. + i * .05,
                 status='MINCO_FOLLOW' if i < 600 else 'FOLLOW') for i in range(1201)]
    authority = dict(discovered_samples=1, all_discovered_single_tracker=True, conflicts=[])
    monkeypatch.setattr(p4_follow_analysis, 'read', lambda path: (rows, []))
    monkeypatch.setattr(p45_execution_analysis, 'analyze', lambda path: dict(
        authority=authority, lifecycle=dict(accepted_unique=1, active_unique=1),
        native_command=dict(period_seconds=dict(count=1))))
    output, task = tmp_path / 'output', tmp_path / 'task'
    output.mkdir()
    task.mkdir()
    summary = harness.assess(output, 'f2', 'MINCO_OBSERVATION_COMPLETE', 100.,
                             dict(reason='WATCHDOG', monitor_returncode=0, still_alive=False),
                             task_root=task, observation_seconds=60.)
    assert summary['observation_seconds'] > 59.9
    assert .49 < summary['minco_fraction'] < .51


def test_p6_disabled_constraint_fingerprint_matches_actual_r11_reference():
    entry = driver()
    assert hasattr(entry.harness, 'constraint_metadata'), 'actual P6 constraint binding is missing'
    research = yaml.safe_load(entry.DEFAULT_RESEARCH.read_text())
    research['p4_follow_planner_node']['ros__parameters'].update(
        p6_visibility_mode='nominal', p6_adaptive_viewpoint=False)
    original = json.loads((entry.R11.parent / 'condition.json').read_text())
    expected = original['constraint_fingerprint']
    metadata = entry.harness.constraint_metadata(expected, research, p6_enabled=True)
    assert metadata['constraint_fingerprint'] == expected
    assert metadata['baseline_reference_constraint_fingerprint'] == expected
    assert 'p6' not in metadata['actual_constraint_snapshot']


def test_p6_enabled_constraints_do_not_relabel_historical_fingerprint():
    entry = driver()
    assert hasattr(entry.harness, 'constraint_metadata'), 'actual P6 constraint binding is missing'
    research = yaml.safe_load(entry.DEFAULT_RESEARCH.read_text())
    research['p4_follow_planner_node']['ros__parameters'].update(
        p6_visibility_mode='nominal', p6_adaptive_viewpoint=True)
    metadata = entry.harness.constraint_metadata('historical-reference', research, p6_enabled=True)
    assert metadata['baseline_reference_constraint_fingerprint'] == 'historical-reference'
    assert metadata['constraint_fingerprint'] != 'historical-reference'
    assert metadata['actual_constraint_snapshot']['p6']['adaptive_viewpoint'] is True
    assert entry.harness.constraint_metadata('legacy', research) == dict(
        constraint_fingerprint='legacy')


def test_nominal_p6_refuses_actual_constraint_difference_before_attempt():
    entry = driver()
    assert hasattr(entry.harness, 'constraint_metadata'), 'actual P6 constraint binding is missing'
    research = yaml.safe_load(entry.DEFAULT_RESEARCH.read_text())
    research['p4_follow_planner_node']['ros__parameters'].update(
        p6_visibility_mode='nominal', p6_adaptive_viewpoint=False)
    with pytest.raises(ValueError, match='P6_NOMINAL_BASELINE_CONSTRAINT_MISMATCH'):
        entry.harness.constraint_metadata('wrong-baseline', research, p6_enabled=True)


def qualification_files(entry, directory, case, *, reviewed=True):
    directory.mkdir()
    verification = directory / 'verification.json'
    verification.write_text(json.dumps(dict(passed=True, independent_review_pass=reviewed)))
    import p61r_imm_qualification as primary
    replay, review, gate = [directory / name for name in (
        'replay.json', 'review.json', 'primary.json')]
    replay.write_text(json.dumps(dict(attempts=1200, accepted=1200, strict_identity_matches=1200,
                                     expired_rejections=1200, unsafe_output_count=0,
                                     sample_counts={'41': 1200}, compute_seconds={'p95': .01})))
    review.write_text(json.dumps(dict(review_pass=reviewed)))
    gate.write_text(json.dumps(dict(
        kind=primary.KIND, scope='linear_nominal_pilot', passed=reviewed,
        evidence=[dict(path=str(p), sha256=entry.harness.sha(p), role=role)
                  for p, role in ((replay, 'replay'), (review, 'review'),
                                  (verification, 'verification'))],
        source_checks=[dict(path=str(p), sha256=entry.harness.sha(p)) for p in primary.sources()])))
    qualification = directory / 'qualification.json'
    metadata = json.loads((case / 'case.json').read_text())
    qualification.write_text(json.dumps(dict(
        kind=entry.KIND, candidate=entry.harness.CANDIDATE,
        offline_gate_pass=True, shadow_gate_pass=False, robust_gate_pass=False,
        imm_primary_evidence=dict(path=str(gate), sha256=entry.harness.sha(gate)),
        qualified_cases=[metadata['case_id']],
        qualified_case_manifests={metadata['case_id']: entry.harness.sha(case / 'SHA256SUMS')},
        source_checks=[dict(path=str(path.relative_to(entry.WORKSPACE)),
                            current_sha256=entry.harness.sha(path))
                       for path in entry.critical_sources()])))
    return verification, qualification


def test_readiness_binds_case_config_source_review_and_qualification(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    case = entry.prepare_case(root / 'pilot', 'linear', 'B', flight, research,
                              role='pilot', imm_shadow=False)
    verification, qualification = qualification_files(entry, root.parent / 'q', case)
    readiness = root.parent / 'readiness.json'
    entry.write_readiness(readiness, case, verification, qualification)
    entry.require_readiness(readiness, case, qualification)
    record = json.loads(readiness.read_text())
    assert record['kind'] == entry.KIND
    sources = {row['path'] for row in record['source_checks']}
    for name in ('guidance/robust_visibility.py', 'guidance/adaptive_viewpoint.py',
                 'guidance/p6_follow.py', 'tracking/imm_predictor.py',
                 'controllers/p4_follow_planner_node.py', 'control/trajectory_tracker_node.py'):
        assert 'src/uav_control/uav_control/' + name in sources
    mutated = copy.deepcopy(record)
    mutated['source_checks'][0]['sha256'] = '0' * 64
    readiness.write_text(json.dumps(mutated))
    with pytest.raises(ValueError, match='P6_READINESS_SOURCE_CHANGED'):
        entry.require_readiness(readiness, case, qualification)


def test_readiness_cannot_be_created_from_missing_review_or_fake_flight_pass(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    case = entry.prepare_case(root / 'pilot', 'linear', 'B', flight, research,
                              role='pilot', imm_shadow=False)
    verification, qualification = qualification_files(
        entry, root.parent / 'q', case, reviewed=False)
    with pytest.raises(ValueError, match='P6_VERIFICATION_AND_REVIEW_REQUIRED'):
        entry.write_readiness(root.parent / 'readiness.json', case, verification, qualification)
    assert not (root.parent / 'readiness.json').exists()


def test_existing_case_name_cannot_qualify_a_different_full_config(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    case = entry.prepare_case(root / 'first', 'linear', 'B', flight, research,
                              role='pilot', imm_shadow=False)
    verification, qualification = qualification_files(entry, root.parent / 'q', case)
    changed = yaml.safe_load(flight.read_text())
    changed['moving_target']['ros__parameters']['horizontal_speed'] = 3.5
    flight.write_text(yaml.safe_dump(changed))
    replacement = entry.prepare_case(root / 'second', 'linear', 'B', flight, research,
                                     role='pilot', imm_shadow=False)
    with pytest.raises(ValueError, match='P6_CASE_CONFIG_NOT_QUALIFIED'):
        entry.write_readiness(root.parent / 'readiness.json', replacement,
                              verification, qualification)


def test_qualification_must_cover_all_integrated_runtime_sources(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    case = entry.prepare_case(root / 'pilot', 'linear', 'B', flight, research,
                              role='pilot', imm_shadow=False)
    verification, qualification = qualification_files(entry, root.parent / 'q', case)
    record = json.loads(qualification.read_text())
    record['source_checks'].pop()
    qualification.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='P6_SOURCES_NOT_QUALIFIED'):
        entry.write_readiness(root.parent / 'readiness.json', case, verification, qualification)


def test_new_driver_requires_explicit_execute_without_touching_processes(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    case = entry.prepare_case(root / 'pilot', 'linear', 'B', flight, research,
                              role='pilot', imm_shadow=False)
    with pytest.raises(ValueError, match='P6_EXPLICIT_EXECUTION_REQUIRED'):
        entry.run_case(case, root.parent / 'missing-q', root.parent / 'missing-r')
    assert not (case / 'attempt_f2.json').exists()


def test_baseline_requires_real_healthy_60_second_linear_pilot(bundle_inputs):
    entry, root, flight, research = bundle_inputs
    case = entry.prepare_case(root / 'baseline', 'figure_eight', 'B', flight, research,
                              role='baseline')
    with pytest.raises(ValueError, match='P6_PRIOR_ACTUAL_RUN_REQUIRED'):
        entry.require_prior_run(case, None)
    prior = root / 'pilot_summary.json'
    prior.write_text(json.dumps(dict(
        completed=True, healthy=True, observation_seconds=60.,
        motion_mode='linear', role='pilot')))
    with pytest.raises(ValueError, match='P6_PRIOR_ACTUAL_RUN_REQUIRED'):
        entry.require_prior_run(case, prior)


def test_driver_does_not_call_a_partial_recording_a_complete_60s_window(
        bundle_inputs, monkeypatch):
    entry, root, flight, research = bundle_inputs
    case = entry.prepare_case(root / 'pilot', 'linear', 'B', flight, research,
                              role='pilot', imm_shadow=False)
    verification, qualification = qualification_files(entry, root.parent / 'q', case)
    readiness = root.parent / 'readiness.json'
    entry.write_readiness(readiness, case, verification, qualification)
    original_environment = entry.os.environ.get('UAV_USV_P6_PREDICTOR')

    def external_flight(output, qualification_path, scene, **options):
        # The external flight process is the only replaced boundary. Its
        # recorded result deliberately contains inadequate 60s coverage.
        assert options['observation_seconds'] == 60.
        assert options['overlay_config'] == case / 'config/flight.yaml'
        assert options['extra_audit'] is entry._actual_execution_audit
        assert entry.os.environ['UAV_USV_P6_PREDICTOR'] == 'imm'
        assert entry.os.environ['UAV_USV_P6_IMM_SHADOW'] == 'false'
        assert entry.os.environ['UAV_USV_P6_RECORD'] == '1'
        output.mkdir()
        (output / 'p5_run_summary.json').write_text(json.dumps(dict(
            completed=True, healthy=True, observation_seconds=30., diagnostic_coverage=1.)))
        return 0

    monkeypatch.setattr(entry.harness, 'run', external_flight)
    assert entry.run_case(case, qualification, readiness, execute=True) == 1
    saved = json.loads((case / 'p6_run_summary.json').read_text())
    assert saved['window_evidence_complete'] is False
    assert saved['observation_seconds'] == 30.
    assert entry.os.environ.get('UAV_USV_P6_PREDICTOR') == original_environment
