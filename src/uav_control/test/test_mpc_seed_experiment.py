# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""Lightweight offline experiment contracts; no optimizer benchmarks here."""

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


PATH = Path(__file__).resolve().parents[3] / 'scripts/mpc_seed_experiment.py'


def _module():
    assert PATH.exists(), 'offline experiment tool has not been implemented'
    spec = importlib.util.spec_from_file_location('mpc_seed_experiment_test', PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_synthetic_straight_prediction_uses_analytic_same_epoch_motion():
    scene = _module().build_scenario('constant_velocity', cycle=2)
    assert scene['context']['prediction_source'] == 'synthetic'
    assert scene['context']['execution_start_stamp'] == pytest.approx(100.4)
    assert scene['target_positions'][0] == pytest.approx((0.4, 0.0, 0.0))
    assert scene['target_positions'][5] == pytest.approx((1.4, 0.0, 0.0))
    assert scene['target_velocities'][5] == pytest.approx((1.0, 0.0, 0.0))
    assert np.all(np.diff(scene['prediction_times']) > 0)


def test_turn_scenarios_have_opposite_lateral_acceleration_and_motion():
    module = _module()
    left = module.build_scenario('left_turn')
    right = module.build_scenario('right_turn')
    assert left['target_positions'][5][1] < 0 < right['target_positions'][5][1]
    assert left['target_velocities'][5][1] < 0 < right['target_velocities'][5][1]
    assert np.allclose(left['target_positions'][:, 0], right['target_positions'][:, 0])


def test_rolling_navigation_position_matches_its_reported_velocity():
    module = _module()
    first = module.build_scenario('figure_eight', cycle=0)
    later = module.build_scenario('figure_eight', cycle=2)
    assert later['state'][:3] - first['state'][:3] == pytest.approx((.24, .24, 0.))
    assert later['state'][3:6] == pytest.approx((.6, .6, 0.))


def test_invalid_scenarios_preserve_the_failure_instead_of_repairing_input():
    module = _module()
    expired = module.build_scenario('expired_prediction')
    assert expired['context']['prediction_valid_until'] < expired['now_stamp']
    times = module.build_scenario('nonmonotonic_prediction')['prediction_times']
    assert np.any(np.diff(times) <= 0)
    assert module.build_scenario('wrong_frame')['context']['frame_id'] == 'local_enu'


def test_percentiles_count_nonfinite_samples_and_use_standard_linear_quantiles():
    result = _module().percentiles([0.0, 10.0, 20.0, float('nan')])
    assert result == {'count': 4, 'finite_count': 3, 'p50': 10.0, 'p95': 19.0,
                      'p99': 19.8, 'minimum': 0.0, 'maximum': 20.0}
    assert _module().percentiles([])['p50'] is None


@pytest.mark.parametrize('steps', [1, 12])
def test_baseline_controls_keep_identical_initial_state_and_constant_motion(steps):
    module = _module()
    config = SimpleNamespace(dt=0.2, horizon_steps=steps, follow_distance=5., flight_altitude=-5.)
    scene = module.build_scenario('constant_velocity')
    request = SimpleNamespace(**scene)
    request.context = SimpleNamespace(**scene['context'])
    initial = np.asarray(request.state).copy()
    for kind in ('quintic', 'viewpoint_interpolation'):
        controls = module.baseline_controls(request, config, kind)
        assert controls.shape == (steps, 4)
        assert np.max(np.abs(controls)) < 1e-8
        assert np.array_equal(initial, request.state)


def test_json_manifest_contains_relative_paths_and_verifiable_content(tmp_path):
    module = _module()
    record = {'array': np.array([1., 2.]), 'valid': np.bool_(True),
              'invalid': float('nan')}
    module.write_json(tmp_path / 'case.json', record)
    decoded = json.loads((tmp_path / 'case.json').read_text())
    assert decoded['array'] == [1., 2.] and decoded['valid'] is True
    assert decoded['invalid'] == {'__nonfinite_float__': 'nan'}
    module.write_manifest(tmp_path, ['case.json'], {'git_head': 'example'})
    manifest = json.loads((tmp_path / 'manifest.json').read_text())
    assert manifest['files'][0]['path'] == 'case.json'
    assert manifest['files'][0]['sha256'] == hashlib.sha256(
        (tmp_path / 'case.json').read_bytes()).hexdigest()


@pytest.mark.parametrize('path', [
    '/home/qin/data/uav_usv/review_output',
    '/home/qin/data/uav_usv_mpc/data/baselines/20261006_pre_mpc/new_output',
])
def test_experiment_refuses_old_workspace_and_frozen_evidence_output(path):
    with pytest.raises(ValueError, match='protected'):
        _module().validate_output_path(path)


def test_metrics_report_unsafe_ratio_and_real_integrated_kinematics():
    module = _module()
    request = SimpleNamespace(
        state=np.array([0., 0., -5., 1., 0., 0., 0., 0., 0., 0.]),
        prediction_times=np.array([0., 1.]),
        target_positions=np.array([[5., 0., 0.], [6., 0., 0.]]),
        target_velocities=np.array([[1., 0., 0.], [1., 0., 0.]]),
        context=SimpleNamespace(execution_start_stamp=100., prediction_source_stamp=100.))
    output = SimpleNamespace(relative_times=np.array([0., 1.]),
                             positions=np.array([[0., 0., -5.], [1., 0., -5.]]),
                             velocities=np.array([[1., 0., 0.], [2., 0., 0.]]),
                             accelerations=np.array([[0., 0., 0.], [3., 0., 0.]]),
                             jerks=np.array([[4., 0., 0.]]),
                             visibility_margins=np.array([[.1, .2], [-.1, .3]]))
    config = SimpleNamespace(follow_distance=5., flight_altitude=-5.)
    metrics = module.output_metrics(request, output, config)
    assert metrics['visibility_ratio'] == .5
    assert metrics['minimum_horizontal_margin_rad'] == -.1
    assert metrics['maximum_speed_mps'] == 2.
    assert metrics['maximum_acceleration_mps2'] == 3.
    assert metrics['maximum_jerk_mps3'] == 4.
    assert metrics['follow_error_rms_m'] == 0.


def test_summary_does_not_merge_warm_and_cold_timing():
    module = _module()
    records = [
        {'method': 'mpc_warm', 'output': {'valid': True, 'reason': ''},
         'timing': {'solve_external_s': .1, 'end_to_end_s': .2},
         'metrics': {'visibility_ratio': 1.}},
        {'method': 'mpc_cold', 'output': {'valid': False, 'reason': 'DEADLINE_EXCEEDED'},
         'timing': {'solve_external_s': .5, 'end_to_end_s': .6},
         'metrics': {'visibility_ratio': .5}},
    ]
    summary = module.summarize(records)
    assert summary['mpc_warm']['feasible_rate'] == 1.
    assert summary['mpc_cold']['timeout_rate'] == 1.
    assert summary['mpc_warm']['solve_external_s']['p50'] == .1
    assert summary['mpc_cold']['solve_external_s']['p50'] == .5


def test_timeout_rate_uses_solver_status_when_reason_is_human_readable():
    record = {'method': 'mpc_cold',
              'output': {'valid': False, 'solver_status': 'DEADLINE_EXCEEDED',
                         'reason': 'whole preparation/solve/validation budget expired'},
              'timing': {'solve_external_s': .5, 'end_to_end_s': .6}, 'metrics': {}}
    assert _module().summarize([record])['mpc_cold']['timeout_rate'] == 1.


def test_pipeline_budget_can_fail_a_geometrically_valid_core_output():
    record = {'method': 'mpc_warm',
              'output': {'valid': True, 'solver_status': 'SUCCESS', 'reason': 'passed'},
              'timing': {'solve_external_s': .49, 'end_to_end_s': .51,
                         'external_budget_s': .5}, 'metrics': {}}
    summary = _module().summarize([record])['mpc_warm']
    assert summary['feasible_rate'] == 1.
    assert summary['pipeline_budget_exceeded_rate'] == 1.
    assert summary['pipeline_feasible_rate'] == 0.


def test_baseline_real_core_preserves_initial_pva_and_does_not_fake_residual():
    module = _module()
    FollowMpcSeed, MpcConfig, MpcRequest, PlanningContext = module._core()
    scene = module.build_scenario('constant_velocity')
    request = MpcRequest(**{**scene, 'context': PlanningContext(**scene['context'])})
    config = MpcConfig(allow_synthetic_predictions=True)
    solver = FollowMpcSeed(config=config)
    original_sha = module.fingerprint(request)
    for kind in ('quintic', 'viewpoint_interpolation'):
        controls = module.baseline_controls(request, config, kind)
        output = solver.evaluate_controls(request, controls, candidate_kind=kind)
        assert output.valid, output.reason
        assert output.positions[0] == pytest.approx((-5., 0., -5.))
        assert output.velocities[0] == pytest.approx((1., 0., 0.))
        assert output.accelerations[0] == pytest.approx((0., 0., 0.))
        assert output.positions[-1] == pytest.approx((-2.6, 0., -5.))
        assert output.metrics['dynamics_residual'] < 1e-9
    assert module.fingerprint(request) == original_sha


def test_cli_control_blocks_is_applied_to_the_actual_experiment_config(tmp_path):
    module = _module()
    arguments = module.parse_arguments(['--output-dir', str(tmp_path), '--control-blocks', '1'])
    config = module.configuration_from_args(arguments)
    assert config.control_blocks == 1
    assert config.allow_synthetic_predictions
    default_arguments = module.parse_arguments(['--output-dir', str(tmp_path)])
    defaults = module.configuration_from_args(default_arguments)
    assert defaults.control_blocks == module._core()[1]().control_blocks
