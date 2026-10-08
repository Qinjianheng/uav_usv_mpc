# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""Recorded epoch/configuration and replay input-selection contracts."""

from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[3] / 'scripts/mpc_seed_replay.py'


def _module():
    assert SCRIPT.exists(), 'shadow replay tool has not been implemented'
    spec = importlib.util.spec_from_file_location('mpc_seed_replay_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _record(module, cycle=0):
    experiment = module.experiment
    FollowMpcSeed, MpcConfig, _, _ = experiment._core()
    config = MpcConfig(control_blocks=1)
    solver = FollowMpcSeed(config)
    request = experiment.json_safe(experiment.build_scenario('constant_velocity', cycle))
    request['context']['prediction_source'] = 'tracking'
    return {'schema_version': 1, 'event': 'completion', 'request': request,
            'accepted_by_tracker': False, 'event_stamp': request['now_stamp'] + .2,
            'input_provenance': {'model_config': {
                'mpc': asdict(config), 'intrinsics': asdict(solver.intrinsics),
                'extrinsics': asdict(solver.extrinsics), 'target': asdict(solver.target),
                'visibility': asdict(solver.visibility),
                'attitude': asdict(solver.attitude_config)}},
            'output': {'valid': False, 'reason': 'CYCLE_DEADLINE_EXCEEDED'},
            'core_result': {'valid': True}, 'admission_status': 'CYCLE_DEADLINE_EXCEEDED'}


def test_replay_restores_original_navigation_and_now_not_late_event_stamp():
    module = _module()
    record = _record(module)
    request, solver = module.restore_case(record)
    assert request.now_stamp == 100.
    assert request.context.navigation_stamp == 100.
    assert request.context.observation_stamp == 99.99
    assert request.context.execution_start_stamp == 100.
    assert request.now_stamp != record['event_stamp']
    assert solver.config.control_blocks == 1
    assert not solver.config.allow_synthetic_predictions
    assert module.experiment.json_safe(request) == record['request']


def test_replay_baseline_uses_recorded_mount_instead_of_current_default():
    module = _module()
    record = _record(module)
    record['input_provenance']['model_config']['extrinsics']['translation_flu'] = [1., 2., 3.]
    _, solver = module.restore_case(record)
    assert solver.extrinsics.translation_flu == (1., 2., 3.)


def test_loader_skips_rejections_and_retains_original_line_numbers_and_limit(tmp_path):
    module = _module()
    path = tmp_path / 'shadow.jsonl'
    records = [{'schema_version': 1, 'event': 'reject', 'request': None},
               _record(module, 0), _record(module, 1), _record(module, 2)]
    path.write_text(''.join(json.dumps(record) + '\n' for record in records))
    cases, audit = module.load_replay_cases(path, maximum_records=2)
    assert [case['line_number'] for case in cases] == [2, 3]
    assert cases[1]['record']['request']['context']['cycle_id'] == 1
    assert audit['skipped_noncompletion_count'] == 1
    assert audit['selected_count'] == 2


@pytest.mark.parametrize('limit', [0, 21, -1])
def test_loader_enforces_twenty_record_cap(tmp_path, limit):
    with pytest.raises(ValueError, match='1.*20'):
        _module().load_replay_cases(tmp_path / 'missing.jsonl', maximum_records=limit)


def test_missing_geometry_configuration_is_rejected_instead_of_using_defaults():
    module = _module()
    record = _record(module)
    del record['input_provenance']['model_config']['extrinsics']
    with pytest.raises(ValueError, match='configuration'):
        module.restore_case(record)


def test_replay_refuses_synthetic_or_tracker_accepted_record():
    module = _module()
    record = _record(module)
    record['request']['context']['prediction_source'] = 'synthetic'
    with pytest.raises(ValueError, match='tracking'):
        module.restore_case(record)
    record = _record(module)
    record['accepted_by_tracker'] = True
    with pytest.raises(ValueError, match='research'):
        module.restore_case(record)


def test_replay_baselines_have_one_identical_input_fingerprint_and_real_residual():
    module = _module()
    record = _record(module)
    request, solver = module.restore_case(record)
    original = module.experiment.fingerprint(record['request'])
    for kind in ('quintic', 'viewpoint_interpolation'):
        controls = module.experiment.baseline_controls(request, solver.config, kind)
        result = solver.evaluate_controls(request, controls, kind)
        assert result.valid, result.reason
        assert result.positions[0] == (-5., 0., -5.)
        assert result.metrics['dynamics_residual'] < 1e-9
        assert module.experiment.fingerprint(request) == original
