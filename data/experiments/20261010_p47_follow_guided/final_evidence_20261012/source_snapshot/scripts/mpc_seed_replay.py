#!/usr/bin/env python3
# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""Replay at most twenty recorded shadow requests, preserving their original epochs.

Recomputed seeds are offline research comparisons. They cannot reverse the live
node's admission rejection and have no flight tracker acceptance or publication.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


SCRIPTS = str(Path(__file__).resolve().parent)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
import mpc_seed_experiment as experiment  # noqa: E402


def _freeze(value):
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, dict):
        return {key: _freeze(item) for key, item in value.items()}
    return value


def load_replay_cases(path, maximum_records=20):
    """Select request-bearing completion lines in recorded order without repairs."""
    if not isinstance(maximum_records, int) or not 1 <= maximum_records <= 20:
        raise ValueError('maximum_records must be between 1 and 20')
    cases = []
    skipped = scanned = 0
    with Path(path).open() as stream:
        for number, line in enumerate(stream, 1):
            scanned += 1
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f'invalid shadow JSONL at line {number}') from error
            if not isinstance(record, dict):
                raise ValueError(f'shadow JSONL line {number} must be an object')
            if record.get('event') != 'completion' or record.get('request') is None:
                skipped += 1
                continue
            cases.append({'line_number': number, 'record': record})
            if len(cases) == maximum_records:
                break
    return cases, {'scanned_line_count': scanned, 'selected_count': len(cases),
                   'skipped_noncompletion_count': skipped, 'maximum_records': maximum_records}


def restore_case(record):
    """Use the recorded configuration and request; no current-time substitution."""
    if record.get('schema_version') != 1:
        raise ValueError('unsupported shadow schema version')
    if record.get('accepted_by_tracker') is not False:
        raise ValueError('replay requires an explicitly unaccepted research record')
    try:
        model = record['input_provenance']['model_config']
        required = ('mpc', 'intrinsics', 'extrinsics', 'target', 'visibility', 'attitude')
        if not all(isinstance(model.get(key), dict) for key in required):
            raise ValueError('incomplete recorded model configuration')
        data = _freeze(record['request'])
        if data['context']['prediction_source'] != 'tracking':
            raise ValueError('recorded replay requires tracking predictions')
        FollowMpcSeed, MpcConfig, MpcRequest, PlanningContext = experiment._core()
        from uav_control.guidance.camera_visibility import (
            CameraExtrinsics, CameraIntrinsics, TargetBoundingSphere, VisibilityConstraints,
        )
        from uav_control.guidance.planned_attitude import AttitudeConfig
        config = MpcConfig(**model['mpc'])
        if config.allow_synthetic_predictions:
            raise ValueError('research shadow configuration must close the synthetic gate')
        request = MpcRequest(**{**data, 'context': PlanningContext(**data['context'])})
        solver = FollowMpcSeed(config=config, intrinsics=CameraIntrinsics(**model['intrinsics']),
                               extrinsics=CameraExtrinsics(**_freeze(model['extrinsics'])),
                               target=TargetBoundingSphere(**_freeze(model['target'])),
                               visibility=VisibilityConstraints(**model['visibility']),
                               attitude_config=AttitudeConfig(**model['attitude']))
    except (KeyError, TypeError) as error:
        raise ValueError('missing or invalid recorded request/configuration') from error
    if experiment.json_safe(request) != record['request']:
        raise ValueError('request restoration changed recorded values')
    return request, solver


def run_replay(input_jsonl, output_dir, maximum_records=20):
    """Compare four methods on original inputs/configurations, then hash final artifacts."""
    overall_start = time.perf_counter()
    output_dir = experiment.validate_output_path(output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError('output directory must be empty')
    input_jsonl = Path(input_jsonl).resolve()
    cases, audit = load_replay_cases(input_jsonl, maximum_records)
    if not cases:
        raise ValueError('no request-bearing completion records were found')
    # Validate all selected cases before making an output directory.
    restored = [restore_case(case['record']) for case in cases]
    input_load_s = time.perf_counter() - overall_start
    input_bytes = input_jsonl.read_bytes()
    first_solver = restored[0][1]
    provenance = experiment._provenance(first_solver.config, first_solver)
    for relative in ('scripts/mpc_seed_replay.py',
                     'src/uav_control/uav_control/controllers/follow_mpc_shadow_node.py'):
        path = experiment.WORKSPACE / relative
        if path.exists():
            provenance['source_sha256'][relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    provenance.update({
        'input_jsonl': str(input_jsonl),
        'input_jsonl_sha256': hashlib.sha256(input_bytes).hexdigest(),
        'prediction_provenance': 'recorded tracking predictions, original offline epochs retained',
        'rolling_state_source': 'original recorded navigation and measured attitude snapshots',
        'acceptance_boundary': 'offline replay only; original node rejection remains unchanged',
        'input_selection': audit, 'input_load_and_validation_s': input_load_s,
        'selected_line_numbers': [case['line_number'] for case in cases],
        'model_configurations': [case['record']['input_provenance']['model_config']
                                 for case in cases],
    })
    output_dir.mkdir(parents=True, exist_ok=True)
    experiment.write_json(output_dir / 'configuration.json', provenance)
    files = ['configuration.json']
    records = []
    pairs = []
    warm_solver = None
    previous_config_sha = None
    for case in cases:
        prep_start = time.perf_counter()
        source = case['record']
        request, cold_template = restore_case(source)
        config = cold_template.config
        model = source['input_provenance']['model_config']
        config_sha = experiment.fingerprint(model)
        if warm_solver is None or config_sha != previous_config_sha:
            warm_solver = cold_template
            previous_config_sha = config_sha
        input_sha = experiment.fingerprint(request)
        source_sha = experiment.fingerprint(source)
        prep_s = time.perf_counter() - prep_start
        pair = {}
        for method in ('quintic', 'viewpoint_interpolation', 'mpc_warm', 'mpc_cold'):
            start = time.perf_counter()
            _, solver = restore_case(source)
            if method == 'mpc_warm':
                solver = warm_solver
            controls = None
            if method in ('quintic', 'viewpoint_interpolation'):
                controls = experiment.baseline_controls(request, config, method)
            solve_start = time.perf_counter()
            if controls is None:
                output = solver.solve(request)
            else:
                output = solver.evaluate_controls(request, controls, candidate_kind=method)
            solve_s = time.perf_counter() - solve_start
            if experiment.fingerprint(request) != input_sha:
                raise RuntimeError('solver mutated recorded replay input')
            record = {
                'scenario': 'recorded_shadow', 'source_line_number': case['line_number'],
                'method': method, 'replay_only': True, 'accepted_by_tracker': False,
                'input_sha256': input_sha, 'model_config_sha256': config_sha,
                'source_record_sha256': source_sha, 'source_record': source,
                'original_admission_status': source.get('admission_status'),
                'configuration': model, 'input': experiment.json_safe(request),
                'baseline_controls': experiment.json_safe(controls),
                'output': experiment.json_safe(output),
                'metrics': experiment.output_metrics(request, output, config),
                'timing': {'shared_data_prep_s': prep_s, 'external_budget_s': config.solve_budget,
                           'solve_external_s': solve_s,
                           'core_timing': experiment.json_safe(output.timing)},
            }
            relative = f'line_{case["line_number"]:08d}_{method}.json'
            experiment.write_json(output_dir / relative, record)
            record['timing']['end_to_end_s'] = prep_s + time.perf_counter() - start
            experiment.write_json(output_dir / relative, record)
            files.append(relative)
            records.append(record)
            pair[method] = record
        pairs.append(pair)
    summary = {
        'methods': experiment.summarize(records), 'request_count': len(cases),
        'candidate_count': len(records), 'same_input_pair_count': len(pairs),
        'paired_warm_started_count': sum(pair['mpc_warm']['output']['warm_started']
                                         for pair in pairs),
        'paired_warm_minus_cold_s': experiment.percentiles([
            pair['mpc_warm']['timing']['solve_external_s']
            - pair['mpc_cold']['timing']['solve_external_s'] for pair in pairs]),
        'total_wall_s_before_summary': time.perf_counter() - overall_start,
        'replay_only': True, 'accepted_by_tracker': False,
    }
    experiment.write_json(output_dir / 'summary.json', summary)
    files.append('summary.json')
    experiment.write_manifest(output_dir, files, provenance)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-jsonl', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--max-records', type=int, default=20)
    args = parser.parse_args(argv)
    summary = run_replay(args.input_jsonl, args.output_dir, args.max_records)
    keys = ('count', 'feasible_rate', 'timeout_rate', 'pipeline_feasible_rate', 'end_to_end_s')
    compact = {name: {key: values[key] for key in keys}
               for name, values in summary['methods'].items()}
    print(json.dumps({'output_dir': str(args.output_dir.resolve()),
                      'request_count': summary['request_count'], 'methods': compact}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
