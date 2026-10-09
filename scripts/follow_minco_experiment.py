#!/usr/bin/env python3
# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""
Sequential, same-boundary MINCO comparisons using P2 scenarios or immutable live replay.

All methods share endpoint P/V/A, total horizon, calibration and one optimizer.
MPC contributes intermediate samples only, with its original solver unchanged.
Replay cannot reverse a live rejection or authorize tracker execution.
"""

import argparse
import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np

import mpc_seed_experiment as base
import mpc_seed_replay as replay


METHODS = ('simple', 'greedy', 'mpc')
VARIANTS = ('fixed', 'q', 'qt', 'qt_no_visibility_cost')
SCENARIOS = (*base.SCENARIOS, 'deceleration', 'epoch_mismatch')


def scenario(name, cycle=0):
    """Reuse P2 generation, adding a deceleration and inconsistent acquisition epoch."""
    _, _, Request, Context = base._core()
    data = base.build_scenario('speed_jump' if name == 'deceleration' else
                               'constant_velocity' if name == 'epoch_mismatch' else name, cycle)
    request = Request(**{**data, 'context': Context(**data['context'])})
    if name == 'epoch_mismatch':
        return replace(request, context=replace(request.context, attitude_stamp=99.))
    if name == 'deceleration':
        times = np.asarray(request.prediction_times) + cycle * .2
        positions = np.c_[3 * times - 2 * np.maximum(times - 1., 0.),
                          np.zeros((len(times), 2))]
        velocities = np.c_[np.where(times < 1., 3., 1.), np.zeros((len(times), 2))]
        state = (-5. + 3 * cycle * .2, 0., -5., 3., 0., 0., 0., 0., 0., 0.)
        return replace(request, state=state, target_positions=tuple(map(tuple, positions)),
                       target_velocities=tuple(map(tuple, velocities)))
    return request


def restore_record(record):
    """Reuse P2 recorded calibration validation; restore explicit future-boundary metadata."""
    base._core()
    from uav_control.guidance.follow_problem import FutureRequest
    stripped = copy.deepcopy(record)
    measurement = stripped['request'].pop('measurement_state', None)
    policy = stripped['request'].pop('boundary_policy', None)
    request, model = replay.restore_case(stripped)
    if measurement is not None:
        request = FutureRequest(**request.__dict__, measurement_state=tuple(measurement),
                                boundary_policy=policy)
    if base.json_safe(request) != record['request']:
        raise ValueError('replay changed original request')
    return request, model


def compare(request, model, variants=VARIANTS):
    """Run one optimizer at a time; preserve every invalid initialization and timing."""
    base._core()
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    initializer = GreedyFollowInitializer(model)
    common_start = time.perf_counter()
    common = initializer.build(request, method='simple')
    common_time = time.perf_counter() - common_start
    records = []
    for method in METHODS:
        init_start = time.perf_counter()
        mpc_result = None
        seed = initializer.build(request) if method == 'greedy' else common
        if method == 'mpc':
            mpc_result = model.solve(request)
            if common.valid_input and mpc_result.valid:
                times = np.asarray(mpc_result.relative_times)
                q = tuple(tuple(float(np.interp(t, times, np.asarray(
                    mpc_result.positions)[:, axis])) for axis in range(3)) for t in (.8, 1.6))
                yaw = tuple(float(np.interp(t, times, np.unwrap(mpc_result.yaw_refs)))
                            for t in (0., .8, 1.6, 2.4))
                seed = replace(common, q=q, yaw=yaw)
            else:
                seed = replace(common, valid_input=False, reason=mpc_result.reason)
        init_time = common_time + time.perf_counter() - init_start
        for variant in variants:
            start = time.perf_counter()
            remaining = model.config.solve_budget - init_time
            if remaining <= 0:
                result = MpcSeedResult(request.context, solver_status='DEADLINE_EXCEEDED',
                                       reason='initialization exhausted whole budget')
            else:
                config = MincoConfig(mode='qt' if variant.startswith('qt') else variant,
                                     visibility_weight=0. if variant.endswith('cost') else 20.,
                                     budget=remaining)
                result = FollowMincoOptimizer(config, model).solve(request, seed)
            record = {'method': method, 'variant': variant, 'request_sha256':
                      base.fingerprint(request), 'seed': base.json_safe(seed),
                      'common_endpoint': common.end, 'output': base.json_safe(result),
                      'initialization_s': init_time,
                      'mpc_initialization_status': (mpc_result.solver_status
                                                    if mpc_result else None),
                      'accepted_by_tracker': False}
            json.dumps(base.json_safe(record), allow_nan=False)
            total = init_time + time.perf_counter() - start
            record['end_to_end_s'] = total
            record['pipeline_feasible'] = result.valid and total < model.config.solve_budget
            records.append(record)
    return records


def summary(records):
    """Retain infeasible counts; quantiles describe recorded, finite phase timings only."""
    groups = {}
    for method in METHODS:
        for variant in VARIANTS:
            group = [r for r in records if r['method'] == method and r['variant'] == variant]
            if not group:
                continue
            reasons = {}
            for record in group:
                reason = record['output']['solver_status']
                reasons[reason] = reasons.get(reason, 0) + 1
            phases = {'initialization': base.percentiles([r['initialization_s'] for r in group]),
                      'end_to_end': base.percentiles([r['end_to_end_s'] for r in group])}
            for phase in ('preparation', 'minco_optimization',
                          'yaw_construction_within_optimization', 'validation'):
                phases[phase] = base.percentiles([
                    r['output']['timing'][phase] for r in group if phase in r['output']['timing']])
            # Yaw and Q/T share a single optimizer: this duration overlaps, not an extra phase.
            phases['joint_yaw_optimization_overlaps_minco'] = phases['minco_optimization']
            metrics = {key: base.percentiles([r['output']['metrics'][key] for r in group
                                             if key in r['output']['metrics']]) for key in (
                'visible_fraction', 'minimum_horizontal_margin', 'minimum_vertical_margin',
                'follow_rmse', 'jerk_integral')}
            groups[method + '/' + variant] = {
                'count': len(group), 'sampled_feasible': sum(r['output']['valid'] for r in group),
                'pipeline_feasible': sum(r['pipeline_feasible'] for r in group),
                'optimizer_success': sum(r['output']['metrics'].get('optimizer_success', False)
                                         for r in group),
                'timeout': sum(r['end_to_end_s'] >= .5 or r['output']['solver_status'] in
                               ('DEADLINE_EXCEEDED', 'TIMEOUT') for r in group),
                'statuses': reasons, 'seconds': phases, 'metrics': metrics}
    return groups


def run(output_dir, cycles=2, input_jsonl=None, max_records=20):
    """Preserve input hashes and source hashes in a fresh, independently manifested directory."""
    Solver, Config, _, _ = base._core()
    output = base.validate_output_path(output_dir)
    if output.exists() and any(output.iterdir()):
        raise ValueError('output must be empty')
    if not isinstance(cycles, int) or not 1 <= cycles <= 10:
        raise ValueError('cycles must be between 1 and 10')
    if input_jsonl:
        cases, selection = replay.load_replay_cases(input_jsonl, max_records)
        requests = [(str(case['line_number']), *restore_record(case['record'])) for case in cases]
    else:
        selection = {'scenarios': SCENARIOS, 'cycles': cycles}
        requests = [(name + '/' + str(cycle), scenario(name, cycle),
                     Solver(Config(allow_synthetic_predictions=True)))
                    for cycle in range(cycles) for name in SCENARIOS]
    if not requests:
        raise ValueError('no comparison inputs')
    provenance = base._provenance(requests[0][2].config, requests[0][2])
    sources = [Path(__file__), *base.WORKSPACE.glob(
        'src/uav_control/uav_control/guidance/*follow*.py'),
        base.WORKSPACE / 'src/uav_control/uav_control/guidance/yaw_trajectory.py',
        base.WORKSPACE / 'src/uav_control/uav_control/guidance/minco_trajectory.py']
    for path in sources:
        provenance['source_sha256'][str(path.relative_to(base.WORKSPACE))] = hashlib.sha256(
            path.read_bytes()).hexdigest()
    provenance.update(selection=selection, common_boundary_policy='rear reference endpoint P/V/A',
                      mpc_adapter='intermediate positions and yaw only; common endpoint retained',
                      initialization_reused_across_ablations=True,
                      continuous_time_guarantee=False, variants=VARIANTS)
    if input_jsonl:
        provenance.update(
                          input_jsonl=str(Path(input_jsonl).resolve()),
                          input_sha256=hashlib.sha256(Path(input_jsonl).read_bytes()).hexdigest(),
                          prediction_provenance='recorded tracking; immutable acquisition epochs',
                          rolling_state_source='recorded PVA and explicit future projection')
    output.mkdir(parents=True, exist_ok=True)
    base.write_json(output / 'configuration.json', provenance)
    records = []
    with (output / 'records.jsonl').open('x') as stream:
        for name, request, model in requests:
            for record in compare(request, model):
                record['case'] = name
                stream.write(json.dumps(base.json_safe(record), allow_nan=False) + '\n')
                records.append(record)
    result = {'request_count': len(requests), 'groups': summary(records),
              'accepted_by_tracker': False, 'continuous_time_guarantee': False}
    base.write_json(output / 'summary.json', result)
    base.write_manifest(output, ['configuration.json', 'records.jsonl', 'summary.json'],
                        provenance)
    return result


def main():
    """Compare synthetic or actual recorded inputs, without ROS or simulator startup."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--input-jsonl', type=Path)
    parser.add_argument('--max-records', type=int, default=20)
    args = parser.parse_args()
    result = run(args.output_dir, args.cycles, args.input_jsonl, args.max_records)
    print(json.dumps({'requests': result['request_count'], 'groups': {
        name: {k: group[k] for k in ('count', 'sampled_feasible', 'pipeline_feasible', 'timeout')}
        for name, group in result['groups'].items()}}, indent=2))


if __name__ == '__main__':
    main()
