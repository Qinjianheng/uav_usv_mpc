#!/usr/bin/env python3
# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""Synthetic, transport-free MPC seed comparisons; never a flight acceptance.

Each method receives the same immutable request. Baseline acceleration samples
are converted to interval jerk and integrated by the core's real validator.
Rolling inputs are analytical synthetic navigation samples, not executed seeds.
No ROS, truth subscriber, PX4, network, or simulator is used by this tool.
"""

import argparse
from dataclasses import asdict, is_dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation


WORKSPACE = Path(__file__).resolve().parents[1]
SCENARIOS = (
    'stationary', 'constant_velocity', 'left_turn', 'right_turn', 'figure_eight',
    'speed_jump', 'turn_jump', 'wrong_yaw', 'fov_edge', 'recoverable_lost_view',
    'too_close', 'too_far', 'dynamics_infeasible', 'expired_prediction',
    'short_prediction', 'stale_observation', 'nonmonotonic_prediction', 'wrong_frame',
)


def percentiles(values):
    """Keep invalid counts visible; use NumPy's linear empirical quantiles."""
    array = np.asarray(values, dtype=float)
    finite = array[np.isfinite(array)]
    result = {'count': int(array.size), 'finite_count': int(finite.size),
              'p50': None, 'p95': None, 'p99': None, 'minimum': None, 'maximum': None}
    if finite.size:
        result.update(zip(('p50', 'p95', 'p99', 'minimum', 'maximum'),
                          map(float, np.percentile(finite, [50, 95, 99, 0, 100]))))
    return result


def json_safe(value):
    """Serialize complete dataclasses, preserving nonfinite diagnostics explicitly."""
    if is_dataclass(value):
        return json_safe(asdict(value))
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {'__nonfinite_float__': str(value)}
    return value


def write_json(path, value):
    """Write strict JSON; never rely on nonstandard NaN JSON tokens."""
    encoded = json.dumps(json_safe(value), indent=2, sort_keys=True, allow_nan=False)
    Path(path).write_text(encoded + '\n')


def fingerprint(value):
    data = json.dumps(json_safe(value), sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode()
    return hashlib.sha256(data).hexdigest()


def write_manifest(output_dir, paths, provenance):
    """Hash final artifacts, excluding the self-referential manifest itself."""
    output_dir = Path(output_dir)
    files = []
    for relative in sorted(paths):
        path = output_dir / relative
        files.append({'path': str(relative), 'bytes': path.stat().st_size,
                      'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    write_json(output_dir / 'manifest.json', {'provenance': provenance, 'files': files})


def _target_motion(name, times):
    """Analytical targets, including declared discontinuities; these are not BCTRA."""
    times = np.asarray(times)
    positions = np.zeros((len(times), 3))
    velocities = np.zeros_like(positions)
    if name == 'stationary':
        return positions, velocities
    positions[:, 0] = times
    velocities[:, 0] = 1.
    if name in ('left_turn', 'right_turn'):
        sign = -1. if name == 'left_turn' else 1.
        omega = .25
        positions[:, 0] = np.sin(omega * times) / omega
        positions[:, 1] = sign * (1. - np.cos(omega * times)) / omega
        velocities[:, 0] = np.cos(omega * times)
        velocities[:, 1] = sign * np.sin(omega * times)
    elif name == 'figure_eight':
        positions[:, 0] = 2. * np.sin(.3 * times)
        positions[:, 1] = np.sin(.6 * times)
        velocities[:, 0] = .6 * np.cos(.3 * times)
        velocities[:, 1] = .6 * np.cos(.6 * times)
    elif name == 'speed_jump':
        positions[:, 0] = times + 2. * np.maximum(times - 1., 0.)
        velocities[:, 0] = np.where(times < 1., 1., 3.)
    elif name == 'turn_jump':
        positions[:, 0] = np.minimum(times, 1.)
        positions[:, 1] = np.maximum(times - 1., 0.)
        velocities[:, 0] = np.where(times < 1., 1., 0.)
        velocities[:, 1] = np.where(times < 1., 0., 1.)
    return positions, velocities


def build_scenario(name, cycle=0, dt=.2, prediction_duration=4.):
    """Build a deterministic same-epoch synthetic request; retain invalid cases."""
    if name not in SCENARIOS:
        raise ValueError('unknown scenario: ' + str(name))
    if cycle < 0 or dt <= 0 or prediction_duration <= 0:
        raise ValueError('cycle/dt/prediction duration out of range')
    elapsed = cycle * dt
    stamp = 100. + elapsed
    times = np.arange(0., prediction_duration + dt / 2., dt)
    positions, velocities = _target_motion(name, elapsed + times)
    origin_positions, origin_velocities = _target_motion(name, np.array([0.]))
    # Analytical navigation moves at its own constant initial velocity;
    # target curvature cannot silently rewrite the synthetic vehicle's P/V/A.
    velocity = origin_velocities[0]
    yaw = math.atan2(velocity[1], velocity[0]) if np.linalg.norm(velocity[:2]) > 0 else 0.
    direction = np.array((math.cos(yaw), math.sin(yaw), 0.))
    initial_position = origin_positions[0] - 5. * direction + velocity * elapsed
    initial_position[2] = -5.
    initial_velocity = velocity.copy()
    if name == 'wrong_yaw':
        yaw += math.pi / 2.
    elif name == 'fov_edge':
        yaw += .8
    elif name == 'recoverable_lost_view':
        yaw += 1.1
    elif name == 'too_close':
        initial_position = positions[0] - .1 * direction
        initial_position[2] = -.1
    elif name == 'too_far':
        initial_position = positions[0] - 35. * direction
        initial_position[2] = -5.
    elif name == 'dynamics_infeasible':
        initial_velocity[0] = 20.
    context = {
        'mission_id': 1, 'cycle_id': cycle, 'execution_start_stamp': stamp,
        'navigation_stamp': stamp, 'attitude_stamp': stamp,
        'prediction_source_stamp': stamp, 'observation_stamp': stamp - .01,
        'prediction_sequence_id': cycle + 1, 'prediction_valid_until': stamp + times[-1],
        'clock_generation': 0, 'frame_id': 'local_ned', 'prediction_source': 'synthetic',
    }
    if name == 'expired_prediction':
        context['prediction_valid_until'] = stamp - .1
    elif name == 'short_prediction':
        times, positions, velocities = times[:3], positions[:3], velocities[:3]
        context['prediction_valid_until'] = stamp + times[-1]
    elif name == 'stale_observation':
        context['observation_stamp'] = stamp - 2.
    elif name == 'nonmonotonic_prediction':
        times[2] = times[1]
    elif name == 'wrong_frame':
        context['frame_id'] = 'local_enu'
    state = np.r_[initial_position, initial_velocity, np.zeros(3), yaw]
    return {'context': context, 'state': state, 'prediction_times': times,
            'target_positions': positions, 'target_velocities': velocities,
            'actual_rotation': Rotation.from_euler('z', yaw).as_matrix(), 'now_stamp': stamp}


def _viewpoints(request, config, relative_times):
    query = (request.context.execution_start_stamp - request.context.prediction_source_stamp
             + np.asarray(relative_times))
    times = np.asarray(request.prediction_times)
    target_positions = np.asarray(request.target_positions)
    target_velocities = np.asarray(request.target_velocities)
    positions = np.column_stack([np.interp(query, times, target_positions[:, i])
                                 for i in range(3)])
    velocities = np.column_stack([np.interp(query, times, target_velocities[:, i])
                                  for i in range(3)])
    speeds = np.linalg.norm(velocities[:, :2], axis=1)
    yaw = np.where(speeds > 1e-8, np.arctan2(velocities[:, 1], velocities[:, 0]),
                   float(request.state[9]))
    direction = np.column_stack((np.cos(yaw), np.sin(yaw), np.zeros(len(yaw))))
    viewpoints = positions - config.follow_distance * direction
    viewpoints[:, 2] = config.flight_altitude
    # np.interp clamps only baseline construction; the common core separately
    # rejects insufficient coverage and every method preserves the original request.
    return viewpoints, velocities, np.unwrap(yaw)


def baseline_controls(request, config, kind):
    """Derive interval controls; core integrates them with the original P/V/A."""
    times = np.arange(config.horizon_steps + 1) * config.dt
    views, target_velocity, yaw = _viewpoints(request, config, times)
    state = np.asarray(request.state, dtype=float)
    if kind == 'quintic':
        duration = times[-1]
        c0, c1, c2 = state[:3], state[3:6], state[6:9] / 2.
        endpoint_velocity = target_velocity[-1].copy()
        endpoint_velocity[2] = 0.
        rhs = np.vstack((views[-1] - c0 - c1 * duration - c2 * duration**2,
                         endpoint_velocity - c1 - 2. * c2 * duration, -2. * c2))
        matrix = np.array(((duration**3, duration**4, duration**5),
                           (3. * duration**2, 4. * duration**3, 5. * duration**4),
                           (6. * duration, 12. * duration**2, 20. * duration**3)))
        c3, c4, c5 = np.linalg.solve(matrix, rhs)
        acceleration = (2. * c2 + 6. * times[:, None] * c3
                        + 12. * times[:, None]**2 * c4 + 20. * times[:, None]**3 * c5)
    elif kind == 'viewpoint_interpolation':
        # Use numerical P/V/A from target viewpoints, then honestly integrate
        # the derived jerk instead of presenting interpolated P as feasible.
        edge_order = min(2, len(views) - 1)
        velocity = np.gradient(views, config.dt, axis=0, edge_order=edge_order)
        acceleration = np.gradient(velocity, config.dt, axis=0, edge_order=edge_order)
    else:
        raise ValueError('unknown baseline: ' + str(kind))
    acceleration[0] = state[6:9]
    yaw = yaw + 2. * math.pi * round((state[9] - yaw[0]) / (2. * math.pi))
    yaw[0] = state[9]
    return np.column_stack((np.diff(acceleration, axis=0) / config.dt,
                            np.diff(yaw) / config.dt))


def output_metrics(request, output, config):
    """Measure integrated trajectories; prefer core's denser safety evaluations."""
    times = np.asarray(output.relative_times, dtype=float)
    result = {'visibility_ratio': None, 'minimum_horizontal_margin_rad': None,
              'minimum_vertical_margin_rad': None, 'follow_error_rms_m': None,
              'maximum_speed_mps': None, 'maximum_acceleration_mps2': None,
              'maximum_jerk_mps3': None, 'maximum_tilt_rad': None}
    if times.size:
        desired, _, _ = _viewpoints(request, config, times)
        positions = np.asarray(output.positions, dtype=float)
        squared_errors = np.sum((positions - desired)**2, axis=1)
        result['follow_error_rms_m'] = float(np.sqrt(np.mean(squared_errors)))
        for field, key in (('velocities', 'maximum_speed_mps'),
                           ('accelerations', 'maximum_acceleration_mps2'),
                           ('jerks', 'maximum_jerk_mps3')):
            array = np.asarray(getattr(output, field), dtype=float)
            if array.size:
                result[key] = float(np.max(np.linalg.norm(array, axis=1)))
        margins = np.asarray(output.visibility_margins, dtype=float)
        if margins.size:
            result['visibility_ratio'] = float(np.mean(np.all(np.isfinite(margins), axis=1)
                                                       & np.all(margins >= 0., axis=1)))
            for i, key in enumerate(('minimum_horizontal_margin_rad',
                                     'minimum_vertical_margin_rad')):
                if np.any(np.isfinite(margins[:, i])):
                    result[key] = float(np.nanmin(margins[:, i]))
    dense = getattr(output, 'metrics', {})
    for source, destination in (
        ('visible_fraction', 'visibility_ratio'),
        ('minimum_horizontal_margin', 'minimum_horizontal_margin_rad'),
        ('minimum_vertical_margin', 'minimum_vertical_margin_rad'),
        ('follow_rmse', 'follow_error_rms_m'), ('maximum_tilt_rad', 'maximum_tilt_rad'),
    ):
        if source in dense:
            result[destination] = dense[source]
    for key in ('dynamics_residual', 'maximum_horizontal_speed', 'maximum_vertical_speed',
                'maximum_horizontal_acceleration', 'maximum_vertical_acceleration',
                'maximum_horizontal_jerk', 'maximum_vertical_jerk', 'maximum_yaw_rate'):
        if key in dense:
            result[key] = dense[key]
    result['core_dense_metrics'] = dense
    return result


def summarize(records):
    """Keep methods separate, including same-input paired warm/cold runs."""
    result = {}
    for method in sorted({record['method'] for record in records}):
        selected = [record for record in records if record['method'] == method]
        count = len(selected)
        valid = sum(bool(record['output']['valid']) for record in selected)
        timeout = sum(any(token in str(record['output'].get('solver_status', ''))
                          or token in str(record['output']['reason'])
                          for token in ('DEADLINE', 'TIMEOUT')) for record in selected)
        group = {'count': count, 'feasible_rate': valid / count, 'failure_rate': 1.-valid/count,
                 'timeout_rate': timeout / count}
        budgeted = [record for record in selected if 'external_budget_s' in record['timing']]
        if budgeted:
            late = sum(record['timing']['end_to_end_s'] > record['timing']['external_budget_s']
                       for record in budgeted)
            admitted = sum(record['output']['valid'] and record['timing']['end_to_end_s']
                           <= record['timing']['external_budget_s'] for record in budgeted)
            group['pipeline_budget_sample_count'] = len(budgeted)
            group['pipeline_budget_exceeded_rate'] = late / len(budgeted)
            group['pipeline_feasible_rate'] = admitted / len(budgeted)
        for key in ('solve_external_s', 'end_to_end_s'):
            group[key] = percentiles([record['timing'][key] for record in selected])
        metric_keys = sorted({key for record in selected for key in record['metrics']
                              if key != 'core_dense_metrics'})
        for key in metric_keys:
            group[key] = percentiles([record['metrics'].get(key) if record['metrics'].get(key)
                                      is not None else math.nan for record in selected])
        reasons = {}
        for record in selected:
            reason = str(record['output']['reason'])
            reasons[reason] = reasons.get(reason, 0) + 1
        group['reason_counts'] = reasons
        result[method] = group
    return result


def _core():
    package_path = str(WORKSPACE / 'src/uav_control')
    if package_path not in sys.path:
        sys.path.insert(0, package_path)
    from uav_control.controllers.follow_mpc_seed import (
        FollowMpcSeed, MpcConfig, MpcRequest, PlanningContext,
    )
    return FollowMpcSeed, MpcConfig, MpcRequest, PlanningContext


def _provenance(config, solver):
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=WORKSPACE, check=True,
                          capture_output=True, text=True).stdout.strip()
    status = subprocess.run(['git', 'status', '--short'], cwd=WORKSPACE, check=True,
                            capture_output=True, text=True).stdout
    sources = {}
    for relative in ('scripts/mpc_seed_experiment.py',
                     'src/uav_control/uav_control/controllers/follow_mpc_seed.py',
                     'src/uav_control/uav_control/guidance/planned_attitude.py',
                     'src/uav_control/uav_control/guidance/camera_visibility.py'):
        path = WORKSPACE / relative
        if path.exists():
            sources[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    calibration = {key: json_safe(getattr(solver, key)) for key in (
        'intrinsics', 'extrinsics', 'target', 'visibility', 'attitude_config')}
    return {'git_head': head, 'git_status': status, 'workspace': str(WORKSPACE),
            'source_sha256': sources, 'config': json_safe(config),
            'geometry_and_attitude_config': calibration,
            'prediction_provenance': 'synthetic analytical targets; no BCTRA measurement',
            'acceptance_boundary': 'research seed only; never accepted by flight tracker',
            'rolling_state_source': 'analytical synthetic navigation; seeds are not executed',
            'timing_boundary': 'data prep through first full record JSON output; timing metadata '
            'and final summaries/manifest are excluded from per-candidate end_to_end_s'}


def validate_output_path(output_dir):
    """Reject output under the read-only old workspace and frozen evidence."""
    path = Path(output_dir).resolve()
    for protected in (WORKSPACE.parent / 'uav_usv', WORKSPACE.parent / 'uav_usv_mpc_archive',
                      WORKSPACE / 'data/baselines/20261006_pre_mpc'):
        if path == protected or protected in path.parents:
            raise ValueError('protected output directory: ' + str(path))
    return path


def run_experiment(output_dir, repeats=3, rolling_cycles=3, scenarios=SCENARIOS, config=None):
    """Execute deterministic comparisons with fresh warm solver per scenario/repeat."""
    if repeats <= 0 or rolling_cycles <= 0:
        raise ValueError('repeats and rolling cycles must be positive')
    FollowMpcSeed, MpcConfig, MpcRequest, PlanningContext = _core()
    config = config or MpcConfig(allow_synthetic_predictions=True)
    if not config.allow_synthetic_predictions:
        raise ValueError('synthetic experiments require allow_synthetic_predictions=True')
    output_dir = validate_output_path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    if any(output_dir.iterdir()):
        raise ValueError('output directory must be empty; select a fresh output directory')
    provenance = _provenance(config, FollowMpcSeed(config=config))
    write_json(output_dir / 'configuration.json', provenance)
    files = ['configuration.json']
    records = []
    overall_start = time.perf_counter()
    for name in scenarios:
        for repeat in range(repeats):
            warm_solver = FollowMpcSeed(config=config)
            for cycle in range(rolling_cycles):
                shared_start = time.perf_counter()
                scene = build_scenario(name, cycle, config.dt)
                payload = {**scene, 'context': PlanningContext(**scene['context'])}
                request = MpcRequest(**payload)
                input_sha = fingerprint(request)
                data_prep = time.perf_counter() - shared_start
                # Warm and cold receive exactly the same request in this cycle.
                for method in ('quintic', 'viewpoint_interpolation', 'mpc_warm', 'mpc_cold'):
                    start = time.perf_counter()
                    controls = None
                    solver = warm_solver if method == 'mpc_warm' else FollowMpcSeed(config=config)
                    if method in ('quintic', 'viewpoint_interpolation'):
                        controls = baseline_controls(request, config, method)
                    solve_start = time.perf_counter()
                    if controls is None:
                        output = solver.solve(request)
                    else:
                        output = solver.evaluate_controls(request, controls, candidate_kind=method)
                    solve_external = time.perf_counter() - solve_start
                    if fingerprint(request) != input_sha:
                        raise RuntimeError('solver mutated shared request')
                    record = {
                        'scenario': name, 'repeat': repeat, 'cycle': cycle, 'method': method,
                        'input_sha256': input_sha, 'input': json_safe(request),
                        'baseline_controls': json_safe(controls), 'output': json_safe(output),
                        'metrics': output_metrics(request, output, config),
                        'timing': {'shared_data_prep_s': data_prep,
                                   'external_budget_s': config.solve_budget,
                                   'solve_external_s': solve_external,
                                   'core_timing': json_safe(output.timing)},
                    }
                    relative = f'{name}_r{repeat:03d}_c{cycle:03d}_{method}.json'
                    write_json(output_dir / relative, record)
                    record['timing']['end_to_end_s'] = data_prep + time.perf_counter() - start
                    # Add timing after a complete first output, rather than falsely
                    # labelling core optimization time as the full processing budget.
                    write_json(output_dir / relative, record)
                    files.append(relative)
                    records.append(record)
    summary = {'methods': summarize(records), 'scenario_methods': {},
               'repeats': repeats, 'rolling_cycles': rolling_cycles, 'case_count': len(records),
               'total_wall_s_before_summary': time.perf_counter() - overall_start}
    for name in scenarios:
        selected = [r for r in records if r['scenario'] == name]
        summary['scenario_methods'][name] = summarize(selected)
    paired = {}
    for record in records:
        if record['method'] in ('mpc_warm', 'mpc_cold'):
            key = (record['scenario'], record['repeat'], record['cycle'])
            paired.setdefault(key, {})[record['method']] = record
    summary['paired_warm_minus_cold_s'] = percentiles([
        pair['mpc_warm']['timing']['solve_external_s']
        - pair['mpc_cold']['timing']['solve_external_s'] for pair in paired.values()])
    summary['same_input_pair_count'] = len(paired)
    summary['paired_warm_started_count'] = sum(
        pair['mpc_warm']['output']['warm_started'] for pair in paired.values())
    write_json(output_dir / 'summary.json', summary)
    files.append('summary.json')
    write_manifest(output_dir, files, provenance)
    return summary


def parse_arguments(argv=None):
    """Expose experiment options with defaults drawn from the core config."""
    _, MpcConfig, _, _ = _core()
    defaults = MpcConfig()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--rolling-cycles', type=int, default=3)
    parser.add_argument('--scenarios', nargs='+', choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument('--solve-budget', type=float, default=defaults.solve_budget)
    parser.add_argument('--maximum-iterations', type=int, default=defaults.maximum_iterations)
    parser.add_argument('--control-blocks', type=int, default=defaults.control_blocks)
    return parser.parse_args(argv)


def configuration_from_args(args):
    """Apply CLI values without modifying core defaults or the online config."""
    _, MpcConfig, _, _ = _core()
    return replace(MpcConfig(allow_synthetic_predictions=True), control_blocks=args.control_blocks,
                   solve_budget=args.solve_budget, maximum_iterations=args.maximum_iterations)


def main(argv=None):
    args = parse_arguments(argv)
    config = configuration_from_args(args)
    summary = run_experiment(args.output_dir, args.repeats, args.rolling_cycles,
                             tuple(args.scenarios), config)
    compact = {method: {key: values[key] for key in (
        'count', 'feasible_rate', 'timeout_rate', 'pipeline_budget_exceeded_rate',
        'pipeline_feasible_rate', 'solve_external_s', 'end_to_end_s')}
        for method, values in summary['methods'].items()}
    print(json.dumps({'output_dir': str(args.output_dir.resolve()),
                      'case_count': summary['case_count'], 'methods': compact},
                     indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
