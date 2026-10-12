#!/usr/bin/env python3
"""Same-request F2/F3 scalar-rollout equivalence and 5/10/20 ms numerical study.

No controller, future measurement or truth source is substituted. Strict seed
admission is offline and excludes optimizer/fallback time. The chosen baseline
remains 10 ms even when a coarser numerical step happens to be feasible.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE/'src/uav_control'))
sys.path.insert(0, str(WORKSPACE/'scripts'))

from p46_follow_study import read_scenes, distribution, safe, write_json  # noqa: E402
from uav_control.controllers.direct_reference_minco import (  # noqa: E402
    follow_guided_seed, reference_fit,
)
from uav_control.guidance.follow_fast_validation import validate_seed  # noqa: E402
from uav_control.guidance.follow_limits import constraint_snapshot  # noqa: E402
from uav_control.guidance.follow_problem import FollowProblem  # noqa: E402
from uav_control.guidance.follow_rollout import (  # noqa: E402
    FollowRolloutConfig, rollout_follow, rollout_follow_fast,
)
from uav_control.guidance.p44_invariant_cache import P44InvariantCache  # noqa: E402


METHODS = dict(numpy_10ms=(rollout_follow, .01), scalar_10ms=(rollout_follow_fast, .01),
               scalar_5ms=(rollout_follow_fast, .005), scalar_20ms=(rollout_follow_fast, .02))


def digest(path):
    """Hash the exact bytes used as evidence or source."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reference_samples(reference, query):
    """Batch the existing piecewise constant-jerk interface, with its exact support check."""
    query = np.asarray(query)
    if (not np.all(np.isfinite(query)) or np.any(query < 0)
            or np.any(query > reference.times[-1]+1e-9)):
        raise ValueError('FOLLOW_REFERENCE_OUTSIDE')
    ts = np.asarray(reference.times)
    index = np.minimum(np.searchsorted(ts, query, side='right')-1, len(ts)-2)
    dt = query-ts[index]
    p, v, a = [np.asarray(field)[index] for field in (
        reference.positions, reference.velocities, reference.accelerations)]
    jerk = (np.asarray(reference.accelerations)[index+1]-a)/(ts[index+1]-ts[index])[:, None]
    return (p+v*dt[:, None]+a*dt[:, None]**2/2+jerk*dt[:, None]**3/6,
            v+a*dt[:, None]+jerk*dt[:, None]**2/2, a+jerk*dt[:, None])


def compare(reference, baseline, horizon):
    """Compare all P/V/A on a common 5 ms grid and commands on original 20 Hz ticks."""
    query = np.linspace(0., horizon, int(round(horizon/.005))+1)
    actual, original = reference_samples(reference, query), reference_samples(baseline, query)
    report = {}
    for name, a, b in zip(('position', 'velocity', 'acceleration'), actual, original):
        error = a-b
        report[name+'_max_abs'] = float(np.max(np.abs(error)))
        report[name+'_rmse'] = float(np.sqrt(np.mean(np.sum(error*error, axis=1))))
        report[name+'_z_rmse'] = float(np.sqrt(np.mean(error[:, 2]**2)))
    commands = np.asarray(reference.commands)-np.asarray(baseline.commands)
    report['command_max_abs'] = float(np.max(np.abs(commands)))
    report['command_rmse'] = float(np.sqrt(np.mean(np.sum(commands*commands, axis=1))))
    return report


def run(output, limit=None):
    """Use all immutable P46 same-input requests; warmup and fit are outside rollout timers."""
    if limit is not None and limit < 1:
        raise ValueError('INVALID_LIMIT')
    scenes, inputs = read_scenes(limit)
    output.mkdir(parents=True, exist_ok=False)
    source_paths = [WORKSPACE/name for name in (
        'scripts/p47_rollout_performance.py', 'scripts/p46_follow_study.py',
        'src/uav_control/uav_control/guidance/follow_rollout.py',
        'src/uav_control/uav_control/control/flight_guidance.py',
        'src/uav_control/uav_control/common/sea_safety.py',
        'src/uav_control/uav_control/guidance/follow_problem.py',
        'src/uav_control/uav_control/controllers/direct_reference_minco.py',
        'src/uav_control/uav_control/guidance/follow_fast_validation.py',
        'src/uav_control/uav_control/guidance/follow_minco_optimizer.py',
        'src/uav_control/uav_control/guidance/p44_invariant_cache.py',
        'src/uav_control/test/test_p47_follow_rollout_fast.py')]
    sources = {str(p.relative_to(WORKSPACE)): digest(p) for p in source_paths}
    rollout_path = 'src/uav_control/uav_control/guidance/follow_rollout.py'
    original = subprocess.check_output(['git', 'show', 'HEAD:'+rollout_path], cwd=WORKSPACE)
    old_bytes_preserved = (WORKSPACE/rollout_path).read_bytes().startswith(original)
    if not old_bytes_preserved:
        raise ValueError('ORIGINAL_ROLLOUT_MODULE_PREFIX_CHANGED')
    manifest = dict(
        inputs=inputs, source_sha256=sources, original_module_prefix_preserved=True,
        original_module_sha256=hashlib.sha256(original).hexdigest(),
        selected_step_seconds=.01, original_control_period_seconds=.05,
        default_original_function_unchanged=True, no_sitl=True, quick_run=limit is not None,
        request_policy='all original F2/F3 single-input requests; same PVA/start/constraints',
        initial_command_policy='bounded execution velocity, same for each method; no emitted '
        'command was recorded in these P32 inputs',
        strict_validation='same follow_guided_seed + validate_seed with a fresh '
        'P44InvariantCache for each method; frozen clock budget for numerical comparison',
        timing='rollout only; rotating method order, one untimed warmup each; '
        'seed construction/strict validation/fit excluded from rollout timer',
        methods={k: asdict(FollowRolloutConfig(integration_dt=step))
                 for k, (_, step) in METHODS.items()})
    write_json(output/'manifest.json', manifest)
    all_rows = []
    with (output/'per_request.jsonl').open('x') as stream:
        for scene, subsets in scenes.items():
            for horizon in (1.2, 1.6):
                cases = subsets['single']
                warm = FollowProblem(cases[0][3], cases[0][4], horizon)
                for function, step in METHODS.values():
                    function(warm, FollowRolloutConfig(integration_dt=step))
                for index, (path, line, sha, request, model) in enumerate(cases):
                    problem = FollowProblem(request, model, horizon)
                    references, elapsed = {}, {}
                    names = list(METHODS)
                    order = names[index % len(names):]+names[:index % len(names)]
                    for name in order:
                        function, step = METHODS[name]
                        started = time.perf_counter()
                        references[name] = function(
                            problem, FollowRolloutConfig(integration_dt=step))
                        elapsed[name] = time.perf_counter()-started
                    outcomes = {}
                    fits = {}
                    for name in names:
                        seed = follow_guided_seed(request, problem, references[name])
                        outcomes[name] = validate_seed(request, seed, model, P44InvariantCache(),
                                                       budget=1., clock=lambda: 0.)
                        fits[name] = reference_fit(seed, references[name])
                    baseline = references['numpy_10ms']
                    for name in names:
                        result = outcomes[name]
                        equivalent = compare(references[name], baseline, horizon)
                        row = dict(
                            scene=scene, horizon=horizon, method=name, source_path=path,
                            source_line=line, source_line_sha256=sha,
                            execution_start=request.context.execution_start_stamp,
                            constraint_fingerprint=constraint_snapshot(model).fingerprint,
                            rollout_seconds=elapsed[name],
                            samples=len(references[name].times),
                            commands=len(references[name].commands),
                            differences_from_original_10ms=equivalent,
                            strict_seed_valid=result.valid,
                            strict_seed_status=result.solver_status,
                            strict_status_matches_original=(
                                result.valid == outcomes['numpy_10ms'].valid and
                                result.solver_status == outcomes['numpy_10ms'].solver_status),
                            fit=fits[name],
                            reference_height_rmse=float(np.sqrt(np.mean(
                                (np.asarray(references[name].positions)[:, 2]
                                 - model.config.flight_altitude)**2))),
                            reference_metrics=references[name].metrics)
                        all_rows.append(row)
                        stream.write(json.dumps(safe(row), allow_nan=False)+'\n')
                    if index % 100 == 0:
                        print(scene, horizon, index, 'of', len(cases), flush=True)
    summary = dict(selected_step_seconds=.01, quick_run=limit is not None,
                   total_same_request_horizon_cases=len(all_rows)//len(METHODS), groups={})
    for scene in scenes:
        for horizon in (1.2, 1.6):
            for name in METHODS:
                rows = [r for r in all_rows if (r['scene'], r['horizon'], r['method'])
                        == (scene, horizon, name)]
                keys = rows[0]['differences_from_original_10ms']
                summary['groups'][f'{scene}/h{horizon}/{name}'] = dict(
                    count=len(rows), rollout_seconds=distribution(
                        [r['rollout_seconds'] for r in rows]),
                    strict_seed_valid=sum(r['strict_seed_valid'] for r in rows),
                    strict_status_counts=dict(Counter(r['strict_seed_status'] for r in rows)),
                    strict_status_mismatches=sum(
                        not r['strict_status_matches_original'] for r in rows),
                    max_difference={k: max(r['differences_from_original_10ms'][k] for r in rows)
                                    for k in keys},
                    fit_distributions={k: distribution([r['fit'][k] for r in rows])
                                       for k in rows[0]['fit']})
    source_changes = [name for name, sha in sources.items() if digest(WORKSPACE/name) != sha]
    summary['source_changed_during_analysis'] = source_changes
    summary['scalar_10ms_equivalence_passed'] = all(
        r['strict_status_matches_original'] and all(
            v < 3e-11 for v in r['differences_from_original_10ms'].values())
        for r in all_rows if r['method'] == 'scalar_10ms')
    write_json(output/'summary.json', summary)
    plot(output, summary)
    write_json(output/'source_after.json', {
        str(p.relative_to(WORKSPACE)): digest(p) for p in source_paths})
    (output/'SHA256SUMS').write_text(''.join(
        f'{digest(p)}  {p.name}\n' for p in sorted(output.iterdir()) if p.is_file()))
    if not summary['scalar_10ms_equivalence_passed']:
        raise RuntimeError('SCALAR_10MS_EQUIVALENCE_FAILED')
    print(json.dumps(dict(cases=summary['total_same_request_horizon_cases'],
                          scalar_10ms_equivalence_passed=True,
                          source_changed_during_analysis=source_changes)))
    return summary


def plot(output, summary):
    """Separate actual measured rollout cost from numerical approximation error."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for i, (scene, horizon) in enumerate((('f2', 1.2), ('f2', 1.6), ('f3', 1.2), ('f3', 1.6))):
        names = list(METHODS)
        selected = [summary['groups'][f'{scene}/h{horizon}/{name}'] for name in names]
        axes[0].plot(range(4), [r['rollout_seconds']['p50']*1000 for r in selected],
                     marker='o', label=f'{scene} H={horizon}')
        axes[1].plot(range(4), [r['max_difference']['position_max_abs'] for r in selected],
                     marker='o', label=f'{scene} H={horizon}')
    for ax in axes:
        ax.set_xticks(range(4), ['numpy10', 'scalar10', 'scalar5', 'scalar20'])
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    axes[0].set_ylabel('Rollout p50 (ms); original control law at 20 Hz')
    axes[1].set_ylabel('Worst |P - original 10 ms P| (m), common 5 ms grid')
    fig.suptitle('Same input / start / limits; chosen integration step remains 10 ms')
    fig.tight_layout()
    fig.savefig(output/'rollout_cost_and_numerics.png', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=WORKSPACE/(
        'data/experiments/20261010_p47_follow_guided/rollout_performance/full_matrix_01'))
    parser.add_argument('--limit', type=int,
                        help='explicit quick run only; final matrix has no limit')
    args = parser.parse_args()
    run(args.output.resolve(), args.limit)


if __name__ == '__main__':
    main()
