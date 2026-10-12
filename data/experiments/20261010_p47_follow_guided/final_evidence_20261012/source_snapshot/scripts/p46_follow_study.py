#!/usr/bin/env python3
"""Frozen four-way MINCO research, with single continuous prediction-only ideal episodes."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from p4_follow_replay import load, to_curve
from p41_follow_replay import errors
from uav_control.controllers.direct_reference_minco import FollowGuidedMinco, follow_guided_seed
from uav_control.controllers.p44_follow_solver import P44FollowSolver
from uav_control.guidance.follow_fast_validation import fields
from uav_control.guidance.follow_limits import constraint_snapshot
from uav_control.guidance.follow_problem import FollowProblem, hypothetical_request
from uav_control.guidance.follow_rollout import FollowRolloutConfig, rollout_follow


WORKSPACE = Path(__file__).resolve().parents[1]
INPUT_ROOT = WORKSPACE/'data/experiments/20261009_p32_realtime_follow'
INITIAL_ROOT = WORKSPACE/'data/experiments/20261009_p43_follow_optimization/common_initial'
GROUPS = dict(A='p44_D', B='follow_guided_fixed', C='follow_guided_Q', D='follow_guided_QT')
METRICS = ('fixed', 'selected', 'distance', 'relative_velocity', 'height')
PHASES = ('initialization', 'coefficient_construction', 'optimization', 'validation',
          'fallback', 'total')


def safe(value):
    """Encode unavailable/nonfinite evidence as null, retaining full dataclass payloads."""
    if hasattr(value, '__dataclass_fields__'):
        return safe(asdict(value))
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, np.ndarray)):
        return [safe(v) for v in value]
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, value):
    """New files only; never amend a previous experiment or frozen source."""
    with path.open('x') as stream:
        json.dump(safe(value), stream, indent=2, allow_nan=False)
        stream.write('\n')


def distribution(values):
    """Absent timings are unknown, rather than measured zero-duration work."""
    values = [float(v) for v in values if v is not None and np.isfinite(v)]
    if not values:
        return None
    return dict(count=len(values), minimum=min(values), maximum=max(values),
                **dict(zip(('p50', 'p95', 'p99'), np.percentile(values, (50, 95, 99)))))


def make(model, group, horizon, now):
    """All groups share original constraints, frozen wall clock and disabled warm hints."""
    kwargs = dict(duration=horizon, rolling=False, wall_clock=now)
    if group == 'A':
        return P44FollowSolver(model, ablation='D', refinement='none', **kwargs)
    return FollowGuidedMinco(model, refinement=dict(B='none', C='q', D='qt')[group],
                             fallback_enabled=True, **kwargs)


def model_payload(model):
    """The complete source calibration accompanies the raw request, not just limits."""
    return {key: asdict(value) for key, value in (
        ('mpc', model.config), ('intrinsics', model.intrinsics), ('extrinsics', model.extrinsics),
        ('target', model.target), ('visibility', model.visibility),
        ('attitude', model.attitude_config))}


def sample_errors(raw, model, sample, beta, horizon):
    """Horizontal prediction-reference errors and an independent constant-altitude error."""
    result = errors(raw, model, sample, beta, horizon)
    result['height'] = float(sample[2]-model.config.flight_altitude)
    return result


def evaluation(request, model, result, horizon):
    """Post-timer fit of every final valid curve, including A and P44 fallback, to FOLLOW."""
    problem = FollowProblem(request, model, horizon)
    reference = rollout_follow(problem)
    seed = follow_guided_seed(request, problem, reference)
    report = dict(
        final_curve_to_follow_fit=None, reference_assumptions=reference.metrics,
        reference_config=asdict(FollowRolloutConfig()),
        reference=dict(times=reference.times, positions=reference.positions,
                       velocities=reference.velocities, accelerations=reference.accelerations,
                       commands=reference.commands), initial_seed=asdict(seed),
        seed_role='FOLLOW-guided initialization; evaluation-only reference for group A',
        analysis_outside_solver_timer=True)
    if not result.valid:
        return report
    metrics = result.metrics
    durations = metrics['durations']
    times = np.asarray(reference.times)
    if times[-1] > sum(durations)+1e-7:
        raise ValueError('FIT_HORIZON_MISMATCH')
    samples = fields(np.asarray(metrics['xyz_coefficients']), durations,
                     np.asarray(metrics['yaw_coefficients'])[::-1].T,
                     np.clip(times, 0., sum(durations)))
    report['final_curve_yaw'] = dict(
        initial=float(samples[4][0]), final=float(samples[4][-1]),
        initial_rate=float(samples[5][0]), maximum_absolute_rate=float(abs(samples[5]).max()))
    fit = {}
    for name, actual, wanted in zip(('position', 'velocity', 'acceleration'), samples[:3],
                                    (reference.positions, reference.velocities,
                                     reference.accelerations)):
        delta = actual-np.asarray(wanted)
        fit[name+'_rmse'] = float(np.sqrt(np.mean(np.sum(delta**2, axis=1))))
        fit[name+'_maximum'] = float(np.max(np.linalg.norm(delta, axis=1)))
        fit[name+'_vertical_rmse'] = float(np.sqrt(np.mean(delta[:, 2]**2)))
    fit['height_rmse'] = float(np.sqrt(np.mean(
        (samples[0][:, 2]-model.config.flight_altitude)**2)))
    report['final_curve_to_follow_fit'] = fit
    return report


def physical(result, model):
    """Record accepted sampled dynamics separately from the solver's continuous-bound metrics."""
    if not result.valid:
        return None
    v, a, j = map(np.asarray, (result.velocities, result.accelerations, result.jerks))
    forces = np.array((0., 0., model.attitude_config.gravity))-a
    maxima = dict(horizontal_speed=float(np.linalg.norm(v[:, :2], axis=1).max()),
                  vertical_speed=float(np.abs(v[:, 2]).max()),
                  horizontal_acceleration=float(np.linalg.norm(a[:, :2], axis=1).max()),
                  vertical_acceleration=float(np.abs(a[:, 2]).max()),
                  horizontal_jerk=float(np.linalg.norm(j[:, :2], axis=1).max()),
                  vertical_jerk=float(np.abs(j[:, 2]).max()),
                  yaw_rate=float(np.abs(result.yaw_rates).max()),
                  specific_thrust=float(np.linalg.norm(forces, axis=1).max()),
                  tilt=float(np.arctan2(
                      np.linalg.norm(forces[:, :2], axis=1), forces[:, 2]).max()))
    certified = {k: v for k, v in result.metrics.items()
                 if k.startswith(('maximum_', 'minimum_'))}
    return dict(sampled_maxima=maxima, solver_bound_metrics=certified,
                samples_are_continuous_bounds=False)


def handover(proposed, previous, epoch):
    """P/V/A and wrapped yaw/rate residuals at exactly the shared execution epoch."""
    if previous is None:
        return None
    delta = np.asarray(proposed.sample(epoch))-np.asarray(previous.sample(epoch))
    delta[9] = np.arctan2(np.sin(delta[9]), np.cos(delta[9]))
    return dict(position=float(np.linalg.norm(delta[:3])),
                velocity=float(np.linalg.norm(delta[3:6])),
                acceleration=float(np.linalg.norm(delta[6:9])), yaw=float(abs(delta[9])),
                yaw_rate=float(abs(delta[10])), maximum_absolute=float(abs(delta).max()))


def run_case(cases, path, group, horizon, rolling=False, factory=make):
    """Rejects keep the prior deadline; the first uncovered epoch ends this episode forever."""
    rows, previous, stopped, stop_epoch = [], None, None, None
    beta = 0.
    origin = cases[0][3].context.execution_start_stamp
    identity = (cases[0][3].context.mission_id, cases[0][3].context.clock_generation)
    last = -float('inf')
    with path.open('x') as stream:
        for source, line, sha, raw, model in cases:
            if rolling and raw.now_stamp-last < .2-1e-7:
                continue
            last = raw.now_stamp
            epoch = raw.context.execution_start_stamp
            if rolling and stopped is None:
                if (raw.context.mission_id, raw.context.clock_generation) != identity:
                    stopped, stop_epoch = 'EPOCH_CHANGED', min(
                        epoch, previous.end if previous else epoch)
                elif previous is not None and epoch > previous.end:
                    stopped, stop_epoch = 'CURVE_EXPIRED', previous.end
            snapshot = constraint_snapshot(model)
            row = dict(path=source, line=line, source_sha256=sha, group=group, horizon=horizon,
                       epoch=epoch, raw_request=safe(raw), model_config=model_payload(model),
                       constraint_fingerprint=snapshot.fingerprint,
                       request=None, result=None, seconds=None, evaluation=None,
                       analysis_seconds=None, physical=None, handover=None,
                       sampled_state=None, error=None, continuous_age=None,
                       stop_reason=stopped, stop_epoch=stop_epoch,
                       hypothetical_only=True, tracker_accepted=False)
            if not rolling or stopped is None:
                req = hypothetical_request(raw, previous, epoch) if previous and rolling else raw
                row['request'] = safe(req)
                # A fresh solver also keeps rolling cache order out of timing comparisons.
                instance = factory(model, group, horizon, lambda: raw.now_stamp)
                began = time.perf_counter()
                result = instance.solve(req)
                row['seconds'] = time.perf_counter()-began
                row['result'] = safe(result)
                analysis_started = time.perf_counter()
                try:
                    row['evaluation'] = evaluation(req, model, result, horizon)
                except (ValueError, TypeError, FloatingPointError, np.linalg.LinAlgError) as error:
                    row['evaluation'] = dict(unavailable_reason=str(error))
                row['physical'] = physical(result, model)
                if result.valid:
                    proposed = to_curve(req, result)
                    row['handover'] = handover(proposed, previous, epoch) if rolling else None
                    if row['handover'] and row['handover']['maximum_absolute'] > 1e-6:
                        raise ValueError('OWN_REFERENCE_DISCONTINUITY')
                    previous = proposed
                    beta = result.metrics.get('progress', {}).get('beta', 0.)
                if previous is not None and previous.start <= epoch <= previous.end:
                    sample = previous.sample(epoch)
                    row['sampled_state'] = safe(sample)
                    row['error'] = sample_errors(raw, model, sample, beta, horizon)
                    row['continuous_age'] = epoch-origin if rolling else 0.
                elif rolling:
                    stopped, stop_epoch = 'INITIAL_REQUEST_REJECTED', epoch
                    row.update(stop_reason=stopped, stop_epoch=stop_epoch)
                row['analysis_seconds'] = time.perf_counter()-analysis_started
                if not rolling:
                    previous = None
            if rolling:
                row['continuous_coverage_until'] = (
                    stop_epoch if stopped is not None else previous.end if previous else origin)
            stream.write(json.dumps(safe(row), allow_nan=False)+'\n')
            rows.append(safe(row))
    return rows


def rmse(rows):
    """One shared sample population; empty steady state is explicitly unavailable."""
    return {name: float(np.sqrt(np.mean([r['error'][name]**2 for r in rows])))
            if rows else None for name in METRICS}


def source_key(row):
    return row['source_sha256'], row['line']


def matched(groups):
    """Match immutable requests, epochs, and twenty continuous seconds in every group."""
    populations = {g: {source_key(r): r for r in rows} for g, rows in groups.items()}
    aligned = set.intersection(*(set(p) for p in populations.values()))
    common, steady = [], []
    for key in sorted(aligned):
        rows = [p[key] for p in populations.values()]
        if max(r['epoch'] for r in rows)-min(r['epoch'] for r in rows) > 1e-7:
            raise ValueError('MATCHED_EPOCH_MISMATCH')
        if all(r['error'] is not None and r['sampled_state'] is not None for r in rows):
            common.append(key)
            if all(r.get('continuous_age') is not None and r['continuous_age'] >= 20
                   for r in rows):
                steady.append(key)
    first = next(iter(populations.values()))
    epochs = [first[k]['epoch'] for k in common]
    return dict(count=len(common), steady_count=len(steady), aligned_count=len(aligned),
                coverage_fraction=len(common)/len(aligned) if aligned else None,
                start=min(epochs) if epochs else None, end=max(epochs) if epochs else None,
                selection='source line SHA intersection, same epoch, continuous episode age >=20',
                groups={g: dict(all=rmse([p[k] for k in common]),
                                after20=rmse([p[k] for k in steady]))
                        for g, p in populations.items()})


def summarize(rows, group, origin, end):
    """Final feasibility, independent method success and budgeted fallback remain separate."""
    attempts = [r for r in rows if r.get('result') is not None]
    detail = [r['result']['metrics'].get('follow_guided', {}) for r in attempts]
    valid = [r for r in attempts if r['result']['valid']]
    sampled = [r for r in rows if r['sampled_state'] is not None]
    stopped = next((r for r in rows if r.get('stop_reason')), None)
    cover = rows[-1].get('continuous_coverage_until', end) if rows else origin
    end_of_survival = min(end, cover)
    rolling = any('continuous_coverage_until' in r for r in rows)
    fits = [r['evaluation']['final_curve_to_follow_fit'] for r in valid
            if (r.get('evaluation') or {}).get('final_curve_to_follow_fit')]
    guided_fits = {}
    for field in ('initial_fit', 'fit'):
        available = [d[field] for d in detail if d.get(field)]
        guided_fits[field] = {key: distribution([fit.get(key) for fit in available])
                              for key in set(k for fit in available for k in fit)}
    dimensions = ('position', 'velocity', 'acceleration', 'yaw', 'yaw_rate')
    bounds = [r['physical']['solver_bound_metrics'] for r in valid if r.get('physical')]
    bound_keys = set(k for b in bounds for k in b)
    return dict(count=len(rows), attempted=len(attempts), not_attempted=len(rows)-len(attempts),
                feasible=len(valid), independent_valid=(len(valid) if group == 'A' else sum(
                    r['result']['valid'] and d.get('independent_valid', False)
                    for r, d in zip(attempts, detail))),
                fallback_used=sum(d.get('fallback_used', False) for d in detail),
                fallback_valid=sum(r['result']['valid'] and d.get('fallback_valid', False)
                                   for r, d in zip(attempts, detail)),
                final_statuses=dict(Counter(r['result']['solver_status'] for r in attempts)),
                coverage_fraction=len(sampled)/len(rows) if rows else None,
                observed_span=end-origin,
                continuous_survival_seconds=max(0., end_of_survival-origin) if rolling else None,
                stop_reason=stopped['stop_reason'] if stopped else None,
                stop_epoch=stopped.get('stop_epoch') if stopped else None,
                own_sample_errors=rmse(sampled),
                solver_wall_seconds=distribution([r['seconds'] for r in attempts]),
                analysis_wall_seconds=distribution([r.get('analysis_seconds') for r in attempts]),
                timing={phase: distribution([r['result']['timing'].get(phase) for r in attempts])
                        for phase in sorted(set(PHASES).union(
                            k for r in attempts for k in r['result']['timing']))},
                final_curve_to_follow_fit={key: distribution([f.get(key) for f in fits])
                                           for key in set(k for f in fits for k in f)},
                follow_guided_fit=guided_fits,
                handover={name: distribution([r['handover'][name] for r in rows
                                              if r.get('handover')]) for name in dimensions},
                solver_dynamic_and_fov={key: distribution([b.get(key) for b in bounds])
                                        for key in bound_keys},
                reference_quality_is_not_px4_execution=True)


def read_scenes(limit):
    """Read full frozen populations before any explicitly declared per-scene quick-run limit."""
    scenes, manifest = {}, {}
    for scene, folder, expected in (('f2', 'f2_retry', 118), ('f3', 'f3_short', 489)):
        paths = sorted((INPUT_ROOT/folder/'shadow').glob('mpc_seed_shadow*.jsonl'))
        cases = load(paths)
        if len(cases) != expected:
            raise ValueError(f'{scene}: expected {expected} original requests, got {len(cases)}')
        initial_path = INITIAL_ROOT/f'{scene}_initial.json'
        initial = json.loads(initial_path.read_text())
        first = next(i for i, c in enumerate(cases) if c[1] == initial['line']
                     and c[2] == initial['sha'])
        scenes[scene] = dict(single=cases[:limit], rolling=cases[first:][:limit])
        manifest[scene] = dict(
            original_request_count=len(cases), common_initial=initial,
            initial_file_sha256=hashlib.sha256(initial_path.read_bytes()).hexdigest(),
            sources={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            selected={mode: [dict(path=c[0], line=c[1], line_sha256=c[2]) for c in subset]
                      for mode, subset in scenes[scene].items()})
    return scenes, manifest


def run(output, limit=None, horizons=(1.2, 1.6)):
    """Write isolated research evidence; formal matrix execution remains caller controlled."""
    if limit is not None and limit < 1:
        raise ValueError('limit must be positive')
    scenes, inputs = read_scenes(limit)
    output.mkdir(exist_ok=False, parents=True)
    source_paths = sorted((WORKSPACE/'src/uav_control/uav_control').rglob('*.py'))
    source_paths += sorted((WORKSPACE/'scripts').glob('*.py'))
    code = {str(p.relative_to(WORKSPACE)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in source_paths}
    manifest = dict(schema_version=1, inputs=inputs, source_sha256=code,
                    groups=GROUPS, horizons=horizons, quick_run_limit_per_scene_per_mode=limit,
                    frozen_wall_clock='original raw.now_stamp',
                    rolling_policy='own curves, one episode, stop at first uncovered epoch',
                    independent_solver_cache='fresh solver per request in every group',
                    response_assumptions=asdict(FollowRolloutConfig()),
                    initial_command='bounded execution velocity; emitted command unavailable',
                    reference_target_source='recorded tracking prediction only',
                    fit_analysis_outside_solver_timer=True, tracker_accepted=False)
    timing_interpretation = dict(
        coefficient_construction='CycleProfile minco_coefficients total including child phases',
        initial_coefficient_construction='initial diagnostic construction only',
        optimization_and_validation='inclusive timings; overlap coefficient construction',
        coefficient_plus_optimization_plus_validation_is_additive=False,
        fit_analysis='post-solver evaluation; excluded from solver wall time',
        missing_phase='null, never a fabricated zero')
    manifest['timing_interpretation'] = timing_interpretation
    write_json(output/'manifest.json', manifest)
    summary = dict(groups=GROUPS, horizons=horizons, quick_run=limit is not None,
                   source_manifest='manifest.json', configurations={}, matched_rolling={},
                   same_input_checks={}, timing_interpretation=timing_interpretation)
    for scene, subsets in scenes.items():
        for horizon in horizons:
            for mode, cases in subsets.items():
                groups = {}
                for group in GROUPS:
                    name = f'{scene}_h{horizon}_{group}_{mode}'
                    rows = run_case(cases, output/(name+'.jsonl'), group, horizon,
                                    rolling=mode == 'rolling')
                    groups[group] = rows
                    summary['configurations'][name] = summarize(
                        rows, group, cases[0][3].context.execution_start_stamp,
                        cases[-1][3].context.execution_start_stamp)
                    print(name, len(rows), summary['configurations'][name]['feasible'], flush=True)
                if mode == 'rolling':
                    summary['matched_rolling'][f'{scene}_h{horizon}'] = matched(groups)
                else:
                    common = {}
                    for group, rows in groups.items():
                        common[group] = [(source_key(r), r['raw_request'],
                                          r['constraint_fingerprint']) for r in rows]
                    if any(rows != common['A'] for rows in common.values()):
                        raise ValueError('INDEPENDENT_INPUT_OR_CONSTRAINT_MISMATCH')
                    summary['same_input_checks'][f'{scene}_h{horizon}'] = dict(
                        count=len(cases), all_four_identical=True)
    write_json(output/'summary.json', summary)
    with (output/'SHA256SUMS').open('x') as stream:
        for path in sorted(output.iterdir()):
            if path.is_file() and path.name != 'SHA256SUMS':
                stream.write(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+path.name+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--limit', type=int, help='quick run: requests per scene per mode')
    parser.add_argument('--horizons', type=float, nargs='+', default=[1.2, 1.6],
                        choices=(1.2, 1.6))
    args = parser.parse_args()
    run(args.output.resolve(), args.limit, tuple(dict.fromkeys(args.horizons)))


if __name__ == '__main__':
    main()
