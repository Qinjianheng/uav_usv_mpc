#!/usr/bin/env python3
"""Frozen FOLLOW-to-MINCO derivative diagnosis and bounded offline counterfactuals."""
import argparse
from collections import Counter
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import lsq_linear

from p46_follow_study import read_scenes, safe
from uav_control.controllers.direct_reference_minco import follow_guided_seed
from uav_control.guidance.follow_attitude_bounds import attitude_extrema
from uav_control.guidance.follow_fast_validation import DERIVATIVES, fields, validate_seed
from uav_control.guidance.follow_minco_optimizer import _trajectory
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_rollout import rollout_follow
from uav_control.guidance.greedy_follow_initializer import FollowSeed
from uav_control.guidance.polynomial_extrema import derivative_peak
from uav_control.guidance.yaw_trajectory import YawTrajectory


DOMAINS = dict(A=1.5, V=.5, Q=.2, time_fraction=.2, yaw=.3)


def weighted_jerk(seed):
    """Three-point Gauss-Legendre integrates squared quadratic jerk exactly per piece."""
    nodes, weights = np.polynomial.legendre.leggauss(3)
    durations = np.asarray(seed.durations)
    starts = np.r_[0., np.cumsum(durations)[:-1]]
    query = np.concatenate([start+(nodes+1)*duration/2
                            for start, duration in zip(starts, durations)])
    scale = np.sqrt(np.concatenate([weights*t/2 for t in durations]))
    curve = _trajectory(seed, seed.q, seed.durations)
    jerk = fields(curve.coefficients, seed.durations, np.zeros((3, 4)), query)[3]
    return (jerk*scale[:, None]).ravel()


def yaw_peak(seed, start_rate=0.):
    """Report the analytic quadratic yaw-rate maximum and its original local support."""
    yaw = YawTrajectory(np.r_[0., np.cumsum(seed.durations)], seed.yaw, start_rate=start_rate)
    best = dict(value=-1.)
    offset = 0.
    for i, duration in enumerate(seed.durations):
        cubic, quadratic = yaw.spline.c[:2, i]
        ts = [0., duration]
        if abs(cubic) > 1e-12 and 0 < -quadratic/(3*cubic) < duration:
            ts.append(-quadratic/(3*cubic))
        for local in ts:
            value = abs(float(yaw.spline(offset+local, 1)))
            if value > best['value']:
                best = dict(value=value, time=offset+local, piece=i, local_time=local)
        offset += duration
    return best


def derivatives(seed, start_rate=0.):
    """All six exact norm peaks are retained even when independent admission stops early."""
    curve = _trajectory(seed, seed.q, seed.durations)
    weighted = weighted_jerk(seed)
    return dict(peaks={name: derivative_peak(curve.coefficients, seed.durations, d, axes)
                       for name, d, axes in DERIVATIVES},
                yaw_rate=yaw_peak(seed, start_rate), jerk_integral=float(weighted@weighted),
                legacy_reported_control_effort=curve.control_effort(),
                jerk_integral_source='three-point Gauss exact squared quadratic jerk integral')


def variables(mode):
    """Only terminal V/A and intermediate Q move; the real start and terminal P stay fixed."""
    result = []
    if mode not in ('A', 'VA', 'QVA', 'Q'):
        raise ValueError('INVALID_COUNTERFACTUAL_MODE')
    if 'Q' in mode:
        result += [('Q', i, DOMAINS['Q']) for i in range(6)]
    if 'V' in mode:
        result += [('V', i, DOMAINS['V']) for i in range(3)]
    if 'A' in mode:
        result += [('A', i, DOMAINS['A']) for i in range(3)]
    return result


def apply_delta(seed, descriptors, delta):
    q, end = np.asarray(seed.q).copy(), np.asarray(seed.end).copy()
    for (name, i, _), value in zip(descriptors, delta):
        if name == 'Q':
            q.ravel()[i] += value
        else:
            end[(3 if name == 'V' else 6)+i] += value
    return replace(seed, q=tuple(map(tuple, q)), end=tuple(end))


def quadratic_counterfactual(seed, mode):
    """Bounded convex least squares of exact jerk energy using the existing linear MINCO map."""
    descriptors = variables(mode)
    base = weighted_jerk(seed)
    columns = []
    for i in range(len(descriptors)):
        unit = np.zeros(len(descriptors))
        unit[i] = 1.
        columns.append(weighted_jerk(apply_delta(seed, descriptors, unit))-base)
    matrix = np.column_stack(columns)
    bound = np.asarray([v[2] for v in descriptors])
    solution = lsq_linear(matrix, -base, bounds=(-bound, bound), tol=1e-11, max_iter=200)
    modified = apply_delta(seed, descriptors, solution.x)
    after = weighted_jerk(modified)
    return modified, dict(mode=mode, objective='exact integrated squared 3D jerk',
                          objective_before=float(base@base), objective_after=float(after@after),
                          success=bool(solution.success), status=int(solution.status),
                          optimality=float(solution.optimality),
                          variables=descriptors, delta=solution.x.tolist(),
                          domains_are_safety_thresholds=False)


def time_counterfactual(seed):
    """Finite fixed-total segment-time scan, Q and both PVA boundaries unchanged."""
    durations = np.asarray(seed.durations)
    choices = []
    for a in (-.2, -.1, 0., .1, .2):
        for b in (-.2, -.1, 0., .1, .2):
            delta = np.array([a*durations[0], b*durations[1],
                              -a*durations[0]-b*durations[1]])
            if np.all(np.abs(delta) <= .2*durations+1e-12):
                candidate = replace(seed, durations=tuple(durations+delta))
                vector = weighted_jerk(candidate)
                choices.append((float(vector@vector), candidate))
    value, selected = min(choices, key=lambda item: item[0])
    return selected, dict(mode='T', objective_after=value, candidate_count=len(choices),
                          fixed_total_horizon=sum(seed.durations),
                          relative_duration_domain=[-.2, .2],
                          grid_fraction_step=.1, domains_are_safety_thresholds=False)


def yaw_counterfactual(seed, start_rate):
    """Minimize exact yaw-rate integral in a bounded knot box, without changing XYZ."""
    nodes, weights = np.polynomial.legendre.leggauss(3)
    knots = np.r_[0., np.cumsum(seed.durations)]
    query = np.concatenate([a+(nodes+1)*t/2 for a, t in zip(knots, seed.durations)])
    scale = np.sqrt(np.concatenate([weights*t/2 for t in seed.durations]))

    def vector(values):
        return YawTrajectory(knots, values, start_rate=start_rate).spline(query, 1)*scale

    base = vector(seed.yaw)
    columns = []
    for i in range(1, 4):
        values = np.asarray(seed.yaw).copy()
        values[i] += 1.
        columns.append(vector(values)-base)
    solved = lsq_linear(np.column_stack(columns), -base, bounds=(-.3, .3), tol=1e-11)
    values = np.asarray(seed.yaw).copy()
    values[1:] += solved.x
    modified = replace(seed, yaw=tuple(values))
    return modified, dict(mode='yaw', maximum_rate_before=yaw_peak(seed, start_rate)['value'],
                          maximum_rate_after=yaw_peak(modified, start_rate)['value'],
                          delta=solved.x.tolist(), success=bool(solved.success),
                          knot_delta_domain_rad=[-.3, .3], domains_are_safety_thresholds=False)


def inspect_seed(request, seed, model):
    """Full independent validation outcome plus simultaneous failures and exact witnesses."""
    rate = getattr(request, 'reference_yaw_rate', 0.)
    detail = derivatives(seed, rate)
    violations = {name: peak['value']-getattr(model.config, 'maximum_'+name)
                  for name, peak in detail['peaks'].items()
                  if peak['value'] > getattr(model.config, 'maximum_'+name)+1e-6}
    if detail['yaw_rate']['value'] > model.config.maximum_yaw_rate+1e-6:
        violations['yaw_rate'] = detail['yaw_rate']['value']-model.config.maximum_yaw_rate
    curve = _trajectory(seed, seed.q, seed.durations)
    attitude = attitude_extrema(curve.coefficients, seed.durations, model.attitude_config)
    if not attitude['valid']:
        violations['attitude'] = attitude
    result = validate_seed(request, seed, model, budget=10.)
    detail.update(seed=asdict(seed), simultaneous_violations=violations,
                  continuous_attitude=attitude,
                  validation=dict(valid=result.valid, status=result.solver_status,
                                  reason=result.reason, violations=result.constraint_violations,
                                  seconds=result.solve_time,
                                  minimum_horizontal_margin=result.metrics.get(
                                      'minimum_horizontal_margin'),
                                  minimum_vertical_margin=result.metrics.get(
                                      'minimum_vertical_margin')))
    return detail


def reference_diagnostics(reference):
    times = np.asarray(reference.times)
    jerk = np.diff(np.asarray(reference.accelerations), axis=0)/np.diff(times)[:, None]
    peaks = {}
    for name, axes in (('horizontal_jerk', (0, 1)), ('vertical_jerk', (2,))):
        values = np.linalg.norm(jerk[:, axes], axis=1)
        i = int(np.argmax(values))
        peaks[name] = dict(value=float(values[i]), interval_index=i,
                           interval_start=float(times[i]), interval_end=float(times[i+1]),
                           vector=jerk[i].tolist())
    return dict(peaks=peaks, intervals=len(jerk), integration_dt=.01,
                start_pva=[*reference.positions[0], *reference.velocities[0],
                           *reference.accelerations[0]],
                end_pva=[*reference.positions[-1], *reference.velocities[-1],
                         *reference.accelerations[-1]])


def start_diagnostics(request, model):
    values = dict(horizontal_speed=np.linalg.norm(request.state[3:5]),
                  vertical_speed=abs(request.state[5]),
                  horizontal_acceleration=np.linalg.norm(request.state[6:8]),
                  vertical_acceleration=abs(request.state[8]))
    failures = {name: float(value-getattr(model.config, 'maximum_'+name))
                for name, value in values.items()
                if value > getattr(model.config, 'maximum_'+name)+1e-6}
    return dict(measured_start_pva=request.state[:9], values=values,
                violates_derivative_limits=failures, dynamically_feasible=not failures,
                note='Necessary start V/A feasibility only; not a whole-curve safety approval')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def diagnose(output, horizons, limit=None):
    scenes, source_manifest = read_scenes(limit)
    cases = scenes['f3']['single']
    input_paths = [Path(p) for p in source_manifest['f3']['sources']]
    comparison = Path('data/experiments/20261010_p46_follow_guided/comparison')
    original_by_horizon = {}
    for horizon in horizons:
        original_path = comparison/f'f3_h{horizon:.1f}_B_single.jsonl'
        input_paths.append(original_path)
        with original_path.open() as stream:
            original_by_horizon[horizon] = {
                (r['line'], r['source_sha256']): r
                for r in (json.loads(line) for line in stream)}
    input_hashes = {str(p): digest(p) for p in input_paths}
    output.mkdir(parents=True, exist_ok=True)
    summaries = []
    for horizon in horizons:
        rows, began = [], time.perf_counter()
        path = output/f'f3_h{horizon:.1f}.jsonl'
        if path.exists():
            raise ValueError('REFUSE_OVERWRITE_DIAGNOSTIC_OUTPUT')
        with path.open('x') as stream:
            for i, (source, line, sha, request, model) in enumerate(cases):
                problem = FollowProblem(request, model, horizon)
                reference = rollout_follow(problem)
                seed = follow_guided_seed(request, problem, reference)
                initial = inspect_seed(request, seed, model)
                original = original_by_horizon[horizon][(line, sha)]
                original_status = original['result']['metrics']['follow_guided'][
                    'initial_status']
                row = dict(source=source, line=line, source_line_sha256=sha,
                           horizon=horizon, request=asdict(request),
                           initial=initial, follow=reference_diagnostics(reference),
                           start=start_diagnostics(request, model), counterfactuals={},
                           original_p46_initial_status=original_status,
                           original_p46_horizontal_jerk_population=(
                               original_status == 'DYNAMIC_INFEASIBLE'
                               and 'horizontal_jerk' in initial['simultaneous_violations']),
                           frozen_only=True, tracker_accepted=False)
                if not initial['validation']['valid']:
                    for mode in ('A', 'VA', 'QVA', 'Q'):
                        candidate, detail = quadratic_counterfactual(seed, mode)
                        row['counterfactuals'][mode] = dict(
                            optimization=detail,
                            assessment=inspect_seed(request, candidate, model))
                    for mode, solve in (('T', lambda: time_counterfactual(seed)),
                                        ('yaw', lambda: yaw_counterfactual(
                                            seed, getattr(request, 'reference_yaw_rate', 0.)))):
                        candidate, detail = solve()
                        row['counterfactuals'][mode] = dict(
                            optimization=detail,
                            assessment=inspect_seed(request, candidate, model))
                stream.write(json.dumps(safe(row), allow_nan=False)+'\n')
                stream.flush()
                rows.append(row)
                if i % 50 == 0:
                    print(f'H={horizon}: {i}/{len(cases)}', flush=True)
        rejected = [r for r in rows if not r['initial']['validation']['valid']]
        original_jerk = [r for r in rows if r['original_p46_horizontal_jerk_population']]
        summary = dict(horizon=horizon, requests=len(rows), initial_rejects=len(rejected),
                       horizontal_jerk_failures=sum(
                           'horizontal_jerk' in r['initial']['simultaneous_violations']
                           for r in rows),
                       initial_violation_counts=dict(Counter(
                           k for r in rows for k in r['initial']['simultaneous_violations'])),
                       start_infeasible=sum(not r['start']['dynamically_feasible'] for r in rows),
                       original_p46_initial_statuses=dict(Counter(
                           r['original_p46_initial_status'] for r in rows)),
                       original_p46_horizontal_jerk_population=len(original_jerk),
                       original_jerk_peak_locations=dict(Counter(
                           r['initial']['peaks']['horizontal_jerk']['location']
                           for r in original_jerk)),
                       original_jerk_peak_pieces=dict(Counter(
                           r['initial']['peaks']['horizontal_jerk']['piece']
                           for r in original_jerk)),
                       counterfactuals={}, seconds=time.perf_counter()-began)
        for mode in ('A', 'VA', 'QVA', 'Q', 'T', 'yaw'):
            items = [r['counterfactuals'][mode]['assessment'] for r in rejected]
            summary['counterfactuals'][mode] = dict(
                attempted=len(items),
                full_validation_valid=sum(r['validation']['valid'] for r in items),
                horizontal_jerk_failures=sum('horizontal_jerk' in r['simultaneous_violations']
                                             for r in items),
                remaining_violations=dict(Counter(
                    k for r in items for k in r['simultaneous_violations'])),
                validation_statuses=dict(Counter(r['validation']['status'] for r in items)))
            original_items = [r['counterfactuals'][mode]['assessment'] for r in original_jerk]
            summary['counterfactuals'][mode]['original_jerk_population'] = dict(
                count=len(original_items),
                horizontal_jerk_resolved=sum('horizontal_jerk' not in
                                             r['simultaneous_violations'] for r in original_items),
                full_validation_valid=sum(r['validation']['valid'] for r in original_items))
        write(output/f'summary_h{horizon:.1f}.json', summary)
        summaries.append(summary)
        print(json.dumps(summary), flush=True)
    current = {str(p): digest(p) for p in input_paths}
    if current != input_hashes:
        raise RuntimeError('FROZEN_SOURCE_CHANGED_DURING_ANALYSIS')
    write(output/'source_manifest.json', dict(
        source=source_manifest, source_files_unchanged=True, input_sha256=input_hashes,
        counterfactual_domains=DOMAINS,
        domains_are_safety_thresholds=False, offline_validation_budget_seconds=10.,
        endpoint_policy=('fixed real start PVA and fixed terminal P; '
                         'terminal V/A are counterfactual'),
        counterfactual_is_control_approval=False,
        code_sha256={str(p): digest(p) for p in (
            Path(__file__), Path('scripts/p46_follow_study.py'),
            Path('src/uav_control/uav_control/controllers/direct_reference_minco.py'),
            Path('src/uav_control/uav_control/guidance/minco_trajectory.py'))}))
    return summaries


def plot_products(output, horizons):
    """Separate reproducible visual/CSV export, using already frozen diagnostic rows."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    scenes, _ = read_scenes(None)
    cases = {(c[1], c[2]): c for c in scenes['f3']['single']}
    names = ('initial', 'A', 'VA', 'QVA', 'Q', 'T', 'yaw')
    flat = []
    for horizon in horizons:
        with (output/f'f3_h{horizon:.1f}.jsonl').open() as stream:
            rows = [json.loads(line) for line in stream]
        population = [r for r in rows if r['original_p46_horizontal_jerk_population']]
        for row in rows:
            for name in names:
                if name != 'initial' and name not in row['counterfactuals']:
                    continue
                detail = row['initial'] if name == 'initial' else row['counterfactuals'][name][
                    'assessment']
                flat.append(dict(
                    horizon=horizon, source_line=row['line'], source_sha=row['source_line_sha256'],
                    mode=name, original_p46_initial_status=row['original_p46_initial_status'],
                    original_horizontal_jerk_population=row[
                        'original_p46_horizontal_jerk_population'],
                    start_feasible=row['start']['dynamically_feasible'],
                    start_pva=detail['seed']['start'][:9], end_pva=detail['seed']['end'],
                    q=detail['seed']['q'], durations=detail['seed']['durations'],
                    exact_peaks=detail['peaks'], yaw_peak=detail['yaw_rate'],
                    exact_jerk_integral=detail['jerk_integral'],
                    legacy_reported_control_effort=detail['legacy_reported_control_effort'],
                    simultaneous_violations=detail['simultaneous_violations'],
                    validation=detail['validation']))
        median = np.median([r['initial']['peaks']['horizontal_jerk']['value'] for r in population])
        selected = min(population, key=lambda r: abs(
            r['initial']['peaks']['horizontal_jerk']['value']-median))
        _, _, _, request, model = cases[(selected['line'], selected['source_line_sha256'])]
        reference = rollout_follow(FollowProblem(request, model, horizon))
        reference_times = np.asarray(reference.times)
        reference_jerk = np.diff(np.asarray(reference.accelerations), axis=0)/np.diff(
            reference_times)[:, None]
        fig, axes = plt.subplots(2, 1, figsize=(11, 8))
        for name in ('initial', 'A', 'Q', 'QVA'):
            detail = selected['initial'] if name == 'initial' else selected['counterfactuals'][
                name]['assessment']
            seed = FollowSeed(**detail['seed'])
            query = np.linspace(0., sum(seed.durations), 601)
            curve = _trajectory(seed, seed.q, seed.durations)
            jerk = fields(curve.coefficients, seed.durations, np.zeros((3, 4)), query)[3]
            axes[0].plot(query, np.linalg.norm(jerk[:, :2], axis=1), label=name)
        axes[0].step(reference_times[:-1], np.linalg.norm(reference_jerk[:, :2], axis=1),
                     where='post', color='black', ls=':', label='FOLLOW 10 ms reference jerk')
        axes[0].axhline(model.config.maximum_horizontal_jerk, color='red', ls='--',
                        label='unchanged horizontal jerk limit')
        axes[0].set(xlabel='Time from execution start (s)', ylabel='Horizontal jerk (m/s^3)',
                    title=f'F3 H={horizon}: frozen source line {selected["line"]}')
        values = [[r['initial']['peaks']['horizontal_jerk']['value'] if name == 'initial'
                   else r['counterfactuals'][name]['assessment']['peaks'][
                       'horizontal_jerk']['value'] for r in population] for name in names]
        axes[1].boxplot(values, labels=names, showfliers=False)
        axes[1].axhline(model.config.maximum_horizontal_jerk, color='red', ls='--')
        axes[1].set(ylabel='Exact peak horizontal jerk (m/s^3)',
                    title=('Identical original P4.6 horizontal-jerk population, '
                           f'n={len(population)}'))
        for ax in axes:
            ax.grid(alpha=.2)
        axes[0].legend(fontsize=8, ncol=2)
        fig.tight_layout()
        fig.savefig(output/f'jerk_and_boundary_h{horizon:.1f}.png', dpi=160)
        plt.close(fig)
        write(output/f'plot_witness_h{horizon:.1f}.json', dict(
            line=selected['line'], source_sha=selected['source_line_sha256'],
            initial_seed=selected['initial']['seed'],
            corrected_seeds={name: selected['counterfactuals'][name]['assessment']['seed']
                             for name in ('A', 'Q', 'QVA')},
            unchanged_real_start=True, unchanged_terminal_position=True,
            counterfactual_domain=DOMAINS))
        fig, axes = plt.subplots(3, 1, figsize=(10, 9))
        baseline = np.asarray(selected['initial']['seed']['end'])
        points = np.asarray(selected['initial']['seed']['q'])
        modes = ('initial', 'A', 'Q', 'QVA')
        for j, name in enumerate(modes):
            seed = (selected['initial']['seed'] if name == 'initial' else
                    selected['counterfactuals'][name]['assessment']['seed'])
            terminal = np.asarray(seed['end'])
            for ax, part in zip(axes[:2], (slice(3, 6), slice(6, 9))):
                ax.bar(np.arange(3)+(j-1.5)*.18, terminal[part], width=.18, label=name)
            axes[2].bar(np.arange(2)+(j-1.5)*.18,
                        np.linalg.norm(np.asarray(seed['q'])-points, axis=1),
                        width=.18, label=name)
            if np.max(np.abs(terminal[:3]-baseline[:3])) > 1e-9:
                raise ValueError('COUNTERFACTUAL_CHANGED_TERMINAL_POSITION')
        axes[0].set_ylabel('Terminal velocity (m/s)')
        axes[1].set_ylabel('Terminal acceleration (m/s^2)')
        for ax in axes[:2]:
            ax.set_xticks(np.arange(3), ['N', 'E', 'D'])
        axes[2].set_xticks(np.arange(2), ['Q1', 'Q2'])
        axes[2].set_ylabel('Waypoint displacement (m)')
        for ax in axes:
            ax.legend(fontsize=8, ncol=4)
            ax.grid(axis='y', alpha=.2)
        fig.suptitle(f'H={horizon}: real start PVA and terminal position fixed; '
                     f'source line {selected["line"]}')
        fig.tight_layout()
        fig.savefig(output/f'boundary_pva_h{horizon:.1f}.png', dpi=160)
        plt.close(fig)
    with (output/'per_request_diagnosis.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows({k: json.dumps(v) if isinstance(v, (list, dict)) else v
                         for k, v in row.items()} for row in flat)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(
        'data/experiments/20261010_p47_follow_guided/derivative_diagnosis'))
    parser.add_argument('--horizons', nargs='+', type=float, default=[1.2, 1.6])
    parser.add_argument('--limit', type=int)
    parser.add_argument('--plots-only', action='store_true')
    args = parser.parse_args()
    if args.plots_only:
        plot_products(args.output, args.horizons)
    else:
        diagnose(args.output, args.horizons, args.limit)


if __name__ == '__main__':
    main()
