#!/usr/bin/env python3
"""New-only P47 frozen-input comparisons; all rolling states are hypothetical, not PX4."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np

from p46_follow_study import (
    FollowGuidedMinco, FollowRolloutConfig, P44FollowSolver, WORKSPACE, distribution,
    evaluation as evaluation, matched, read_scenes, run_case, safe, source_key,
    summarize as p46_summary, write_json)
from uav_control.guidance.fast_follow_minco import FastConfig, freshness
from uav_control.guidance.follow_fast_validation import DERIVATIVES
from uav_control.guidance.follow_minco_optimizer import _trajectory
from uav_control.guidance.polynomial_extrema import derivative_peak


OUTPUT_ROOT = WORKSPACE/'data/experiments/20261010_p47_follow_guided/comparison'
HISTORICAL = WORKSPACE/'data/experiments/20261010_p46_follow_guided/comparison/summary.json'
SUITES = ('main', 'terminal', 'budget', 'budget_stress')
GROUPS = dict(A='p44_D', B='p46_follow_guided_fixed', C='p47_boundary_fixed_qva',
              D='p47_boundary_jerk_qva')
REJECTION_CATEGORIES = ('jerk', 'speed', 'acceleration', 'yaw', 'deadline', 'visibility',
                        'attitude', 'dynamic_unresolved', 'other')
__all__ = ('evaluation', 'read_scenes', 'run_case', 'matched')


def p47_engine():
    """Lazy import keeps the unchanged A/B baselines independently runnable."""
    from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
    return P47FollowGuidedMinco


def experiment_arms(suite):
    """Separate terminal variables and budget scheduling from the four primary groups."""
    common = dict(engine='p47', terminal_mode='qva', refinement=True, scheduling='adaptive')
    if suite == 'main':
        return dict(A=dict(engine='p44'), B=dict(engine='p46'),
                    C=dict(common, refinement=False), D=dict(common))
    if suite == 'terminal':
        return {mode: dict(common, terminal_mode=mode) for mode in ('a', 'va', 'qva')}
    if suite == 'budget':
        return {policy: dict(common, scheduling=policy) for policy in ('legacy', 'adaptive')}
    if suite == 'budget_stress':
        return {f'{policy}_{ms}ms': dict(common, scheduling=policy, remaining_seconds=ms/1000)
                for ms in (8, 12, 20, 24) for policy in ('legacy', 'adaptive')}
    raise ValueError('UNKNOWN_P47_SUITE')


class BudgetStress:
    """Delay solver arrival against original expiry; never alter any logged request epoch."""

    def __init__(self, model, arm, horizon, now):
        self.original_now, self.stamp = now, now()
        self.remaining = arm['remaining_seconds']
        self.reserve = FastConfig().publication_reserve
        plain = {k: v for k, v in arm.items() if k != 'remaining_seconds'}
        self.solver = make(model, plain, horizon, lambda: self.stamp)

    def solve(self, request):
        """The remaining value excludes the unchanged publication/receiver reserve."""
        until, original_remaining = freshness(request, self.original_now())
        self.stamp = max(self.original_now(), until-self.reserve-self.remaining)
        result = self.solver.solve(request)
        stress = dict(requested_compute_seconds=self.remaining,
                      remaining_compute_seconds=until-self.stamp-self.reserve,
                      original_remaining_freshness=original_remaining,
                      original_expiry=until, delayed_arrival=self.stamp,
                      publication_reserve=self.reserve, request_epochs_unchanged=True)
        return replace(result, metrics=dict(result.metrics, budget_stress=stress))


def make(model, arm, horizon, now):
    """Fresh instances, identical hard constraints, and original P44/P46 constructors."""
    if 'remaining_seconds' in arm:
        return BudgetStress(model, arm, horizon, now)
    kwargs = dict(duration=horizon, rolling=False, wall_clock=now)
    if arm['engine'] == 'p44':
        instance = P44FollowSolver(model, ablation='D', refinement='none', **kwargs)
    elif arm['engine'] == 'p46':
        instance = FollowGuidedMinco(model, refinement='none', fallback_enabled=True, **kwargs)
    elif arm['engine'] == 'p47':
        instance = p47_engine()(model, terminal_mode=arm['terminal_mode'],
                                refinement=arm['refinement'],
                                scheduling=arm['scheduling'], **kwargs)
    else:
        raise ValueError('UNKNOWN_P47_ENGINE')
    instance.profile_enabled = True
    return instance


def cadence(cases):
    """Describe actual source epochs; the shared replay never inserts interpolated requests."""
    epochs = [c[3].context.execution_start_stamp for c in cases]
    now = [c[3].now_stamp for c in cases]
    return dict(count=len(cases), first_epoch=epochs[0] if epochs else None,
                last_epoch=epochs[-1] if epochs else None,
                interval_seconds=distribution(np.diff(epochs)),
                request_interval_seconds=distribution(np.diff(now)), fabricated_samples=0,
                replay_minimum_spacing_seconds=.2,
                policy='native recorded cadence; shared P46 rolling minimum spacing, no upsample')


def boundary_offset(row):
    """Separate offset from recorded navigation and its explicit CA projection to execution."""
    request = row.get('request')
    if request is None:
        return None
    boundary = np.asarray(request['state'])
    measured = np.asarray(request.get('measurement_state') or request['state'])
    context = request['context']
    dt = context['execution_start_stamp']-context['navigation_stamp']
    projected = measured.copy()
    projected[:3] += measured[3:6]*dt+measured[6:9]*dt**2/2
    projected[3:6] += measured[6:9]*dt
    report = dict(navigation_epoch=context['navigation_stamp'],
                  execution_epoch=context['execution_start_stamp'], projection_seconds=dt,
                  policy=request.get('boundary_policy', 'measured'),
                  projection_is_not_measurement=True)
    for name, reference in (('measurement', measured), ('projected_measurement', projected)):
        delta = boundary[:9]-reference[:9]
        report[name] = {key: float(np.linalg.norm(delta[part])) for key, part in (
            ('position', slice(0, 3)), ('velocity', slice(3, 6)), ('acceleration', slice(6, 9)))}
        report[name].update(horizontal_position=float(np.linalg.norm(delta[:2])),
                            vertical_position=float(delta[2]))
    return report


def rejection_categories(status, violations=None):
    """Generic dynamic failures remain unresolved unless a recorded component proves the cause."""
    text = str(status).upper()
    categories = set()
    for name, value in (violations or {}).items():
        if isinstance(value, (float, int)) and value > 1e-6:
            text += ' '+str(name).upper()
    for category, tokens in dict(
            jerk=('JERK',), speed=('SPEED',), acceleration=('ACCELERATION',), yaw=('YAW',),
            deadline=('DEADLINE', 'BUDGET', 'EXPIRED', 'STALE'),
            visibility=('VISIBILITY', 'FOV', 'PREDICTION_REVALIDATION_FAILED'),
            attitude=('ATTITUDE', 'THRUST', 'TILT')).items():
        if any(token in text for token in tokens):
            categories.add(category)
    if 'DYNAMIC' in text and not categories.intersection(('jerk', 'speed', 'acceleration')):
        categories.add('dynamic_unresolved')
    return categories or {'other'}


def coefficient_quality(metrics):
    """Exact derivative extrema and jerk integral, independently of legacy control_effort."""
    if not all(k in metrics for k in ('xyz_coefficients', 'durations')):
        return None
    xyz, durations = np.asarray(metrics['xyz_coefficients']), np.asarray(metrics['durations'])
    if xyz.shape != (len(durations), 6, 3) or not np.all(np.isfinite(xyz)):
        raise ValueError('INVALID_OFFLINE_COEFFICIENTS')
    jerk = np.polynomial.polynomial.polyder(xyz, 3, axis=1)
    integral = 0.
    for part, duration in zip(jerk, durations):
        for axis in range(3):
            squared = np.polynomial.polynomial.polymul(part[:, axis], part[:, axis])
            powers = np.arange(1, len(squared)+1)
            integral += float(np.sum(squared*duration**powers/powers))
    return dict(derivative_peaks={name: derivative_peak(xyz, durations, order, axes)
                                  for name, order, axes in DERIVATIVES},
                exact_jerk_integral=integral,
                legacy_reported_jerk_integral=metrics.get('jerk_integral'),
                quality_analysis_outside_solver_timer=True)


def postprocess(row):
    """Persist independent quality evidence and true measured-versus-reference offsets."""
    result = row.get('result')
    final = coefficient_quality(result['metrics']) if result else None
    detail = result['metrics'].get('follow_guided', {}) if result else {}
    correction = detail.get('terminal_correction') or {}
    evaluation_report = row.get('evaluation') or {}
    seed = evaluation_report.get('initial_seed')
    initial = None
    if seed:
        curve = _trajectory(SimpleNamespace(**seed), seed['q'], seed['durations'])
        initial = coefficient_quality(dict(xyz_coefficients=curve.coefficients,
                                           durations=seed['durations']))
        limits = row['model_config']['mpc']
        initial['violations'] = {name: max(0., peak['value']-limits['maximum_'+name])
                                 for name, peak in initial['derivative_peaks'].items()}
    independent = None
    if all(key in correction for key in ('start', 'end', 'q', 'durations')):
        curve = _trajectory(SimpleNamespace(**correction),
                            correction['q'], correction['durations'])
        independent = coefficient_quality(dict(xyz_coefficients=curve.coefficients,
                                               durations=correction['durations']))
        limits = row['model_config']['mpc']
        independent['violations'] = {name: max(0., peak['value']-limits['maximum_'+name])
                                     for name, peak in independent['derivative_peaks'].items()}
    return dict(source_key=source_key(row), epoch=row['epoch'], boundary=boundary_offset(row),
                final_coefficients=final, original_follow_seed=initial,
                independent_candidate=independent,
                seed_note='unmodified FOLLOW reference seed; not necessarily selected P47 seed',
                solver_timer_excludes_this_analysis=True)


def summarize(rows, group, origin, end):
    """Augment the unchanged population metrics with explicit rates and rejection provenance."""
    report = p46_summary(rows, group, origin, end)
    attempts = [r for r in rows if r.get('result') is not None]
    rejections = {phase: Counter() for phase in ('initial', 'independent', 'final')}
    offsets = [v for r in rows if (v := boundary_offset(r)) is not None]
    correction = []
    budgets = []
    for row in attempts:
        result = row['result']
        detail = result['metrics'].get('follow_guided', {})
        if detail.get('terminal_correction'):
            correction.append(detail['terminal_correction'])
        if result['metrics'].get('budget_stress'):
            budgets.append(result['metrics']['budget_stress'])
        for phase in ('initial', 'independent', 'final'):
            status = (result['solver_status'] if phase == 'final' else
                      detail.get(phase+'_status', result['solver_status']
                                 if phase == 'independent' else None))
            if phase == 'final':
                failed = not result['valid']
            elif phase == 'independent':
                failed = not detail.get('independent_valid', result['valid'])
            else:
                failed = status not in (None, 'SAMPLED_FEASIBLE', 'NOT_ATTEMPTED')
            if failed:
                violations = (result.get('constraint_violations', {}) if phase == 'final'
                              else detail.get(phase+'_violations', {}))
                # P46 fixed uses this exact seed. New P47 terminal corrections do not.
                if (not violations and group == 'B' and 'DYNAMIC' in str(status)
                        and (phase != 'final' or not detail.get('fallback_used'))
                        and row.get('_analysis')):
                    original = row['_analysis'].get('original_follow_seed')
                    if original:
                        violations = original['violations']
                if (not violations and 'DYNAMIC' in str(status) and row.get('_analysis')
                        and (phase == 'independent' or (
                            phase == 'initial' and status == detail.get('independent_status')))):
                    independent = row['_analysis'].get('independent_candidate')
                    if independent:
                        violations = independent['violations']
                rejections[phase].update(rejection_categories(status, violations))
    count = report['attempted']
    report.update(
        independent_feasible_fraction=report['independent_valid']/count if count else None,
        final_feasible_fraction=report['feasible']/count if count else None,
        rejections={phase: {name: counts[name] for name in REJECTION_CATEGORIES}
                    for phase, counts in rejections.items()},
        rejection_note='categories may overlap; generic dynamic failures are unresolved',
        boundary_measurement_offsets={basis: {
            key: distribution([o[basis][key] for o in offsets])
            for key in ('position', 'velocity', 'acceleration',
                        'horizontal_position', 'vertical_position')}
            for basis in ('measurement', 'projected_measurement')},
        terminal_corrections=correction, budget_stress=budgets,
        own_sample_errors_comparable_between_arms=False,
        comparison_population='use matched_rolling all/after20 for cross-arm RMSE comparisons',
        metric_scope='prediction-only ideal rolling model; not measured UAV/PX4 execution')
    report['fully_covered'] = ((report['not_attempted'] == 0
                                and report['coverage_fraction'] == 1.)
                               if report['continuous_survival_seconds'] is not None else None)
    report['longest_continuous_valid_seconds'] = report['continuous_survival_seconds']
    qualities = [r['_analysis']['final_coefficients'] for r in rows if r.get('_analysis')
                 and r['_analysis']['final_coefficients'] and r['result']['valid']]
    report['coefficient_quality'] = dict(
        population='independently validated final curves only, including successful fallback',
        exact_jerk_integral=distribution([q['exact_jerk_integral'] for q in qualities]),
        legacy_reported_jerk_integral=distribution(
            [q['legacy_reported_jerk_integral'] for q in qualities]),
        derivative_extrema={name: distribution(
            [q['derivative_peaks'][name]['value'] for q in qualities])
            for name, _, _ in DERIVATIVES},
        analysis_outside_solver_timer=True)
    bounds = report['solver_dynamic_and_fov']
    report['minimum_complete_target_fov_margin_rad'] = min(
        (bounds[name]['minimum'] for name in (
            'minimum_horizontal_margin', 'minimum_vertical_margin')
         if bounds.get(name)), default=None)
    return report


def validate_output(output):
    """Keep every rerun new and strictly under the dedicated P47 comparison directory."""
    output = Path(output).resolve()
    root = OUTPUT_ROOT.resolve()
    if output != root and root not in output.parents:
        raise ValueError('P47_COMPARISON_OUTPUT_REQUIRED')
    if output.exists():
        raise FileExistsError(output)


def source_manifest():
    """Record working-file hashes, Git base and dirty state as distinct evidence."""
    paths = sorted((WORKSPACE/'src/uav_control/uav_control').rglob('*.py'))
    paths += sorted((WORKSPACE/'scripts').glob('*.py'))
    return dict(
        sha256={str(p.relative_to(WORKSPACE)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in paths},
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=WORKSPACE,
                                         text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--short'], cwd=WORKSPACE,
                                           text=True).splitlines())


def same_inputs(groups):
    """Compare full raw requests and immutable constraint fingerprints for independent trials."""
    populations = {group: [(source_key(r), r['raw_request'], r['constraint_fingerprint'])
                           for r in rows] for group, rows in groups.items()}
    first = next(iter(populations.values()))
    if any(rows != first for rows in populations.values()):
        raise ValueError('INDEPENDENT_INPUT_OR_CONSTRAINT_MISMATCH')
    return dict(count=len(first), all_identical=True)


def plot_groups(output, key, groups, single):
    """Plot only recorded request epochs; no curve between samples represents a new request."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plot_root = output/'plots'
    plot_root.mkdir(exist_ok=True)
    axes_sets = dict(trajectory=('N (m)', 'E (m)', 'D (m)'),
                     jerk=('horizontal jerk (m/s³)', 'vertical jerk (m/s³)', 'FOV margin (rad)'),
                     boundary=('P offset (m)', 'V offset (m/s)', 'A offset (m/s²)'))
    for kind, labels in axes_sets.items():
        figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for group, rows in groups.items():
            origin = rows[0]['epoch'] if rows else 0.
            times, values = [], []
            for row in rows:
                value = None
                if kind == 'trajectory' and row.get('sampled_state'):
                    value = row['sampled_state'][:3]
                elif kind == 'jerk' and (row.get('result') or {}).get('valid'):
                    metrics = row['result']['metrics']
                    value = [metrics.get('maximum_horizontal_jerk'),
                             metrics.get('maximum_vertical_jerk'),
                             min(metrics.get('minimum_horizontal_margin', float('nan')),
                                 metrics.get('minimum_vertical_margin', float('nan')))]
                elif kind == 'boundary' and (offset := boundary_offset(row)):
                    value = [offset['projected_measurement'][name]
                             for name in ('position', 'velocity', 'acceleration')]
                if value is not None:
                    times.append(row['epoch']-origin)
                    values.append(value)
            if values:
                for index, axis in enumerate(axes):
                    axis.plot(times, [v[index] for v in values], '.-', label=group, markersize=2)
        for axis, label in zip(axes, labels):
            axis.set_ylabel(label)
            axis.grid(alpha=.25)
            if axis.lines:
                axis.legend()
        axes[-1].set_xlabel('recorded execution epoch from shared origin (s)')
        figure.suptitle(key+' — hypothetical ideal rolling; native request samples')
        figure.tight_layout()
        figure.savefig(plot_root/(key+'_'+kind+'.png'), dpi=140)
        plt.close(figure)
    figure, axes = plt.subplots(1, 2, figsize=(11, 4))
    for axis, phase in zip(axes, ('initialization', 'total')):
        names, values = [], []
        for group, rows in single.items():
            timings = [r['result']['timing'].get(phase) for r in rows if r.get('result')]
            timings = [1000*t for t in timings if t is not None]
            if timings:
                names.append(group)
                values.append(timings)
        if values:
            axis.boxplot(values, labels=names, showfliers=False)
        axis.set_title(phase)
        axis.set_ylabel('fresh replay wall time (ms)')
        axis.grid(alpha=.25)
    figure.suptitle(key+' — solver timing; post-solver reference analysis excluded')
    figure.tight_layout()
    figure.savefig(plot_root/(key+'_timing.png'), dpi=140)
    plt.close(figure)


def run(output, limit=None, horizons=(1.2, 1.6), suites=SUITES, plots=True):
    """Write fresh full raw results, separate ablations and SHA256 evidence without old writes."""
    output = Path(output).resolve()
    validate_output(output)
    if limit is not None and limit < 1:
        raise ValueError('limit must be positive')
    if any(h not in (1.2, 1.6) for h in horizons) or not horizons:
        raise ValueError('INVALID_P47_HORIZONS')
    arm_sets = {suite: experiment_arms(suite) for suite in suites}
    scenes, inputs = read_scenes(limit)
    output.mkdir(exist_ok=False, parents=True)
    historical = (dict(path=str(HISTORICAL),
                       sha256=hashlib.sha256(HISTORICAL.read_bytes()).hexdigest(),
                       copied_as='historical_p46_summary.json') if HISTORICAL.exists() else None)
    manifest = dict(
        schema_version=1, inputs=inputs, source_version=source_manifest(), groups=GROUPS,
        arms=arm_sets, horizons=horizons, quick_run_limit_per_scene_per_mode=limit,
        cadence={scene: {mode: cadence(cases) for mode, cases in subsets.items()}
                 for scene, subsets in scenes.items()},
        frozen_wall_clock='original raw.now_stamp; stress delays arrival within unchanged TTL',
        rolling_policy='unchanged P46 own curves, single episode, first uncovered epoch stops',
        independent_solver_cache='fresh solver per request, every arm',
        profiling='same CycleProfile enabled for every fresh arm; overhead included in timing',
        response_assumptions=asdict(FollowRolloutConfig()),
        reference_target_source='recorded tracking prediction only',
        input_freshness_seconds=.125, original_expiry_never_extended=True,
        fit_analysis_outside_solver_timer=True, tracker_accepted=False,
        main_terminal_decision=dict(
            mode='qva', decided_before_comparison=True,
            evidence='Stage B frozen F3 H1.2: A/VA repair 1/4; QVA repair 155/162 jerk cases',
            rationale='piece0/start jerk dominates; A/VA alone cannot repair early-segment jerk'),
        historical_p46=historical,
        historical_timing_note='historical timing retained separately; fresh replay is a new run',
        timing_note='inclusive phase timers may overlap; missing timings are null, never zero')
    write_json(output/'manifest.json', manifest)
    if historical:
        write_json(output/'historical_p46_summary.json', json.loads(HISTORICAL.read_text()))
    summary = dict(groups=GROUPS, arms=arm_sets, horizons=horizons, quick_run=limit is not None,
                   configurations={}, matched_rolling={}, same_input_checks={},
                   source_manifest='manifest.json', historical_p46=historical,
                   metric_scope='ideal prediction-only rolling; no real UAV tracking claim')
    for scene, subsets in scenes.items():
        for horizon in horizons:
            for suite, arms in arm_sets.items():
                per_mode = {}
                modes = ('single',) if suite == 'budget_stress' else ('single', 'rolling')
                for mode in modes:
                    cases = subsets[mode]
                    groups = {}
                    for group, arm in arms.items():
                        name = f'{scene}_h{horizon}_{suite}_{group}_{mode}'

                        def factory(model, unused, h, now, spec=arm):
                            return make(model, spec, h, now)

                        rows = run_case(cases, output/(name+'.jsonl'), group, horizon,
                                        rolling=mode == 'rolling', factory=factory)
                        with (output/(name+'_analysis.jsonl')).open('x') as stream:
                            for row in rows:
                                row['_analysis'] = postprocess(row)
                                stream.write(json.dumps(safe(row['_analysis']),
                                                        allow_nan=False)+'\n')
                        groups[group] = rows
                        summary['configurations'][name] = summarize(
                            rows, 'A' if arm['engine'] == 'p44' else group,
                            cases[0][3].context.execution_start_stamp,
                            cases[-1][3].context.execution_start_stamp)
                        print(name, len(rows), summary['configurations'][name]['feasible'],
                              flush=True)
                    key = f'{scene}_h{horizon}_{suite}'
                    per_mode[mode] = groups
                    if mode == 'rolling':
                        summary['matched_rolling'][key] = matched(groups)
                    else:
                        summary['same_input_checks'][key] = same_inputs(groups)
                if plots and suite == 'main':
                    plot_groups(output, f'{scene}_h{horizon}',
                                per_mode['rolling'], per_mode['single'])
    write_json(output/'summary.json', summary)
    with (output/'SHA256SUMS').open('x') as stream:
        for path in sorted(output.rglob('*')):
            if path.is_file() and path.name != 'SHA256SUMS':
                stream.write(hashlib.sha256(path.read_bytes()).hexdigest()+'  ' +
                             str(path.relative_to(output))+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--limit', type=int, help='explicit quick-run limit per scene and mode')
    parser.add_argument('--horizons', type=float, nargs='+', default=[1.2, 1.6],
                        choices=(1.2, 1.6))
    parser.add_argument('--suites', nargs='+', choices=SUITES, default=SUITES)
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()
    run(args.output, args.limit, tuple(dict.fromkeys(args.horizons)),
        tuple(dict.fromkeys(args.suites)), not args.no_plots)


if __name__ == '__main__':
    main()
