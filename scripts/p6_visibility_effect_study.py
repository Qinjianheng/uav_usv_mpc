#!/usr/bin/env python3
"""Bounded frozen-input visibility diagnosis; no ROS, execution or new calibration."""
import argparse
from collections import Counter, defaultdict
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/uav_control'))

from uav_control.controllers.follow_mpc_seed import (  # noqa: E402
    FollowMpcSeed, MpcConfig, MpcRequest,
)
from uav_control.guidance.adaptive_viewpoint import (  # noqa: E402
    AdaptiveViewpointConfig, AdaptiveViewpointSelector,
)
from uav_control.guidance.camera_visibility_batch import (  # noqa: E402
    attitude_batch, visibility_batch,
)
from uav_control.guidance.follow_minco_execution import request_from_payload  # noqa: E402
from uav_control.guidance.follow_problem import FollowProblem  # noqa: E402
from uav_control.guidance.follow_reference import (  # noqa: E402
    forecast_target_kinematics, forecast_viewpoint, viewpoint_state_batch,
)
from uav_control.guidance.p6_follow import P6Policy, assess_risk  # noqa: E402

MODES = ('linear', 'figure_eight', 'variable_speed_eight',
         'variable_speed_s', 'variable_speed_z')
CANDIDATES = tuple((d, b) for d in (4.5, 5., 6.)
                   for b in (-math.radians(20.), 0., math.radians(20.)))
CONFIG = AdaptiveViewpointConfig(candidate_distances=(4.5, 5., 6.),
                                 candidate_betas=(-math.radians(20.), 0., math.radians(20.)))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_safe(value):
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('Infinity' if value > 0. else '-Infinity')
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


def read_jsonl(path):
    with Path(path).open() as stream:
        for line in stream:
            yield json.loads(line)


def summarize_frozen(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[row['group']+'/'+row['pressure']].append(row)
    result = {}
    for key, values in sorted(groups.items()):
        metrics = [r['truth_follow_rmse'] for r in values
                   if r.get('valid') and r.get('truth_follow_rmse') is not None]
        result[key] = dict(
            attempts=len(values), nominal_valid=sum(bool(r.get('valid')) for r in values),
            robust_pass=sum(bool(r.get('risk', {}).get('robust_ok')) for r in values),
            fallback=sum(bool(r.get('risk', {}).get('used_nominal_fallback')) for r in values),
            statuses=dict(Counter(r.get('status', 'MISSING') for r in values)),
            conditional_metric_denominator=len(metrics),
            truth_follow_rmse_mean=float(np.mean(metrics)) if metrics else None)
    return result


def score_components(candidates, risks, accelerations, jerks, config=CONFIG):
    """Decompose the existing selector score from its stationary 5m/zero incumbent."""
    result = []
    for (d, beta), risk, acceleration, jerk in zip(candidates, risks, accelerations, jerks):
        incumbent = d == 5. and beta == 0.
        terms = dict(
            risk=config.risk_weight*risk,
            preferred=config.follow_error_weight*((d-5.)**2+2*d*5.*(1.-math.cos(beta))),
            extra_error=config.follow_error_weight*((d-5.)**2+beta**2),
            smooth=config.smooth_weight*((d-5.)**2+25.*beta**2),
            switch=config.switch_weight*(not incumbent),
            acceleration=config.acceleration_weight*acceleration,
            jerk=config.jerk_weight*jerk)
        terms['total'] = sum(terms.values())
        terms['risk_gain_needed_with_hysteresis'] = (
            terms['total']-terms['risk']+config.hysteresis_cost if not incumbent else 0.)
        result.append(terms)
    return result


def transition_profile(candidate):
    """Force one existing bounded transition for diagnosis, preserving all slew bounds."""
    selector = AdaptiveViewpointSelector(CONFIG)
    risks = np.full(len(selector.candidates), math.inf)
    risks[selector.candidates.index(candidate)] = 0.
    report = selector.select(.01, risks)
    if not report.valid:
        raise ValueError(report.reason)
    return selector.reference_profile()


def preparation_request(saved):
    """Reconstruct this frozen runner's pre-future_request boundary; offline only."""
    values = {name: getattr(saved, name) for name in MpcRequest.__dataclass_fields__}
    values.update(context=replace(saved.context,
                                  execution_start_stamp=saved.context.prediction_source_stamp),
                  state=saved.measurement_state, viewpoint_profile=())
    return MpcRequest(**values)


def risk_summary(report, detail, relative_times):
    """Retain fused and every conditional branch, including minor probability failures."""
    if report is None or not report.valid_input:
        return dict(robust_ok=False, valid_input=False, detail=detail, branches=[])
    checked = list(detail['checked_indices'])
    branches = []
    sets = [('fused', None, report.samples)] + [
        (str(branch.model_index), branch.probabilities, branch.samples)
        for branch in report.model_reports]
    failures = []
    for identity, probability, samples in sets:
        first = next((relative_times[checked[i]] for i, sample in enumerate(samples)
                      if not sample.robust_ok), None)
        if first is not None:
            failures.append(float(first))
        branches.append(dict(
            branch=identity, model_probability=probability,
            robust_ok=all(sample.robust_ok for sample in samples),
            nominal_ok=all(sample.nominal_ok for sample in samples),
            first_failure_seconds=first,
            minimum_horizontal_margin=min(s.horizontal_margin_rad for s in samples),
            minimum_vertical_margin=min(s.vertical_margin_rad for s in samples),
            samples=[asdict(s) for s in samples]))
    return dict(robust_ok=all(b['robust_ok'] for b in branches), valid_input=True,
                first_failure_seconds=min(failures) if failures else None,
                checked_seconds=[float(relative_times[i]) for i in checked],
                detail=detail, branches=branches)


def coverage_by_horizon(rows, calibration, family):
    """Evaluate already train-fitted per-horizon scales on untouched heldout residuals."""
    scales = {r['horizon']: r['covariance_scale'] for r in calibration['per_horizon']}
    distances = defaultdict(list)
    for row in rows:
        if (row.get('backend') != family.upper() or not row.get('valid')
                or row.get('comparison') != 'complete_chain'):
            continue
        error = np.asarray(row['error'])
        value = float(error @ np.linalg.solve(np.asarray(row['covariance']), error))
        distances[row['horizon']].append(value)
    return [dict(horizon=h, samples=len(values), horizon_scale=scales[h],
                 global_scale=calibration['covariance_scale'],
                 heldout_coverage_horizon_scale=float(np.mean(
                     np.asarray(values) <= calibration['empirical_scale']**2*scales[h])),
                 heldout_coverage_global_scale=float(np.mean(
                     np.asarray(values) <= calibration['empirical_scale']**2
                     * calibration['covariance_scale'])))
            for h, values in sorted(distances.items())]


def evaluate_reference(request, model, candidate, transition, times):
    profile = transition_profile(candidate) if transition else None
    problem = FollowProblem(replace(request, viewpoint_profile=(
        profile['duration'], profile['elapsed'], profile['distance_coefficients'],
        profile['beta_coefficients'])) if profile else request, model, 1.6)
    targets, _ = problem.target_state(times)
    if transition:
        samples = [forecast_viewpoint(problem, t) for t in times]
        positions = np.asarray([s[0] for s in samples])
        accelerations = np.asarray([s[2] for s in samples])
    else:
        states = [viewpoint_state_batch(*forecast_target_kinematics(problem, t)[:6],
                                        np.array((candidate[0],)), np.array((candidate[1],)),
                                        model.config.flight_altitude) for t in times]
        positions = np.asarray([s[0][0] for s in states])
        accelerations = np.asarray([s[2][0] for s in states])
    yaw = np.arctan2(targets[:, 1]-positions[:, 1], targets[:, 0]-positions[:, 0])
    attitudes = attitude_batch(accelerations, yaw, model.attitude_config)
    rotations = np.asarray([s.rotation_frd_to_ned for s in attitudes])
    views = visibility_batch(positions, rotations, targets, model.intrinsics,
                             model.extrinsics, model.target, model.visibility)
    query = times+request.context.execution_start_stamp-request.context.prediction_source_stamp
    report, detail = assess_risk(model, request, query, positions, rotations, targets)
    margins = np.asarray([min(v.horizontal_margin_rad, v.vertical_margin_rad) for v in views])
    if report is not None and report.valid_input:
        sets = (report.samples, *(branch.samples for branch in report.model_reports))
        margins[np.asarray(detail['checked_indices'])] = np.min([
            [min(s.horizontal_margin_rad, s.vertical_margin_rad) for s in samples]
            for samples in sets], axis=0)
    return dict(candidate=candidate, transition=transition, profile=profile,
                nominal_geometry_ok=all(v.whole_target_safe for v in views),
                risk=risk_summary(report, detail, times),
                minimum_scoring_margin=float(min(margins)),
                risk_penalty=float(max(0., .12-min(margins))**2*100.),
                acceleration_penalty=float(np.mean(np.sum(accelerations**2, axis=1))),
                jerk_penalty=float(np.mean(np.sum((np.diff(accelerations, axis=0)
                                                  / np.diff(times)[:, None])**2, axis=1))),
                executable_minco_checked=False, actual_attitude_checked=False)


def probe_row(row, protocol):
    request = request_from_payload(row['request'])
    model = FollowMpcSeed(MpcConfig(**protocol['model_config']))
    model.p6_policy = P6Policy(**dict(row['policy'], risk_horizon_seconds=.45))
    current_times = np.array((0., .225, .45, .8, 1.2))
    raw = preparation_request(request)
    static = [evaluate_reference(raw, model, c, False, current_times) for c in CANDIDATES]
    execution_aligned = [evaluate_reference(request, model, c, False, current_times)
                         for c in CANDIDATES]
    risks = [s['risk_penalty'] if s['nominal_geometry_ok'] else math.inf for s in static]
    costs = score_components(CANDIDATES, risks, [s['acceleration_penalty'] for s in static],
                             [s['jerk_penalty'] for s in static])
    selector = AdaptiveViewpointSelector(CONFIG)
    selection = selector.select(.01, risks,
                                candidate_errors=[(d-5.)**2+b**2 for d, b in CANDIDATES],
                                candidate_accelerations=[s['acceleration_penalty']
                                                         for s in static],
                                candidate_jerks=[s['jerk_penalty'] for s in static])
    for evidence, components in zip(static, costs):
        evidence['score_components'] = components
    transitions = [evaluate_reference(request, model, c, True, np.linspace(0., 1.6, 33))
                   for c in CANDIDATES]
    return dict(mode=row['mode'], event=row['plan_event'], pressure=row['pressure'],
                family=row['family'], input_sha256=row['input_sha256'],
                common_boundary_sha256=row['common_boundary_sha256'],
                selection=asdict(selection), static_candidates=static,
                execution_aligned_static_candidates=execution_aligned,
                bounded_transition_references=transitions,
                prepare_execution_start=raw.context.execution_start_stamp,
                future_execution_start=request.context.execution_start_stamp,
                incumbent_assumption='Stationary 5m/zero beta; existing history not rerun.',
                note='Ideal references only. Current prepare static ranking, execution-aligned '
                     'static geometry and new bounded transition evaluated separately; '
                     'no vehicle response, MINCO solve or Receiver acceptance.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ablation', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    rows = list(read_jsonl(args.ablation/'attempts.jsonl'))
    protocol = json.loads((args.ablation/'protocol.json').read_text())
    calibrations = {family: json.loads((args.calibration/f'{family}_calibration.json').read_text())
                    for family in ('bctra', 'imm')}
    sources = [Path(__file__), WORKSPACE/'src/uav_control/test/test_p6_visibility_effect_study.py',
               args.ablation/'attempts.jsonl', args.ablation/'protocol.json',
               args.replay/'analysis.json', args.calibration/'heldout_forecasts.jsonl',
               *(args.calibration/f'{family}_calibration.json' for family in calibrations)]
    sources += list((WORKSPACE/'src/uav_control/uav_control/guidance').glob('*.py'))
    sources += list((WORKSPACE/'src/uav_control/uav_control/controllers').glob('*.py'))
    before = {str(p): sha256(p) for p in sources}
    for p in sources:
        if p.is_relative_to(WORKSPACE) and 'data/experiments' not in str(p):
            target = args.output/'source_snapshot'/p.relative_to(WORKSPACE)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, target)
    preregistration = dict(
        schema='p61-visibility-effect-preregistration-v1', synthetic_only=True,
        event=24., families=['bctra', 'imm'], normal_modes=MODES,
        additional_existing_pressures=dict(family='imm', mode='figure_eight',
                                           pressures=['lateral_2m', 'yaw_0p25']),
        candidates=CANDIDATES, maximum_reference_windows=12*9*3,
        covariance_modified=False, conditional_models_dropped=False, solver_rerun=False,
        risk_horizon_seconds=.45, nominal_full_horizon_seconds=1.6,
        coverage_study='Existing train-fitted discrete horizon scales; untouched heldout '
                       'residuals. No interpolated schedule or geometric success claimed.',
        source_sha256=before,
        qualification_gate='Nominal supply and matched F2/F3 first; then real RGB-D calibration, '
                           'shadow and independent Receiver qualification before A/B/C/D SITL.')
    (args.output/'protocol.json').write_text(json.dumps(json_safe(preregistration), indent=2)+'\n')
    selected = [row for row in rows if row['plan_event'] == 24.
                and ((row['pressure'] == 'normal' and row['group'] in ('C', 'D_lease_prefix'))
                     or (row['mode'] == 'figure_eight' and row['group'] == 'D'
                         and row['pressure'] in ('lateral_2m', 'yaw_0p25')))]
    probes = [probe_row(row, protocol) for row in selected]
    heldout = list(read_jsonl(args.calibration/'heldout_forecasts.jsonl'))
    coverage = {f: coverage_by_horizon(heldout, c, f) for f, c in calibrations.items()}
    fixed = {(r['mode'], r['plan_event'], r['pressure']): r for r in rows
             if r['group'] == 'D_fixed_profile'}
    prefix = [r for r in rows if r['group'] == 'D_lease_prefix']
    equality = [r.get('curve_coefficients') == fixed[(r['mode'], r['plan_event'],
                r['pressure'])].get('curve_coefficients') for r in prefix]
    replay = json.loads((args.replay/'analysis.json').read_text())
    summary = dict(schema='p61-visibility-effect-study-v1', synthetic_only=True,
                   frozen=summarize_frozen(rows), replay_summary=replay['summary'],
                   fixed_prefix_identical_curves=sum(equality), fixed_prefix_pairs=len(equality),
                   horizon_coverage=coverage, probe_count=len(probes),
                   source_changed_during_run=[p for p, h in before.items() if sha256(p) != h],
                   qualifications=dict(actual_control=False, full_curve_robust=False,
                                       adaptive_effect=False, real_calibration=False))
    for name, value in [('analysis.json', summary), ('candidate_probes.json', probes)]:
        (args.output/name).write_text(json.dumps(json_safe(value), indent=2)+'\n')
    artifacts = sorted(p for p in args.output.rglob('*') if p.is_file())
    (args.output/'SHA256SUMS').write_text(''.join(
        f'{sha256(p)}  {p.relative_to(args.output)}\n' for p in artifacts))
    print(json.dumps(dict(output=str(args.output), probe_count=len(probes),
                          source_changed=summary['source_changed_during_run'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
