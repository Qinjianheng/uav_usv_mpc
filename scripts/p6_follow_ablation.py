#!/usr/bin/env python3
"""Matched synthetic-only P47 A/B/C/D study with retained failures and stress cases."""

import argparse
from collections import Counter, defaultdict
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]
CONTROL = WORKSPACE / 'src/uav_control/uav_control'
sys.path.insert(0, str(WORKSPACE / 'src/uav_control'))

from p6_prediction_study import load_parameters  # noqa: E402
from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco  # noqa: E402
from uav_control.controllers.follow_mpc_seed import (  # noqa: E402
    FollowMpcSeed, MpcConfig, MpcRequest, PlanningContext,
)
from uav_control.guidance.camera_visibility_batch import (  # noqa: E402
    attitude_batch, visibility_batch,
)
from uav_control.guidance.follow_fast_validation import fields  # noqa: E402
from uav_control.guidance.follow_problem import FollowProblem, future_request  # noqa: E402
from uav_control.guidance.follow_terminal_correction import linear_geometry  # noqa: E402
from uav_control.guidance.p6_follow import (  # noqa: E402
    P6Policy, P6Runtime, assess_risk, load_calibration,
)
from uav_control.tracking.constant_velocity_kalman import (  # noqa: E402
    ConstantVelocityKalmanFilter,
)
from uav_control.tracking.imm_predictor import ImmConfig, ImmPredictor  # noqa: E402
from uav_control.tracking.maneuvering_target_predictor import (  # noqa: E402
    ManeuveringTargetPredictor,
)
from uav_control.usv_motion_profiles import MOTION_MODES, UsvMotionProfile  # noqa: E402


DEFAULT_SEED = 1913
DEFAULT_EPOCHS = (8., 16., 24., 40., 56.)
HORIZON = 1.6
FORECAST_TIMES = np.arange(41)*.1
LOOKBACK = 1.6
PRESSURES = dict(normal=(0., 0.), lateral_2m=(2., 0.), yaw_0p25=(0., .25))
GROUPS = dict(
    A=('bctra', 'nominal', False, 'full'),
    B=('imm', 'nominal', False, 'full'),
    C=('bctra', 'robust', True, 'full'),
    D=('imm', 'robust', True, 'full'),
    D_no_covariance=('imm', 'robust', True, 'without_covariance'),
    D_no_models=('imm', 'robust', True, 'without_models'),
    D_no_yaw=('imm', 'robust', True, 'without_yaw'),
    D_no_adaptive=('imm', 'robust', False, 'without_adaptive'),
    C_lease_prefix=('bctra', 'robust', True, 'lease_prefix'),
    D_lease_prefix=('imm', 'robust', True, 'lease_prefix'),
    D_fixed_profile=('imm', 'robust', False, 'fixed_profile'))


def json_safe(value):
    """Keep nonfinite diagnostic evidence as explicit text instead of JSON NaN."""
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else 'Infinity' if value > 0 else '-Infinity'
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def digest(value):
    """Bind explicit immutable study inputs without timestamps invented by receipt."""
    data = json.dumps(json_safe(value), sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(data.encode()).hexdigest()


def validate_evaluation_seed(seed, calibrations):
    """Keep both calibration and heldout random seeds outside this study."""
    for evidence in calibrations:
        for split in ('source_provenance', 'heldout_provenance'):
            if seed == evidence.get(split, {}).get('seed'):
                raise ValueError('EVALUATION_SEED_OVERLAPS_CALIBRATION')


def robust_claim_eligible(component):
    """Exclude removed uncertainty/model/error terms from full robust qualification."""
    return component in ('full', 'without_adaptive', 'lease_prefix', 'fixed_profile')


def remove_component(request, component):
    """Explicit study interventions; never relabel IMM to bypass missing-model guards."""
    if component == 'without_covariance':
        covariance = np.zeros_like(np.asarray(request.target_covariances, dtype=float))
        conditional = np.zeros_like(np.asarray(request.model_covariances, dtype=float))
        return replace(request, target_covariances=tuple(map(tuple, covariance)),
                       model_covariances=tuple(conditional.tolist()))
    if component == 'without_models':
        return replace(request, model_positions=(), model_covariances=(), model_probabilities=())
    if component == 'fixed_profile':
        return replace(request, viewpoint_profile=(0., 0., (5., 0., 0., 0., 0., 0.), (0.,)*6))
    return request


def _truth_at(positions, velocities, times, query):
    query = np.atleast_1d(query)
    return tuple(np.column_stack([np.interp(query, times, field[:, i]) for i in range(3)])
                 for field in (positions, velocities))


def matched_snapshots(mode, seed, epochs, config_path):
    """
    Feed identical causal raw positions to KF+BCTRA and raw-position IMM.

    Synthetic ground truth creates measurements and later scores predictions; it
    never enters an estimator update or planner request. Each case has a 1.6 s
    causal adaptive-reference warmup at real 5 Hz epochs, with a common virtual
    UAV boundary from the causal KF estimate. This is snapshot planning, not a
    vehicle or tracking closed-loop simulation.
    """
    bctra_parameters, kf_parameters, motion = load_parameters(config_path)
    dt, duration = .05, max(epochs)+4.2
    profile = UsvMotionProfile(mode, motion)
    truth = [profile.state()] + [profile.advance(dt) for _ in range(round(duration/dt))]
    ground_p, ground_v = (np.array([getattr(s, key) for s in truth])
                          for key in ('position', 'velocity'))
    truth_times = np.arange(len(truth))*dt
    wanted = {round((event+offset)/dt) for event in epochs
              for offset in np.arange(-LOOKBACK, .0001, .2)}
    rng = np.random.default_rng(seed)
    covariance = np.diag((.1**2, .1**2, .05**2))
    kf, bctra = ConstantVelocityKalmanFilter(**kf_parameters), ManeuveringTargetPredictor(
        **bctra_parameters)
    imm, previous = ImmPredictor(), None
    records, snapshots, history = [], {}, hashlib.sha256()
    for index in range(round(max(epochs)/dt)+1):
        elapsed, stamp = index*dt, 10.+index*dt
        measurement = ground_p[index]+rng.normal(0., (.1, .1, .05))
        dropped = bool(rng.uniform() < .02)
        record = dict(mode=mode, seed=seed, observation_stamp=stamp, position=measurement.tolist(),
                      covariance=covariance.tolist(), valid=not dropped)
        records.append(record)
        history.update(json.dumps(record, sort_keys=True).encode())
        if dropped:
            if index in wanted:
                snapshots[index] = dict(valid=False, reason='SYNTHETIC_DROPPED_FRAME',
                                        elapsed=elapsed, input_sha256=history.hexdigest())
            continue
        if previous is None:
            kf.initialize(measurement, covariance)
        else:
            kf.predict(stamp-previous)
            kf.update(measurement, covariance)
        previous = stamp
        bctra.update_velocity(kf.state[3], kf.state[4], stamp)
        bctra.update_vertical(kf.state[2], kf.state[5], stamp)
        status = imm.update(stamp, measurement, covariance)
        if not status.accepted:
            raise ValueError('VALID_SYNTHETIC_OBSERVATION_REJECTED:'+status.reason)
        if index not in wanted:
            continue
        age, source = .02, stamp+.02
        common, _ = kf.project(age)
        started = time.perf_counter()
        bf = np.asarray(bctra.predict_many(*common[:3], *common[3:], FORECAST_TIMES))
        btime = time.perf_counter()-started
        started = time.perf_counter()
        imf = imm.forecast_at(source, FORECAST_TIMES)
        itime = time.perf_counter()-started
        # Matched groups share delivery after both forecasts are ready. CPU time
        # is counted in freshness and never removed by freezing a wall clock.
        delivery = max(btime, itime)
        snapshots[index] = dict(
            valid=True, elapsed=elapsed, source_stamp=source, observation_stamp=stamp,
            now_stamp=source+delivery, delivery_delay=delivery, common_state=common.tolist(),
            input_sha256=history.hexdigest(), stage=truth[index].stage,
            bctra=dict(positions=bf[:, :3].tolist(), velocities=bf[:, 3:].tolist(),
                       covariances=np.array([np.eye(3)*(.02+.1*t)**2
                                             for t in FORECAST_TIMES]).tolist(),
                       forecast_cpu_ms=btime*1000.),
            imm=dict(positions=imf.position.tolist(), velocities=imf.velocity.tolist(),
                     covariances=imf.position_covariance.tolist(),
                     model_positions=imf.conditional_position.tolist(),
                     model_covariances=imf.conditional_position_covariance.tolist(),
                     model_probabilities=imf.model_probabilities.tolist(),
                     forecast_cpu_ms=itime*1000.))
    return records, snapshots, (ground_p, ground_v, truth_times), dict(
        bctra=bctra_parameters, kf=kf_parameters, motion=asdict(motion),
        imm=json_safe(asdict(ImmConfig())))


def snapshot_request(snapshot, family, pressure, sequence):
    """Use a common synthetic UAV boundary and the unchanged acquired prediction epochs."""
    estimate = np.asarray(snapshot['common_state'])
    heading = math.atan2(estimate[4], estimate[3])
    rear = np.array((math.cos(heading), math.sin(heading)))
    side = np.array((-rear[1], rear[0]))
    lateral, yaw_offset = PRESSURES[pressure]
    position = estimate[:3].copy()
    position[:2] += -5.*rear + lateral*side
    position[2] = -5.
    yaw = heading+yaw_offset
    rotation = ((math.cos(yaw), -math.sin(yaw), 0.),
                (math.sin(yaw), math.cos(yaw), 0.), (0., 0., 1.))
    source, observation = snapshot['source_stamp'], snapshot['observation_stamp']
    context = PlanningContext(1, sequence, source, source, source, source, observation,
                              sequence, observation+.125)
    forecast = snapshot[family]
    request = MpcRequest(
        context, (*position, estimate[3], estimate[4], 0., 0., 0., 0., yaw),
        tuple(FORECAST_TIMES), tuple(map(tuple, forecast['positions'])),
        tuple(map(tuple, forecast['velocities'])), rotation, snapshot['now_stamp'],
        target_covariances=tuple(np.asarray(forecast['covariances']).reshape(-1, 9).tolist()),
        model='IMM_CV_CTRV_CTRA' if family == 'imm' else 'BCTRA_BOUNDED_Z')
    if family == 'imm':
        request = replace(request, model_positions=tuple(forecast['model_positions']),
                          model_covariances=tuple(forecast['model_covariances']),
                          model_probabilities=tuple(forecast['model_probabilities']),
                          uncertainty_identity=(1, sequence, source, observation))
    return request


def _finite_number(value):
    return float(value) if value is not None and math.isfinite(value) else None


def evaluate_curve(result, request, model, snapshot, ground):
    """Score ideal MINCO coefficients against hidden truth only after planning."""
    metrics = result.metrics
    if not all(key in metrics for key in ('xyz_coefficients', 'yaw_coefficients', 'durations')):
        return {}
    query = np.linspace(0., HORIZON, 33)
    xyz, duration = np.asarray(metrics['xyz_coefficients']), np.asarray(metrics['durations'])
    yaw = np.asarray(metrics['yaw_coefficients'])[::-1].T
    p, v, a, j, angles, rates = fields(xyz, duration, yaw, query)
    problem = FollowProblem(request, model, HORIZON)
    reference_p, _, _ = problem.reference(query)
    elapsed = (snapshot['elapsed']+.02+request.context.execution_start_stamp
               - request.context.prediction_source_stamp)
    truth_p, truth_v = _truth_at(*ground, elapsed+query)
    heading = np.arctan2(truth_v[:, 1], truth_v[:, 0])
    truth_reference = truth_p.copy()
    truth_reference[:, :2] -= 5.*np.c_[np.cos(heading), np.sin(heading)]
    truth_reference[:, 2] = -5.
    attitudes = attitude_batch(a, angles, model.attitude_config)
    rotations = np.asarray([x.rotation_frd_to_ned if x.valid else np.full((3, 3), np.nan)
                            for x in attitudes])
    views = visibility_batch(p, rotations, truth_p, model.intrinsics, model.extrinsics,
                             model.target, model.visibility)
    targets, _ = problem.target_state(query)
    _, detail = assess_risk(model, request, query+request.context.execution_start_stamp
                            - request.context.prediction_source_stamp, p, rotations, targets)
    minimum_h = min((r.horizontal_margin_rad for r in views
                     if r.horizontal_margin_rad is not None), default=-math.pi)
    minimum_v = min((r.vertical_margin_rad for r in views
                     if r.vertical_margin_rad is not None), default=-math.pi)
    return dict(
        reference_rmse=float(np.sqrt(np.mean(np.sum((p[:, :2]-reference_p[:, :2])**2, axis=1)))),
        truth_follow_rmse=float(np.sqrt(np.mean(np.sum(
            (p[:, :2]-truth_reference[:, :2])**2, axis=1)))),
        actual_target_visible_fraction=float(np.mean([r.whole_target_safe for r in views])),
        truth_minimum_horizontal_margin=_finite_number(minimum_h),
        truth_minimum_vertical_margin=_finite_number(minimum_v),
        maximum_horizontal_acceleration=float(np.linalg.norm(a[:, :2], axis=1).max()),
        maximum_horizontal_jerk=float(np.linalg.norm(j[:, :2], axis=1).max()),
        maximum_yaw_rate=float(np.abs(rates).max()), risk=detail,
        curve_coefficients=dict(xyz=xyz.tolist(), yaw=yaw.tolist(), durations=duration.tolist()))


def run_case(group, mode, event, pressure, snapshots, ground, policies):
    """Attempt one matched real implementation solve; retain every failed warmup and solve."""
    family, visibility, adaptive, component = GROUPS[group]
    policy = replace(policies[family], visibility_mode=visibility, adaptive_viewpoint=adaptive)
    risk_horizon = .45 if component == 'lease_prefix' else 0.
    if hasattr(policy, 'risk_horizon_seconds'):
        policy = replace(policy, risk_horizon_seconds=risk_horizon)
    elif risk_horizon:
        raise ValueError('LEASE_PREFIX_POLICY_NOT_IMPLEMENTED')
    if component == 'without_yaw':
        policy = replace(policy, yaw_error_bound_rad=0., attitude_error_bound_rad=0.)
    policy.validate()
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    model.p6_policy = policy
    runtime = P6Runtime(model)
    timer = [0., 0.]
    solver = P47FollowGuidedMinco(
        model, duration=HORIZON, rolling=False, online_budget_guard=True,
        wall_clock=lambda: timer[0]+time.perf_counter()-timer[1])
    warm_errors, prepared, started = [], None, None
    indices = [round((event+offset)/.05) for offset in np.arange(-LOOKBACK, .0001, .2)]
    final = snapshots[indices[-1]]
    row = dict(group=group, mode=mode, plan_event=event, pressure=pressure, family=family,
               component=component, robust_claim_eligible=robust_claim_eligible(component),
               risk_horizon_seconds=risk_horizon,
               full_curve_robust_claim_eligible=component in (
                   'full', 'without_adaptive', 'fixed_profile'),
               valid=False, status='NOT_ATTEMPTED', input_sha256=final['input_sha256'],
               accepted_by_tracker=False, synthetic_only=True)
    if not final['valid']:
        return dict(row, status=final['reason'], total_ms=0., warmup_errors=[])
    common_boundary = snapshot_request(final, family, pressure, indices[-1]).state
    row['common_boundary_sha256'] = digest(common_boundary)
    for index in indices:
        snapshot = snapshots[index]
        if not snapshot['valid']:
            warm_errors.append(dict(epoch=snapshot['elapsed'], reason=snapshot['reason']))
            continue
        raw = remove_component(snapshot_request(snapshot, family, pressure, index), component)
        if index == indices[-1]:
            started = time.perf_counter()
            timer[:] = [raw.now_stamp, started]
        try:
            prepared = runtime.prepare(raw)
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            warm_errors.append(dict(epoch=snapshot['elapsed'], reason=str(error)))
            if index == indices[-1]:
                return dict(row, status=str(error), total_ms=(time.perf_counter()-started)*1000.,
                            warmup_errors=warm_errors)
    prepare_time = time.perf_counter()-started
    prepared = future_request(prepared, .15)
    result = solver.solve(prepared)
    total = time.perf_counter()-started
    row.update(valid=result.valid, status=result.solver_status, prepare_ms=prepare_time*1000.,
               solve_ms=result.solve_time*1000., total_ms=total*1000.,
               delivery_cpu_delay_ms=final['delivery_delay']*1000., warmup_errors=warm_errors,
               request=json_safe(asdict(prepared)), policy=json_safe(asdict(policy)),
               metrics=json_safe(result.metrics),
               violations=json_safe(result.constraint_violations))
    row.update(evaluate_curve(result, prepared, model, final, ground))
    return json_safe(row)


def summarize(rows):
    """Include every attempt; ideal curve metrics are explicitly conditional on admission."""
    groups = defaultdict(list)
    for row in rows:
        groups[(row['mode'], row['pressure'], row['group'])].append(row)
    result = []
    for (mode, pressure, group), values in sorted(groups.items()):
        valid = [r for r in values if r['valid']]
        item = dict(mode=mode, pressure=pressure, group=group, attempts=len(values),
                    valid_count=len(valid), rejection_fraction=1.-len(valid)/len(values),
                    status_counts=dict(Counter(r['status'] for r in values)),
                    conditional_metrics_denominator=len(valid),
                    total_ms_median=float(np.median([r['total_ms'] for r in values])),
                    total_ms_p95=float(np.percentile([r['total_ms'] for r in values], 95)))
        for metric in ('truth_follow_rmse', 'reference_rmse', 'actual_target_visible_fraction',
                       'truth_minimum_horizontal_margin', 'truth_minimum_vertical_margin',
                       'maximum_horizontal_acceleration', 'maximum_horizontal_jerk'):
            measured = [r[metric] for r in valid if isinstance(r.get(metric), (float, int))]
            if measured:
                item[metric+'_mean'] = float(np.mean(measured))
        result.append(item)
    return result


def write_jsonl(path, values):
    """Write explicit finite JSON while retaining rejected attempts."""
    with path.open('w') as output:
        for value in values:
            output.write(json.dumps(json_safe(value), allow_nan=False, separators=(',', ':'))+'\n')


def main(argv=None):
    """Run the fixed matched protocol and save full inputs, rejected cases and hashes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=WORKSPACE /
                        'data/experiments/20261014_p6_development_01/visibility/ablation_01')
    parser.add_argument('--calibration-directory', type=Path, default=WORKSPACE /
                        'data/experiments/20261014_p6_development_01/prediction_04')
    parser.add_argument('--config', type=Path, default=WORKSPACE /
                        'src/uav_usv_bringup/config/baseline.yaml')
    parser.add_argument('--modes', nargs='+', choices=tuple(MOTION_MODES.values()),
                        default=tuple(MOTION_MODES.values()))
    parser.add_argument('--epochs', nargs='+', type=float, default=DEFAULT_EPOCHS)
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED)
    parser.add_argument('--groups', nargs='+', choices=tuple(GROUPS),
                        default=tuple(g for g, (_, _, _, component) in GROUPS.items()
                                      if component not in ('lease_prefix', 'fixed_profile')))
    parser.add_argument('--include-lease-prefix', action='store_true',
                        help='Add C/D .45s risk scope on identical normal snapshots (50 rows).')
    parser.add_argument('--include-fixed-profile', action='store_true',
                        help='Add fixed5m/beta0 analytic reference on normal snapshots (25 rows).')
    args = parser.parse_args(argv)
    if args.include_lease_prefix:
        extra = [g for g in ('C_lease_prefix', 'D_lease_prefix') if g not in args.groups]
        args.groups = list(args.groups)+extra
    if args.include_fixed_profile and 'D_fixed_profile' not in args.groups:
        args.groups = list(args.groups)+['D_fixed_profile']
    if min(args.epochs) < 2. or max(args.epochs) > 60. or not np.all(np.isfinite(args.epochs)):
        parser.error('finite preregistered epochs must lie in 2..60 seconds')
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('output already contains evidence; choose a new directory')
    policies, evidence = {}, []
    for family in ('bctra', 'imm'):
        path = args.calibration_directory/(family+'_calibration.json')
        evidence.append(json.loads(path.read_text()))
        policies[family] = load_calibration(P6Policy(
            visibility_mode='robust', calibration_file=str(path)))
    validate_evaluation_seed(args.seed, evidence)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'runner_source.py').write_bytes(Path(__file__).read_bytes())
    # Target-independent numerical map warmed equally before every population.
    linear_geometry((HORIZON/3,)*3, 'qva')
    protocol = dict(
        schema='p6-offline-ablation-v1', seed=args.seed, modes=args.modes, epochs=args.epochs,
        groups=GROUPS, selected_groups=args.groups, pressures=PRESSURES,
        source='synthetic_iid_raw_position', observation_dt=.05, dropout_probability=.02,
        adaptive_lookback=LOOKBACK, adaptive_update_dt=.2, horizon=HORIZON, execution_lead=.15,
        same_raw_input_and_common_uav_boundary=True, no_truth_in_predictors_or_planner=True,
        real_cpu_freshness_accounted=True, future_truth_used_only_after_solve=True,
        closed_loop=False, tracker_control_authority=False, probability_guarantee=False,
        calibration_scope='synthetic empirical envelopes; conditional minor-model scope limited',
        component_ablation_full_robust_claim=False,
        calibration_policies={family: asdict(policy) for family, policy in policies.items()},
        planner_config=dict(duration=HORIZON, rolling=False, online_budget_guard=True,
                            terminal_mode='qva', refinement=True, scheduling='adaptive',
                            fallback_enabled=True),
        virtual_uav_boundary='causal KF estimate minus 5m with declared side/yaw pressure',
        model_config=asdict(MpcConfig(allow_synthetic_predictions=True)),
        limitations=['Independent planning snapshots with causal reference warmups.',
                     'Ideal MINCO PVA and planned attitude; no PX4 response or vision image.',
                     'Matched synthetic sensor seeds, shared deterministic motion paths.',
                     'No actual visual dropout/servo ownership or continuous-time guarantee.',
                     'Missing IMM models intentionally trigger explicit nominal fallback.',
                     'CPU-dependent freshness failures retained, no retry or deadline extension.'],
        fingerprints={str(p.relative_to(WORKSPACE)): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in (Path(__file__), args.config,
                                CONTROL/'guidance/p6_follow.py',
                                CONTROL/'guidance/adaptive_viewpoint.py',
                                CONTROL/'guidance/robust_visibility.py',
                                CONTROL/'tracking/imm_predictor.py',
                                CONTROL/'tracking/constant_velocity_kalman.py',
                                CONTROL/'tracking/maneuvering_target_predictor.py')})
    (args.output/'protocol.json').write_text(json.dumps(json_safe(protocol), indent=2)+'\n')
    rows, all_observations = [], []
    with (args.output/'attempts.jsonl').open('w') as output:
        for mode in args.modes:
            records, snapshots, ground, parameters = matched_snapshots(
                mode, args.seed, args.epochs, args.config)
            all_observations.extend(records)
            write_jsonl(args.output/(mode+'_prediction_snapshots.jsonl'), snapshots.values())
            (args.output/(mode+'_parameters.json')).write_text(
                json.dumps(parameters, indent=2)+'\n')
            for event in args.epochs:
                for pressure in PRESSURES:
                    for group in args.groups:
                        if pressure != 'normal' and group not in ('A', 'B', 'C', 'D'):
                            continue
                        row = run_case(group, mode, event, pressure, snapshots, ground, policies)
                        rows.append(row)
                        output.write(json.dumps(json_safe(row), allow_nan=False)+'\n')
                        output.flush()
                print(f'{mode} epoch={event:g}s: {sum(r["valid"] for r in rows)}/{len(rows)} '
                      'admitted (offline only)', flush=True)
    write_jsonl(args.output/'raw_observations.jsonl', all_observations)
    report = dict(protocol=protocol, attempts=len(rows), valid_count=sum(r['valid'] for r in rows),
                  rejected_count=sum(not r['valid'] for r in rows), by_group=summarize(rows))
    (args.output/'summary.json').write_text(json.dumps(json_safe(report), indent=2)+'\n')
    artifact_paths = sorted(p for p in args.output.iterdir()
                            if p.is_file() and p.name != 'SHA256SUMS')
    (args.output/'SHA256SUMS').write_text(''.join(
        hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in artifact_paths))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
