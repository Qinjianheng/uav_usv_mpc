#!/usr/bin/env python3
"""Matched, synthetic-only prediction study with disjoint residual calibration."""

import argparse
import csv
from dataclasses import asdict, fields
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time

import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/uav_control'))

from uav_control.tracking.constant_velocity_kalman import (  # noqa: E402
    ConstantVelocityKalmanFilter)
from uav_control.tracking.imm_predictor import ImmConfig, ImmPredictor  # noqa: E402
from uav_control.tracking.imm_predictor import propagate_model  # noqa: E402
from uav_control.tracking.maneuvering_target_predictor import (  # noqa: E402
    ManeuveringTargetPredictor)
from uav_control.usv_motion_profiles import MotionParameters, UsvMotionProfile  # noqa: E402


HORIZONS = np.array((.2, .5, .8, 1.2, 1.6, 2.4, 4.))
MODES = ('linear', 'figure_eight', 'variable_speed_eight',
         'variable_speed_s', 'variable_speed_z')
CHI_SQUARED_95_2D = 5.991464547107979
CHI_SQUARED_95_3D = 7.814727903251179


def sha256(path):
    """Return a source/input fingerprint without changing the file."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(rows):
    """Count all attempts and keep severe valid residuals in every metric."""
    valid = [item for item in rows if item['valid']]
    result = dict(attempts=len(rows), valid_count=len(valid),
                  invalid_count=len(rows) - len(valid),
                  valid_fraction=len(valid) / max(len(rows), 1))
    if not valid:
        return result
    for key, label in (('horizontal_error', 'horizontal'), ('velocity_error', 'velocity')):
        values = np.array([item[key] for item in valid])
        result[label + '_rmse'] = float(np.sqrt(np.mean(values ** 2)))
        result[label + '_p95'] = float(np.percentile(values, 95))
    result['severe_over_5m'] = sum(item['horizontal_error'] > 5. for item in valid)
    normalized = [item['mahalanobis_squared'] for item in valid
                  if item['mahalanobis_squared'] is not None]
    result['nominal_95_coverage'] = float(np.mean(
        np.asarray(normalized) <= CHI_SQUARED_95_2D)) if normalized else None
    heading = [item['heading_error'] for item in valid if item['heading_error'] is not None]
    result['heading_rmse_rad'] = float(np.sqrt(np.mean(np.square(heading)))) if heading else None
    result['compute_ms'] = dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(
        [item['compute_time_ms'] for item in valid], (50, 95, 99)))))
    for name in ('initial_state_effect', 'model_increment', 'pure_model_increment',
                 'current_turn_rate_error'):
        values = [item[name] for item in valid if name in item and item[name] is not None]
        if values:
            result[name + '_rmse'] = float(np.sqrt(np.mean(np.square(values))))
    return result


def fit_calibration(calibration_rows, heldout_rows, family, provenance):
    """Fit only calibration residuals; heldout coverage never adjusts the scale."""
    if {item['seed'] for item in calibration_rows} & {item['seed'] for item in heldout_rows}:
        raise ValueError('calibration and heldout seeds must be disjoint')
    name = family.upper()
    train = [item for item in calibration_rows if item['backend'] == name and item['valid']
             and item['comparison'] == 'complete_chain']
    test = [item for item in heldout_rows if item['backend'] == name and item['valid']
            and item['comparison'] == 'complete_chain']
    if not train or not test:
        raise ValueError('both independent splits require valid residuals')

    def distances(item, conditional=False):
        error = np.asarray(item['error'], dtype=float)
        covariance = np.asarray(item['covariance'], dtype=float)
        if not conditional:
            return [float(error @ np.linalg.solve(covariance, error))]
        # IMM conditionals share the independent vertical filter; existing
        # evidence records conditional XY distances and the common z block.
        if np.max(np.abs(covariance[:2, 2])) > 1e-12:
            raise ValueError('conditional xyz reconstruction requires independent vertical block')
        vertical = float(error[2] ** 2 / covariance[2, 2])
        return [value + vertical for value in item.get('conditional_mahalanobis_squared', [])]

    scales = []
    for horizon in sorted({item['horizon'] for item in train}):
        group = [item for item in train if item['horizon'] == horizon]
        normalized = [distances(item)[0] for item in group]
        conditional = [value for item in group for value in distances(item, True)]
        quantile = float(np.quantile(normalized, .95))
        conditional_quantile = float(np.quantile(conditional, .95)) if conditional else quantile
        scales.append(dict(horizon=horizon, covariance_scale=max(
            1., quantile / CHI_SQUARED_95_3D, conditional_quantile / CHI_SQUARED_95_3D),
            calibration_samples=len(group), q95_mahalanobis_squared=quantile,
            q95_conditional_mahalanobis_squared=conditional_quantile))
    scale = max(item['covariance_scale'] for item in scales)

    def coverage(items, conditional=False):
        values = [value for item in items for value in distances(item, conditional)]
        return float(np.mean(np.asarray(values) <= CHI_SQUARED_95_3D * scale)) if values else None

    for item in scales:
        item['calibration_coverage'] = coverage([
            row for row in train if row['horizon'] == item['horizon']])
        item['heldout_coverage'] = coverage([
            row for row in test if row['horizon'] == item['horizon']])
    return dict(
        schema='p6-calibration-v1', id=f'p6_{family}_synthetic_{provenance["calibration_seed"]}',
        family=family, distribution='empirical_envelope', covariance_scale=scale,
        covariance_floor_m2=0., empirical_scale=math.sqrt(CHI_SQUARED_95_3D),
        probability_guarantee=False, calibration_domain='xyz',
        normalization_basis='chi_squared_95_3d_radius_used_as_empirical_reference_only',
        synthetic_only=True, suggested_policy='nominal_fallback',
        calibrated_horizons=[item['horizon'] for item in scales], per_horizon=scales,
        source_provenance=dict(provenance, split='calibration',
                               seed=provenance['calibration_seed']),
        heldout_provenance=dict(provenance, split='heldout', seed=provenance['heldout_seed']),
        coverage=dict(calibration=coverage(train), heldout=coverage(test),
                      calibration_samples=len(train), heldout_samples=len(test),
                      conditional_calibration=coverage(train, True),
                      conditional_heldout=coverage(test, True),
                      nominal_pointwise=.95, joint_trajectory_claim=False),
        limitations=[
            'Synthetic iid position noise; shared motion paths; disjoint noise seeds.',
            'Overlapping horizons are dependent; no iid confidence interval or joint guarantee.',
            'No attitude, depth bias, dropout bursts or real RGB-D residual calibration.',
            'BCTRA preset variance is an empirical envelope basis, not a propagated covariance.',
            'IMM conditional models with posterior probability >=.05 are included in calibration.',
            'No heldout value is used to fit noise, scale, floor, or model parameters.'])


def load_parameters(config_path):
    """Read actual baseline parameters instead of recopying algorithm defaults."""
    config = yaml.safe_load(Path(config_path).read_text())
    bctra_parameters = config['target_predictor_node']['ros__parameters']
    kf_parameters = config['target_kalman_filter']['ros__parameters']
    mapping = dict(turn_rate_filter_alpha='turn_rate_filter_alpha',
                   max_turn_rate='max_target_turn_rate',
                   maneuver_horizon='maneuver_prediction_horizon',
                   minimum_speed='minimum_target_speed', minimum_updates='minimum_turn_updates',
                   turn_acceleration_filter_alpha='turn_acceleration_filter_alpha',
                   max_turn_acceleration='max_target_turn_acceleration',
                   speed_acceleration_filter_alpha='speed_acceleration_filter_alpha',
                   max_longitudinal_acceleration='max_target_longitudinal_acceleration',
                   acceleration_decay_time='maneuver_acceleration_decay_time',
                   vertical_velocity_decay_time='vertical_velocity_decay_time',
                   maximum_vertical_displacement='maximum_vertical_displacement',
                   vertical_observation_margin='vertical_observation_margin',
                   vertical_history_window='vertical_history_window')
    predictor = {name: bctra_parameters[key] for name, key in mapping.items()}
    kf = {name: kf_parameters[name] for name in (
        'process_acceleration_std', 'measurement_position_std', 'initial_velocity_std')}
    motion_config = config.get('moving_target', {}).get('ros__parameters', {})
    motion = {item.name: motion_config[item.name] for item in fields(MotionParameters)
              if item.name in motion_config}
    for old, new in (('velocity_x', 'linear_vx'), ('velocity_y', 'linear_vy')):
        if old in motion_config:
            motion[new] = motion_config[old]
    return predictor, kf, MotionParameters(**motion)


def stage_labels(sample, next_samples):
    """Nonexclusive evaluation-only physical stage labels at the source epoch."""
    omega, alpha = sample.turn_rate, sample.angular_acceleration
    acceleration = sample.longitudinal_acceleration
    labels = [sample.stage]
    if abs(omega) < .01 and abs(acceleration) < .03:
        labels.append('uniform_straight')
    if abs(omega) >= .01 and abs(alpha) < .015:
        labels.append('constant_turn_rate')
    if abs(alpha) >= .015:
        labels.append('turn_rate_growth' if omega * alpha >= 0. else 'turn_rate_decay')
    if acceleration > .03:
        labels.append('longitudinal_acceleration')
    elif acceleration < -.03:
        labels.append('longitudinal_deceleration')
    if abs(omega) >= .01 and abs(acceleration) >= .03:
        labels.append('compound_speed_turn')
    if any(omega * item.turn_rate < 0. for item in next_samples) and abs(omega) > .005:
        labels.append('turn_reversal_within_1p6s')
    return '|'.join(sorted(set(labels)))


def heading_error(velocity, reference):
    """Return circular velocity-heading error only when both headings exist."""
    if min(np.linalg.norm(velocity[:2]), np.linalg.norm(reference[:2])) < .2:
        return None
    delta = math.atan2(velocity[1], velocity[0]) - math.atan2(reference[1], reference[0])
    return math.atan2(math.sin(delta), math.cos(delta))


def metric_row(seed, mode, backend, comparison, stamp, source, horizon, position,
               velocity, covariance, truth_position, truth_velocity, stage,
               compute_time_ms, conditional=None, **extras):
    """Build an independently recomputable row from explicit forecast and truth."""
    error = position - truth_position
    model_only = comparison == 'matched_initial_state_model_only'
    mahalanobis = (None if model_only else float(
        error[:2] @ np.linalg.solve(covariance[:2, :2], error[:2])))
    result = dict(seed=seed, mode=mode, backend=backend, comparison=comparison,
                  observation_stamp=stamp, source_stamp=source, input_age=source - stamp,
                  horizon=float(horizon), valid=True, stage=stage,
                  horizontal_error=float(np.linalg.norm(error[:2])),
                  velocity_error=float(np.linalg.norm(velocity[:2] - truth_velocity[:2])),
                  heading_error=heading_error(velocity, truth_velocity),
                  mahalanobis_squared=mahalanobis, compute_time_ms=compute_time_ms,
                  position=position.tolist(), velocity=velocity.tolist(),
                  truth_position=truth_position.tolist(), truth_velocity=truth_velocity.tolist(),
                  error=error.tolist(), covariance=None if model_only else covariance.tolist(),
                  conditional_mahalanobis_squared=conditional or [])
    result.update(extras)
    return result


def run_mode(mode, seed, duration, observation_dt, evaluation_dt, noise_std,
             predictor_parameters, kf_parameters, motion_parameters):
    """Feed only causal synthetic measurements to all complete-chain filters."""
    rng = np.random.default_rng(seed)
    profile = UsvMotionProfile(mode, motion_parameters)
    count = int(round((duration + HORIZONS[-1] + observation_dt) / observation_dt))
    truth = [profile.state()] + [profile.advance(observation_dt) for _ in range(count)]
    true_position = np.array([item.position for item in truth])
    true_velocity = np.array([item.velocity for item in truth])
    true_omega = np.array([item.turn_rate for item in truth])
    true_alpha = np.array([item.angular_acceleration for item in truth])
    true_longitudinal_acceleration = np.array([item.longitudinal_acceleration for item in truth])
    times = np.arange(len(truth)) * observation_dt

    def interpolate(values, epoch):
        epoch = np.atleast_1d(epoch)
        return np.stack([np.interp(epoch, times, values[:, axis]) for axis in range(3)], axis=-1)

    kf = ConstantVelocityKalmanFilter(**kf_parameters)
    bctra = ManeuveringTargetPredictor(**predictor_parameters)
    oracle_bctra = ManeuveringTargetPredictor(**predictor_parameters)
    filters = {'IMM': ImmPredictor()}
    for index, name in enumerate(('CV', 'CTRV', 'CTRA')):
        probability = np.zeros(3)
        probability[index] = 1.
        filters[name] = ImmPredictor(ImmConfig(
            transition_matrix=np.eye(3), initial_probabilities=probability))
    measurement_covariance = np.diag(np.asarray(noise_std) ** 2)
    last_observation = None
    observations, rows, probability_rows, timings = [], [], [], []
    stride = max(1, round(evaluation_dt / observation_dt))
    for index in range(int(round(duration / observation_dt)) + 1):
        elapsed = times[index]
        stamp = 10. + elapsed
        age = .02 + rng.uniform(0., .005)
        measurement = true_position[index] + rng.normal(0., noise_std)
        dropout = bool(rng.uniform() < .02)
        observations.append(dict(seed=seed, mode=mode, stamp=stamp, valid=not dropout,
                                 position=measurement.tolist(),
                                 covariance=measurement_covariance.tolist(),
                                 truth_position=true_position[index].tolist(),
                                 raw_horizontal_error=float(np.linalg.norm(
                                     measurement[:2] - true_position[index, :2])),
                                 age=age, stage=truth[index].stage))
        if dropout:
            if elapsed >= 2. and index % stride == 0:
                for name in ('BCTRA', 'IMM', 'CV', 'CTRV', 'CTRA'):
                    for horizon in HORIZONS:
                        rows.append(dict(seed=seed, mode=mode, backend=name,
                                         comparison='complete_chain', horizon=float(horizon),
                                         observation_stamp=stamp, source_stamp=stamp + age,
                                         valid=False, invalid_reason='SYNTHETIC_DROPPED_FRAME',
                                         stage=truth[index].stage))
            continue
        update_started = time.perf_counter()
        if last_observation is None:
            kf.initialize(measurement, measurement_covariance)
        else:
            kf.predict(stamp - last_observation)
            kf.update(measurement, measurement_covariance)
        last_observation = stamp
        bctra.update_velocity(kf.state[3], kf.state[4], stamp)
        bctra.update_vertical(kf.state[2], kf.state[5], stamp)
        timings.append(dict(seed=seed, mode=mode, backend='BCTRA_update',
                            milliseconds=(time.perf_counter() - update_started) * 1000.))
        for name, predictor in filters.items():
            started = time.perf_counter()
            status = predictor.update(stamp, measurement, measurement_covariance)
            if not status.accepted:
                raise RuntimeError(
                    f'{mode} {name} rejected valid synthetic observation: {status.reason}')
            timings.append(dict(seed=seed, mode=mode, backend=name + '_update',
                                milliseconds=(time.perf_counter() - started) * 1000.))
        observations[-1].update(
            kf_position=kf.state[:3].tolist(), kf_velocity=kf.state[3:].tolist(),
            kf_position_horizontal_error=float(np.linalg.norm(
                kf.state[:2] - true_position[index, :2])),
            kf_velocity_horizontal_error=float(np.linalg.norm(
                kf.state[3:5] - true_velocity[index, :2])))
        observations[-1]['posterior_projection_counts'] = {
            name: predictor.projection_count for name, predictor in filters.items()}
        if elapsed < 2. or index % stride != 0:
            continue
        source = stamp + age
        source_position = interpolate(true_position, elapsed + age)[0]
        source_velocity = interpolate(true_velocity, elapsed + age)[0]
        source_omega = float(np.interp(elapsed + age, times, true_omega))
        source_alpha = float(np.interp(elapsed + age, times, true_alpha))
        source_longitudinal_acceleration = float(np.interp(
            elapsed + age, times, true_longitudinal_acceleration))
        future_position = interpolate(true_position, elapsed + age + HORIZONS)
        future_velocity = interpolate(true_velocity, elapsed + age + HORIZONS)
        stage = stage_labels(truth[index], truth[index + 1:index + 33])
        projected, _ = kf.project(age)
        started = time.perf_counter()
        bctra_predicted = np.asarray(bctra.predict_many(*projected[:3], *projected[3:], HORIZONS))
        bctra_time = (time.perf_counter() - started) * 1000.
        # This branch is offline scoring only. It calls a pure forecast with
        # explicit true initial P/V; it never updates any online estimator.
        same_parameters_true_initial = np.asarray(bctra.predict_many(
            *source_position, *source_velocity, HORIZONS))
        oracle_bctra.turn_rate = source_omega
        oracle_bctra.turn_acceleration = source_alpha
        oracle_bctra.speed_acceleration = source_longitudinal_acceleration
        oracle_bctra.valid_turn_updates = oracle_bctra.minimum_updates
        oracle_prediction = np.asarray(oracle_bctra.predict_many(
            *source_position, *source_velocity, HORIZONS))
        for sample_index, horizon in enumerate(HORIZONS):
            covariance = np.eye(3) * (.02 + .1 * horizon) ** 2
            increment = (same_parameters_true_initial[sample_index, :2]
                         - future_position[sample_index, :2])
            initial_effect = (bctra_predicted[sample_index, :2]
                              - same_parameters_true_initial[sample_index, :2])
            rows.append(metric_row(
                seed, mode, 'BCTRA', 'complete_chain', stamp, source, horizon,
                bctra_predicted[sample_index, :3], bctra_predicted[sample_index, 3:], covariance,
                future_position[sample_index], future_velocity[sample_index], stage, bctra_time,
                initial_state_effect=float(np.linalg.norm(initial_effect)),
                model_increment=float(np.linalg.norm(increment)),
                pure_model_increment=float(np.linalg.norm(
                    oracle_prediction[sample_index, :2] - future_position[sample_index, :2])),
                initial_state_effect_vector=initial_effect.tolist(),
                model_increment_vector=increment.tolist(),
                current_turn_rate_error=bctra.turn_rate - source_omega))
            rows.append(metric_row(
                seed, mode, 'BCTRA', 'matched_initial_state_model_only', stamp, source, horizon,
                oracle_prediction[sample_index, :3], oracle_prediction[sample_index, 3:],
                covariance,
                future_position[sample_index], future_velocity[sample_index], stage, 0.))
        for name, predictor in filters.items():
            started = time.perf_counter()
            forecast = predictor.forecast_at(source, HORIZONS)
            forecast_time = (time.perf_counter() - started) * 1000.
            if name == 'IMM':
                probability_rows.append(dict(seed=seed, mode=mode, time=elapsed,
                                             source_stamp=source, observation_stamp=stamp,
                                             model_probabilities=(
                                                 forecast.model_probabilities.tolist()),
                                             true_turn_rate=truth[index].turn_rate, stage=stage))
            for sample_index, horizon in enumerate(HORIZONS):
                conditional = []
                if name == 'IMM':
                    for model_index, probability in enumerate(forecast.model_probabilities):
                        if probability >= .05:
                            error = (forecast.conditional_position[model_index, sample_index, :2]
                                     - future_position[sample_index, :2])
                            covariance = forecast.conditional_position_covariance[
                                model_index, sample_index, :2, :2]
                            conditional.append(float(error @ np.linalg.solve(covariance, error)))
                rows.append(metric_row(
                    seed, mode, name, 'complete_chain', stamp, source, horizon,
                    forecast.position[sample_index], forecast.velocity[sample_index],
                    forecast.position_covariance[sample_index], future_position[sample_index],
                    future_velocity[sample_index], stage, forecast_time, conditional,
                    current_turn_rate_error=float(forecast.state[4] - source_omega)))
        # Separate matched true initial state model comparison, evaluation only.
        oracle_state = np.array((*source_position[:2], *source_velocity[:2],
                                 source_omega, source_longitudinal_acceleration, source_alpha))
        model_states = []
        for model in ('CV', 'CTRV', 'CTRA'):
            predicted = np.array([propagate_model(oracle_state, np.eye(7) * .01,
                                                  horizon, model)[0] for horizon in HORIZONS])
            model_states.append(predicted)
            for sample_index, horizon in enumerate(HORIZONS):
                rows.append(metric_row(
                    seed, mode, model, 'matched_initial_state_model_only', stamp, source, horizon,
                    np.array((*predicted[sample_index, :2], source_position[2])),
                    np.array((*predicted[sample_index, 2:4], 0.)), np.eye(3),
                    future_position[sample_index], future_velocity[sample_index], stage, 0.))
        fused = np.einsum('m,mni->ni', filters['IMM'].probabilities, np.asarray(model_states))
        for sample_index, horizon in enumerate(HORIZONS):
            rows.append(metric_row(
                seed, mode, 'IMM', 'matched_initial_state_model_only', stamp, source, horizon,
                np.array((*fused[sample_index, :2], source_position[2])),
                np.array((*fused[sample_index, 2:4], 0.)), np.eye(3),
                future_position[sample_index], future_velocity[sample_index], stage, 0.))
    return rows, observations, probability_rows, timings


def grouped_summaries(rows, keys):
    """Group explicit records; no high-error row is removed."""
    groups = {}
    for row in rows:
        group = tuple(row[key] for key in keys)
        groups.setdefault(group, []).append(row)
    return [dict(zip(keys, key), **summarize(value)) for key, value in sorted(groups.items())]


def write_jsonl(path, values):
    """Save original per-sample evidence with explicit finite JSON."""
    with Path(path).open('w') as output:
        for value in values:
            output.write(json.dumps(value, allow_nan=False, separators=(',', ':')) + '\n')


def plot_results(output, summaries, probability_rows):
    """Export scientific figures with matplotlib, retaining all horizons."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 5, figsize=(20, 4), sharey=True)
    for axis, mode in zip(axes, MODES):
        for name in ('CV', 'CTRV', 'CTRA', 'BCTRA', 'IMM'):
            values = [item for item in summaries
                      if item['mode'] == mode and item['backend'] == name
                      and item['comparison'] == 'complete_chain']
            axis.plot([item['horizon'] for item in values],
                      [item['horizontal_rmse'] for item in values], marker='o', label=name)
        axis.set_title(mode)
        axis.set_xlabel('Horizon [s]')
        axis.grid(alpha=.3)
    axes[0].set_ylabel('Horizontal position RMSE [m]')
    axes[-1].legend()
    fig.tight_layout()
    fig.savefig(output / 'heldout_prediction_rmse.png', dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(5, 1, figsize=(12, 13))
    for axis, mode in zip(axes, MODES):
        values = [item for item in probability_rows if item['mode'] == mode]
        for index, name in enumerate(('CV', 'CTRV', 'CTRA')):
            axis.plot([item['time'] for item in values],
                      [item['model_probabilities'][index] for item in values], label=name)
        axis.set_title(mode)
        axis.set_ylim(0., 1.)
        axis.grid(alpha=.3)
    axes[0].legend()
    axes[-1].set_xlabel('Acquisition time from startup [s]')
    fig.tight_layout()
    fig.savefig(output / 'heldout_model_probabilities.png', dpi=160)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=WORKSPACE /
                        'data/experiments/20261014_p6_development_01/prediction_04')
    parser.add_argument('--config', type=Path, default=WORKSPACE /
                        'src/uav_usv_bringup/config/baseline.yaml')
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--observation-period', type=float, default=.05)
    parser.add_argument('--evaluation-period', type=float, default=.2)
    parser.add_argument('--calibration-seed', type=int, default=127)
    parser.add_argument('--heldout-seed', type=int, default=3731)
    args = parser.parse_args(argv)
    if (args.duration < 4. or args.observation_period <= 0. or args.evaluation_period <= 0.
            or args.calibration_seed == args.heldout_seed):
        parser.error('duration>=4 and positive periods and disjoint seeds required')
    args.output.mkdir(parents=True, exist_ok=True)
    bctra, kf, motion = load_parameters(args.config)
    config = ImmConfig()
    config_json = {name: (value.tolist() if isinstance(value, np.ndarray) else value)
                   for name, value in asdict(config).items()}
    provenance = dict(
        calibration_seed=args.calibration_seed, heldout_seed=args.heldout_seed,
        source='synthetic_motion_profiles_iid_position_sensor', synthetic_only=True,
        duration=args.duration, observation_period=args.observation_period,
        evaluation_period=args.evaluation_period, warmup_seconds=2.,
        noise_std_xyz=[.1, .1, .05], noise='independent zero-mean Gaussian by acquisition',
        dropout_probability=.02, latency_seconds=[.02, .025], modes=list(MODES),
        trajectory_paths_shared_between_splits=True, no_truth_in_online_filters=True,
        model_parameters_fitted_on_heldout=False, full_bctra_parameters=bctra,
        revision='CTRA_stop_Q_recovery_conservative_posterior_motion_priors_v4',
        state_prior_source='Declared USV max4m/s plus25percent current-speed estimation margin; '
                           'latent bounds match original BCTRA engineering ranges',
        state_projection='posterior mean only; covariance increases by outer(mean shift); '
                         'future velocity is not globally forced to endpoint speed',
        previous_study_heldout_seed=731,
        previous_study='prediction/ remains unchanged; fresh heldout noise seed for revised code',
        full_kf_parameters=kf, full_imm_parameters=config_json,
        full_motion_parameters=asdict(motion),
        fingerprints={str(path.relative_to(WORKSPACE)): sha256(path) for path in (
            Path(__file__), args.config,
            WORKSPACE / 'src/uav_control/uav_control/tracking/imm_predictor.py',
            WORKSPACE / 'src/uav_control/uav_control/tracking/imm_target_predictor_node.py',
            WORKSPACE / 'src/uav_control/uav_control/tracking/constant_velocity_kalman.py',
            WORKSPACE / 'src/uav_control/uav_control/tracking/maneuvering_target_predictor.py',
            WORKSPACE / 'src/uav_control/uav_control/usv_motion_profiles.py')})
    (args.output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    snapshot_rows = []
    for relative, digest in provenance['fingerprints'].items():
        source = WORKSPACE / relative
        destination = args.output / 'source_snapshot' / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        if sha256(destination) != digest:
            raise RuntimeError('SOURCE_SNAPSHOT_CHANGED_DURING_COPY')
        snapshot_rows.append(dict(path=relative, sha256=digest))
    (args.output / 'source_snapshot_manifest.json').write_text(json.dumps(
        dict(scope='exact source and parameter bytes before study', files=snapshot_rows),
        indent=2) + '\n')
    split_rows, split_observations, split_probabilities, split_timings = {}, {}, {}, {}
    for split, seed in (('calibration', args.calibration_seed), ('heldout', args.heldout_seed)):
        rows, observations, probabilities, timings = [], [], [], []
        for mode in MODES:
            started = time.perf_counter()
            result = run_mode(mode, seed, args.duration, args.observation_period,
                              args.evaluation_period, provenance['noise_std_xyz'],
                              bctra, kf, motion)
            for destination, values in zip((rows, observations, probabilities, timings), result):
                destination.extend(values)
            print(f'{split} {mode}: {len(result[0])} forecast rows '
                  f'in {time.perf_counter() - started:.2f}s', flush=True)
        split_rows[split], split_observations[split] = rows, observations
        split_probabilities[split], split_timings[split] = probabilities, timings
        write_jsonl(args.output / f'{split}_forecasts.jsonl', rows)
        write_jsonl(args.output / f'{split}_observations.jsonl', observations)
        write_jsonl(args.output / f'{split}_model_probabilities.jsonl', probabilities)
        write_jsonl(args.output / f'{split}_timings.jsonl', timings)
    summaries = grouped_summaries(
        split_rows['heldout'], ('mode', 'backend', 'comparison', 'horizon'))
    stage_rows = []
    for row in split_rows['heldout']:
        for stage in row['stage'].split('|'):
            stage_rows.append(dict(row, stage=stage))
    report = dict(provenance=provenance, by_mode_horizon=summaries,
                  by_stage=grouped_summaries(
                      stage_rows, ('backend', 'comparison', 'horizon', 'stage')),
                  primary_0p5_to_1p6=grouped_summaries([
                      row for row in split_rows['heldout'] if .5 <= row['horizon'] <= 1.6],
                      ('mode', 'backend', 'comparison')))
    for split, observations in split_observations.items():
        valid = [item for item in observations if item['valid']]
        report[split + '_observation_kf_errors'] = dict(
            raw_horizontal_rmse=float(np.sqrt(np.mean([item['raw_horizontal_error'] ** 2
                                                       for item in valid]))),
            kf_horizontal_position_rmse=float(np.sqrt(np.mean([
                item['kf_position_horizontal_error'] ** 2 for item in valid]))),
            kf_horizontal_velocity_rmse=float(np.sqrt(np.mean([
                item['kf_velocity_horizontal_error'] ** 2 for item in valid]))),
            dropped=sum(not item['valid'] for item in observations), total=len(observations))
    (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    with (args.output / 'heldout_summary.csv').open('w') as output:
        keys = sorted({key for item in summaries for key in item if key != 'compute_ms'})
        writer = csv.DictWriter(output, fieldnames=keys)
        writer.writeheader()
        writer.writerows({key: item.get(key) for key in keys} for item in summaries)
    for family in ('bctra', 'imm'):
        calibration = fit_calibration(split_rows['calibration'], split_rows['heldout'],
                                      family, provenance)
        path = args.output / f'{family}_calibration.json'
        path.write_text(json.dumps(calibration, indent=2) + '\n')
        print(f'{family} scale={calibration["covariance_scale"]:.3f}; '
              f'heldout point coverage={calibration["coverage"]["heldout"]:.4f}', flush=True)
    plot_results(args.output, summaries, split_probabilities['heldout'])
    artifact_paths = sorted(path for path in args.output.rglob('*') if path.is_file()
                            and path.name != 'SHA256SUMS')
    (args.output / 'SHA256SUMS').write_text(''.join(
        f'{sha256(path)}  {path.relative_to(args.output)}\n' for path in artifact_paths))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
