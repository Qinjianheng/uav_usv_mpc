#!/usr/bin/env python3
"""Offline audit of shadow visual observations and the unchanged KF."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from uav_control.tracking.constant_velocity_kalman import (
    ConstantVelocityKalmanFilter,
)


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def read_observations(directory):
    path = directory / 'observation.jsonl'
    if path.exists():
        return read_jsonl(path), 'recorded_covariance'
    csv_path = directory.with_suffix('.csv')
    rows = list(csv.DictReader(csv_path.open()))
    observations = []
    for row in rows:
        value = [float(row[f'position_{axis}']) for axis in 'xyz']
        rng = float(row['target_range'])
        standard = 0.08 + 0.01 * rng
        observations.append({
            'stamp': float(row['measurement_stamp']),
            'valid': row['valid'] == 'True',
            'rejection_reason': row['rejection_reason'],
            'position_ned': value,
            'covariance': (np.eye(3) * standard ** 2).ravel().tolist(),
        })
    return observations, 'reconstructed_from_logged_range'


def truth_at(truth, stamp, max_gap=0.2):
    stamps = truth['stamps']
    index = int(np.searchsorted(stamps, stamp))
    if index == 0 or index >= len(stamps):
        return None
    left, right = stamps[index - 1], stamps[index]
    if stamp - left > max_gap or right - stamp > max_gap:
        return None
    alpha = (stamp - left) / (right - left)
    position = (1 - alpha) * truth['position'][index - 1] + (
        alpha * truth['position'][index]
    )
    velocity = (1 - alpha) * truth['velocity'][index - 1] + (
        alpha * truth['velocity'][index]
    )
    return position, velocity


def error_summary(errors):
    if not errors:
        return {'n': 0}
    array = np.asarray(errors, dtype=float)
    norm = np.linalg.norm(array, axis=1)
    return {
        'n': len(norm),
        'rmse': float(np.sqrt(np.mean(norm ** 2))),
        'p50': float(np.percentile(norm, 50)),
        'p95': float(np.percentile(norm, 95)),
        'max': float(np.max(norm)),
        'signed_mean_xyz': array.mean(axis=0).tolist(),
    }


def audit(directory, scenario):
    obs, covariance_source = read_observations(directory)
    kf = read_jsonl(directory / 'kf.jsonl')
    truth_rows = sorted((row for row in read_jsonl(directory / 'truth.jsonl')
                         if row['valid']), key=lambda row: row['stamp'])
    truth = {
        'stamps': np.asarray([row['stamp'] for row in truth_rows]),
        'position': np.asarray([row['position_ned'] for row in truth_rows]),
        'velocity': np.asarray([row['velocity_ned'] for row in truth_rows]),
    }
    raw_errors, raw_moving, raw_stationary = [], [], []
    rejection = {}
    intervals = []
    previous = None
    for row in obs:
        if not row['valid']:
            reason = row['rejection_reason'] or 'UNSPECIFIED'
            rejection[reason] = rejection.get(reason, 0) + 1
            continue
        stamp = float(row['stamp'])
        if previous is not None:
            intervals.append(stamp - previous)
        previous = stamp
        state = truth_at(truth, stamp)
        if state is None:
            continue
        err = np.asarray(row['position_ned']) - state[0]
        if not np.all(np.isfinite(err)):
            continue
        raw_errors.append(err)
        (raw_moving if np.linalg.norm(state[1][:2]) > 0.5
         else raw_stationary).append(err)
    kf_errors, kf_moving, kf_stationary, velocity_errors = [], [], [], []
    measurement_ages = []
    for row in kf:
        if not row['valid']:
            continue
        stamp = float(row['stamp'])
        state = truth_at(truth, stamp)
        if state is None:
            continue
        err = np.asarray(row['position_ned']) - state[0]
        vel_err = np.asarray(row['velocity_ned']) - state[1]
        if not np.all(np.isfinite(err)):
            continue
        kf_errors.append(err)
        if np.all(np.isfinite(vel_err)):
            velocity_errors.append(vel_err)
        (kf_moving if np.linalg.norm(state[1][:2]) > 0.5
         else kf_stationary).append(err)
        measurement_ages.append(stamp - float(row['source_stamp']))
    replay = ConstantVelocityKalmanFilter(1.5, 0.25, 5.0)
    filter_stamp = None
    previous_stamp = None
    innovations, variances = [], []
    replay_rejections = {'nonmonotonic': 0, 'invalid_input': 0}
    for row in obs:
        if not row['valid']:
            continue
        stamp = float(row['stamp'])
        if previous_stamp is not None and stamp <= previous_stamp:
            replay_rejections['nonmonotonic'] += 1
            continue
        value = np.asarray(row['position_ned'], dtype=float)
        cov = np.asarray(row['covariance'], dtype=float).reshape(3, 3)
        try:
            if filter_stamp is None:
                replay.initialize(value, cov)
            else:
                replay.predict(max(stamp - filter_stamp, 0))
                innovation = value - replay.state[:3]
                variance = replay.covariance[:3, :3] + cov
                innovations.append(innovation)
                variances.append(variance)
                replay.update(value, cov)
        except ValueError:
            replay_rejections['invalid_input'] += 1
            continue
        filter_stamp = stamp
        previous_stamp = stamp
    interval_array = np.asarray(intervals)
    age_array = np.asarray(measurement_ages)
    return {
        'scenario': scenario, 'directory': str(directory),
        'covariance_source': covariance_source,
        'raw': error_summary(raw_errors),
        'kf': error_summary(kf_errors),
        'raw_stationary': error_summary(raw_stationary),
        'kf_stationary': error_summary(kf_stationary),
        'raw_moving': error_summary(raw_moving),
        'kf_moving': error_summary(kf_moving),
        'kf_velocity': error_summary(velocity_errors),
        'invalid_observation_reasons': rejection,
        'replay_rejections': replay_rejections,
        'valid_observation_interval_p50_p95_s': (
            np.percentile(interval_array, [50, 95]).tolist()
            if len(interval_array) else []
        ),
        'kf_source_age_p50_p95_s': (
            np.percentile(age_array, [50, 95]).tolist()
            if len(age_array) else []
        ),
        'innovation_norm_p50_p95_m': (
            np.percentile(np.linalg.norm(innovations, axis=1),
                          [50, 95]).tolist() if innovations else []
        ),
        'innovation_covariance_diag_m2_mean': (
            np.mean(np.asarray(variances)[:, range(3), range(3)],
                    axis=0).tolist() if variances else []
        ),
        'turning_baseline': 'not_identifiable_in_linear_scenarios',
        'explicit_kf_measurement_gate': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', action='append', required=True,
                        help='name=/absolute/path/to/capture_ROI_directory')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {}
    for entry in args.scenario:
        name, path = entry.split('=', 1)
        result[name] = audit(Path(path), name)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False,
                  allow_nan=True)
        stream.write('\n')
    for name, item in result.items():
        print(name, 'raw', item['raw'], 'KF', item['kf'],
              'velocity', item['kf_velocity'])


if __name__ == '__main__':
    main()
