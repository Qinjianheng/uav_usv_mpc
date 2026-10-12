#!/usr/bin/env python3
"""Pair current B observations and KF without cross-frame truth."""

import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path

import numpy as np

from uav_control.tracking.constant_velocity_kalman import (
    ConstantVelocityKalmanFilter,
)


def scalar_stats(values):
    array = np.asarray([value for value in values
                        if math.isfinite(value)], dtype=float)
    if not len(array):
        return {'n': 0, 'p50': None, 'p95': None, 'max': None}
    return {
        'n': int(len(array)), 'p50': float(np.percentile(array, 50)),
        'p95': float(np.percentile(array, 95)), 'max': float(array.max()),
    }


def vector_stats(values):
    if not values:
        return {'n': 0, 'rmse': None, 'p95': None, 'signed_mean_xyz': None}
    array = np.asarray(values, dtype=float)
    norms = np.linalg.norm(array, axis=1)
    return {
        'n': int(len(array)),
        'rmse': float(np.sqrt(np.mean(norms ** 2))),
        'p50': float(np.percentile(norms, 50)),
        'p95': float(np.percentile(norms, 95)),
        'max': float(norms.max()),
        'signed_mean_xyz': array.mean(axis=0).tolist(),
    }


def interpolate_kf(records, stamps, stamp, maximum_side_gap=0.12):
    """Interpolate two valid KF states; never extrapolate or bridge a gap."""
    index = bisect_left(stamps, stamp)
    if index == 0 or index == len(stamps):
        return None
    left, right = records[index - 1], records[index]
    t0, t1 = stamps[index - 1], stamps[index]
    if stamp - t0 > maximum_side_gap or t1 - stamp > maximum_side_gap:
        return None
    fraction = (stamp - t0) / (t1 - t0)
    position = (1 - fraction) * np.asarray(left['position_ned']) + (
        fraction * np.asarray(right['position_ned']))
    velocity = (1 - fraction) * np.asarray(left['velocity_ned']) + (
        fraction * np.asarray(right['velocity_ned']))
    return position, velocity, stamp - float(left['source_stamp'])


def local_linear_residuals(stamps, values, half_window=0.5):
    """Measure high-frequency variation about each local linear trend."""
    stamps = np.asarray(stamps)
    values = np.asarray(values)
    residuals = []
    for index, stamp in enumerate(stamps):
        mask = np.abs(stamps - stamp) <= half_window
        if mask.sum() < 5:
            continue
        design = np.column_stack((np.ones(mask.sum()), stamps[mask] - stamp))
        coefficients = np.linalg.lstsq(design, values[mask], rcond=None)[0]
        residuals.append(values[index] - coefficients[0])
    return residuals


def replay_innovations(rows):
    """Replay innovations using range-reconstructed covariance."""
    replay = ConstantVelocityKalmanFilter(1.5, 0.25, 5.0)
    previous = None
    innovations, variances = [], []
    for row in rows:
        stamp = float(row['measurement_stamp'])
        if previous is not None and stamp <= previous:
            continue
        value = np.array([float(row[f'position_{axis}']) for axis in 'xyz'])
        standard = 0.08 + 0.01 * float(row['target_range'])
        covariance = np.eye(3) * standard ** 2
        if not np.all(np.isfinite(value)) or not np.isfinite(standard):
            continue
        try:
            if previous is None:
                replay.initialize(value, covariance)
            else:
                replay.predict(stamp - previous)
                innovations.append(value - replay.state[:3])
                variances.append(np.diag(
                    replay.covariance[:3, :3] + covariance))
                replay.update(value, covariance)
        except ValueError:
            continue
        previous = stamp
    return {
        'source': 'offline_replay_reconstructed_range_covariance',
        'innovation_norm_m': scalar_stats(
            [float(np.linalg.norm(value)) for value in innovations]),
        'innovation_covariance_diag_m2_mean': (
            np.mean(variances, axis=0).tolist() if variances else None),
    }


def audit(csv_path, kf_path, frame_path):
    with csv_path.open(newline='', encoding='utf-8') as stream:
        rows = [row for row in csv.DictReader(stream)
                if row['valid'] == 'True']
    rows.sort(key=lambda row: float(row['measurement_stamp']))
    kf = [json.loads(line) for line in kf_path.read_text().splitlines()]
    kf = sorted((row for row in kf if row['valid']
                 and row['frame_id'] == 'local_ned'),
                key=lambda row: row['stamp'])
    stamps = [float(row['stamp']) for row in kf]
    pairs = []
    for row in rows:
        stamp = float(row['measurement_stamp'])
        estimate = interpolate_kf(kf, stamps, stamp)
        if estimate is None:
            continue
        raw = np.array([float(row[f'position_{axis}']) for axis in 'xyz'])
        if not np.all(np.isfinite(raw)):
            continue
        position, velocity, source_age = estimate
        pairs.append((stamp, raw, position, velocity, source_age))
    pair_times = [item[0] for item in pairs]
    raw = [item[1] for item in pairs]
    filtered = [item[2] for item in pairs]
    velocities = [item[3] for item in pairs]
    lag = []
    for _, raw_position, kf_position, velocity, _ in pairs:
        speed_squared = float(np.dot(velocity, velocity))
        if speed_squared >= 0.04:
            lag.append(float(np.dot(raw_position - kf_position, velocity)
                             / speed_squared))
    frame = json.loads(frame_path.read_text(encoding='utf-8'))
    if 'phases' not in frame:
        frame = next(iter(frame.values()))
    phases = frame['phases']
    fixed_holdout = phases['all']['fixed_map_holdout']['rmse_3d_m']
    return {
        'source_csv': str(csv_path), 'source_kf_jsonl': str(kf_path),
        'valid_raw_observations': len(rows),
        'valid_kf_messages': len(kf),
        'same_time_paired_count': len(pairs),
        'pairing_method': 'linear KF interpolation at raw measurement_stamp; '
                          'each bracket side <=0.12 s, no extrapolation',
        'kf_minus_raw_position_m': vector_stats([
            item[2] - item[1] for item in pairs]),
        'raw_high_frequency_residual_m': vector_stats(
            local_linear_residuals(pair_times, raw)),
        'kf_high_frequency_residual_m': vector_stats(
            local_linear_residuals(pair_times, filtered)),
        'along_track_lag_proxy_s': scalar_stats(lag),
        'kf_source_age_at_observation_s': scalar_stats(
            [item[4] for item in pairs]),
        'valid_observation_interval_s': scalar_stats([
            right - left for left, right in zip(
                [float(row['measurement_stamp']) for row in rows],
                [float(row['measurement_stamp']) for row in rows][1:])
        ]),
        'kf_velocity_step_mps': scalar_stats([
            float(np.linalg.norm(right - left)) for left, right
            in zip(velocities, velocities[1:])]),
        'kf_speed_mps': scalar_stats([
            float(np.linalg.norm(value)) for value in velocities]),
        'innovation_replay': replay_innovations(rows),
        'fixed_g_to_p_map_holdout_raw_error_rmse_m': fixed_holdout,
        'fixed_map_yaw_std_rad': phases['all']['frame_yaw_rad']['std'],
        'fixed_map_translation_std_xyz_m': phases[
            'all']['frame_translation_m']['std_xyz'],
        'absolute_raw_and_kf_truth_rmse': None,
        'absolute_truth_rmse_limitation': (
            'Fixed G-to-P map drifts substantially in this dynamic run; '
            'per-frame E4 map is not a stable KF time-series truth frame.'),
        'kf_velocity_truth_rmse': None,
        'kf_velocity_truth_rmse_limitation': (
            'The fixed G-to-P map is unstable; velocity truth in P frame '
            'cannot be audited reliably from this capture.'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', action='append', required=True,
                        help='name=csv_path=kf_jsonl=frame_json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {}
    for entry in args.scenario:
        name, csv_path, kf_path, frame_path = entry.split('=', 3)
        result[name] = audit(Path(csv_path), Path(kf_path), Path(frame_path))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    for name, item in result.items():
        print(name, 'paired', item['same_time_paired_count'],
              'fixed-map holdout',
              item['fixed_g_to_p_map_holdout_raw_error_rmse_m'])


if __name__ == '__main__':
    main()
