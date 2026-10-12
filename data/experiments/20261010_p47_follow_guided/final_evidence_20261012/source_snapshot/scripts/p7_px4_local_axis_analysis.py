#!/usr/bin/env python3
"""Evaluate fixed PX4/Gazebo XY axes from independent UAV motion.

The fit uses position increments, never body attitude. Groundtruth and EKF
alignment are diagnostic outputs, not inputs to any online controller.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np


def enu_to_ned(position):
    x, y, z = (float(value) for value in position)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError('nonfinite ENU position')
    return [y, x, -z]


def xy_displacement(start, end):
    return (np.asarray(end, dtype=float)[:2]
            - np.asarray(start, dtype=float)[:2]).tolist()


def rotation(theta):
    cosine, sine = math.cos(theta), math.sin(theta)
    return np.array(((cosine, -sine), (sine, cosine)))


def require_noncollinear(displacements):
    values = np.asarray(displacements, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 2:
        raise ValueError('noncollinear displacement set requires two axes')
    singular = np.linalg.svd(values, compute_uv=False)
    if singular[0] < 1.0 or singular[1] / singular[0] < 0.15:
        raise ValueError('noncollinear displacement set required')


def fit_fixed_rotation(gazebo_deltas, px4_deltas):
    gazebo = np.asarray(gazebo_deltas, dtype=float)
    px4 = np.asarray(px4_deltas, dtype=float)
    if gazebo.shape != px4.shape:
        raise ValueError('displacement shapes differ')
    require_noncollinear(gazebo)
    dot = float(np.sum(gazebo * px4))
    cross = float(np.sum(gazebo[:, 0] * px4[:, 1]
                         - gazebo[:, 1] * px4[:, 0]))
    return math.atan2(cross, dot)


def displacement_rmse(gazebo_deltas, px4_deltas, theta):
    gazebo = np.asarray(gazebo_deltas, dtype=float)
    px4 = np.asarray(px4_deltas, dtype=float)
    errors = px4 - gazebo @ rotation(theta).T
    return float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))


def bracket_gap(stamp, left, right, limit):
    values = (stamp, left, right, limit)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('nonfinite timestamp bracket')
    if left > stamp + 1e-6 or right < stamp - 1e-6:
        raise ValueError('timestamp not bracketed')
    gap = max(stamp - left, right - stamp)
    if gap > limit:
        raise ValueError('timestamp gap exceeds limit')
    return gap


def segment_epochs(rows):
    epochs = []
    previous_stamp = None
    previous_key = None
    epoch = 0
    for row in rows:
        stamp = float(row['ros_stamp'])
        key = tuple(row['reset_key'])
        if previous_stamp is not None and (
                stamp <= previous_stamp or key != previous_key):
            epoch += 1
        epochs.append(epoch)
        previous_stamp, previous_key = stamp, key
    return epochs


def _stats(values):
    array = np.asarray(values, dtype=float)
    if not len(array):
        return {'n': 0}
    return {
        'n': int(len(array)), 'p50': float(np.percentile(array, 50)),
        'p95': float(np.percentile(array, 95)),
        'max': float(np.max(array)),
    }


def _xy(row, key):
    return np.asarray(row[key], dtype=float)[:2]


def _phase_samples(rows, phase):
    selected = sorted((row for row in rows if row.get('valid')
                       and row.get('phase') == phase),
                      key=lambda row: row['ros_stamp'])
    if len(selected) < 8:
        raise ValueError(f'{phase}: too few valid samples')
    epochs = {row['reset_epoch'] for row in selected}
    if len(epochs) != 1:
        raise ValueError(f'{phase}: crosses reset epoch')
    return selected


def _endpoint(rows, fraction, key):
    times = np.asarray([row['ros_stamp'] for row in rows], dtype=float)
    target = times[0] + fraction * (times[-1] - times[0])
    nearest = np.argsort(abs(times - target))[:5]
    return np.median(np.asarray([_xy(rows[i], key) for i in nearest]), axis=0)


def phase_displacement(rows, phase, start_fraction=0.0,
                       end_fraction=1.0):
    selected = _phase_samples(rows, phase)
    if selected[-1]['ros_stamp'] - selected[0]['ros_stamp'] < 2.0:
        raise ValueError(f'{phase}: duration too short')
    gazebo = (_endpoint(selected, end_fraction,
                        'gazebo_uav_position_ned')
              - _endpoint(selected, start_fraction,
                          'gazebo_uav_position_ned'))
    px4 = (_endpoint(selected, end_fraction, 'px4_local_position_ned')
           - _endpoint(selected, start_fraction,
                       'px4_local_position_ned'))
    return {'phase': phase, 'fraction': [start_fraction, end_fraction],
            'gazebo_xy_m': gazebo.tolist(), 'px4_xy_m': px4.tolist(),
            'gazebo_distance_m': float(np.linalg.norm(gazebo)),
            'px4_distance_m': float(np.linalg.norm(px4))}


def analyze(rows):
    valid = [row for row in rows if row.get('valid')]
    if len(valid) < 20:
        raise ValueError('insufficient valid captured samples')
    phases = ('NORTH_OUT', 'NORTH_RETURN',
              'EAST_OUT', 'EAST_RETURN')
    parts = {}
    for phase in phases:
        if phase.endswith('OUT'):
            parts[f'{phase}_first'] = phase_displacement(
                valid, phase, 0.0, 0.5)
            parts[f'{phase}_second'] = phase_displacement(
                valid, phase, 0.5, 1.0)
        else:
            parts[phase] = phase_displacement(valid, phase)
    train = [parts['NORTH_OUT_first'], parts['EAST_OUT_first']]
    hold = [parts['NORTH_OUT_second'], parts['EAST_OUT_second'],
            parts['NORTH_RETURN'], parts['EAST_RETURN']]
    train_g = np.array([part['gazebo_xy_m'] for part in train])
    train_p = np.array([part['px4_xy_m'] for part in train])
    hold_g = np.array([part['gazebo_xy_m'] for part in hold])
    hold_p = np.array([part['px4_xy_m'] for part in hold])
    theta = fit_fixed_rotation(train_g, train_p)
    scale = float(np.sum(train_g * train_p) /
                  np.sum(train_g * train_g))
    hover = _phase_samples(valid, 'HOVER')
    offset = np.median(np.array([
        _xy(row, 'px4_local_position_ned')
        - rotation(theta) @ _xy(row, 'gazebo_uav_position_ned')
        for row in hover]), axis=0)
    residual = np.array([
        _xy(row, 'px4_local_position_ned')
        - rotation(theta) @ _xy(row, 'gazebo_uav_position_ned') - offset
        for row in valid])
    yaw_rows = [row for row in valid if row['phase'].startswith('YAW_')]
    yaw_drift = np.array([
        _xy(row, 'px4_local_position_ned')
        - _xy(row, 'gazebo_uav_position_ned') for row in yaw_rows])
    body_residual = []
    for part in train + hold:
        subset = _phase_samples(valid, part['phase'])
        yaw = np.median([row['px4_yaw_ned'] - row['gazebo_yaw_ned']
                         for row in subset])
        body_residual.append((
            np.asarray(part['px4_xy_m'])
            - rotation(yaw) @ np.asarray(part['gazebo_xy_m'])
        ).tolist())
    timing = {}
    for label, lower, upper in (
            ('lt10ms', 0.0, 0.01), ('10to25ms', 0.01, 0.025),
            ('25to50ms', 0.025, 0.05)):
        selected = [np.linalg.norm(error) for row, error in zip(valid,
                    residual) if lower <= row['alignment_gap_s'] < upper]
        timing[label] = _stats(selected)
    return {
        'valid_count': len(valid), 'phase_displacements': parts,
        'fit': {
            'theta_rad': theta, 'theta_deg': math.degrees(theta),
            'scale_ratio_diagnostic': scale,
            'model0_train_rmse_m': displacement_rmse(train_g, train_p, 0),
            'model0_holdout_rmse_m': displacement_rmse(hold_g, hold_p, 0),
            'model1_train_rmse_m': displacement_rmse(train_g, train_p,
                                                     theta),
            'model1_holdout_rmse_m': displacement_rmse(hold_g, hold_p,
                                                       theta),
            'model2_body_residual_displacement_rmse_m': float(np.sqrt(
                np.mean(np.sum(np.asarray(body_residual) ** 2, axis=1))))},
        'origin_offset_xy_m': offset.tolist(),
        'world_position_residual_norm_m': _stats(
            np.linalg.norm(residual, axis=1)),
        'yaw_only_offset_xy_span_m': (
            np.ptp(yaw_drift, axis=0).tolist() if len(yaw_drift) else None),
        'yaw_only_samples': len(yaw_rows),
        'timing_groups': timing,
        'reset_epochs': sorted({row['reset_epoch'] for row in valid}),
        'groundtruth_ros_count': sum(
            row.get('groundtruth_status') in ('EXACT', 'INTERPOLATED')
            for row in valid),
    }


def interpolated_ulog_value(stamp, times, values, maximum_gap):
    """Interpolate only inside a bounded ULog timestamp bracket."""
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and abs(times[index] - stamp) <= 1e-8:
        return values[index]
    if index == 0 or index == len(times):
        raise ValueError('ULog timestamp not bracketed')
    left, right = times[index - 1], times[index]
    if max(stamp - left, right - stamp) > maximum_gap:
        raise ValueError('ULog timestamp gap exceeds limit')
    return values[index - 1] + (values[index] - values[index - 1]) * (
        (stamp - left) / (right - left))


def align_ulog_ekf_to_capture(ros_stamps, captured_positions,
                              ulog_times, ulog_positions):
    """Estimate clock offset only from PX4 EKF capture vs PX4 EKF ULog."""
    from scipy.optimize import minimize_scalar

    ros = np.asarray(ros_stamps, dtype=float)
    captured = np.asarray(captured_positions, dtype=float)
    source = np.asarray(ulog_times, dtype=float)
    positions = np.asarray(ulog_positions, dtype=float)
    if len(ros) < 10 or captured.shape != (len(ros), 3):
        raise ValueError('too few PX4 self-alignment samples')
    relative = ros - ros[0]
    lower, upper = source[0], source[-1] - relative[-1]
    if lower >= upper:
        raise ValueError('ULog does not cover capture span')
    selected = np.unique(np.linspace(
        0, len(ros) - 1, min(300, len(ros))).astype(int))

    def mse(first_ulog_time):
        query = relative[selected] + first_ulog_time
        predicted = np.column_stack([
            np.interp(query, source, positions[:, axis])
            for axis in range(3)
        ])
        errors = predicted - captured[selected]
        return float(np.mean(np.sum(errors * errors, axis=1)))

    grid = np.linspace(lower, upper, max(1001, int((upper - lower) * 20)))
    best = int(np.argmin([mse(value) for value in grid]))
    left = grid[max(0, best - 1)]
    right = grid[min(len(grid) - 1, best + 1)]
    optimum = minimize_scalar(mse, bounds=(left, right), method='bounded',
                              options={'xatol': 1e-8})
    return ros[0] - float(optimum.x), math.sqrt(float(optimum.fun))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    with args.input.open(encoding='utf-8') as stream:
        rows = [json.loads(line) for line in stream]
    result = analyze(rows)
    result['source_capture'] = str(args.input.resolve())
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps(result['fit'], ensure_ascii=False))


if __name__ == '__main__':
    main()
