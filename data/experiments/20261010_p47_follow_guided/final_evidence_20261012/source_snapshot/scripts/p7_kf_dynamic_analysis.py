#!/usr/bin/env python3
"""Same-image-time P7.3 B/KF baseline with bounded Gazebo and PX4 truth."""

import argparse
from bisect import bisect_right
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
import yaml

from uav_control.perception.rgbd_target_localizer import (
    camera_target_to_local_ned,
)

from kf_truth_capture import convert_rows
from p7_px4_local_axis_analysis import (
    align_ulog_ekf_to_capture, interpolated_ulog_value,
)


def metrics(vectors):
    values = np.asarray(vectors, dtype=float)
    if not values.size:
        return {'n': 0}
    norms = np.linalg.norm(values, axis=1)
    return {
        'n': len(values), 'rmse_m': float(np.sqrt(np.mean(norms ** 2))),
        'p50_m': float(np.percentile(norms, 50)),
        'p95_m': float(np.percentile(norms, 95)),
        'max_m': float(max(norms)),
        'signed_mean_xyz_m': values.mean(axis=0).tolist(),
    }


def bounded_interpolate(stamp, times, values, gap=0.075):
    """Use two nearby samples, with no extrapolation or nearest-value fill."""
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and abs(times[index] - stamp) < 1e-7:
        return values[index]
    if index == 0 or index == len(times):
        return None
    left, right = times[index - 1:index + 1]
    if max(stamp - left, right - stamp) > gap or right <= left:
        return None
    return values[index - 1] + (values[index] - values[index - 1]) * (
        (stamp - left) / (right - left))


def causal_kf_at_image(states, stamp, maximum_age=0.075):
    """Project last *published* state to image time, with no future update."""
    stamps = [state['stamp'] for state in states]
    index = bisect_right(stamps, stamp) - 1
    if index < 0:
        return None, 'NO_PRIOR_KF'
    state = states[index]
    age = stamp - state['stamp']
    if age < -1e-8 or age > maximum_age:
        return None, 'KF_PUBLISH_GAP'
    if state['source_stamp'] > stamp + 1e-6:
        return None, 'KF_FUTURE_SOURCE'
    if not state['valid'] or state['frame_id'] != 'local_ned':
        return None, 'KF_INVALID_OR_FRAME'
    position = np.asarray(state['position_ned'], dtype=float)
    velocity = np.asarray(state['velocity_ned'], dtype=float)
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
        return None, 'KF_NONFINITE'
    return {
        'position': (position + age * velocity).tolist(),
        'velocity': velocity.tolist(),
        'publish_stamp': state['stamp'],
        'source_stamp': state['source_stamp'],
        'publish_age_s': age,
        'source_age_s': stamp - state['source_stamp'],
        'status': 'UPDATED_SINCE_PREVIOUS_PUBLISH' if (
            index > 0 and state['source_stamp']
            > states[index - 1]['source_stamp'] + 1e-8)
        else 'PREDICT_ONLY_SINCE_PREVIOUS_PUBLISH',
    }, ''


def finite_difference_velocity(rows, key, maximum_dt=0.15):
    """Raw B has no velocity field; use adjacent image-time difference."""
    result = {}
    previous = None
    for row in rows:
        if not row.get('frame_valid') or key not in row:
            previous = None
            continue
        stamp = row['timestamp']
        if previous is not None:
            dt = stamp - previous['timestamp']
            if 0 < dt <= maximum_dt and row['reset_epoch'] == previous[
                            'reset_epoch']:
                result[stamp] = ((np.asarray(row[key]) - np.asarray(
                    previous[key])) / dt).tolist()
        previous = row
    return result


def _ulog_series(dataset, fields):
    data = dataset.data
    times = np.asarray(data['timestamp_sample'], dtype=float) * 1e-6
    values = np.column_stack([data[name] for name in fields])
    valid = np.isfinite(times) & np.all(np.isfinite(values), axis=1)
    times, values = times[valid], values[valid]
    unique, indices = np.unique(times, return_index=True)
    return unique, values[indices]


def _clock_fit(rows, ekf_time, ekf_state):
    # Takeoff supplies variation even in the static-target scenario.
    selected = [row for row in rows if row['frame_valid']
                and -4.7 < row['px4_uav_position_ned'][2] < -0.5]
    if len(selected) < 20:
        selected = [row for row in rows if row['frame_valid']][20:180]
    if len(selected) < 20:
        raise ValueError('insufficient takeoff samples for PX4 self clock fit')
    selected = selected[:min(len(selected), 150)]
    return align_ulog_ekf_to_capture(
        [row['timestamp'] for row in selected],
        [row['px4_uav_position_ned'] for row in selected],
        ekf_time, ekf_state[:, :3])


def lag_sweep(pairs, estimate_key, truth_key):
    """Compare every lag on the same bounded-interpolation image subset."""
    times = np.asarray([pair['timestamp'] for pair in pairs])
    truth = np.asarray([pair[truth_key] for pair in pairs])
    lags = np.arange(-0.2, 0.20001, 0.01)
    common = []
    for pair in pairs:
        shifted = [bounded_interpolate(pair['timestamp'] + lag,
                                       times, truth, gap=0.15)
                   for lag in lags]
        if all(value is not None for value in shifted):
            common.append((pair, shifted))
    if len(common) < 20:
        return {'curve': [], 'best': None, 'common_count': len(common)}
    curve = []
    for index, lag in enumerate(lags):
        errors = [np.asarray(pair[estimate_key]) - shifted[index]
                  for pair, shifted in common]
        curve.append({'lag_s': round(float(lag), 3), **metrics(errors)})
    return {'curve': curve,
            'best': min(curve, key=lambda item: item['rmse_m']),
            'common_count': len(common)}


def _correlation(x, y):
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    if len(x) < 5 or np.std(x) < 1e-6 or np.std(y) < 1e-6:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def analyze(csv_path, kf_path, ulog_path, scenario, config_path):
    config = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    camera_config = config['rgbd_target_localizer']['ros__parameters']
    camera_translation = tuple(camera_config[
        f'camera_translation_{axis}'] for axis in 'xyz')
    camera_pitch = camera_config['camera_pitch_down']
    with Path(csv_path).open(newline='', encoding='utf-8') as stream:
        raw_rows = list(csv.DictReader(stream))
    truth = convert_rows(raw_rows, max_alignment_gap=0.05)
    states = sorted((json.loads(line) for line in Path(kf_path).read_text(
        encoding='utf-8').splitlines()), key=lambda item: item['stamp'])
    log = ULog(str(ulog_path), message_name_filter_list=[
        'vehicle_local_position', 'vehicle_local_position_groundtruth'])
    datasets = {item.name: item for item in log.data_list}
    ekf_t, ekf_values = _ulog_series(
        datasets['vehicle_local_position'], ('x', 'y', 'z', 'vx', 'vy', 'vz'))
    gt_t, gt_values = _ulog_series(
        datasets['vehicle_local_position_groundtruth'],
        ('x', 'y', 'z', 'vx', 'vy', 'vz'))
    offset, clock_rmse = _clock_fit(truth, ekf_t, ekf_values)
    if clock_rmse > 0.02:
        raise ValueError(f'PX4 self clock fit too inaccurate: {clock_rmse}')
    invalid = Counter()
    paired = []
    for raw, reference in zip(raw_rows, truth):
        if not raw.get('valid') == 'True':
            invalid['RGBD_' + (raw.get('rejection_reason') or 'INVALID')] += 1
            continue
        if not reference['frame_valid']:
            invalid['TRUTH_' + reference['invalid_reason']] += 1
            continue
        stamp = reference['timestamp']
        query = stamp - offset
        try:
            gt = interpolated_ulog_value(query, gt_t, gt_values, 0.05)
            ekf = interpolated_ulog_value(query, ekf_t, ekf_values, 0.05)
        except ValueError:
            invalid['ULOG_BRACKET'] += 1
            continue
        # Separate the captured PX4 sample from the same ULog EKF stream.
        px4 = np.asarray(reference['px4_uav_position_ned'])
        if np.linalg.norm(px4 - ekf[:3]) > 0.02:
            invalid['PX4_SELF_CLOCK_MISMATCH'] += 1
            continue
        kf, reason = causal_kf_at_image(states, stamp)
        if reason:
            invalid[reason] += 1
            continue
        observed = np.asarray([float(raw[f'position_{axis}'])
                               for axis in 'xyz'])
        target_world = np.asarray(reference['gazebo_target_position_enu'])
        target_ned = np.array((target_world[1], target_world[0],
                               -target_world[2]))
        target_ned[2] += reference['target_reference_z_offset_m']
        operational = np.asarray(reference['target_truth_px4_ned'])
        physical = target_ned
        model = np.asarray(reference['gazebo_uav_position_enu'])[[1, 0, 2]]
        model = model * np.array((1, 1, -1))
        camera_vector = [float(raw[f'center_camera_{axis}'])
                         for axis in 'xyz']
        camera_counterfactual = camera_target_to_local_ned(
            camera_vector, px4,
            reference['gazebo_uav_attitude_ned_frd_wxyz'],
            camera_translation, camera_pitch,
            reference['target_reference_z_offset_m'])
        ekf_minus_gt = ekf[:3] - gt[:3]
        if not np.allclose(operational - physical, px4 - model,
                           atol=1e-6):
            raise ValueError('operational frame decomposition failed')
        target_velocity = np.asarray([float(raw[f'truth_velocity_{axis}'])
                                      for axis in 'xyz'])
        operational_velocity = target_velocity + ekf[3:] - gt[3:]
        gt_index = int(np.searchsorted(gt_t, query))
        gt_side_gap = max(query - gt_t[gt_index - 1],
                          gt_t[gt_index] - query)
        math_target = np.asarray([float(raw[f'truth_{axis}'])
                                  for axis in 'xyz'])
        gazebo_offset = (float(raw['image_rgb_mapped_stamp'])
                         - float(raw['image_rgb_raw_stamp']))
        clock_gap = gazebo_offset - offset
        timing_prediction = gt[3:] * clock_gap
        relative_skew_shift = np.linalg.norm(
            (target_velocity - gt[3:])
            * float(raw['rgb_depth_acquisition_skew']))
        paired.append({
            'timestamp': stamp, 'reset_epoch': reference['reset_epoch'],
            'raw_position_ned': observed.tolist(),
            'kf_position_ned': kf['position'],
            'kf_velocity_ned': kf['velocity'],
            'kf_status': kf['status'],
            'kf_source_age_s': kf['source_age_s'],
            'kf_publish_gap_s': kf['publish_age_s'],
            'target_physical_position_ned': physical.tolist(),
            'target_operational_position_ned': operational.tolist(),
            'target_commanded_velocity_ned': target_velocity.tolist(),
            'entity_minus_math_target_ned': (physical - math_target).tolist(),
            'camera_geometry_error_ned': (
                camera_counterfactual - operational).tolist(),
            'px4_attitude_contribution_ned': (
                observed - camera_counterfactual).tolist(),
            'target_operational_velocity_ned': operational_velocity.tolist(),
            'gazebo_uav_position_ned': (gt[:3] - (
                gt[:3] - np.asarray(reference['gazebo_uav_position_enu'])[
                    [1, 0, 2]] * np.array((1, 1, -1)))).tolist(),
            'px4_ekf_position_ned': ekf[:3].tolist(),
            'px4_ekf_velocity_ned': ekf[3:].tolist(),
            'px4_groundtruth_position_ned': gt[:3].tolist(),
            'px4_groundtruth_velocity_ned': gt[3:].tolist(),
            'px4_position_error_ned': ekf_minus_gt.tolist(),
            'px4_minus_gazebo_model_ned': (ekf[:3] - model).tolist(),
            'captured_px4_minus_gazebo_model_ned': (px4 - model).tolist(),
            'groundtruth_minus_gazebo_model_ned': (gt[:3] - model).tolist(),
            'gazebo_minus_px4_ulog_clock_offset_s': clock_gap,
            'timing_predicted_gt_model_ned': timing_prediction.tolist(),
            'timing_residual_gt_model_ned': (
                gt[:3] - model - timing_prediction).tolist(),
            'relative_rgbd_skew_shift_m': float(relative_skew_shift),
            'px4_velocity_error_ned': (ekf[3:] - gt[3:]).tolist(),
            'raw_operational_error_ned': (observed - operational).tolist(),
            'raw_physical_error_ned': (observed - physical).tolist(),
            'kf_operational_error_ned': (
                np.asarray(kf['position']) - operational).tolist(),
            'kf_physical_error_ned': (
                np.asarray(kf['position']) - physical).tolist(),
            'alignment_gap_s': reference['timestamp_alignment_error_s'],
            'ulog_groundtruth_side_gap_s': float(gt_side_gap),
            'source_query': reference['source_query'],
            'rgb_depth_skew_s': float(raw['rgb_depth_acquisition_skew']),
            'observation_publish_age_s': float(raw[
                'observation_published_stamp_online']) - stamp,
            'kf_source_stamp': kf['source_stamp'],
            'kf_publish_stamp': kf['publish_stamp'],
        })
    if not paired:
        raise ValueError('no strictly paired B/KF/truth frames')
    stable = [float(row['measurement_stamp']) for row in raw_rows
              if float(row.get('gazebo_model_world_z', 'nan')) >= 4.7]
    if not stable:
        raise ValueError('no airborne stable-altitude interval')
    window_start = min(stable) + 2.0
    window_end = window_start + 30.0
    window_rows = [row for row in raw_rows
                   if window_start <= float(row['measurement_stamp'])
                   <= window_end]
    paired = [item for item in paired
              if window_start <= item['timestamp'] <= window_end]
    if len(paired) < 20:
        raise ValueError('too few strictly paired FOLLOW frames')
    window_invalid = Counter(
        row.get('rejection_reason') or 'VALID' for row in window_rows
        if row.get('valid') != 'True')
    raw_velocity = finite_difference_velocity([
        {'frame_valid': True, 'timestamp': item['timestamp'],
         'reset_epoch': item['reset_epoch'],
         'raw_position_ned': item['raw_position_ned']}
        for item in paired], 'raw_position_ned')
    for item in paired:
        item['raw_velocity_fd_ned'] = raw_velocity.get(item['timestamp'])

    def _values(key):
        return [item[key] for item in paired]
    report = {
        'scenario': scenario, 'source_csv': str(Path(csv_path).resolve()),
        'source_kf': str(Path(kf_path).resolve()),
        'source_ulog': str(Path(ulog_path).resolve()),
        'source_config': str(Path(config_path).resolve()),
        'total_observations': len(raw_rows),
        'follow_window_s': [window_start, window_end],
        'follow_window_observations': len(window_rows),
        'paired_count': len(paired),
        'paired_rate': len(paired) / len(window_rows),
        'invalid_reasons_all': dict(invalid),
        'rgbd_invalid_reasons_follow': dict(window_invalid),
        'clock_self_fit_rmse_m': clock_rmse,
        'ros_minus_ulog_boot_offset_s': offset,
        'raw_operational_position': metrics(
            _values('raw_operational_error_ned')),
        'kf_operational_position': metrics(
            _values('kf_operational_error_ned')),
        'raw_physical_position': metrics(
            _values('raw_physical_error_ned')),
        'kf_physical_position': metrics(
            _values('kf_physical_error_ned')),
        'px4_position': metrics(_values('px4_position_error_ned')),
        'px4_minus_gazebo_model': metrics(
            _values('px4_minus_gazebo_model_ned')),
        'captured_px4_minus_gazebo_model': metrics(
            _values('captured_px4_minus_gazebo_model_ned')),
        'groundtruth_minus_gazebo_model': metrics(
            _values('groundtruth_minus_gazebo_model_ned')),
        'timing_predicted_gt_model': metrics(
            _values('timing_predicted_gt_model_ned')),
        'timing_residual_gt_model': metrics(
            _values('timing_residual_gt_model_ned')),
        'gazebo_minus_px4_ulog_clock_offset_p50_p95_s': np.percentile(
            _values('gazebo_minus_px4_ulog_clock_offset_s'),
            [50, 95]).tolist(),
        'relative_rgbd_skew_shift_p50_p95_max_m': np.percentile(
            _values('relative_rgbd_skew_shift_m'), [50, 95, 100]).tolist(),
        'entity_minus_math_target': metrics(
            _values('entity_minus_math_target_ned')),
        'camera_geometry_contribution': metrics(
            _values('camera_geometry_error_ned')),
        'px4_attitude_contribution': metrics(
            _values('px4_attitude_contribution_ned')),
        'px4_velocity': metrics(_values('px4_velocity_error_ned')),
        'kf_operational_velocity': metrics([
            np.asarray(item['kf_velocity_ned']) - np.asarray(
                item['target_operational_velocity_ned']) for item in paired]),
        'raw_fd_operational_velocity': metrics([
            np.asarray(item['raw_velocity_fd_ned']) - np.asarray(
                item['target_operational_velocity_ned']) for item in paired
            if item['raw_velocity_fd_ned'] is not None]),
        'kf_status_counts': dict(Counter(
            item['kf_status'] for item in paired)),
        'kf_source_age_p50_p95_s': np.percentile(
            _values('kf_source_age_s'), [50, 95]).tolist(),
        'alignment_gap_p50_p95_s': np.percentile(
            _values('alignment_gap_s'), [50, 95]).tolist(),
        'ulog_gt_gap_p50_p95_s': np.percentile(
            _values('ulog_groundtruth_side_gap_s'), [50, 95]).tolist(),
        'observation_publish_age_p50_p95_s': np.percentile(
            _values('observation_publish_age_s'), [50, 95]).tolist(),
        'raw_lag_sweep': lag_sweep(
            paired, 'raw_position_ned', 'target_operational_position_ned'),
        'kf_lag_sweep': lag_sweep(
            paired, 'kf_position_ned', 'target_operational_position_ned'),
        'correlation': {
            'raw_physical_y_vs_target_velocity_y': _correlation(
                [item['raw_physical_error_ned'][1] for item in paired],
                [item['target_commanded_velocity_ned'][1]
                 for item in paired]),
            'raw_physical_y_vs_px4_error_y': _correlation(
                [item['raw_physical_error_ned'][1] for item in paired],
                [item['px4_position_error_ned'][1] for item in paired]),
            'relative_skew_shift_vs_camera_error_norm': _correlation(
                [item['relative_rgbd_skew_shift_m'] for item in paired],
                [np.linalg.norm(item['camera_geometry_error_ned'])
                 for item in paired]),
            'raw_operational_y_vs_px4_error_y': _correlation(
                [item['raw_operational_error_ned'][1] for item in paired],
                [item['px4_position_error_ned'][1] for item in paired]),
        },
    }
    return paired, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', required=True, type=Path)
    parser.add_argument('--kf', required=True, type=Path)
    parser.add_argument('--ulog', required=True, type=Path)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    paired, report = analyze(
        args.csv, args.kf, args.ulog, args.scenario, args.config)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / 'paired_frames.jsonl').open('x') as stream:
        for item in paired:
            stream.write(json.dumps(item, allow_nan=False) + '\n')
    with (args.output / 'analysis.json').open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2,
                  allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: report[key] for key in (
        'scenario', 'total_observations', 'paired_count',
        'raw_operational_position', 'kf_operational_position',
        'px4_position')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
