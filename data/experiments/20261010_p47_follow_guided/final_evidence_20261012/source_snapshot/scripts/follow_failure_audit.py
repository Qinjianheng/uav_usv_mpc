#!/usr/bin/env python3
"""Read-only CSV/ULog follow failure audit; no online correction or delay fit."""

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from p7_attitude_clock_analysis import quaternion_at, series


def read_csv(path):
    """Read original evidence without changing it."""
    with path.open() as stream:
        return list(csv.DictReader(stream))


def vector_at(stamp, times, vectors, maximum_gap=.05):
    """Interpolate only a finite native-time bracket, never extrapolate."""
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and abs(times[index] - stamp) < 1e-8:
        return vectors[index]
    if index == 0 or index == len(times):
        raise ValueError('position_not_bracketed')
    left, right = times[index - 1:index + 1]
    if right <= left or max(stamp-left, right-stamp) > maximum_gap:
        raise ValueError('position_bracket_too_wide')
    return vectors[index-1] + (vectors[index]-vectors[index-1]) * (
        (stamp-left)/(right-left))


def attitude_surrogate(position, vision, reference, estimated, truth):
    """Hold position fixed and change only attitude using a ULog surrogate.

    Valid production frames lack saved camera/body vectors and quaternions.
    Thus this is NOT exact reconstruction of the production pose or a new
    physical measurement. No truth is sent back to a running system.
    """
    body = estimated.inv().apply(vision-position)
    return position + truth.apply(body) - reference


def error_stats(errors):
    """Report horizontal RMSE and signed vector mean on identical samples."""
    errors = np.asarray(errors)
    if not len(errors):
        return {'n': 0}
    return {'n': len(errors), 'horizontal_rmse_m': float(np.sqrt(np.mean(
        np.sum(errors[:, :2]**2, axis=1)))), 'mean_xyz_m': errors.mean(axis=0).tolist()}


def percentiles(values):
    """Summarize finite timing/angle samples."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return None if not len(values) else dict(zip(
        ('min', 'p50', 'p95', 'max'), np.percentile(values, [0, 50, 95, 100]).tolist()))


def audit(main_path, vision_path, ulog_path):
    """Use Gazebo raw image time = native SITL hrt, without fitted offsets."""
    main, vision = read_csv(main_path), read_csv(vision_path)
    log = ULog(str(ulog_path), message_name_filter_list=[
        'vehicle_attitude', 'vehicle_attitude_groundtruth', 'vehicle_local_position',
        'vehicle_magnetometer', 'sensor_mag', 'trajectory_setpoint'])
    datasets = {d.name: d for d in log.data_list if d.multi_id == 0}
    estimated = series(datasets['vehicle_attitude'])
    truth = series(datasets['vehicle_attitude_groundtruth'])
    position_data = datasets['vehicle_local_position'].data
    position_times, indices = np.unique(position_data['timestamp_sample'] * 1e-6,
                                        return_index=True)
    positions = np.column_stack([position_data[a] for a in 'xyz'])[indices]
    frames, rejected = [], Counter()
    for row in vision:
        if row['observation_valid'] != 'True' or row['truth_available'] != 'True':
            continue
        stamp = float(row['rgb_raw_stamp'])
        v, reference = (np.array([float(row[f'{prefix}_{a}']) for a in 'xyz'])
                        for prefix in ('position', 'truth'))
        try:
            p = vector_at(stamp, position_times, positions)
            e, g = quaternion_at(stamp, *estimated), quaternion_at(stamp, *truth)
        except ValueError as exc:
            rejected[str(exc)] += 1
            continue
        yaw_delta = (e.as_euler('xyz')[2]-g.as_euler('xyz')[2]+np.pi) % (2*np.pi)-np.pi
        frames.append({'image_raw_stamp': stamp, 'measurement_stamp': float(
            row['measurement_stamp']), 'geometry_saved': row['geometry_diagnostics_enabled'],
            'raw_error_m': (v-reference).tolist(), 'surrogate_error_m':
            attitude_surrogate(p, v, reference, e, g).tolist(),
            'estimated_minus_truth_yaw_deg': float(np.degrees(yaw_delta))})
    valid = [r for r in vision if r['observation_valid'] == 'True']
    # Main CSV time is elapsed, whereas vision measurement_stamp is ROS epoch.
    # Read the recorded run origin; do not infer an offset from error curves.
    run_summary_path = main_path.with_name(main_path.stem+'_summary.json')
    start = float(json.loads(run_summary_path.read_text())['run_started_at'])
    subset = [r for r in frames if r['measurement_stamp'] >= start]
    magnetic = {}
    for name, keys in [('sensor_mag', list('xyz')), ('vehicle_magnetometer', [
            f'magnetometer_ga[{i}]' for i in range(3)])]:
        data = datasets[name].data
        fields = []
        for i, stamp in enumerate(data['timestamp_sample']*1e-6):
            try:
                rotation = quaternion_at(stamp, *truth)
            except ValueError:
                continue
            # Initial static heading only: cached calibration rotates with the body.
            angles = rotation.as_euler('xyz', degrees=True)
            if np.max(np.abs(angles)) > .1:
                continue
            fields.append(rotation.apply([data[k][i] for k in keys]))
        fields = np.asarray(fields)
        magnetic[name] = {'static_samples': len(fields), 'ned_field_median_gauss':
                          np.median(fields, axis=0).tolist() if len(fields) else None,
                          'declination_deg': percentiles(np.degrees(np.arctan2(
                              fields[:, 1], fields[:, 0]))) if len(fields) else None}
    setpoints = datasets['trajectory_setpoint'].data
    summary = {
        'sources': {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (main_path, vision_path, ulog_path, run_summary_path)},
        'run_started_at_ros': start,
        'method': 'native_raw_sim_time_no_fit; ULog attitude-only SURROGATE; no extrapolation',
        'phase_rows': dict(Counter(r['phase'] for r in main)),
        'uav_xy_span_m': [float(np.ptp([float(r[f'uav_{a}']) for r in main])) for a in 'xy'],
        'target_horizontal_distance_m': percentiles([r['horizontal_distance'] for r in main]),
        'vision_rows': len(vision), 'valid_observations': len(valid),
        'rejections': dict(Counter(r['rejection_reason'] for r in vision
                                   if r['observation_valid'] != 'True')),
        'latency_seconds': {
            name: percentiles([float(r[end])-float(r[begin]) for r in valid])
            for name, begin, end in [
                                ('delivery', 'measurement_stamp', 'receipt_stamp'),
                                ('receipt_to_processed', 'receipt_stamp', 'processed_stamp'),
                                ('total', 'measurement_stamp', 'published_stamp')]},
        'all_paired_frames': {
            'raw': error_stats([r['raw_error_m'] for r in frames]),
            'surrogate': error_stats([r['surrogate_error_m'] for r in frames])},
        'after_x_paired_frames': {
            'raw': error_stats([r['raw_error_m'] for r in subset]),
            'surrogate': error_stats([r['surrogate_error_m'] for r in subset])},
        'yaw_delta_deg': percentiles([r['estimated_minus_truth_yaw_deg'] for r in frames]),
        'ulog_bracket_rejections': dict(rejected), 'static_magnetic_field': magnetic,
        'px4_initial_magnetic_parameters': {
            k: v for k, v in log.initial_parameters.items()
            if k in ('EKF2_MAG_DECL', 'CAL_MAG0_ID', 'CAL_MAG0_XOFF',
                     'CAL_MAG0_YOFF', 'CAL_MAG0_ZOFF')},
        'ulog_xy_position_setpoint_span_m': [
            percentiles(setpoints[f'position[{i}]']) for i in range(2)],
        'limitations': ['valid production frames did not save geometry/online quaternion',
                        'surrogate is not post-fix simulation accuracy',
                        'raw image time uses existing native SITL clock identity',
                        'cached magnetic calibration and geomagnetic models remain to validate'],
    }
    return summary, frames


def main():
    """Write new evidence artifacts without overwriting input logs."""
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ('main', 'vision', 'ulog', 'output'):
        parser.add_argument('--'+option, type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve() in {p.resolve() for p in (args.main, args.vision, args.ulog)}:
        parser.error('output must not be an input')
    summary, frames = audit(args.main, args.vision, args.ulog)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    (args.output/'paired_frames.jsonl').write_text(''.join(
        json.dumps(r)+'\n' for r in frames))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
