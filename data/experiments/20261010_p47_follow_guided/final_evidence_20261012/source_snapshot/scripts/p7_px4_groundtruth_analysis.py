#!/usr/bin/env python3
"""Pair PX4 ULog groundtruth with P7 ROS/Gazebo axis capture offline."""

import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from p7_px4_local_axis_analysis import (
    align_ulog_ekf_to_capture, interpolated_ulog_value,
)


def _series(data, fields):
    times = np.asarray(data['timestamp_sample'], dtype=float) * 1e-6
    values = np.column_stack([data[field] for field in fields])
    finite = np.isfinite(times) & np.all(np.isfinite(values), axis=1)
    times, values = times[finite], values[finite]
    unique, indices = np.unique(times, return_index=True)
    return unique, values[indices]


def _metrics(vectors):
    array = np.asarray(vectors, dtype=float)
    if not len(array):
        return {'n': 0}
    norms = np.linalg.norm(array, axis=1)
    return {
        'n': int(len(array)),
        'rmse_m': float(np.sqrt(np.mean(norms ** 2))),
        'p50_m': float(np.percentile(norms, 50)),
        'p95_m': float(np.percentile(norms, 95)),
        'max_m': float(np.max(norms)),
        'signed_mean_xyz_m': array.mean(axis=0).tolist(),
    }


def _training_rows(rows):
    selected = []
    phases = (
        ('NORTH_OUT', 'EAST_OUT')
        if any(row.get('phase') == 'EAST_OUT' for row in rows)
        else ('NORTH_OUT', 'NORTH_RETURN')
    )
    for phase in phases:
        phase_rows = [row for row in rows if row.get('valid')
                      and row.get('phase') == phase]
        if len(phase_rows) < 30:
            raise ValueError(f'{phase} lacks clock train samples')
        selected.extend(phase_rows[:len(phase_rows) // 2])
    return sorted(selected, key=lambda row: row['ros_stamp']), phases


def analyze(capture, ulog_path, maximum_gap=0.05):
    log = ULog(str(ulog_path), message_name_filter_list=[
        'vehicle_local_position', 'vehicle_local_position_groundtruth',
    ])
    datasets = {dataset.name: dataset.data for dataset in log.data_list}
    if ('vehicle_local_position' not in datasets
            or 'vehicle_local_position_groundtruth' not in datasets):
        raise ValueError('ULog lacks EKF or simulator local-position truth')
    ekf = datasets['vehicle_local_position']
    ground = datasets['vehicle_local_position_groundtruth']
    ekf_time, ekf_xyz = _series(ekf, 'xyz')
    gt_time, gt_state = _series(
        ground, ('x', 'y', 'z', 'vx', 'vy', 'vz'))
    train, clock_train_phases = _training_rows(capture)
    offset, train_rmse = align_ulog_ekf_to_capture(
        [row['ros_stamp'] for row in train],
        [row['px4_local_position_ned'] for row in train],
        ekf_time, ekf_xyz)
    enriched = []
    self_errors = []
    bridge_errors = []
    ekf_errors = []
    by_phase = {}
    by_timing = {'lt10ms': [], '10to25ms': [], '25to50ms': []}
    gt_side_gaps = []
    for row in capture:
        result = dict(row)
        result['source_ulog'] = str(Path(ulog_path).resolve())
        result['ros_minus_ulog_boot_offset_s'] = offset
        result['ulog_query_stamp_s'] = row['ros_stamp'] - offset
        result['groundtruth_ulog_valid'] = False
        if not row.get('valid'):
            enriched.append(result)
            continue
        query = result['ulog_query_stamp_s']
        try:
            truth = interpolated_ulog_value(
                query, gt_time, gt_state, maximum_gap)
            same_ekf = interpolated_ulog_value(
                query, ekf_time, ekf_xyz, maximum_gap)
        except ValueError as exc:
            result['groundtruth_ulog_invalid_reason'] = str(exc)
            enriched.append(result)
            continue
        index = int(np.searchsorted(gt_time, query))
        exact = (index < len(gt_time)
                 and abs(gt_time[index] - query) <= 1e-8)
        left_index = index if exact else max(index - 1, 0)
        right_index = min(index, len(gt_time) - 1)
        left = float(gt_time[left_index])
        right = float(gt_time[right_index])
        side_gap = max(query - left, right - query)
        result['groundtruth_ulog_left_stamp_s'] = left
        result['groundtruth_ulog_right_stamp_s'] = right
        result['groundtruth_ulog_max_side_gap_s'] = side_gap
        result['px4_groundtruth_ulog_position_ned'] = truth[:3].tolist()
        result['px4_groundtruth_ulog_velocity_ned'] = truth[3:].tolist()
        result['groundtruth_ulog_valid'] = True
        gazebo = np.asarray(row['gazebo_uav_position_ned'])
        px4 = np.asarray(row['px4_local_position_ned'])
        bridge_error = truth[:3] - gazebo
        bridge_errors.append(bridge_error)
        ekf_errors.append(px4 - truth[:3])
        self_errors.append(same_ekf - px4)
        by_phase.setdefault(row['phase'], []).append(bridge_error)
        capture_gap = row['alignment_gap_s']
        if capture_gap < 0.01:
            by_timing['lt10ms'].append(bridge_error)
        elif capture_gap < 0.025:
            by_timing['10to25ms'].append(bridge_error)
        elif capture_gap <= 0.05:
            by_timing['25to50ms'].append(bridge_error)
        gt_side_gaps.append(side_gap)
        enriched.append(result)
    if not bridge_errors:
        raise ValueError('no bounded ULog groundtruth pairs')
    capture_times = [row['ros_stamp'] - offset for row in capture
                     if row.get('valid')]
    raw_ulog_times = np.asarray(ekf['timestamp_sample']) * 1e-6
    in_capture = ((raw_ulog_times >= min(capture_times))
                  & (raw_ulog_times <= max(capture_times)))
    reference_fields = (
        'ref_timestamp', 'ref_lat', 'ref_lon', 'ref_alt',
        'xy_reset_counter', 'z_reset_counter',
        'heading_reset_counter',
    )
    reference = {
        field: np.unique(ekf[field][in_capture]).tolist()
        for field in reference_fields if field in ekf
    }
    result = {
        'source_ulog': str(Path(ulog_path).resolve()),
        'clock_mapping_basis': (
            'fit PX4 EKF captured position to same ULog EKF position '
            'on first half of phases; no Gazebo fit'),
        'clock_train_phases': list(clock_train_phases),
        'ros_minus_ulog_boot_offset_s': offset,
        'clock_train_self_rmse_m': train_rmse,
        'clock_all_self_error': _metrics(self_errors),
        'groundtruth_pairs': len(bridge_errors),
        'gazebo_to_px4_groundtruth': _metrics(bridge_errors),
        'px4_groundtruth_to_ekf': _metrics(ekf_errors),
        'gazebo_to_groundtruth_by_phase': {
            phase: _metrics(errors) for phase, errors in by_phase.items()},
        'gazebo_to_groundtruth_by_capture_gap': {
            group: _metrics(errors) for group, errors in by_timing.items()},
        'groundtruth_ulog_side_gap_p95_s': float(np.percentile(
            gt_side_gaps, 95)),
        'capture_window_px4_reference_and_reset_values': reference,
    }
    return enriched, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--ulog', required=True, type=Path)
    parser.add_argument('--enriched-output', required=True, type=Path)
    parser.add_argument('--report-output', required=True, type=Path)
    args = parser.parse_args()
    with args.capture.open(encoding='utf-8') as stream:
        capture = [json.loads(line) for line in stream]
    enriched, result = analyze(capture, args.ulog)
    result['source_capture'] = str(args.capture.resolve())
    with args.enriched_output.open('x', encoding='utf-8') as stream:
        for row in enriched:
            stream.write(json.dumps(row, allow_nan=False) + '\n')
    with args.report_output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps({
        'clock_self_rmse_m': result['clock_train_self_rmse_m'],
        'groundtruth_pairs': result['groundtruth_pairs'],
        'bridge_rmse_m': result['gazebo_to_px4_groundtruth']['rmse_m'],
        'ekf_vs_truth_rmse_m': result['px4_groundtruth_to_ekf']['rmse_m'],
    }))


if __name__ == '__main__':
    main()
