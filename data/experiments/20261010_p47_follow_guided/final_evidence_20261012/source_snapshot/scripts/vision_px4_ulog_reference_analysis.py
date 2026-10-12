#!/usr/bin/env python3
"""Compare PX4 EKF, PX4 simulator truth, and Gazebo UAV pose offline."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation

CASES = {
    'plus_y': ('08_33_47.ulg', '163417_715972'),
    'minus_y': ('12_23_02.ulg', '202327_524289'),
}


def xyz(data, times):
    sample_times = data['timestamp_sample'].astype(float) / 1e6
    return np.array([
        np.interp(times, sample_times, data[a]) for a in 'xyz'
    ]).T


def yaw(quaternions):
    return Rotation.from_quat(quaternions[:, [1, 2, 3, 0]]).as_euler(
        'xyz', degrees=False,
    )[:, 2]


def source_quaternions(data, indices):
    return np.array([data[f'q[{i}]'][indices] for i in range(4)]).T


def analyze_session(root, ulog_dir, name, ulog_name, csv_suffix):
    ulog_path = ulog_dir / ulog_name
    csv_path = (root / 'data/experiments/current'
                / f'vision_static_capture_20260923_{csv_suffix}.csv')
    log = ULog(str(ulog_path), message_name_filter_list=[
        'vehicle_local_position', 'vehicle_local_position_groundtruth',
        'vehicle_attitude', 'vehicle_attitude_groundtruth',
        'estimator_status_flags',
    ])
    data = {dataset.name: dataset.data for dataset in log.data_list}
    estimated = data['vehicle_local_position']
    ground = data['vehicle_local_position_groundtruth']
    et = estimated['timestamp_sample'].astype(float) / 1e6
    gt = ground['timestamp_sample'].astype(float) / 1e6
    with csv_path.open(newline='') as file:
        all_rows = list(csv.DictReader(file))
    rows = [
        row for row in all_rows
        if row['valid'] == 'True'
        and row['model_status'] in ('EXACT', 'INTERPOLATED')
        and 3.8 <= abs(float(row['truth_velocity_y'])) <= 4.1
    ]
    csv_time = np.array([float(row['measurement_stamp']) for row in rows])
    px4_csv = np.array([
        [float(row[f'interpolated_uav_{a}']) for a in 'xyz']
        for row in rows
    ])
    model = np.array([
        [float(row['gazebo_model_world_y']),
         float(row['gazebo_model_world_x']),
         -float(row['gazebo_model_world_z'])]
        for row in rows
    ])
    relative_time = csv_time - csv_time[0]
    lower = max(et[0], gt[0]) - relative_time[0]
    upper = min(et[-1], gt[-1]) - relative_time[-1]
    if lower >= upper:
        raise ValueError(f'{name}: no common ULog time range')

    def self_alignment(first_ulog_time):
        return float(np.sqrt(np.mean(np.sum((
            xyz(estimated, relative_time + first_ulog_time) - px4_csv
        ) ** 2, axis=1))))

    # This fits each session's clock epoch using PX4 against itself. It does
    # not fit a PX4/Gazebo physical latency or modify either data stream.
    grid = np.linspace(lower, upper, 2001)
    best = int(np.argmin([self_alignment(t) for t in grid]))
    local_lower = grid[max(0, best - 2)]
    local_upper = grid[min(len(grid) - 1, best + 2)]
    optimum = minimize_scalar(
        self_alignment, method='bounded',
        bounds=(local_lower, local_upper),
        options={'xatol': 1e-8},
    )
    aligned_time = relative_time + optimum.x
    ground_at_image = xyz(ground, aligned_time)
    px4_at_image = xyz(estimated, aligned_time)
    velocities = np.array([
        np.interp(aligned_time, gt, ground[f'v{a}']) for a in 'xyz'
    ]).T
    position = {
        'n': len(rows),
        'csv_first_image_ros_stamp': float(csv_time[0]),
        'csv_first_image_ulog_seconds': float(optimum.x),
        'px4_self_alignment_rmse_m': self_alignment(optimum.x),
        'px4_self_alignment_max_m': float(np.max(np.linalg.norm(
            px4_at_image - px4_csv, axis=1,
        ))),
        'uav_groundtruth_velocity_mean_xyz_mps': (
            velocities.mean(axis=0).tolist()),
        'model_minus_px4_mean_xyz_m': (model - px4_csv).mean(axis=0).tolist(),
        'px4_groundtruth_minus_px4_mean_xyz_m': (
            ground_at_image - px4_csv).mean(axis=0).tolist(),
        'model_minus_px4_groundtruth_mean_xyz_m': (
            model - ground_at_image).mean(axis=0).tolist(),
        'model_minus_px4_groundtruth_rmse_3d_m': float(np.sqrt(np.mean(
            np.sum((model - ground_at_image) ** 2, axis=1),
        ))),
    }
    attitude = data['vehicle_attitude']
    truth_attitude = data['vehicle_attitude_groundtruth']
    at = attitude['timestamp_sample'].astype(float) / 1e6
    gat = truth_attitude['timestamp_sample'].astype(float) / 1e6
    nearest = np.searchsorted(at, gat).clip(1, len(at) - 1)
    nearest = np.where(
        abs(at[nearest] - gat) < abs(at[nearest - 1] - gat),
        nearest, nearest - 1,
    )
    delta = yaw(source_quaternions(truth_attitude, slice(None))) - yaw(
        source_quaternions(attitude, nearest),
    )
    delta = np.arctan2(np.sin(delta), np.cos(delta))
    flags = data['estimator_status_flags']
    flag_times = flags['timestamp_sample'].astype(float) / 1e6
    watched = ('cs_mag_hdg', 'cs_mag_3d', 'cs_mag_aligned_in_flight',
               'cs_gnss_yaw', 'cs_yaw_align', 'cs_in_air',
               'cs_mag_field_disturbed')
    flag_changes = {}
    for key in watched:
        values = flags[key].astype(int)
        changes = np.r_[0, np.flatnonzero(np.diff(values)) + 1]
        flag_changes[key] = [
            [float(flag_times[i] - gat[0]), int(values[i])]
            for i in changes
        ]
    return {
        'source_ulog': str(ulog_path),
        'source_csv': str(csv_path.relative_to(root)),
        'position': position,
        'yaw': {
            'first_5s_median_model_minus_px4_rad': float(np.median(
                delta[gat < gat[0] + 5],
            )),
            'last_15s_median_model_minus_px4_rad': float(np.median(
                delta[gat > gat[-1] - 15],
            )),
            'attitude_match_p95_ms': float(np.percentile(
                abs(at[nearest] - gat) * 1000, 95,
            )),
        },
        'estimator_status_flag_changes_seconds_from_groundtruth_start': (
            flag_changes
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path,
                        default=Path(__file__).resolve().parents[1])
    parser.add_argument('--ulog-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = {
        name: analyze_session(args.root, args.ulog_dir, name, *case)
        for name, case in CASES.items()
    }
    with args.output.open('x') as file:
        json.dump(results, file, indent=2, ensure_ascii=False,
                  allow_nan=False)
        file.write('\n')
    print(args.output)
    for name, case in results.items():
        print(name, case['position'])


if __name__ == '__main__':
    main()
