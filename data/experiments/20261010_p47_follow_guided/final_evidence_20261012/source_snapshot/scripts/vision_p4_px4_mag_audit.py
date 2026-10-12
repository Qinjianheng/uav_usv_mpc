#!/usr/bin/env python3
"""Reproduce ground magnetic heading and in-air fusion evidence from ULogs."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation

ULOG_DIR = Path('/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default'
                '/rootfs/log/2026-09-23')
OUTPUT = (Path(__file__).resolve().parents[1]
          / 'data/experiments/current/p4_20260927_px4_mag_audit.json')
SESSIONS = {
    'ground_and_5m': '08_20_54.ulg',
    '7m': '08_23_27.ulg',
    'motion_plus_y': '08_33_47.ulg',
    'motion_minus_y': '12_23_02.ulg',
}


def attitude_yaw(data, index):
    q = np.array([data[f'q[{i}]'][index] for i in range(4)])
    return float(Rotation.from_quat(q[[1, 2, 3, 0]]).as_euler('xyz')[2])


def audit(path):
    log = ULog(str(path), message_name_filter_list=[
        'vehicle_magnetometer', 'sensor_mag', 'vehicle_attitude',
        'vehicle_attitude_groundtruth', 'estimator_status_flags',
        'estimator_aid_src_mag', 'sensor_gps',
    ])
    data = {dataset.name: dataset.data for dataset in log.data_list}
    mag = data['vehicle_magnetometer']
    attitude = data['vehicle_attitude']
    groundtruth = data['vehicle_attitude_groundtruth']
    flags = data['estimator_status_flags']
    gps = data['sensor_gps']
    t0 = groundtruth['timestamp_sample'][0] / 1e6
    mt = mag['timestamp_sample'].astype(float) / 1e6
    at = attitude['timestamp_sample'].astype(float) / 1e6
    gt = groundtruth['timestamp_sample'].astype(float) / 1e6
    mask = (mt > t0 + 2) & (mt < t0 + 12)
    if not np.any(mask):
        raise ValueError(f'no ground magnetometer samples in {path}')
    field = [float(np.median(mag[f'magnetometer_ga[{i}]'][mask]))
             for i in range(3)]
    declination = math.radians(log.initial_parameters['EKF2_MAG_DECL'])
    predicted = math.atan2(-field[1], field[0]) + declination
    middle = float(np.median(mt[mask]))
    ekf_yaw = attitude_yaw(attitude, np.argmin(abs(at - middle)))
    model_yaw = attitude_yaw(groundtruth, np.argmin(abs(gt - middle)))
    flag_times = flags['timestamp_sample'].astype(float) / 1e6
    changes = {}
    for key in ('cs_yaw_align', 'cs_mag_hdg', 'cs_mag_3d',
                'cs_mag_aligned_in_flight', 'cs_in_air'):
        values = flags[key].astype(int)
        indices = np.r_[0, np.flatnonzero(np.diff(values)) + 1]
        changes[key] = [[float(flag_times[i] - t0), int(values[i])]
                        for i in indices]
    aids = data['estimator_aid_src_mag']
    aid_t = aids['timestamp_sample'].astype(float) / 1e6
    innovation = np.linalg.norm(np.array([
        aids[f'innovation[{i}]'] for i in range(3)
    ]).T, axis=1)
    air_start = next((t0 + t for t, value in changes['cs_in_air']
                      if value == 1), math.inf)
    innovation_sections = {}
    for label, section in (
        ('ground', (aid_t > t0 + 2) & (aid_t < t0 + 12)),
        ('flight', (aid_t > air_start + 4) & (aid_t < aid_t[-1] - 2)),
    ):
        innovation_sections[label] = {
            'n': int(np.count_nonzero(section)),
            'median_innovation_norm_ga': (
                float(np.median(innovation[section]))
                if np.any(section) else None),
            'fused_fraction': (
                float(np.mean(aids['fused'][section]))
                if np.any(section) else None),
        }
    parameters = {key: value for key, value in log.initial_parameters.items()
                  if key in ('EKF2_MAG_DECL', 'EKF2_DECL_TYPE',
                             'EKF2_MAG_TYPE', 'EKF2_MAG_CHECK',
                             'CAL_MAG0_XOFF', 'CAL_MAG0_YOFF',
                             'CAL_MAG0_ZOFF', 'SIM_GZ_EN_MAG')}
    return {
        'ulog': str(path),
        'ground_window_seconds_from_groundtruth_start': [2, 12],
        'ground_calibrated_mag_ga_xyz': field,
        'ground_mag_samples': int(np.count_nonzero(mask)),
        'declination_rad': declination,
        'predicted_ground_yaw_rad': predicted,
        'ekf_ground_yaw_rad': ekf_yaw,
        'model_ground_yaw_rad': model_yaw,
        'predicted_minus_ekf_yaw_rad': predicted - ekf_yaw,
        'first_gps_lat_lon_deg': [float(gps['latitude_deg'][0]),
                                  float(gps['longitude_deg'][0])],
        'parameters': parameters,
        'flag_changes_seconds_from_gt_start': changes,
        'mag_aid_innovation': innovation_sections,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    result = {name: audit(ULOG_DIR / filename)
              for name, filename in SESSIONS.items()}
    with args.output.open('x') as file:
        json.dump(result, file, indent=2, ensure_ascii=False,
                  allow_nan=False)
        file.write('\n')
    print(args.output)
    for name, item in result.items():
        print(name, item['ground_calibrated_mag_ga_xyz'],
              item['predicted_ground_yaw_rad'],
              item['ekf_ground_yaw_rad'],
              item['predicted_minus_ekf_yaw_rad'])


if __name__ == '__main__':
    main()
