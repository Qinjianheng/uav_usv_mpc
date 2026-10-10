#!/usr/bin/env python3
"""Offline physical attitude/thrust and producer-stage accounting; never an online data source."""
from collections import defaultdict
import csv
import json

import numpy as np

from p4_follow_analysis import distribution


def extend(root, report, kinds, start):
    """Use original native PX4 epochs, not fitted lags or monitor arrival derivatives."""
    states = [r for r in kinds.get('uav', []) if r['valid']]
    frames = [r for r in kinds.get('attitude', []) if start+20 <= r['receipt'] <= start+60]
    angles = []
    for row in frames:
        w, x, y, z = row['q']
        angles.append((np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                       np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                       np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))))
    physical = dict(attitude_rad={name: distribution([a[i] for a in angles])
                                  for i, name in enumerate(('roll', 'pitch', 'yaw'))})
    try:
        from pyulog import ULog
        native = np.array([r['native'] for r in states])
        epochs = np.array([r['stamp'] for r in states])
        if not len(native) or not np.all(np.diff(native) > 0):
            raise ValueError('NATIVE_NAVIGATION_MAPPING_NOT_MONOTONIC')
        for path in (root/'ulogs').glob('*.ulg'):
            ulog = ULog(str(path), message_name_filter_list=[
                'vehicle_angular_velocity', 'vehicle_thrust_setpoint', 'trajectory_setpoint'])
            for dataset in ulog.data_list:
                data = dataset.data
                ts = data.get('timestamp_sample', data['timestamp'])
                mapped = np.interp(ts, native, epochs)
                mask = ((ts >= native[0]) & (ts <= native[-1])
                        & (mapped >= start+20) & (mapped <= start+60))
                if dataset.name == 'vehicle_angular_velocity':
                    physical['body_rate_radps'] = {
                        axis: distribution(data[f'xyz[{i}]'][mask])
                        for i, axis in enumerate(('x', 'y', 'z'))}
                elif dataset.name == 'vehicle_thrust_setpoint':
                    physical['normalized_thrust_command'] = distribution(np.linalg.norm(
                        np.column_stack([data[f'xyz[{i}]'][mask] for i in range(3)]), axis=1))
                elif dataset.name == 'trajectory_setpoint':
                    physical['px4_logged_setpoints'] = dict(
                        count=int(sum(mask)), acceleration_ff_finite=int(sum(np.any(
                            np.isfinite(np.column_stack([data[f'acceleration[{i}]'][mask]
                                                         for i in range(3)])), axis=1))),
                        horizontal_speed=distribution(np.linalg.norm(np.column_stack(
                            [data[f'velocity[{i}]'][mask] for i in (0, 1)]), axis=1)))
        physical['epoch_mapping'] = 'ULog native sample to original navigation sample epoch pairs'
    except (ImportError, ValueError, OSError) as error:
        physical['ulog_unavailable_reason'] = str(error)
    report['physical'] = physical
    completions = [json.loads(line) for p in (root/'shadow').glob('mpc_seed_shadow_*.jsonl')
                   for line in p.open() if '"completion"' in line]
    exclusive = defaultdict(list)
    for row in completions:
        profiles = (row.get('preparation_profile'), row.get('core_result', {}).get(
            'metrics', {}).get('profile'))
        for profile in profiles:
            if profile:
                for name, stage in profile.get('stages', {}).items():
                    exclusive[name].append(stage['seconds'])
    report['exclusive_planner_stage_seconds'] = {
        key: distribution(values) for key, values in exclusive.items()}
    originals = [r for p in (root/'evaluator').glob('*mission_1.csv')
                 for r in csv.DictReader(p.open()) if r['phase'] == 'FOLLOW'
                 and r['truth_available'] == 'True' and r['uav_available'] == 'True']
    csv_start = float(originals[0]['time']) if originals else 0
    for name, bounds in (('all_follow', (0, float('inf'))), ('follow_20_to_60', (20, 60))):
        error = []
        for row in originals:
            if not bounds[0] <= float(row['time'])-csv_start <= bounds[1]:
                continue
            p = np.array([float(row['target_'+k]) for k in 'xy'])
            v = np.array([float(row['target_v'+k]) for k in 'xy'])
            up = np.array([float(row['uav_'+k]) for k in 'xy'])
            error.append(float(np.linalg.norm(up-(p-5*v/max(np.linalg.norm(v), 1e-9)))))
        window = report['windows'][name]
        h, z = window['position_horizontal_rmse'], window['altitude_rmse']
        window['position_3d_rmse'] = float(np.hypot(h, z)) if h is not None else None
        window['horizontal_peak_error'] = max(error) if error else None
