#!/usr/bin/env python3
"""Reproduce P3 same-image-frame UAV-pose counterfactuals from P2 CSVs.

Only evaluation data are read. Gazebo pose never feeds the online localizer.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from uav_control.evaluation.pose_frame_diagnostics import (
    four_pose_counterfactuals, model_quaternion_from_residual,
)
from uav_control.perception.rgbd_target_localizer import (
    local_ned_target_to_camera_flu,
)

SCENARIOS = {
    'ground_center': ('162108_442464', 'static5', 'ground'),
    'follow_5m': ('162137_767644', 'static5', 'follow'),
    'follow_7m': ('162407_184449', 'static7', 'follow'),
    'ground_left': ('162559_286251', 'yaw_plus', 'ground'),
    'follow_left': ('162617_441613', 'yaw_plus', 'follow'),
    'ground_right': ('162805_447409', 'yaw_minus', 'ground'),
    'follow_right': ('163025_736132', 'yaw_minus', 'follow'),
    'motion_plus_y': ('163417_715972', 'motion_plus_v2', 'motion'),
    'motion_minus_y': ('202327_524289', 'motion_minus_v2', 'motion'),
}


def vec(row, prefix):
    return np.array([float(row[f'{prefix}_{axis}']) for axis in 'xyz'])


def sample_ok(row):
    return (
        row['valid'] == 'True'
        and row['gazebo_entity_available'] == 'True'
        and row['gazebo_model_available'] == 'True'
        and all(row[k] in ('EXACT', 'INTERPOLATED') for k in (
            'entity_status', 'model_status', 'px4_position_status',
            'px4_attitude_status',
        ))
        and row['px4_position_reset_crossed'] == 'False'
        and row['px4_attitude_reset_crossed'] == 'False'
    )


def summarize(rows):
    if not rows:
        return {'n': 0}
    values = np.array([[float(row[f'error_{axis}']) for axis in 'xyz']
                       for row in rows])
    norms = np.linalg.norm(values, axis=1)
    return {
        'n': len(rows),
        'signed_mean_xyz_m': values.mean(axis=0).tolist(),
        'rmse_3d_m': float(np.sqrt(np.mean(norms ** 2))),
        'median_3d_m': float(np.median(norms)),
        'p95_3d_m': float(np.percentile(norms, 95)),
    }


def analyze(root, output_prefix):
    details = []
    summary = {'definitions': {
        'reference': 'gazebo_entity_reference_xyz, same image time, NED',
        'A': 'PX4 position + PX4 attitude (online reproduction)',
        'B': 'Gazebo model position + Gazebo model attitude',
        'C': 'Gazebo model position + PX4 attitude',
        'D': 'PX4 position + Gazebo model attitude',
        'filter': ('complete valid frames; FOLLOW last 15 s; '
                   'motion |USV vy| in [3.8,4.1] m/s'),
    }, 'scenarios': {}}
    for name, (suffix, config_name, phase) in SCENARIOS.items():
        csv_path = (root / 'data/experiments/current'
                    / f'vision_static_capture_20260923_{suffix}.csv')
        config_path = (root / 'data/experiments/current'
                       / f'p2_20260923_{config_name}_config.yaml')
        with config_path.open() as f:
            params = yaml.safe_load(f)['rgbd_target_localizer'][
                'ros__parameters']
        translation = tuple(
            float(params[f'camera_translation_{axis}']) for axis in 'xyz'
        )
        pitch = float(params['camera_pitch_down'])
        z_offset = float(params['target_reference_z_offset'])
        with csv_path.open(newline='') as f:
            all_rows = list(csv.DictReader(f))
        eligible = [r for r in all_rows if sample_ok(r)]
        if phase == 'follow':
            end = max(float(r['measurement_stamp']) for r in all_rows)
            eligible = [
                r for r in eligible
                if float(r['measurement_stamp']) >= end - 15.0
            ]
        elif phase == 'motion':
            eligible = [
                r for r in eligible
                if 3.8 <= abs(float(r['truth_velocity_y'])) <= 4.1
            ]
        count = 0
        max_online_replay_error = 0.0
        max_model_camera_replay_error = 0.0
        per_scenario = []
        for r in eligible:
            q_px4 = tuple(
                float(r[f'px4_attitude_independent_{a}']) for a in 'wxyz'
            )
            q_model = model_quaternion_from_residual(
                q_px4, tuple(
                    float(r[f'model_px4_{a}_residual'])
                    for a in ('roll', 'pitch', 'yaw')
                ),
            )
            px4_position = vec(r, 'interpolated_uav')
            model_enu = vec(r, 'gazebo_model_world')
            model_position = np.array((
                model_enu[1], model_enu[0], -model_enu[2],
            ))
            camera_vector = vec(r, 'center_camera')
            reference = vec(r, 'gazebo_entity_reference')
            poses = four_pose_counterfactuals(
                camera_vector, px4_position, model_position, q_px4, q_model,
                translation, pitch, z_offset,
            )
            online_replay_error = float(np.linalg.norm(
                poses['A'] - vec(r, 'position'),
            ))
            physical_replay = local_ned_target_to_camera_flu(
                reference - np.array((0.0, 0.0, z_offset)),
                model_position, q_model, translation, pitch,
            )
            camera_replay_error = float(np.linalg.norm(
                physical_replay - vec(r, 'physical_expected_camera'),
            ))
            if online_replay_error > 1e-4 or camera_replay_error > 1e-3:
                raise ValueError(
                    f'{name} image={r["measurement_stamp"]}: replay '
                    f'mismatch online={online_replay_error}, '
                    f'physical={camera_replay_error}'
                )
            max_online_replay_error = max(
                max_online_replay_error, online_replay_error,
            )
            max_model_camera_replay_error = max(
                max_model_camera_replay_error, camera_replay_error,
            )
            common = {
                'scenario': name, 'measurement_stamp': r['measurement_stamp'],
                'model_minus_px4_x': float(
                    model_position[0] - px4_position[0]),
                'model_minus_px4_y': float(
                    model_position[1] - px4_position[1]),
                'model_minus_px4_z': float(
                    model_position[2] - px4_position[2]),
                'model_minus_px4_yaw_rad': float(r['model_px4_yaw_residual']),
                'uav_velocity_latest_x': float(r['uav_velocity_latest_x']),
                'uav_velocity_latest_y': float(r['uav_velocity_latest_y']),
                'uav_velocity_latest_z': float(r['uav_velocity_latest_z']),
            }
            for key, estimate in poses.items():
                error = estimate - reference
                row = {**common, 'pose_case': key,
                       **{f'error_{a}': float(error[i])
                          for i, a in enumerate('xyz')},
                       'error_3d': float(np.linalg.norm(error))}
                details.append(row)
                per_scenario.append(row)
            count += 1
        summary['scenarios'][name] = {
            'source_csv': str(csv_path.relative_to(root)),
            'config_yaml': str(config_path.relative_to(root)),
            'rows_total': len(all_rows), 'rows_selected': count,
            'max_online_replay_error_m': max_online_replay_error,
            'max_model_camera_replay_error_m': max_model_camera_replay_error,
            'cases': {
                key: summarize([
                    row for row in per_scenario if row['pose_case'] == key
                ]) for key in 'ABCD'
            },
        }
    csv_output = output_prefix.with_suffix('.csv')
    json_output = output_prefix.with_suffix('.json')
    with csv_output.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(details[0]))
        writer.writeheader()
        writer.writerows(details)
    with json_output.open('x') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write('\n')
    return summary, csv_output, json_output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--root', type=Path, default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument('--output-prefix', type=Path, required=True)
    args = parser.parse_args()
    summary, csv_output, json_output = analyze(args.root, args.output_prefix)
    print(csv_output)
    print(json_output)
    for name, item in summary['scenarios'].items():
        print(name, item['rows_selected'], ' '.join(
            f'{key}={item["cases"][key]["rmse_3d_m"]:.4f}' for key in 'ABCD'))


if __name__ == '__main__':
    main()
