#!/usr/bin/env python3
"""Compare target estimates only after naming and aligning their frames.

Evaluation only. Per-frame UAV-pose alignment is a diagnostic identity, not a
calibration to feed the online visual localizer or its downstream consumers.
"""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from uav_control.evaluation.pose_frame_diagnostics import (
    model_quaternion_from_residual, quaternion_rotation,
)
from uav_control.perception.rgbd_target_localizer import (
    camera_target_to_local_ned,
)


VERTICAL_NED = np.array((0.0, 0.0, 1.0))


def frame_mapping(model_position, model_rotation, px4_position,
                  px4_rotation):
    """Map one Gazebo-fixed NED point into PX4 local NED at one image time."""
    model_position = np.asarray(model_position, dtype=float)
    px4_position = np.asarray(px4_position, dtype=float)
    model_rotation = np.asarray(model_rotation, dtype=float)
    px4_rotation = np.asarray(px4_rotation, dtype=float)
    rotation = px4_rotation @ model_rotation.T
    translation = px4_position - rotation @ model_position
    return rotation, translation


def frame_error_vectors(
    camera_vector, physical_camera_vector, online_target,
    sphere_center_g, model_position_g, model_rotation_g,
    px4_position_p, px4_rotation_p, camera_translation,
    camera_pitch_down, target_reference_z_offset,
):
    """Return E1-E4 and an exact vector split of the cross-frame error.

    The sphere-center-to-reference offset follows the online convention:
    positive NED Z is added after the camera-to-local-frame rotation.
    E4 rigidly maps the Gazebo reference point as explicitly specified.
    Both conventions are reported because nonzero frame tilt separates them.
    """
    camera_vector = np.asarray(camera_vector, dtype=float)
    physical_camera_vector = np.asarray(physical_camera_vector, dtype=float)
    online_target = np.asarray(online_target, dtype=float)
    sphere_center_g = np.asarray(sphere_center_g, dtype=float)
    model_position_g = np.asarray(model_position_g, dtype=float)
    px4_position_p = np.asarray(px4_position_p, dtype=float)
    offset = float(target_reference_z_offset) * VERTICAL_NED
    reference_g = sphere_center_g + offset
    rotation, translation = frame_mapping(
        model_position_g, model_rotation_g, px4_position_p, px4_rotation_p,
    )
    model_quaternion_xyzw = Rotation.from_matrix(
        model_rotation_g
    ).as_quat()
    model_quaternion_wxyz = (
        model_quaternion_xyzw[3], *model_quaternion_xyzw[:3],
    )
    model_reconstruction = camera_target_to_local_ned(
        camera_vector, model_position_g, model_quaternion_wxyz,
        camera_translation, camera_pitch_down,
        target_reference_z_offset,
    )
    same_frame_reference = rotation @ reference_g + translation
    local_offset_reference = (
        rotation @ sphere_center_g + translation + offset
    )
    same_frame_error = online_target - same_frame_reference
    return {
        'camera_error': camera_vector - physical_camera_vector,
        'gazebo_reconstruction_error': model_reconstruction - reference_g,
        'cross_frame_error': online_target - reference_g,
        'same_frame_error': same_frame_error,
        'local_offset_same_frame_error': (
            online_target - local_offset_reference
        ),
        'reference_convention_difference': (
            same_frame_reference - local_offset_reference
        ),
        'frame_rotation_term': (
            (rotation - np.eye(3)) @ (reference_g - model_position_g)
        ),
        'frame_position_term': px4_position_p - model_position_g,
        'frame_rotation': rotation,
        'frame_translation': translation,
        'same_frame_reference': same_frame_reference,
    }


def vector_metrics(values):
    """Summarize a vector error without subtracting nonadditive RMSEs."""
    if not values:
        return {'n': 0}
    array = np.asarray(values, dtype=float)
    norms = np.linalg.norm(array, axis=1)
    return {
        'n': len(array),
        'rmse_3d_m': float(np.sqrt(np.mean(norms ** 2))),
        'p50_3d_m': float(np.percentile(norms, 50)),
        'p95_3d_m': float(np.percentile(norms, 95)),
        'max_3d_m': float(norms.max()),
        'signed_mean_xyz_m': array.mean(axis=0).tolist(),
    }


def _vector(row, prefix, axes='xyz'):
    return np.array([float(row[f'{prefix}_{axis}']) for axis in axes])


def _model_position_ned(row):
    enu = _vector(row, 'gazebo_model_world')
    return np.array((enu[1], enu[0], -enu[2]))


def _row_eligible(row):
    if row['valid'] != 'True':
        return False, 'invalid_observation'
    if (row['gazebo_entity_available'] != 'True'
            or row['gazebo_model_available'] != 'True'):
        return False, 'missing_gazebo_pose'
    for name in ('entity', 'model', 'px4_position', 'px4_attitude'):
        if row[f'{name}_status'] not in ('EXACT', 'INTERPOLATED'):
            return False, f'{name}_query_unavailable'
    if (row['px4_position_reset_crossed'] == 'True'
            or row['px4_attitude_reset_crossed'] == 'True'):
        return False, 'px4_reset_crossed'
    return True, ''


def _camera_config(path):
    with Path(path).open(encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['rgbd_target_localizer'][
            'ros__parameters'
        ]
    return (
        tuple(float(params[f'camera_translation_{axis}'])
              for axis in 'xyz'),
        float(params['camera_pitch_down']),
        float(params['target_reference_z_offset']),
    )


def _detail(row, camera_config):
    translation, pitch, z_offset = camera_config
    q_px4 = _vector(row, 'px4_attitude_independent', 'wxyz')
    q_model = model_quaternion_from_residual(
        q_px4, tuple(float(row[f'model_px4_{axis}_residual'])
                     for axis in ('roll', 'pitch', 'yaw')),
    )
    rotation_px4 = quaternion_rotation(q_px4)
    rotation_model = quaternion_rotation(q_model)
    online = _vector(row, 'position')
    camera = _vector(row, 'center_camera')
    physical_camera = _vector(row, 'physical_expected_camera')
    px4_position = _vector(row, 'px4_position_independent')
    result = frame_error_vectors(
        camera, physical_camera, online,
        _vector(row, 'gazebo_entity_center'), _model_position_ned(row),
        rotation_model, px4_position, rotation_px4,
        translation, pitch, z_offset,
    )
    replay = camera_target_to_local_ned(
        camera, px4_position, q_px4, translation, pitch, z_offset,
    )
    result['online_replay_error_m'] = float(np.linalg.norm(replay - online))
    for key, logged in (
        ('camera_error', 'physical_camera_center_error_3d'),
        ('cross_frame_error', 'vision_to_entity_3d'),
    ):
        if abs(np.linalg.norm(result[key]) - float(row[logged])) > 1e-5:
            raise ValueError(f'{key} does not reproduce logged {logged}')
    if not all(np.all(np.isfinite(value)) for value in result.values()):
        raise ValueError('nonfinite complete frame')
    result['measurement_stamp'] = float(row['measurement_stamp'])
    result['frame_yaw_rad'] = math.atan2(
        result['frame_rotation'][1, 0], result['frame_rotation'][0, 0],
    )
    return result


def _phase_summary(details):
    if not details:
        return {'n': 0}
    keys = (
        'camera_error', 'gazebo_reconstruction_error',
        'cross_frame_error', 'same_frame_error',
        'local_offset_same_frame_error', 'frame_rotation_term',
        'frame_position_term', 'reference_convention_difference',
    )
    result = {key: vector_metrics([d[key] for d in details])
              for key in keys}
    yaws = np.array([d['frame_yaw_rad'] for d in details])
    shifts = np.array([d['frame_translation'] for d in details])
    result['frame_yaw_rad'] = {
        'mean': float(yaws.mean()), 'std': float(yaws.std()),
        'min': float(yaws.min()), 'max': float(yaws.max()),
    }
    result['frame_translation_m'] = {
        'mean_xyz': shifts.mean(axis=0).tolist(),
        'std_xyz': shifts.std(axis=0).tolist(),
    }
    result['max_online_replay_error_m'] = max(
        d['online_replay_error_m'] for d in details
    )
    calibration_index = max(1, len(details) // 5)
    reference = details[calibration_index - 1]
    fixed_rotation = reference['frame_rotation']
    fixed_translation = reference['frame_translation']
    result['fixed_map_holdout'] = vector_metrics([
        d['online_target'] - (
            fixed_rotation @ d['gazebo_reference'] + fixed_translation
        ) for d in details[calibration_index:]
    ])
    result['fixed_map_calibration_index'] = calibration_index - 1
    return result


def analyze(csv_path, config_path):
    """Return auditable same-image-time metrics from one capture CSV."""
    config = _camera_config(config_path)
    with Path(csv_path).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    excluded = Counter()
    details = []
    for row in rows:
        eligible, reason = _row_eligible(row)
        if not eligible:
            excluded[reason] += 1
            continue
        try:
            item = _detail(row, config)
        except (KeyError, ValueError) as exc:
            excluded[f'complete_frame_error:{exc}'] += 1
            continue
        item['online_target'] = _vector(row, 'position')
        item['gazebo_reference'] = _vector(row, 'gazebo_entity_reference')
        details.append(item)
    if not details:
        raise ValueError(f'no complete valid frames in {csv_path}')
    details.sort(key=lambda item: item['measurement_stamp'])
    first_time = details[0]['measurement_stamp']
    last_time = details[-1]['measurement_stamp']
    phases = {
        'all': details,
        'first_10_seconds': [d for d in details
                             if d['measurement_stamp'] <= first_time + 10],
        'last_15_seconds': [d for d in details
                            if d['measurement_stamp'] >= last_time - 15],
    }
    return {
        'source_csv': str(csv_path), 'source_config': str(config_path),
        'total_rows': len(rows), 'complete_rows': len(details),
        'excluded': dict(excluded),
        'definitions': {
            'G': 'Gazebo world ENU converted to fixed NED',
            'P': 'PX4 EKF local NED', 'B': 'UAV body FRD',
            'E1': 'B camera center minus physical camera sphere center',
            'E2': 'B camera vector with exact model pose minus G reference',
            'E3': 'online P target minus raw G reference (cross-frame)',
            'E4': 'online P target minus rigidly mapped G reference',
            'reference': 'sphere center plus +Z NED offset from config',
            'warning': ('Per-frame map forces the two UAV poses to agree; '
                        'E4~E1 alone does not prove a fixed frame map.'),
        },
        'phases': {name: _phase_summary(items)
                   for name, items in phases.items()},
        'per_frame': [
            {
                'measurement_stamp': d['measurement_stamp'],
                'frame_yaw_rad': d['frame_yaw_rad'],
                'frame_translation_xyz_m': d['frame_translation'].tolist(),
                'target_truth_px4_local_ned': (
                    d['same_frame_reference'].tolist()
                ),
                'same_frame_target_error_3d': float(np.linalg.norm(
                    d['same_frame_error']
                )),
                **{key: d[key].tolist() for key in (
                    'camera_error', 'gazebo_reconstruction_error',
                    'cross_frame_error', 'same_frame_error',
                    'frame_rotation_term', 'frame_position_term',
                )},
            } for d in details
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', action='append', required=True,
                        help='name=csv_path=config_yaml_path')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {}
    for entry in args.scenario:
        name, csv_path, config_path = entry.split('=', 2)
        result[name] = analyze(Path(csv_path), Path(config_path))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    for name, item in result.items():
        metrics = item['phases']['all']
        print(name, 'n=', item['complete_rows'],
              'E1=', metrics['camera_error']['rmse_3d_m'],
              'E2=', metrics['gazebo_reconstruction_error']['rmse_3d_m'],
              'E3=', metrics['cross_frame_error']['rmse_3d_m'],
              'E4=', metrics['same_frame_error']['rmse_3d_m'])


if __name__ == '__main__':
    main()
