#!/usr/bin/env python3
"""Convert synchronized P6 capture rows into evaluation-only PX4 NED truth.

The input CSV already contains interpolated Gazebo and PX4 poses at the RGB
measurement timestamp. No truth produced here is published to a ROS topic.
"""

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

from scipy.spatial.transform import Rotation


SOURCES = ('entity', 'model', 'px4_position', 'px4_attitude')
RESET_FIELDS = (
    'image_clock_reset_count',
    'entity_clock_reset_count_at_write',
    'px4_clock_reset_count_at_write',
    'px4_xy_reset_counter',
    'px4_z_reset_counter',
    'px4_heading_reset_counter',
    'px4_quat_reset_counter',
)


def _number(row, key):
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f'{key} is not finite')
    return value


def _vector(row, prefix, axes='xyz'):
    return [_number(row, f'{prefix}_{axis}') for axis in axes]


def _enu_to_ned(value):
    return [value[1], value[0], -value[2]]


def _reset_signature(row):
    return tuple(row.get(key, '') for key in RESET_FIELDS)


def _model_quaternion(px4, residual_rpy):
    """Use the pose_frame_diagnostics residual convention without ROS."""
    px4_xyzw = (px4[1], px4[2], px4[3], px4[0])
    model = Rotation.from_euler('xyz', residual_rpy) * Rotation.from_quat(
        px4_xyzw,
    )
    x, y, z, w = model.as_quat()
    return [w, x, y, z]


def build_record(row, max_alignment_gap=0.05):
    """Build one frame, rejecting missing brackets, resets and large gaps."""
    stamp = float(row.get('measurement_stamp', 'nan'))
    record = {
        'timestamp': stamp if math.isfinite(stamp) else None,
        'frame_valid': False,
        'invalid_reason': '',
        'timestamp_alignment_error_s': None,
        'reset_counters': {key: row.get(key, '') for key in RESET_FIELDS},
        'source_query': {},
    }
    try:
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('invalid_measurement_stamp')
        for key in ('truth_available', 'gazebo_entity_available',
                    'gazebo_model_available'):
            if row.get(key) != 'True':
                raise ValueError(key)
        for key in ('px4_position_reset_crossed',
                    'px4_attitude_reset_crossed'):
            if row.get(key) == 'True':
                raise ValueError(key)
        gaps = []
        for name in SOURCES:
            status = row.get(f'{name}_status', '')
            if status not in ('EXACT', 'INTERPOLATED'):
                raise ValueError(f'{name}_status={status}')
            left = _number(row, f'{name}_left_ros_stamp')
            right = _number(row, f'{name}_right_ros_stamp')
            if left > stamp + 1e-6 or right < stamp - 1e-6:
                raise ValueError(f'{name}_not_bracketed')
            gaps.extend((abs(stamp - left), abs(right - stamp)))
            source_suffix = (
                'sim_stamp' if name in ('entity', 'model')
                else 'source_stamp'
            )
            record['source_query'][name] = {
                'status': status, 'left_ros_stamp': left,
                'right_ros_stamp': right,
                'left_source_stamp': _number(
                    row, f'{name}_left_{source_suffix}'),
                'right_source_stamp': _number(
                    row, f'{name}_right_{source_suffix}'),
            }
        alignment_error = max(gaps)
        record['timestamp_alignment_error_s'] = alignment_error
        if alignment_error > max_alignment_gap:
            raise ValueError('alignment_gap_exceeded')

        # The CSV entity-center fields are already NED; model-world is ENU.
        target_ned = _vector(row, 'gazebo_entity_center')
        target_enu = _enu_to_ned(target_ned)
        model_enu = _vector(row, 'gazebo_model_world')
        model_ned = _enu_to_ned(model_enu)
        px4_position = _vector(row, 'px4_position_independent')
        px4_attitude = _vector(row, 'px4_attitude_independent', 'wxyz')
        residual = [
            _number(row, f'model_px4_{axis}_residual')
            for axis in ('roll', 'pitch', 'yaw')
        ]
        model_attitude = _model_quaternion(px4_attitude, residual)
        reference_z = _number(row, 'target_reference_z_offset')
        relative = [target_ned[i] - model_ned[i] for i in range(3)]
        relative[2] += reference_z
        target_truth = [px4_position[i] + relative[i] for i in range(3)]
        velocity = _vector(row, 'truth_velocity')
        record.update({
            'gazebo_target_position_enu': target_enu,
            'gazebo_uav_position_enu': model_enu,
            'gazebo_uav_attitude_ned_frd_wxyz': model_attitude,
            'px4_uav_position_ned': px4_position,
            'px4_uav_attitude_ned_frd_wxyz': px4_attitude,
            'target_relative_truth_ned': relative,
            'target_truth_px4_ned': target_truth,
            'target_truth_velocity_px4_ned': velocity,
            'target_truth_velocity_finite_difference_ned': None,
            'target_reference_z_offset_m': reference_z,
            'frame_valid': True,
        })
    except (KeyError, TypeError, ValueError) as exc:
        record['invalid_reason'] = str(exc)
    return record


def convert_rows(rows, max_alignment_gap=0.05):
    """Segment resets and time reversals; never bridge epochs for velocity."""
    records = []
    previous_stamp = None
    previous_signature = None
    epoch = 0
    for row in rows:
        current_stamp = float(row.get('measurement_stamp', 'nan'))
        signature = _reset_signature(row)
        reversed_time = (
            previous_stamp is not None and current_stamp <= previous_stamp
        )
        if previous_signature is not None and (
                signature != previous_signature or reversed_time):
            epoch += 1
        record = build_record(row, max_alignment_gap)
        record['reset_epoch'] = epoch
        if reversed_time:
            record['frame_valid'] = False
            record['invalid_reason'] = 'timestamp_reversal'
        if record['frame_valid'] and records:
            previous = records[-1]
            dt = current_stamp - previous['timestamp']
            if (previous['frame_valid']
                    and previous['reset_epoch'] == epoch
                    and 0 < dt <= 0.3):
                record['target_truth_velocity_finite_difference_ned'] = [
                    (a - b) / dt for a, b in zip(
                        record['target_truth_px4_ned'],
                        previous['target_truth_px4_ned'],
                    )
                ]
        records.append(record)
        previous_stamp = current_stamp
        previous_signature = signature
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--max-alignment-gap', type=float, default=0.05)
    parser.add_argument('--px4-ref-timestamp-us', type=int)
    parser.add_argument('--px4-ulog', type=Path)
    args = parser.parse_args()
    if args.max_alignment_gap <= 0:
        parser.error('--max-alignment-gap must be positive')
    with args.input.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    records = convert_rows(rows, args.max_alignment_gap)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        for record in records:
            record['source_csv'] = str(args.input.resolve())
            record['px4_local_origin_ref_timestamp_us'] = (
                args.px4_ref_timestamp_us
            )
            record['px4_ulog'] = (
                str(args.px4_ulog.resolve()) if args.px4_ulog else None
            )
            stream.write(json.dumps(record, allow_nan=False) + '\n')
    summary = {
        'input': str(args.input.resolve()),
        'output': str(args.output.resolve()),
        'total_frames': len(records),
        'valid_frames': sum(record['frame_valid'] for record in records),
        'reset_epochs': len({record['reset_epoch'] for record in records}),
        'invalid_reasons': dict(Counter(
            record['invalid_reason'] for record in records
            if not record['frame_valid']
        )),
        'max_alignment_gap_s': args.max_alignment_gap,
        'px4_local_origin_ref_timestamp_us': args.px4_ref_timestamp_us,
        'px4_ulog': (
            str(args.px4_ulog.resolve()) if args.px4_ulog else None
        ),
        'position_truth_definition': 'px4_uav_ned + gazebo_world_ned'
        '(target_reference - uav)',
        'velocity_truth_definition': 'logged target commanded NED velocity',
    }
    summary_path = args.output.with_suffix('.summary.json')
    with summary_path.open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
