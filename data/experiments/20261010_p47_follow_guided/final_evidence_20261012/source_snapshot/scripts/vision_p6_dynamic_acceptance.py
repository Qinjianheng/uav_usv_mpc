#!/usr/bin/env python3
"""Combine P6 frame metrics with dynamic availability and skew bins."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

import numpy as np


def scalar_stats(values):
    values = np.asarray([value for value in values
                         if math.isfinite(value)], dtype=float)
    if not len(values):
        return {'n': 0, 'p50': None, 'p95': None, 'max': None}
    return {
        'n': int(len(values)),
        'p50': float(np.percentile(values, 50)),
        'p95': float(np.percentile(values, 95)),
        'max': float(np.max(values)),
    }


def summarize(csv_path, frame_path, pose_path):
    with csv_path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    frame = json.loads(frame_path.read_text(encoding='utf-8'))
    if 'per_frame' not in frame:
        if len(frame) != 1:
            raise ValueError('frame JSON must contain one scenario')
        frame = next(iter(frame.values()))
    pose = json.loads(pose_path.read_text(encoding='utf-8'))
    valid = [row for row in rows if row['valid'] == 'True']
    reasons = Counter(row['rejection_reason'] or 'VALID' for row in rows)
    frame_by_stamp = {
        float(row['measurement_stamp']): row
        for row in frame['per_frame']
    }
    bins = {
        'abs_skew_lt_10_ms': [],
        'abs_skew_40_to_60_ms': [],
        'abs_skew_gt_80_ms': [],
    }
    matched = 0
    for row in valid:
        stamp = float(row['measurement_stamp'])
        detail = frame_by_stamp.get(stamp)
        if detail is None:
            continue
        skew = abs(float(row['rgb_depth_acquisition_skew']))
        if not math.isfinite(skew):
            continue
        matched += 1
        error = float(np.linalg.norm(detail['camera_error']))
        if skew < 0.010:
            bins['abs_skew_lt_10_ms'].append(error)
        if 0.040 <= skew <= 0.060:
            bins['abs_skew_40_to_60_ms'].append(error)
        if skew > 0.080:
            bins['abs_skew_gt_80_ms'].append(error)
    bin_stats = {}
    for name, values in bins.items():
        item = scalar_stats(values)
        item['rmse'] = (float(np.sqrt(np.mean(np.square(values))))
                        if values else None)
        bin_stats[name] = item
    skew = [float(row['rgb_depth_acquisition_skew']) for row in rows]
    all_metrics = frame['phases']['all']
    return {
        'source_csv': str(csv_path),
        'total_observations': len(rows),
        'valid_observations': len(valid),
        'valid_rate': len(valid) / len(rows) if rows else None,
        'rejection_reasons': dict(sorted(reasons.items())),
        'after_history_count': pose[
            'position_timestamp_after_history_count'],
        'after_history_rate': pose[
            'position_timestamp_after_history_rate'],
        'pose_future_gap_seconds': {
            kind: pose['valid'][f'{kind}_future_gap']
            for kind in ('position', 'attitude')
        },
        'rgb_depth_skew_seconds': scalar_stats(skew),
        'absolute_rgb_depth_skew_seconds': scalar_stats(
            [abs(value) for value in skew]),
        'valid_absolute_rgb_depth_skew_seconds': scalar_stats(
            [abs(float(row['rgb_depth_acquisition_skew']))
             for row in valid]),
        'skew_bin_e1_m': bin_stats,
        'skew_bin_matched_frame_count': matched,
        'complete_physical_frame_count': frame['complete_rows'],
        'e1_camera_m': all_metrics['camera_error'],
        'e4_same_frame_m': all_metrics['same_frame_error'],
        'e3_cross_frame_diagnostic_m': all_metrics['cross_frame_error'],
        'frame_yaw_rad': all_metrics['frame_yaw_rad'],
        'frame_translation_m': all_metrics['frame_translation_m'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', action='append', required=True,
                        help='name=csv_path=frame_json=pose_json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {}
    for entry in args.scenario:
        name, csv_path, frame_path, pose_path = entry.split('=', 3)
        result[name] = summarize(
            Path(csv_path), Path(frame_path), Path(pose_path),
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    for name, item in result.items():
        print(name, item['valid_observations'], '/',
              item['total_observations'], 'E1',
              item['e1_camera_m']['rmse_3d_m'], 'E4',
              item['e4_same_frame_m']['rmse_3d_m'])


if __name__ == '__main__':
    main()
