#!/usr/bin/env python3
"""Replay alternative RGB-D representative rays on saved P5/P6 ROIs only."""

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from vision_p5_roi_analysis import camera_vector
from vision_p6_target_frame_analysis import vector_metrics


CANDIDATES = (
    'B0_valid_depth_median', 'B1_red_mask_median',
    'B1_red_mask_centroid', 'B2_bbox_center',
)


def pixel_centers(mask, valid, bounds):
    """Use full-image pixel coordinates; bbox is the red mask, not ROI pad."""
    left, top, _, _ = bounds
    red_rows, red_cols = np.nonzero(mask)
    valid_rows, valid_cols = np.nonzero(valid)
    if not len(red_rows) or not len(valid_rows):
        return {}
    red_u = red_cols.astype(float) + left
    red_v = red_rows.astype(float) + top
    valid_u = valid_cols.astype(float) + left
    valid_v = valid_rows.astype(float) + top
    return {
        'B0_valid_depth_median': (
            float(np.median(valid_u)), float(np.median(valid_v))),
        'B1_red_mask_median': (
            float(np.median(red_u)), float(np.median(red_v))),
        'B1_red_mask_centroid': (
            float(np.mean(red_u)), float(np.mean(red_v))),
        'B2_bbox_center': (
            float((red_u.min() + red_u.max()) / 2),
            float((red_v.min() + red_v.max()) / 2)),
    }


def ray_radius_center(depth_median, pixel, intrinsics, radius):
    """Use the online B formula with only the representative pixel changed."""
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    u, v = pixel
    ray = np.array((1.0, -(u - cx) / fx, -(v - cy) / fy))
    return (float(depth_median) + float(radius) / np.linalg.norm(ray)) * ray


def analyze(directory):
    directory = Path(directory)
    records = [json.loads(line) for line in
               (directory / 'roi.jsonl').read_text().splitlines()]
    errors = {name: [] for name in CANDIDATES}
    exclusions = Counter()
    online_replay_differences = []
    for item in records:
        if not item['valid']:
            exclusions['invalid_online_observation'] += 1
            continue
        entity = item['entity_at_rgb']['value']
        model = item['model_at_rgb']['value']
        if entity is None or model is None:
            exclusions['missing_same_time_truth'] += 1
            continue
        with np.load(directory / item['file']) as roi:
            mask = roi['mask'].astype(bool)
            depth = roi['depth']
        valid = mask & np.isfinite(depth) & (depth >= 0.2) & (depth <= 25.0)
        centers = pixel_centers(mask, valid, item['roi_bounds_ltrb'])
        if not centers:
            exclusions['no_valid_mask_depth'] += 1
            continue
        median_depth = float(np.median(depth[valid]))
        expected = camera_vector(model, entity)
        intrinsics = item['intrinsics_fx_fy_cx_cy']
        radius = item['target_radius']
        for name, pixel in centers.items():
            candidate = ray_radius_center(
                median_depth, pixel, intrinsics, radius,
            )
            errors[name].append(candidate - expected)
            if name == 'B0_valid_depth_median':
                online_replay_differences.append(float(np.linalg.norm(
                    candidate - item['online_center_camera_flu']
                )))
    attempted = len(records) - exclusions['invalid_online_observation']
    return {
        'directory': str(directory), 'saved_roi': len(records),
        'attempted_valid_roi': attempted,
        'excluded': dict(exclusions),
        'maximum_online_B0_replay_difference_m': (
            max(online_replay_differences)
            if online_replay_differences else None
        ),
        'candidates': {
            name: {
                **vector_metrics(errors[name]),
                'valid_rate_on_valid_roi': len(errors[name]) / attempted
                if attempted else 0.0,
            } for name in CANDIDATES
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', action='append', required=True,
                        help='name=absolute_or_relative_roi_directory')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {}
    for entry in args.scenario:
        name, directory = entry.split('=', 1)
        result[name] = analyze(directory)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    for name, item in result.items():
        print(name, {key: (value['n'], value.get('rmse_3d_m'))
                     for key, value in item['candidates'].items()})


if __name__ == '__main__':
    main()
