#!/usr/bin/env python3
"""Audit P2 camera-frame residuals and physics-led depth quality gates."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.stats import spearmanr

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'data/experiments/current'
SCENARIOS = json.loads(
    (SOURCE / 'p3_20260927_pose_counterfactuals.json').read_text()
)['scenarios']
DETAIL = SOURCE / 'p3_20260927_pose_counterfactuals.csv'
OUTPUT = SOURCE / 'p4_20260927_geometry_gate_analysis_v2.json'
RADIUS_M = 0.25


def metrics(rows):
    if not rows:
        return {'n': 0}
    errors = np.array([
        float(r['physical_camera_center_error_3d']) for r in rows
    ])
    xyz = np.array([[float(r[f'physical_camera_center_error_{a}'])
                     for a in 'xyz'] for r in rows])
    return {'n': len(rows), 'rmse_3d_m': float(np.sqrt(np.mean(errors ** 2))),
            'p50_3d_m': float(np.median(errors)),
            'p95_3d_m': float(np.percentile(errors, 95)),
            'max_3d_m': float(np.max(errors)),
            'signed_mean_xyz_m': xyz.mean(axis=0).tolist()}


def ned_metrics(rows, pose_rows, scenario, case):
    matched = [pose_rows[scenario][r['measurement_stamp']][case]
               for r in rows]
    if not matched:
        return {'n': 0}
    xyz = np.array([[float(r[f'error_{a}']) for a in 'xyz']
                    for r in matched])
    norms = np.linalg.norm(xyz, axis=1)
    return {'n': len(matched),
            'rmse_3d_m': float(np.sqrt(np.mean(norms ** 2))),
            'p50_3d_m': float(np.median(norms)),
            'p95_3d_m': float(np.percentile(norms, 95)),
            'max_3d_m': float(np.max(norms)),
            'signed_mean_xyz_m': xyz.mean(axis=0).tolist()}


def all_valid(row):
    return (row['valid'] == 'True'
            and row['geometry_diagnostics_enabled'] == 'True'
            and row['gazebo_entity_available'] == 'True'
            and row['gazebo_model_available'] == 'True'
            and row['red_pixel_count'] not in ('', '0')
            and math.isfinite(float(row['physical_camera_center_error_3d'])))


def ratio(row):
    return float(row['valid_depth_count']) / float(row['red_pixel_count'])


def gate(row, rule):
    count = int(row['valid_depth_count'])
    mad = float(row['depth_mad'])
    if rule == 'count_9':
        return count >= 9
    if rule == 'count_16':
        return count >= 16
    if rule == 'mad_radius':
        return mad <= RADIUS_M
    if rule == 'mad_rel_5pct':
        return mad / float(row['depth_median']) <= 0.05
    if rule == 'ratio_75pct':
        return ratio(row) >= 0.75
    if rule == 'count_9_and_mad_radius':
        return count >= 9 and mad <= RADIUS_M
    if rule == 'count_16_and_mad_radius':
        return count >= 16 and mad <= RADIUS_M
    raise ValueError(rule)


def describe_tail(row):
    fields = ('measurement_stamp', 'depth_median', 'depth_mad',
              'valid_depth_count', 'red_pixel_count', 'target_range',
              'view_angle', 'rgb_depth_acquisition_skew',
              'physical_projection_error_u', 'physical_projection_error_v',
              'physical_camera_center_error_x',
              'physical_camera_center_error_y',
              'physical_camera_center_error_z',
              'physical_camera_center_error_3d')
    return {**{key: float(row[key]) for key in fields},
            'valid_depth_ratio_derived': ratio(row),
            'depth_min': None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    pose_rows = {}
    with DETAIL.open(newline='') as file:
        for row in csv.DictReader(file):
            pose_rows.setdefault(row['scenario'], {}).setdefault(
                row['measurement_stamp'], {}
            )[row['pose_case']] = row
    selected = {name: set(stamps) for name, stamps in pose_rows.items()}
    output = {
        'method': 'P3 same-frame subset; physical_camera_center_error in FLU',
        'missing_fields': ['depth_min', 'per_pixel_depth'],
        'derived_fields': [
            'valid_depth_ratio=valid_depth_count/red_pixel_count'
        ],
        'radius_m': RADIUS_M,
        'rules': {
            'count_9': 'at least a 3x3 support patch',
            'count_16': 'at least a 4x4 support patch',
            'mad_radius': 'median absolute X-depth deviation <= sphere radius',
            'mad_rel_5pct': 'depth MAD / median <= 5%',
            'ratio_75pct': '>=75% of red mask pixels have valid depth',
            'count_9_and_mad_radius': '3x3 support plus physical MAD bound',
            'count_16_and_mad_radius': '4x4 support plus physical MAD bound',
        },
        'scenarios': {},
    }
    independent = ('depth_median', 'depth_mad', 'valid_depth_count',
                   'red_pixel_count', 'target_range', 'view_angle',
                   'rgb_depth_acquisition_skew', 'physical_projection_error_u',
                   'physical_projection_error_v')
    for name, item in SCENARIOS.items():
        with (ROOT / item['source_csv']).open(newline='') as file:
            source_rows = list(csv.DictReader(file))
        full_rows = [row for row in source_rows if all_valid(row)]
        rows = [row for row in full_rows
                if row['measurement_stamp'] in selected[name]]
        baseline = metrics(rows)
        full_rules = {}
        for rule in output['rules']:
            retained = [r for r in full_rows if gate(r, rule)]
            full_rules[rule] = {
                'rejected': len(full_rows) - len(retained),
                'retained_fraction': len(retained) / len(full_rows),
                **metrics(retained),
            }
        rules = {}
        for rule in output['rules']:
            kept = [r for r in rows if gate(r, rule)]
            rules[rule] = {
                'rejected': len(rows) - len(kept),
                'rejected_fraction': (len(rows) - len(kept)) / len(rows),
                **metrics(kept),
                'ned_A': ned_metrics(kept, pose_rows, name, 'A'),
                'ned_B': ned_metrics(kept, pose_rows, name, 'B'),
            }
        errors = np.array([float(r['physical_camera_center_error_3d'])
                           for r in rows])
        associations = {}
        for key in independent + ('valid_depth_ratio_derived',):
            values = np.array([ratio(r) if key.endswith('_derived')
                               else float(r[key]) for r in rows])
            corr = spearmanr(values, errors)
            associations[key] = {
                'spearman_rho': (float(corr.correlation)
                                 if math.isfinite(corr.correlation) else None),
                'pvalue': (float(corr.pvalue)
                           if math.isfinite(corr.pvalue) else None),
            }
        output['scenarios'][name] = {
            'source_rows': len(source_rows),
            'all_valid': {
                'baseline': metrics(full_rows),
                'rules': full_rules,
            },
            'baseline': baseline,
            'ned_baseline': {
                key: ned_metrics(rows, pose_rows, name, key)
                for key in ('A', 'B')
            },
            'rules': rules,
            'associations': associations,
            'top_errors': [describe_tail(r) for r in sorted(
                rows, key=lambda x: float(
                    x['physical_camera_center_error_3d']), reverse=True
            )[:5]],
        }
    with args.output.open('x') as file:
        json.dump(output, file, indent=2, ensure_ascii=False,
                  allow_nan=False)
        file.write('\n')
    print(args.output)
    for name, item in output['scenarios'].items():
        print(name, item['baseline']['n'],
              round(item['baseline']['rmse_3d_m'], 3),
              round(item['rules']['count_9_and_mad_radius']['rmse_3d_m'], 3),
              item['rules']['count_9_and_mad_radius']['rejected'])


if __name__ == '__main__':
    main()
