#!/usr/bin/env python3
"""Offline mount-angle reprojection of measured Y runs, never online truth."""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


def rotate_mount(vectors, delta_degrees):
    """Rotate camera FLU rays into a mount tilted farther down by delta."""
    delta = np.deg2rad(delta_degrees)
    result = np.asarray(vectors, dtype=float).copy()
    result[:, 0] = np.cos(delta) * vectors[:, 0] - np.sin(delta) * vectors[:, 2]
    result[:, 2] = np.sin(delta) * vectors[:, 0] + np.cos(delta) * vectors[:, 2]
    return result


def audit(vision, records):
    y = next(r['receipt'] for r in records if r['topic'] == 'driver'
             and r['message']['command'] == 'Y')
    phases = pd.DataFrame([
        (r['receipt'], r['message']['state_name'])
        for r in records if r['topic'] == 'mission'
    ], columns=['phase_stamp', 'phase']).sort_values('phase_stamp')
    rows = pd.merge_asof(vision.sort_values('measurement_stamp'), phases,
                         left_on='measurement_stamp', right_on='phase_stamp')
    end = next((r['receipt'] for r in records if r['topic'] == 'result'), y + 30.)
    groups = {'FOLLOW': rows.phase.eq('FOLLOW') & (rows.measurement_stamp < y),
              'Y_PREPARATION': rows.phase.eq('FAR_GUIDANCE'),
              'Y_MINCO': rows.phase.isin(['MINCO_READY', 'MINCO_TRACKING', 'TERMINAL_MINCO'])}
    report = {'role': 'OFFLINE_MOUNT_REPROJECTION', 'y_stamp': y,
              'limitation': 'Same recorded flight poses; not a closed-loop counterfactual. '
              'Expected camera rays exist only on geometry-valid samples; missing rays '
              'are counted separately and never declared visible.', 'groups': {}}
    for name, mask in groups.items():
        subset = rows[mask & (rows.measurement_stamp <= end)]
        vectors = subset[['entity_expected_camera_' + a for a in 'xyz']].to_numpy()
        usable = np.isfinite(vectors).all(axis=1)
        rays = vectors[usable]
        fx = subset.camera_fx.to_numpy()[usable]
        fy = subset.camera_fy.to_numpy()[usable]
        pitches = np.rad2deg(subset.camera_pitch_down.to_numpy()[usable])
        group = {'all_frames': len(subset), 'native_rays': len(rays),
                 'missing_rays': int((~usable).sum()),
                 'rgb_visible_fraction': float((subset.red_pixel_count > 0).mean()),
                 'candidates': {}}
        if len(rays):
            desired = pitches + np.rad2deg(np.arctan2(-rays[:, 2], rays[:, 0]))
            group['ideal_vertical_pitch_p5_p50_p95_deg'] = np.percentile(
                desired, [5, 50, 95]).tolist()
            for angle in (20, 25, 27, 28, 29, 30, 35, 40, 45):
                rotated = rotate_mount(rays, angle - pitches)
                u = -rotated[:, 1] / rotated[:, 0] * fx / 320.
                v = -rotated[:, 2] / rotated[:, 0] * fy / 240.
                inside = (rotated[:, 0] > .05) & (np.abs(u) < 1) & (np.abs(v) < 1)
                group['candidates'][str(angle)] = {
                    'center_inside_fraction_on_known_rays': float(inside.mean()),
                    'normalized_abs_vertical_p50_p95': np.percentile(
                        np.abs(v), [50, 95]).tolist(),
                    'normalized_abs_horizontal_p50_p95': np.percentile(
                        np.abs(u), [50, 95]).tolist(),
                }
        report['groups'][name] = group
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision', required=True, type=Path)
    parser.add_argument('--jsonl', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.jsonl.read_text().splitlines()]
    report = audit(pd.read_csv(args.vision), records)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
