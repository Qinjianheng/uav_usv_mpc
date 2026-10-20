#!/usr/bin/env python3
"""Compare actual rendered visibility with an independent calibrated ray/sphere silhouette."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def audit(session, output, horizontal_fov=1.74, radius=.25):
    """Occlusion is visible area divided by the ideal same-frame spherical silhouette."""
    session, output = Path(session), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    records = [json.loads(line) for line in (session/'frames.jsonl').read_text().splitlines()]
    v, u = np.mgrid[:480, :640]
    focal = 320/math.tan(horizontal_fov/2)
    rays = np.stack((np.ones_like(u), -(u-320)/focal, -(v-240)/focal), axis=-1)
    squared = np.sum(rays*rays, axis=-1)
    rows = []
    for record in records:
        center = np.asarray(record['camera_relative_center'])
        projection = rays@center
        discriminant = projection**2-squared*(center@center-radius*radius)
        ideal = (projection > 0) & (discriminant >= 0)
        mask = np.load(session/'masks'/(record['name']+'.npz'))['mask']
        expected = int(ideal.sum())
        fraction = int(mask.sum())/max(1, expected)
        rows.append(dict(name=record['name'], case=record['geometry_case'],
                         ideal_pixels=expected, visible_pixels=int(mask.sum()),
                         visible_fraction=fraction,
                         expected_center_uv=record['projected_sphere_center_uv']))
    follows = [r for r in rows if r['case']['phase'] == 'FOLLOW']
    summary = dict(scope='Static actual composite Gazebo render, not closed-loop flight',
                   qualified=False, cases=len(rows), follow_cases=len(follows),
                   completely_occluded_follow=sum(r['visible_pixels'] == 0 for r in follows),
                   minimum_follow_visible_fraction=min(r['visible_fraction'] for r in follows),
                   level_follow_visible_fraction=[r['visible_fraction'] for r in follows
                                                  if r['case']['pitch'] == r['case']['roll'] == 0],
                   native_capture_geometry=dict(minimum_origin_height=.33, capture_radius=.25,
                                                minimum_target_height_for_any_hit=.08,
                                                low_heave_minimum_distance=.48,
                                                zero_heave_minimum_distance=.33,
                                                high_heave_minimum_distance=.18),
                   reason='AIRFRAME_OCCLUSION_AND_PHASE_DEPENDENT_CAPTURE_REACHABILITY',
                   source_frames_sha256=hashlib.sha256(
                       (session/'frames.jsonl').read_bytes()).hexdigest())
    (output/'cases.json').write_text(json.dumps(rows, indent=2)+'\n')
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    audit(args.session, args.output)
