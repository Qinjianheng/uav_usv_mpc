#!/usr/bin/env python3
"""Screen only A/B/C 32-degree mounts, before at most two actual render sessions."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.common.p76_contract import load_contract  # noqa: E402
from uav_control.common.p76_model import prepare_model  # noqa: E402
from uav_control.common.p761_geometry import (  # noqa: E402
    geometry_cases, mount_conflicts, visibility,
)


def main():
    """Persist every candidate and analytical rejection, without launching Gazebo."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    original = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    results = []
    for name, x in zip('ABC', (.35, .45, .55)):
        directory = args.output/name
        directory.mkdir()
        contract = deepcopy(original)
        contract['camera']['translation_flu'] = [x, 0., .55]
        saved = {k: v for k, v in contract.items() if k != '_path'}
        (directory/'contract.yaml').write_text(yaml.safe_dump(saved, sort_keys=False))
        model = prepare_model(contract['dynamics']['source_camera_model'],
                              contract['dynamics']['px4_model_root'], directory/'models', contract)
        geometry = model['collision_envelope']
        rows = [dict(**p['case'], **visibility(contract, geometry, p))
                for p in geometry_cases(contract, dense=True)]
        follows = [r for r in rows if r['phase'] == 'FOLLOW']
        terminal = [r for r in rows if r['phase'] == 'TERMINAL']
        conflicts = mount_conflicts(geometry)
        legal = [r for r in terminal if r['visible_pixels_estimate'] >= 3
                 and r['lens_surface_gap'] >= .05 and r['body_clearance'] >= .02]
        complete_path = all(r in legal for r in terminal)
        result = dict(name=name, translation_flu=[x, 0., .55], conflicts=conflicts,
                      occluded_follow=sum(r['visible_pixels_estimate'] < 3 for r in follows),
                      complete_slow_geometric_path=complete_path,
                      legal_capture_points=sum(r['origin_distance'] <= .25 for r in legal),
                      scope='Box-based analytical screening; not detector or flight qualification')
        result['eligible_for_render'] = (not conflicts and not result['occluded_follow']
                                         and complete_path and result['legal_capture_points'] > 0)
        (directory/'samples.json').write_text(json.dumps(rows, indent=2)+'\n')
        results.append(result)
    (args.output/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
