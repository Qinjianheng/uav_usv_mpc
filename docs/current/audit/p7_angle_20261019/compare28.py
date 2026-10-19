"""Independent offline comparison, original 28-degree run versus current candidate."""
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path('/home/qin/data/uav_usv_mpc')
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.guidance.camera_visibility import (
    CameraIntrinsics, CameraExtrinsics, TargetBoundingSphere, VisibilityConstraints,
    body_frd_to_ned_from_quaternion, evaluate_visibility,
)


def read(path):
    return [json.loads(line) for line in path.open() if line.strip()]


def distribution(values):
    values = [x for x in values if math.isfinite(x)]
    return (dict(n=len(values), rmse=float(np.sqrt(np.mean(np.square(values)))),
                 p50=float(np.median(values)), p95=float(np.percentile(values, 95)),
                 maximum=float(max(values))) if values else dict(n=0))


def number(row, key):
    try:
        return float(row.get(key, ''))
    except (ValueError, TypeError):
        return math.nan


def main():
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    results = {}
    manifest = {}
    runs = {'28deg_5m': ROOT/'data/experiments/p7_manual_20261019_112340_740393470_984611',
            '38deg_5m': ROOT/'data/experiments/20261019_p7_angle_capture_01'}
    for label, run in runs.items():
        summary_path = next((run/'evaluator').glob('*summary.json'))
        summary = json.loads(summary_path.read_text())
        start = summary['intercept_started_at']
        if start is None:
            results[label] = dict(run=str(run), outcome='NO_Y')
            continue
        config_path = run/'config/flight.yaml'
        config = yaml.safe_load(config_path.read_text())
        evidence = run/'offline_evidence.jsonl'
        rows = read(evidence)
        cap_path = next((run/'evaluator').glob('*capture.jsonl'))
        pairs = [r for r in read(cap_path) if r['kind'] == 'EVALUATED' and r['stamp'] >= start]
        assert all(r['maximum_bracket'] <= .05+1e-9 for r in pairs)
        assert all(0 <= r['decision_stamp']-r['stamp'] <= .125+1e-9 for r in pairs)
        assert all(abs(math.dist(r['uav_position'], r['target_position'])-r['distance']) < 1e-9
                   for r in pairs)
        closest = min(pairs, key=lambda r: r['distance']) if pairs else None
        stop = pairs[-1]['stamp'] if pairs else max(r['receipt'] for r in rows)
        diag = [r for r in rows if r['kind'] == 'diagnostic' and start <= r['stamp'] <= stop]
        guides = [r for r in rows if r['kind'] == 'p7_guidance' and start <= r['stamp'] <= stop]
        obs = [r for r in rows if r['kind'] == 'observation' and start <= r['stamp'] <= stop]
        invalid = next((r for r in obs if not r['valid']), None)
        braking = next((r for r in diag if r['status'] == 'P7_BRAKING'), None)
        transitions, previous = [], None
        for r in diag:
            key = (r['status'], r['locked'], r['visible'], r['search'])
            if key != previous:
                transitions.append(dict(seconds=r['stamp']-start, status=r['status'],
                                        locked=r['locked'], visible=r['visible'], search=r['search']))
                previous = key
        vision_path = next((run/'evaluator').glob('*vision.csv'))
        vision = list(csv.DictReader(vision_path.open()))
        valid = [r for r in vision if r['valid'] == 'True' and start <= number(r, 'measurement_stamp') <= stop]
        if not valid:
            valid = [r for r in vision if r['valid'].lower() in ('true','1')
                     and start <= number(r, 'measurement_stamp') <= stop]
        errors = {}
        for lo, hi in ((0., 1.), (1., 3.), (3., 10.), (10., 30.)):
            selected = [r for r in valid if lo <= number(r, 'target_range') < hi]
            errors[f'range_{lo:g}_{hi:g}m'] = {
                key: distribution([number(r, key) for r in selected]) for key in
                ('position_3d_error', 'vision_to_entity_3d', 'camera_center_error_3d')}
        csv_path = next(p for p in (run/'evaluator').glob('*.csv') if not p.name.endswith('_vision.csv'))
        csv_rows = list(csv.DictReader(csv_path.open()))
        sea = Counter(r.get('sea_safety_state') for r in csv_rows
                      if number(r, 'intercept_elapsed_time') >= 0)
        camera = config['rgbd_target_localizer']['ros__parameters']
        intr = CameraIntrinsics.from_horizontal_fov(640, 480, camera['horizontal_fov'])
        extr = CameraExtrinsics.from_sdf_pose((.18, 0., .39), 0., camera['camera_pitch_down'], 0.)
        target = TargetBoundingSphere(camera['target_radius'])
        limits = VisibilityConstraints(camera['minimum_depth'], camera['maximum_depth'])
        boats = {r['sim']: r for r in rows if r['kind'] == 'rendered_target'}
        clock = {}
        for r in rows:
            if r['kind'] == 'clock':
                clock.setdefault(r['sim'], r['system'])
        ct = sorted(clock)
        views = []
        enu_ned = np.array(((0., 1., 0.), (1., 0., 0.), (0., 0., -1.)))
        for u in (r for r in rows if r['kind'] == 'rendered_uav' and r['sim'] in boats):
            raw = u['sim']
            i = bisect.bisect_left(ct, raw)
            if raw in clock:
                stamp = clock[raw]
            elif 0 < i < len(ct):
                a, b = ct[i-1:i+1]
                if not (0 < b-a <= .05+1e-9 and 0 < clock[b]-clock[a] <= .05+1e-9):
                    continue
                stamp = clock[a]+(raw-a)/(b-a)*(clock[b]-clock[a])
            else:
                continue
            if not start <= stamp <= stop:
                continue
            rotation = enu_ned@body_frd_to_ned_from_quaternion(u['q'])@np.diag((1., -1., -1.))
            view = evaluate_visibility(u['p'], rotation, boats[raw]['p'], intr, extr, target, limits)
            ray = np.array(view.target_camera_flu)
            distance = float(np.linalg.norm(ray))
            yz = math.hypot(ray[0], ray[2])
            center = math.atan2(-ray[2], ray[0])
            upper_limb = center-math.asin(min(1., target.radius/yz)) if yz > target.radius else None
            views.append(dict(stamp=stamp, seconds=stamp-start, raw=raw, ray=ray.tolist(),
                              camera_sphere_distance=distance, center_visible=view.center_visible,
                              whole_safe=view.whole_target_safe, reasons=view.reasons,
                              upper_limb=upper_limb, lower_limit=intr.angle_bounds()[3],
                              entirely_below=upper_limb is not None and upper_limb > intr.angle_bounds()[3]))
        age = [r['stamp']-r['feedback_epoch'] for r in guides if 'feedback_epoch' in r]
        info = dict(run=str(run), y=start, outcome=summary['outcome'],
                    evaluator_summary=summary,
                    native_samples=len(pairs), native_minimum=(dict(
                        closest, seconds=closest['stamp']-start,
                        dxy=math.dist(closest['uav_position'][:2], closest['target_position'][:2]),
                        dz=abs(closest['uav_position'][2]-closest['target_position'][2])) if closest else None),
                    first_invalid=invalid, first_braking=braking, transitions=transitions,
                    observation_valid_fraction=sum(r['valid'] for r in obs)/max(1,len(obs)),
                    guide_modes=dict(Counter(r.get('mode') for r in guides)),
                    geometry_reasons=dict(Counter(r.get('geometry_reason') for r in guides)),
                    common_epoch_age=distribution(age), vision_error_bins=errors,
                    sea_states=dict(sea),
                    native_visibility=dict(n=len(views),
                        center_fraction=sum(r['center_visible'] for r in views)/max(1,len(views)),
                        whole_fraction=sum(r['whole_safe'] for r in views)/max(1,len(views)),
                        minimum_camera_sphere_distance=min((r['camera_sphere_distance'] for r in views),default=None)),
                    minimum_window=[r for r in guides if closest and abs(r['stamp']-closest['stamp']) < .2],
                    first_loss_geometry=[r for r in views if invalid and abs(r['stamp']-invalid['stamp']) < .25])
        results[label] = info
        (OUT/(label+'_native_views.json')).write_text(json.dumps(views, indent=2)+'\n')
        t = np.array([r['stamp']-start for r in pairs])
        axes[0, 0].plot(t, [r['distance'] for r in pairs], label=label)
        axes[0, 1].plot(t, [-r['uav_position'][2] for r in pairs], label=label+' UAV')
        axes[0, 1].plot(t, [-r['target_position'][2] for r in pairs], '--', label=label+' USV')
        axes[1, 0].plot([r['seconds'] for r in views],
                        [r['camera_sphere_distance'] for r in views], label=label)
        axes[1, 1].plot([number(r, 'measurement_stamp')-start for r in valid],
                        [number(r, 'position_3d_error') for r in valid], label=label)
        for f in (summary_path, config_path, evidence, cap_path, vision_path, csv_path,
                  run/'summary.json', run/'source_snapshot.json'):
            if f.exists():
                manifest[str(f.relative_to(ROOT))] = hashlib.sha256(f.read_bytes()).hexdigest()
    axes[0, 0].axhline(.25, color='black', linestyle='--', label='0.25 m capture')
    axes[1, 0].axhline(.27, color='black', linestyle='--', label='0.27 m camera clearance')
    for ax, title, ylabel in zip(axes.flat,
            ('Native origin-to-USV distance', 'Native height',
             'Native camera-to-sphere-center distance', 'Operational vision error to ideal target'),
            ('Distance (m)', 'Height (m)', 'Distance (m)', 'Error (m)')):
        ax.set(title=title, xlabel='Seconds after Y', ylabel=ylabel)
        ax.grid(alpha=.25); ax.legend()
    fig.tight_layout(); fig.savefig(OUT/'comparison_28_38.png', dpi=160); plt.close(fig)
    result = dict(scope='Two individual runs; no capture-rate or optimality claim. Native capture and vision ideal-truth errors use distinct references.', runs=results)
    (OUT/'comparison_28_38.json').write_text(json.dumps(result, indent=2)+'\n')
    (OUT/'comparison_input_SHA256SUMS.json').write_text(json.dumps(manifest, indent=2)+'\n')
    for label, info in results.items():
        print(label, json.dumps({k:info.get(k) for k in ('outcome','native_minimum','first_invalid','first_braking','guide_modes','native_visibility')}, indent=2))


if __name__ == '__main__':
    main()
