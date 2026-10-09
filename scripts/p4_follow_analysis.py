#!/usr/bin/env python3
"""Read-only native/ROS evidence analysis with explicit evaluation layers and phase windows."""
import argparse
from collections import Counter
from dataclasses import replace
import csv
import json
from pathlib import Path

import numpy as np

from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
from uav_control.guidance.camera_visibility import evaluate_visibility
from uav_control.guidance.camera_visibility import body_frd_to_ned_from_quaternion
from p32_latency_analysis import arrival


def distribution(values):
    """Report finite SI values only, null when evidence is absent."""
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return dict(count=len(a), min=float(np.min(a)), p50=float(np.percentile(a, 50)),
                p95=float(np.percentile(a, 95)),
                p99=float(np.percentile(a, 99)), max=float(np.max(a))) if len(a) else None


def read(path):
    """Preserve and count any incomplete JSON instead of silently calling a failed run complete."""
    rows, errors = [], []
    for number, line in enumerate(path.open(), 1):
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            errors.append(number)
    return rows, errors


def rendered_marker_visibility(position, rotation, marker_center, model):
    """Treat Gazebo's rendered entity pose as its centre, without applying height twice."""
    sphere = replace(model.target, center_offset_ned=(0., 0., 0.))
    return evaluate_visibility(position, rotation, marker_center, model.intrinsics,
                               model.extrinsics, sphere, model.visibility)


def analyze(root):
    """Evaluate original flown FOLLOW separately from unexecuted research proposals."""
    raw, malformed = read(root/'offline_evidence.jsonl')
    kinds = {k: [r for r in raw if r['kind'] == k] for k in set(r['kind'] for r in raw)}
    diagnostics = kinds.get('diagnostic', [])
    follow = [r for r in diagnostics if r['status'] == 'FOLLOW']
    start = follow[0]['stamp'] if follow else None
    csv_rows = [r for p in (root/'evaluator').glob('*mission_1.csv')
                for r in csv.DictReader(p.open()) if r['phase'] == 'FOLLOW'
                and r['truth_available'] == 'True' and r['uav_available'] == 'True']
    csv_start = float(csv_rows[0]['time']) if csv_rows else None
    report = dict(kind_counts={k: len(v) for k, v in kinds.items()}, malformed_lines=malformed,
                  actual_controller='ORIGINAL_FOLLOW', minco_closed_loop=False,
                  end=json.loads((root/'watchdog_end.json').read_text()), windows={})
    model = FollowMpcSeed()
    # Same-message native poses: no interpolated attitude, fitted lag or truth input.
    boats = {r['sim']: r for r in kinds.get('rendered_target', [])}
    clocks = kinds.get('clock', [])
    times = np.array([r['sim'] for r in clocks])
    epochs = np.array([r['system'] for r in clocks])
    geometry = []
    if len(times) and np.all(np.diff(times) >= 0):
        for uav in kinds.get('rendered_uav', []):
            target = boats.get(uav['sim'])
            if target is None or not times[0] <= uav['sim'] <= times[-1]:
                continue
            enu_to_ned = np.array(((0., 1., 0.), (1., 0., 0.), (0., 0., -1.)))
            rotation = (enu_to_ned @ body_frd_to_ned_from_quaternion(uav['q'])
                        @ np.diag((1., -1., -1.)))
            result = rendered_marker_visibility(uav['p'], rotation, target['p'], model)
            geometry.append(dict(stamp=float(np.interp(uav['sim'], times, epochs)),
                                 valid=result.whole_target_safe,
                                 horizontal=result.horizontal_margin_rad,
                                 vertical=result.vertical_margin_rad))
    for window, limits in (('all_follow', (0., float('inf'))), ('follow_20_to_60', (20., 60.))):
        selected = [r for r in csv_rows if csv_start is not None
                    and limits[0] <= float(r['time'])-csv_start <= limits[1]]
        error, distance, velocity, altitude = [], [], [], []
        for row in selected:
            p = np.array([float(row['target_'+k]) for k in 'xyz'])
            v = np.array([float(row['target_v'+k]) for k in 'xy'])
            up = np.array([float(row['uav_'+k]) for k in 'xyz'])
            uv = np.array([float(row['uav_v'+k]) for k in 'xy'])
            heading = v/max(np.linalg.norm(v), 1e-9)
            reference = p[:2]-5*heading
            error.append(float(np.linalg.norm(up[:2]-reference)))
            distance.append(float(np.linalg.norm(up[:2]-p[:2])-5))
            velocity.append(float(np.linalg.norm(uv-v)))
            altitude.append(float(up[2]+5))
        diag = [r for r in follow if start is not None
                and limits[0] <= r['stamp']-start <= limits[1]]
        view = [r for r in geometry if start is not None
                and limits[0] <= r['stamp']-start <= limits[1]]
        states = [r for r in kinds.get('uav', []) if r['valid'] and start is not None
                  and limits[0] <= r['stamp']-start <= limits[1]]
        refs = [r for r in kinds.get('reference', []) if start is not None
                and limits[0] <= r['receipt']-start <= limits[1]]
        jerk, command_acceleration = [], []
        for first, second in zip(states, states[1:]):
            dt = (second['native']-first['native'])/1e6
            if 0 < dt <= .1:
                jerk.append(float(np.linalg.norm(np.subtract(second['a'], first['a'])[:2]/dt)))
        for first, second in zip(refs, refs[1:]):
            dt = second['receipt']-first['receipt']
            if 0 < dt <= .15:
                command_acceleration.append(float(np.linalg.norm(
                    np.subtract(second['v'], first['v'])[:2])/dt))
        report['windows'][window] = dict(
            samples=len(selected),
            evaluation='evaluator native UAV snapshot vs ideal scene TargetState; not KF error',
            position_horizontal_rmse=float(np.sqrt(np.mean(np.square(error)))) if error else None,
            distance_rmse=float(np.sqrt(np.mean(np.square(distance)))) if distance else None,
            relative_velocity_rmse=(float(np.sqrt(np.mean(np.square(velocity))))
                                    if velocity else None),
            altitude_rmse=float(np.sqrt(np.mean(np.square(altitude)))) if altitude else None,
            visible_fraction=np.mean([r['visible'] for r in diag]).item() if diag else None,
            locked_fraction=np.mean([r['locked'] for r in diag]).item() if diag else None,
            actual_whole_marker_safe_fraction=(np.mean([r['valid'] for r in view]).item()
                                               if view else None),
            actual_horizontal_margin=distribution([r['horizontal'] for r in view
                                                  if r['horizontal'] is not None]),
            actual_vertical_margin=distribution([r['vertical'] for r in view
                                                if r['vertical'] is not None]),
            actual_horizontal_speed=distribution([np.linalg.norm(r['v'][:2]) for r in states]),
            actual_horizontal_acceleration=distribution(
                [np.linalg.norm(r['a'][:2]) for r in states]),
            estimated_horizontal_jerk=distribution(jerk),
            reference_velocity_delta_over_receipt_dt=distribution(command_acceleration),
            reference_gaps_seconds=distribution([b['receipt']-a['receipt']
                                                for a, b in zip(refs, refs[1:])]))
    graph_rows = kinds.get('graph', [])
    control = ('/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode',
               '/fmu/in/vehicle_command')
    discovered = [r for r in graph_rows if all(r['publishers'].get(t) for t in control)]
    report['authority'] = dict(
        graph_samples=len(graph_rows), discovered_samples=len(discovered),
        all_discovered_single_tracker=all(
            all(len(r['publishers'][t]) == 1
                and r['publishers'][t][0]['node'] == 'trajectory_tracker_node'
                for t in control) for r in discovered),
        conflicts=[r['receipt'] for r in graph_rows
                   if any(len(r['publishers'].get(t, [])) > 1 for t in control)])
    report['ack'] = dict(
                        count=len(kinds.get('follow_ack', [])),
                        states=dict(Counter(r['state'] for r in kinds.get('follow_ack', []))),
                        reasons=dict(Counter(reason for r in kinds.get('follow_ack', [])
                                             for reason in r['reasons'])),
                        actual_accepted=sum(r['state'] in ('ACCEPTED', 'ACTIVE')
                                            for r in kinds.get('follow_ack', [])))
    complete = [r for p in (root/'shadow').glob('mpc_seed_shadow_*.jsonl')
                for r in read(p)[0] if r.get('event') == 'completion']
    report['research'] = dict(
        count=len(complete),
        feasible=sum(r['output']['valid'] for r in complete),
        initialization=distribution([r['core_result']['timing'].get('initialization', 0.)
                                     for r in complete]),
        solver=distribution([r['core_result']['solve_time'] for r in complete]),
        validation=distribution([r['core_result']['timing']['validation'] for r in complete
                                 if 'validation' in r['core_result'].get('timing', {})]),
        whole_cycle=distribution([r['whole_cycle_time'] for r in complete]),
        warm_hits=sum(r['core_result'].get('metrics', {}).get('progress', {}).get(
            'warm_used', False) for r in complete))
    if (root/'planner_crash_injection.json').exists():
        injection = json.loads((root/'planner_crash_injection.json').read_text())
        after = [r for r in kinds.get('reference', []) if r['receipt'] > injection['stamp']]
        report['planner_exit'] = dict(
            injection=injection, reference_samples_after=len(after),
            maximum_reference_gap_after=max(
                (b['receipt']-a['receipt'] for a, b in zip(after, after[1:])), default=None),
            actual_minco_execution=False)
    if (root/'owned_load.json').exists():
        load = json.loads((root/'owned_load.json').read_text())
        report['cpu'] = distribution([r['cpu_percent'] for r in load])
        report['rss_mib'] = distribution([r['rss_mib'] for r in load])
    # Reuse immutable source matching latency analysis, writing only inside this NEW run.
    report['latency'] = arrival(root)
    (root/'analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('roots', type=Path, nargs='+')
    args = parser.parse_args()
    for root in args.roots:
        result = analyze(root)
        print(root.name, json.dumps(result['windows']))
