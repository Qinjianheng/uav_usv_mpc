#!/usr/bin/env python3
"""Separate shadow planning, input admission, original FOLLOW and rendered-motion evidence."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from p32_follow_experiment import stats


def analyze(root):
    """Use native dual-clock interpolation without fitted delay or truth correction."""
    root = Path(root)
    events = [json.loads(line) for p in (root/'shadow').glob('*.jsonl')
              for line in p.read_text().splitlines()]
    complete = [r for r in events if r.get('event') == 'completion']
    real = [r['core_result'].get('metrics', {}).get('realtime', {}) for r in complete]
    profiles = [x for r in real for x in r.get('initialization_profile', [])]
    grouped = {}
    for name in sorted(set(p['function'] for p in profiles)):
        group = [p for p in profiles if p['function'] == name]
        grouped[name] = dict(calls=sum(x['calls'] for x in group),
                             cumulative=sum(x['cumulative_seconds'] for x in group),
                             self_seconds=sum(x['self_seconds'] for x in group),
                             cumulative_per_profile=stats([
                                 x['cumulative_seconds'] for x in group]))
    report = dict(
        completions=len(complete), core_feasible=sum(r['core_result']['valid'] for r in complete),
        published=sum(r.get('published_valid', False) for r in complete),
        core_status=dict(Counter(r['core_result']['solver_status'] for r in complete)),
        admission=dict(Counter(r['output'].get('reason', '') for r in complete)),
        initialization=stats([r['core_result']['timing'].get('initialization', 0.)
                              for r in complete]),
        whole_cycle=stats([r['whole_cycle_time'] for r in complete]),
        before_initialization=stats([r['remaining_before_initialization'] for r in real
                                     if 'remaining_before_initialization' in r]),
        after_planning=stats([r['remaining_after_planning'] for r in real
                              if 'remaining_after_planning' in r]),
        selected_tier=dict(Counter(r.get('selected_tier') for r in real)),
        all_stage_status=dict(Counter(s['tier']+':'+s['status'] for r in real
                                      for s in r.get('stages', []))),
        tier_initialization={tier: stats([s['initialization'] for r in real
                                          for s in r.get('stages', []) if s['tier'] == tier])
                             for tier in ('L0', 'L1', 'L2', 'L3')},
        input_age={key: stats([r['request']['now_stamp']-r['request']['context'][key]
                               for r in complete]) for key in (
            'navigation_stamp', 'observation_stamp', 'prediction_source_stamp')},
        execution_ttl_gap=stats([r['execution_ttl_gap'] for r in real
                                 if 'execution_ttl_gap' in r]),
        profiled_cycles=sum(bool(r.get('initialization_profile')) for r in real),
        initialization_profile=grouped, accepted_by_tracker=False,
        dry_run=dict(Counter(r.get('handover_dry_run', 'UNAVAILABLE') for r in real)))
    short = [r['core_result'].get('metrics', {}).get('short_follow', {}) for r in complete]
    if any(short):
        report['short_follow'] = dict(
            warm_used=sum(bool(r.get('warm_used')) for r in short),
            candidate_attempts=dict(Counter(len(r.get('attempts', [])) for r in short)),
            after_planning=stats([r['remaining_after_planning'] for r in short
                                  if 'remaining_after_planning' in r]),
            durations=sorted(set(r['duration'] for r in short if 'duration' in r)))
    for p in root.glob('*_load.json'):
        samples = json.loads(p.read_text())['samples']
        report[p.stem] = {key: dict(mean=float(np.mean([s[key] for s in samples])),
                                    percentiles=stats([s[key] for s in samples])) for key in (
            'cpu_percent', 'rss_mib', 'research_cpu_percent', 'research_rss_mib')}
    raw = [json.loads(s) for s in (root/'offline_evidence.jsonl').read_text().splitlines()]
    kinds = {kind: [r for r in raw if r['kind'] == kind] for kind in set(r['kind'] for r in raw)}
    clocks = kinds.get('clock', [])
    ideal = kinds.get('ideal', [])
    poses = kinds.get('rendered_target', [])
    residuals, matched, unmapped = [], [], 0
    if clocks and ideal:
        sim = np.array([r['sim'] for r in clocks])
        wall = np.array([r['system'] for r in clocks])
        ts = np.array([r['stamp'] for r in ideal])
        positions = np.array([r['p'] for r in ideal])
        monotonic = bool(np.all(np.diff(sim) >= 0) and np.all(np.diff(ts) >= 0))
        for pose in poses if monotonic else []:
            t = pose['sim']
            if not sim[0] <= t <= sim[-1]:
                unmapped += 1
                continue
            epoch = float(np.interp(t, sim, wall))
            if not ts[0] <= epoch <= ts[-1]:
                unmapped += 1
                continue
            expected = np.array([np.interp(epoch, ts, positions[:, axis]) for axis in range(3)])
            observed = np.array(pose['p']) + (0., 0., .42)
            error = observed-expected
            residuals.append(float(np.linalg.norm(error)))
            matched.append(dict(sim=t, epoch=epoch, ideal=expected.tolist(),
                                rendered_reference=observed.tolist(), error=error.tolist()))
        report['rendered_motion'] = dict(
            clocks=len(clocks), ideal_samples=len(ideal), rendered_samples=len(poses),
            matches=len(matched), unmapped=unmapped, clocks_monotonic=monotonic,
            error_norm=stats(residuals), rmse=float(np.sqrt(np.mean(np.square(residuals))))
            if residuals else None, empirical_time_shift=0., evaluation_only=True,
            comparable_under_perfect_scene_assumption=False)
        (root/'rendered_motion_residuals.jsonl').write_text(''.join(
            json.dumps(row)+'\n' for row in matched))
    else:
        report['rendered_motion'] = dict(status='MISSING_NATIVE_POSE_OR_CLOCK_OR_IDEAL',
                                         counts={k: len(v) for k, v in kinds.items()})
    # These are actual original control commands; use diagnostic receipt epochs for alignment.
    diagnostics = [r for r in kinds.get('diagnostic', []) if r['status'] == 'FOLLOW']
    report['original_follow_commands'] = dict(
        samples=len(diagnostics), status=dict(Counter(r['status'] for r in kinds.get(
            'diagnostic', []))), locks=sum(r['locked'] for r in diagnostics),
        visible=sum(r['visible'] for r in diagnostics),
        command_horizontal_speed=stats([float(np.linalg.norm(r['v'][:2])) for r in diagnostics]),
        command_horizontal_acceleration_field=stats([float(np.linalg.norm(r['a'][:2]))
                                                     for r in diagnostics]),
        acceleration_field_is_actual_acceleration=False)
    csv_rows = []
    for p in (root/'evaluator').glob('*mission_1.csv'):
        csv_rows += list(csv.DictReader(p.open()))
    following = [r for r in csv_rows if r['phase'] == 'FOLLOW' and r['uav_available'] == 'True'
                 and r['truth_available'] == 'True']
    errors, distance_errors, acceleration = [], [], []
    samples = []
    for row in following:
        target = np.array([float(row['target_'+axis]) for axis in ('x', 'y', 'z')])
        velocity = np.array([float(row['target_v'+axis]) for axis in ('x', 'y')])
        uav = np.array([float(row['uav_'+axis]) for axis in ('x', 'y', 'z')])
        speed = np.linalg.norm(velocity)
        direction = velocity/max(speed, 1e-9) if speed > 1e-6 else np.array((1., 0.))
        desired = np.r_[target[:2]-5*direction, -5.]
        errors.append(float(np.linalg.norm(uav-desired)))
        distance_errors.append(float(np.linalg.norm((uav-target)[:2])-5.))
        samples.append((float(row['time']), np.array([float(row['uav_v'+axis])
                                                     for axis in ('x', 'y', 'z')])))
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        if .01 < t1-t0 < .2:
            acceleration.append(float(np.linalg.norm((v1-v0)[:2]/(t1-t0))))
    report['original_follow_evaluator'] = dict(
        samples=len(following), position_rmse=float(np.sqrt(np.mean(np.square(errors))))
        if errors else None,
        distance_error_rmse=float(np.sqrt(np.mean(np.square(distance_errors))))
        if distance_errors else None, estimated_horizontal_acceleration=stats(acceleration),
        state_time_alignment='evaluator snapshots, not image acquisition epochs',
        measurement_error_is_control_error=False,
        shadow_rmse_is_closed_loop_rmse=False)
    (root/'p32_analysis.json').write_text(json.dumps(report, indent=2))
    printed_keys = ('completions', 'core_feasible', 'published', 'initialization',
                    'whole_cycle', 'rendered_motion')
    print(json.dumps({key: report[key] for key in printed_keys}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root')
    arguments = parser.parse_args()
    analyze(arguments.root)
