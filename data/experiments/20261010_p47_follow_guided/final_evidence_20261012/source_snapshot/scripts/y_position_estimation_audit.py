#!/usr/bin/env python3
"""Offline Y audit using acquisition-time Gazebo entities, never online truth."""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


def seconds(stamp):
    return float(stamp['sec']) + float(stamp['nanosec']) * 1e-9


def xyz(vector):
    return np.array([vector[k] for k in 'xyz'], dtype=float)


def bracketed_position(times, positions, stamp, maximum_gap=.15):
    """Interpolate a physical reference without extrapolation or fitted lag."""
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and abs(times[index] - stamp) < 1e-7:
        return positions[index]
    if index == 0 or index == len(times):
        return None
    gap = times[index] - times[index - 1]
    if not 0 < gap <= maximum_gap:
        return None
    ratio = (stamp - times[index - 1]) / gap
    return positions[index - 1] + ratio * (
        positions[index] - positions[index - 1]
    )


def error_stats(rows):
    if not rows:
        return {'count': 0}
    errors = np.array([row[1:] for row in rows])
    horizontal = np.linalg.norm(errors[:, :2], axis=1)
    distance = np.linalg.norm(errors, axis=1)
    return {
        'count': len(rows), 'signed_xyz_mean_m': errors.mean(axis=0).tolist(),
        'horizontal_rmse_m': float(np.sqrt(np.mean(horizontal ** 2))),
        'horizontal_p50_m': float(np.median(horizontal)),
        'horizontal_p95_m': float(np.percentile(horizontal, 95)),
        'position_3d_rmse_m': float(np.sqrt(np.mean(distance ** 2))),
    }


def audit(records, vision):
    y = next(x['receipt'] for x in records
             if x['topic'] == 'driver' and x['message']['command'] == 'Y')
    columns = ['measurement_stamp'] + [
        'gazebo_entity_reference_' + axis for axis in 'xyz'
    ]
    reference = vision.loc[
        vision['gazebo_entity_available'].astype(str).str.lower().eq('true'),
        columns,
    ].dropna().drop_duplicates('measurement_stamp').sort_values('measurement_stamp')
    if len(reference) < 2:
        raise ValueError('Native Gazebo USV reference is missing; enable geometry diagnostics')
    times = reference.iloc[:, 0].to_numpy()
    positions = reference.iloc[:, 1:].to_numpy()
    series = {'rgbd': [], 'kf': [], **{
        f'forecast_{h:g}s': [] for h in (.3, .5, 1., 2., 3.)
    }}
    for _, row in vision.iterrows():
        stamp = float(row['measurement_stamp'])
        if str(row['valid']).lower() != 'true':
            continue
        actual = bracketed_position(times, positions, stamp)
        estimated = np.array([row['position_' + a] for a in 'xyz'])
        if actual is not None and np.isfinite(estimated).all():
            series['rgbd'].append([stamp - y, *(estimated - actual)])
    contacts, pending = [], {}
    for record in records:
        m = record['message']
        if record['topic'] == 'kf' and m['valid']:
            stamp = seconds(m['stamp'])
            actual = bracketed_position(times, positions, stamp)
            if actual is not None:
                series['kf'].append([stamp - y, *(xyz(m['position']) - actual)])
        elif record['topic'] == 'prediction' and m['valid'] and m['samples']:
            stamp = seconds(m['source_stamp'])
            rel = np.array([seconds(p['relative_time']) for p in m['samples']])
            points = np.array([xyz(p['position']) for p in m['samples']])
            for horizon in (.3, .5, 1., 2., 3.):
                actual = bracketed_position(times, positions, stamp + horizon)
                if actual is None or not rel[0] <= horizon <= rel[-1]:
                    continue
                estimated = np.array([np.interp(horizon, rel, points[:, a])
                                      for a in range(3)])
                series[f'forecast_{horizon:g}s'].append(
                    [stamp - y, *(estimated - actual)]
                )
        elif record['topic'] == 'trajectory':
            pending[int(m['plan_id'])] = m
        elif (record['topic'] == 'controller' and m['status'] == 'PLAN_ACCEPTED'
              and m['trajectory_replaced']):
            plan = pending.get(int(m['attempted_plan_id']))
            if plan is None:
                continue
            stamp = seconds(plan['contact_stamp'])
            actual = bracketed_position(times, positions, stamp)
            if actual is None:
                continue
            error = xyz(plan['terminal_position']) - actual
            contacts.append({
                'plan_id': plan['plan_id'], 'accepted_y_seconds': record['receipt'] - y,
                'contact_y_seconds': stamp - y,
                'remaining_t_go_at_accept': stamp - record['receipt'],
                'planned_contact_minus_actual_usv_xyz_m': error.tolist(),
                'horizontal_contact_error_m': float(np.linalg.norm(error[:2])),
                'position_3d_contact_error_m': float(np.linalg.norm(error)),
            })
    windows = {'follow_before_y': (-5., 0.), 'y_early': (0., 2.),
               'y_approach': (2., 6.), 'y_terminal': (6., 8.),
               'y_recovery_and_retry': (8., 30.)}
    stats = {name: {
        key: error_stats([row for row in values if start <= row[0] < end])
        for key, values in series.items()
    } for name, (start, end) in windows.items()}
    return {
        'y_ros_stamp': y, 'reference': 'NATIVE_GAZEBO_USV_AT_ACQUISITION',
        'role': 'OFFLINE_EVALUATION_ONLY', 'no_extrapolation': True,
        'maximum_reference_bracket_gap_seconds': .15,
        'reference_count': len(reference), 'windows_seconds_from_y': windows,
        'statistics': stats, 'accepted_contacts': contacts,
        'results': [x['message'] for x in records if x['topic'] == 'result'],
    }, series


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jsonl', required=True, type=Path)
    parser.add_argument('--vision', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    with args.jsonl.open() as stream:
        records = [json.loads(line) for line in stream]
    report, series = audit(records, pd.read_csv(args.vision))
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 4))
    for key in ('rgbd', 'kf', 'forecast_1s'):
        rows = np.array(series[key])
        if len(rows):
            ax.plot(rows[:, 0], np.linalg.norm(rows[:, 1:3], axis=1),
                    label=key, alpha=.8, linewidth=1.)
    ax.axvline(0, color='black', linestyle='--', label='Y')
    ax.axhline(.5, color='gray', linestyle=':', label='3D capture radius (reference)')
    ax.set(xlim=(-5, 30), ylim=(0, 2), xlabel='Seconds from Y',
           ylabel='Horizontal position error to native USV (m)')
    ax.grid(alpha=.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output.with_suffix('.png'), dpi=180)
    print(json.dumps(report['statistics'], indent=2))


if __name__ == '__main__':
    main()
