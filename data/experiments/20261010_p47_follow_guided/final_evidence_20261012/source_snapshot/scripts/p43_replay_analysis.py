#!/usr/bin/env python3
"""Matched replay populations and phase series; never interpret them as actual UAV execution."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def read(path):
    """Read immutable case rows with explicit source identities."""
    return [json.loads(line) for line in path.open()]


def key(row):
    """Match the frozen source, not a solver's episode or acceptance population."""
    return row['source_sha256'], row['line']


def rmse(rows):
    """Use all provided sample errors, including curve continuation after rejected proposals."""
    return {metric: float(np.sqrt(np.mean([r['error'][metric]**2 for r in rows])))
            if rows else None for metric in ('fixed', 'selected', 'distance', 'relative_velocity')}


def matched(paths):
    """Intersect sample availability and apply one common absolute steady window."""
    cases = {p.stem: read(p) for p in paths}
    populations = {name: {key(r): r for r in rows if r['error'] is not None}
                   for name, rows in cases.items()}
    common = set.intersection(*(set(rows) for rows in populations.values()))
    if not common:
        return dict(count=0, cases={})
    begin = max(min(r['epoch'] for r in rows.values()) for rows in populations.values())
    end = min(max(r['epoch'] for r in rows.values()) for rows in populations.values())
    common = {k for k in common if begin <= next(iter(populations.values()))[k]['epoch'] <= end}
    return dict(count=len(common), start=begin, end=end, steady_start=begin+20,
                selection='source SHA + source line, common sampled population, absolute window',
                cases={name: dict(
                    all=rmse([rows[k] for k in common]),
                    after20=rmse([rows[k] for k in common if rows[k]['epoch'] >= begin+20]))
                    for name, rows in populations.items()})


def physical_summary(rows):
    """Recover nominal thrust from recorded accelerations when old diagnostics omitted it."""
    valid = [r for r in rows if r['result']['valid']]
    thrust, acceleration, yaw, tilt, axis_rate = [], [], [], [], []
    refine, attempts = Counter(), Counter()
    for row in valid:
        result, metrics = row['result'], row['result']['metrics']
        a = np.asarray(result['accelerations'])
        forces = np.array((0., 0., 9.80665))-a
        thrust.extend(np.linalg.norm(forces, axis=1))
        jerk = np.asarray(result['jerks'])
        axis_rate.extend(np.linalg.norm(np.cross(forces, -jerk), axis=1)
                         / np.sum(forces**2, axis=1))
        acceleration.extend(np.linalg.norm(a[:, :2], axis=1))
        yaw.extend(np.abs(result['yaw_rates']))
        tilt.extend(np.arctan2(np.linalg.norm(forces[:, :2], axis=1), forces[:, 2]))
        for r in metrics.get('p43', {}).get('refinements', []):
            refine[str((r.get('mode'), r.get('valid'), r.get('timeout', False)))] += 1
        for r in metrics.get('p43', {}).get('attempts', []):
            attempts[r['status']] += 1
    return dict(valid=len(valid), maximum_sampled_thrust=max(thrust, default=None),
                maximum_sampled_acceleration=max(acceleration, default=None),
                maximum_yaw_rate=max(yaw, default=None),
                maximum_sampled_tilt=max(tilt, default=None),
                maximum_nominal_thrust_axis_rate=max(axis_rate, default=None),
                refinements=dict(refine), candidate_statuses=dict(attempts),
                sampled_values_are_certified_bounds=False)


def run(root):
    """Save comparisons and per-cycle phase/quality plots in this new experiment only."""
    paths = sorted((root/'optimizer_batched').glob('f3*rolling.jsonl'))
    horizon_paths = sorted((root/'horizons_final').glob('f3_a3.0*rolling.jsonl'))
    f2_paths = sorted((root/'optimizer_batched').glob('f2*rolling.jsonl'))
    summaries = dict(optimizer_f3=matched(paths), horizons_f3=matched(horizon_paths),
                     optimizer_f2=matched(f2_paths), physical={
                         p.stem: physical_summary(read(p)) for p in paths+horizon_paths})
    if (root/'common_initial/summary.json').exists():
        common = sorted((root/'common_initial').glob('f3*rolling.jsonl'))
        summaries['common_initial_f3'] = matched(common)
        summaries['common_initial_f2'] = matched(
            sorted((root/'common_initial').glob('f2*rolling.jsonl')))
        summaries['common_initial_physical'] = {p.stem: physical_summary(read(p)) for p in common}
    summaries['limits_physical'] = {p.stem: physical_summary(read(p))
                                    for p in (root/'limits').glob('*rolling.jsonl')}
    (root/'replay_comparison.json').write_text(json.dumps(summaries, indent=2, allow_nan=False))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    selected = [p for p in paths if ('h1.2_p41' in p.name or 'h1.6_p41' in p.name
                                     or 'h1.6_p43_none' in p.name or 'h1.6_p43_q_' in p.name)]
    common_origin = min(read(p)[0]['epoch'] for p in selected)
    for path in selected:
        rows = [r for r in read(path) if r['error']]
        epoch = np.array([r['epoch'] for r in rows])
        label = path.stem.split('_h', 1)[1].replace('_rolling', '')
        axes[0].plot(epoch-common_origin, [r['error']['fixed'] for r in rows], label=label)
        axes[1].plot(epoch-common_origin, [r['seconds']*1000 for r in rows], label=label, alpha=.7)
        axes[2].step(epoch-common_origin, [r['beta'] for r in rows], label=label)
    axes[0].set_ylabel('Hypothetical rear error (m)')
    axes[1].set_ylabel('Solver wall time (ms)')
    axes[2].set_ylabel('Selected beta (rad)')
    axes[2].set_xlabel('Seconds from common source start')
    axes[0].legend(ncol=2)
    for ax in axes:
        ax.axvline(20, color='gray', linestyle='--')
        ax.grid(alpha=.2)
    fig.savefig(root/'replay_timeseries.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    run(parser.parse_args().root)
