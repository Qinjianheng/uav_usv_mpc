#!/usr/bin/env python3
"""Fixed-window actual P7 FOLLOW curves; truth is used only in this offline tool."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from p4_follow_analysis import read  # noqa: E402
from p5_minco_sitl import sha  # noqa: E402


def analyze(run, output):
    summary = json.loads((run/'summary.json').read_text())
    condition = json.loads((run/'condition.json').read_text())
    start = summary['first_minco_receipt']
    if start is None:
        raise ValueError('NO_ACTUAL_MINCO_WINDOW')
    duration = condition['observation_seconds']
    rows, malformed = read(run/'offline_evidence.jsonl')
    if malformed:
        raise ValueError('MALFORMED_NATIVE_EVIDENCE')
    states = sorted((r for r in rows if r['kind'] == 'uav' and r['valid']),
                    key=lambda r: r['stamp'])
    targets = sorted((r for r in rows if r['kind'] == 'ideal' and r['valid']),
                     key=lambda r: r['stamp'])
    times = np.array([r['stamp'] for r in targets])
    up = np.array([r['p'] for r in states])
    ut = np.array([r['stamp'] for r in states])
    actual = [r for r in states if start <= r['stamp'] <= start+duration
              and times[0] <= r['stamp'] <= times[-1]]
    t = np.array([r['stamp'] for r in actual])
    p, v, a = (np.array([r[k] for r in actual]) for k in ('p', 'v', 'a'))

    def target_values(key):
        values = np.array([r[key] for r in targets])
        return np.column_stack([np.interp(t, times, values[:, i]) for i in range(3)])

    tp, tv = target_values('p'), target_values('v')
    speed = np.linalg.norm(tv[:, :2], axis=1)
    desired = tp[:, :2]-5.*tv[:, :2]/np.maximum(speed[:, None], 1e-9)
    error = np.linalg.norm(p[:, :2]-desired, axis=1)
    velocity_error = np.linalg.norm(v[:, :2]-tv[:, :2], axis=1)
    diagnostics = [r for r in rows if r['kind'] == 'diagnostic'
                   and start <= r['stamp'] <= start+duration]
    refs = [r for r in rows if r['kind'] == 'execution_debug' and r.get('reference')
            and start <= r['stamp'] <= start+duration and ut[0] <= r['stamp'] <= ut[-1]]
    rt = np.array([r['stamp'] for r in refs])
    rp = np.array([r['reference'][:3] for r in refs])
    measured = np.column_stack([np.interp(rt, ut, up[:, i]) for i in range(3)])
    rmse = lambda values: float(np.sqrt(np.mean(np.square(values))))  # noqa: E731
    metrics = dict(
        window='fixed first-MINCO receipt + 25/60 s; includes every control owner',
        start=start, requested_duration=duration, completed=summary['completed'],
        measured_span=float(t[-1]-t[0]), native_samples=len(actual), reference_samples=len(refs),
        horizontal_viewpoint_rmse_m=rmse(error), altitude_rmse_m=rmse(p[:, 2]+5.),
        target_relative_velocity_rmse_mps=rmse(velocity_error),
        actual_reference_3d_rmse_m=rmse(np.linalg.norm(measured-rp, axis=1)),
        maximum_actual_horizontal_speed_mps=float(np.max(np.linalg.norm(v[:, :2], axis=1))),
        maximum_actual_horizontal_acceleration_mps2=float(np.max(
            np.linalg.norm(a[:, :2], axis=1))),
        visible_fraction=sum(r['visible'] for r in diagnostics)/len(diagnostics),
        locked_fraction=sum(r['locked'] for r in diagnostics)/len(diagnostics),
        minco_fraction=summary['minco_fraction'],
        longest_minco_seconds=summary['longest_minco_seconds'],
        fallbacks=summary['p51_audit']['fallback_reasons'],
        source_fingerprint=summary['source_fingerprint'],
        evidence={str(run/'summary.json'): sha(run/'summary.json'),
                  str(run/'offline_evidence.jsonl'): sha(run/'offline_evidence.jsonl')},
        interpolation='offline only, original sample epochs, inside recorded coverage')
    output.mkdir(exist_ok=False)
    (output/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    with (output/'native_window.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['elapsed', 'uav_x', 'uav_y', 'uav_z', 'uav_vx', 'uav_vy', 'uav_vz',
                         'target_x', 'target_y', 'target_z', 'viewpoint_error', 'relative_speed'])
        writer.writerows(np.column_stack((t-start, p, v, tp, error, velocity_error)))
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    axes[0, 0].plot(tp[:, 0], tp[:, 1], label='USV')
    axes[0, 0].plot(desired[:, 0], desired[:, 1], label='5 m viewpoint')
    axes[0, 0].plot(p[:, 0], p[:, 1], label='PX4 UAV')
    axes[0, 0].set(xlabel='North (m)', ylabel='East (m)', title='Actual horizontal track')
    axes[0, 0].axis('equal')
    axes[0, 1].plot(t-start, error, label='viewpoint error')
    axes[0, 1].plot(rt-start, np.linalg.norm(measured-rp, axis=1), label='reference error')
    axes[0, 1].set(xlabel='Time (s)', ylabel='Error (m)', title='All owners in fixed window')
    axes[1, 0].plot(t-start, np.linalg.norm(v[:, :2], axis=1), label='UAV speed')
    axes[1, 0].plot(t-start, speed, label='USV speed')
    axes[1, 0].set(xlabel='Time (s)', ylabel='Speed (m/s)')
    axes[1, 1].plot(t-start, -p[:, 2], label='UAV height')
    axes[1, 1].axhline(5., linestyle='--', color='gray', label='5 m reference')
    axes[1, 1].set(xlabel='Time (s)', ylabel='Height (m)')
    for ax in axes.flat:
        ax.grid(alpha=.25)
        ax.legend()
    fig.suptitle('P7 '+condition['scene'].upper()+' — actual PX4 navigation')
    fig.tight_layout()
    fig.savefig(output/'follow.png', dpi=160)
    plt.close(fig)
    (output/'SHA256SUMS.json').write_text(json.dumps(
        {p.name: sha(p) for p in output.iterdir()}, indent=2)+'\n')
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    analyze(args.run.resolve(), args.output.resolve())
