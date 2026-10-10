#!/usr/bin/env python3
"""Offline actual ownership, native command timing and same-epoch reference error analysis."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from p4_follow_analysis import analyze as base_analysis, distribution, read


def ownership(rows, start=None, end=None):
    """Time weight controller diagnostics, explicitly excluding gaps above 150 ms."""
    rows = sorted(rows, key=lambda r: r['stamp'])
    seconds, spans, current = Counter(), [], 0.
    fallback = 0
    for a, b in zip(rows, rows[1:]):
        lo = max(a['stamp'], start) if start is not None else a['stamp']
        hi = min(b['stamp'], end) if end is not None else b['stamp']
        dt = max(0., hi-lo)
        if not dt:
            continue
        if b['stamp']-a['stamp'] > .15:
            seconds['UNKNOWN_GAP'] += dt
            if current:
                spans.append(current)
            current = 0.
            continue
        seconds[a['status']] += dt
        if a['status'] == 'MINCO_FOLLOW':
            current += dt
            if b['status'] != 'MINCO_FOLLOW':
                fallback += 1
                spans.append(current)
                current = 0.
        elif current:
            spans.append(current)
            current = 0.
    if current:
        spans.append(current)
    total = sum(seconds.values())
    return dict(seconds=dict(seconds),
                minco_fraction=seconds['MINCO_FOLLOW']/total if total else 0.,
                continuous_seconds=distribution(spans), fallback_count=fallback)


def norm_rmse(values):
    """Euclidean vector RMSE, null for absent measurements."""
    return float(np.sqrt(np.mean(np.sum(np.asarray(values)**2, axis=1)))) if values else None


def sample(proposal, epoch):
    """Evaluate recorded coefficients at a declared support epoch for offline audit only."""
    from uav_control.guidance.follow_fast_validation import fields
    x = np.asarray(proposal['xyz']).reshape(-1, 6, 3)
    dt = epoch-proposal['start']
    p, v, a, _, yaw, rate = fields(
        x, proposal['durations'], np.asarray(proposal['yaw']).reshape(-1, 4), np.array([dt]))
    return np.r_[p[0], v[0], a[0], yaw[0], rate[0]]


def analyze(root):
    """Retain mixed-control and active-only populations separately; never amend raw evidence."""
    condition = json.loads((root/'condition.json').read_text())
    report = base_analysis(root, ('FOLLOW', 'MINCO_FOLLOW'), condition['actual_controller'])
    rows, malformed = read(root/'offline_evidence.jsonl')
    kinds = {k: [r for r in rows if r['kind'] == k] for k in {r['kind'] for r in rows}}
    diag = [r for r in kinds.get('diagnostic', []) if r['status'] in ('FOLLOW', 'MINCO_FOLLOW')]
    start = diag[0]['stamp'] if diag else 0.
    stable = [r for r in diag if 20 <= r['stamp']-start <= 60]
    report['ownership'] = dict(all_follow=ownership(diag),
                               stable=ownership(diag, start+20, start+60))
    refs = [r for r in kinds.get('reference', []) if start+20 <= r['receipt'] <= start+60]
    periods, steps, accel, switches = [], [], [], []
    stamps = np.array([r['stamp'] for r in diag])
    for a, b in zip(refs, refs[1:]):
        dt = b['stamp']-a['stamp']
        if not 0 < dt <= .5:
            continue
        step = float(np.linalg.norm(np.subtract(b['v'], a['v'])[:2]))
        periods.append(dt)
        steps.append(step)
        accel.append(step/dt)
        i = np.searchsorted(stamps, a['receipt'], side='right')-1
        j = np.searchsorted(stamps, b['receipt'], side='right')-1
        if i >= 0 and j >= 0 and (diag[i]['status'], diag[i]['plan']) != (
                diag[j]['status'], diag[j]['plan']):
            switches.append(step)
    report['native_command'] = dict(period_seconds=distribution(periods),
                                    horizontal_step_mps=distribution(steps),
                                    acceleration_mps2=distribution(accel),
                                    switch_step_mps=distribution(switches),
                                    note='PX4 setpoint timestamp deltas')
    report['tracker'] = dict(
        callback_seconds=distribution([r['compute'] for r in stable if 'compute' in r]),
        prefix_validation_seconds=distribution(
                                 [r['validation_seconds'] for r in kinds.get('execution_debug', [])
                                  if 20 <= r['stamp']-start <= 60]),
        callback_over_50ms=sum(r.get('compute', 0) > .05 for r in stable))
    proposals = {r['plan']: r for r in kinds.get('proposal', [])}
    acks = kinds.get('follow_ack', [])
    active = [r for r in acks if r['state'] == 'ACTIVE']
    accepted = [r for r in acks if r['state'] == 'ACCEPTED']
    deviations, handovers = [], []
    for ack in active:
        c = proposals.get(ack['plan'])
        if not c:
            continue
        deviations.append(ack['stamp']-c['start'])
        parent = proposals.get(c.get('parent'))
        if parent and 'xyz' in c and 'xyz' in parent:
            d = sample(c, c['start'])-sample(parent, c['start'])
            d[9] = np.arctan2(np.sin(d[9]), np.cos(d[9]))
            handovers.append(d.tolist())
    report['lifecycle'] = dict(proposals=len(proposals),
                               accepted_unique=len({r['plan'] for r in accepted}),
                               active_unique=len({r['plan'] for r in active}),
                               reasons=dict(Counter(x for r in acks for x in r['reasons'])),
                               activation_deviation_seconds=distribution(deviations),
                               handover_max_abs_pva_yaw_rate=(
                                   np.max(np.abs(handovers), axis=0).tolist()
                                   if handovers else None),
                               receiver_service_seconds=distribution(
                                   [r['receiver_compute_seconds'] for r in acks
                                    if r['state'] in ('ACCEPTED', 'REJECTED')]),
                               first_active=active[0]['stamp'] if active else None)
    truth = [r for r in kinds.get('ideal', []) if r['valid']]
    ts = np.asarray([r['stamp'] for r in truth])
    errors, trajectory, tracking, traces = [], [], [], []
    for r in kinds.get('execution_debug', []):
        if not r['plan'] or 'reference' not in r or not 20 <= r['stamp']-start <= 60:
            continue
        t = r['state_stamp']
        if not len(ts) or not ts[0] <= t <= ts[-1] or t < r['active_start']:
            continue
        tp = [np.interp(t, ts, [s['p'][i] for s in truth]) for i in (0, 1)]
        tv = np.array([np.interp(t, ts, [s['v'][i] for s in truth]) for i in (0, 1)])
        viewpoint = np.array(tp)-5*tv/max(np.linalg.norm(tv), 1e-9)
        measured, reference = np.array(r['actual_position'][:2]), np.array(r['reference'][:2])
        errors.append((measured-viewpoint).tolist())
        trajectory.append((reference-viewpoint).tolist())
        tracking.append((measured-reference).tolist())
        traces.append(dict(epoch=t, actual_error=float(np.linalg.norm(measured-viewpoint)),
                           planned_error=float(np.linalg.norm(reference-viewpoint)),
                           tracking_error=float(np.linalg.norm(measured-reference))))
    report['active_only_decomposition'] = dict(
        samples=len(errors), actual_viewpoint_rmse=norm_rmse(errors),
        planned_viewpoint_rmse=norm_rmse(trajectory), tracking_rmse=norm_rmse(tracking),
        convention='same navigation epoch; offline TargetState interpolation, no extrapolation')
    report['healthy'] = bool(report['end'].get('reason') == 'FOLLOW_WINDOW_COMPLETE'
                             and report['end'].get('monitor_returncode') == 0
                             and not report['end'].get('still_alive')
                             and report['authority']['all_discovered_single_tracker']
                             and len(stable) >= 500 and not malformed
                             and (condition['actual_controller'] == 'ORIGINAL_FOLLOW'
                                  or len(active) >= 20))
    with (root/'reference_errors.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=('epoch', 'actual_error', 'planned_error',
                                                    'tracking_error'))
        writer.writeheader()
        writer.writerows(traces)
    from p45_physical_analysis import extend
    extend(root, report, kinds, start)
    (root/'execution_analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    args = p.parse_args()
    result = analyze(args.root)
    print(json.dumps({k: result[k] for k in ('healthy', 'ownership', 'tracker', 'lifecycle')}))
