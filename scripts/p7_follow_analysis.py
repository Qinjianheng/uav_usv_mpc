#!/usr/bin/env python3
"""Fixed-window actual P7 FOLLOW curves; truth is used only in this offline tool."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from p4_follow_analysis import read  # noqa: E402
from p5_minco_sitl import sha  # noqa: E402


def guidance_metrics(run, reason, stop=None):
    """Judge actual guidance, reference arbitration and safe finite standoff in a new run."""
    rows, malformed = read(run/'offline_evidence.jsonl')
    guidance = [r for r in rows if r['kind'] == 'p7_guidance' and r.get('valid')]
    if not guidance:
        return dict(completed=False, healthy=False, reason=reason, f3_eligible=False)
    start = guidance[0]['stamp']
    if stop is None:
        stop = json.loads((run/'bounded_stop.json').read_text())['stamp']
    selected = [r for r in rows if start <= r.get('stamp', r['receipt']) <= stop]
    diagnostics = [r for r in selected if r['kind'] == 'diagnostic']
    native = [r for r in selected if r['kind'] == 'uav' and r['valid']]
    references = [r for r in selected if r['kind'] == 'reference']
    graphs = [r for r in rows if r['kind'] == 'graph' and start <= r['receipt'] <= stop]
    topics = ('/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode',
              '/fmu/in/vehicle_command')
    one_tracker = bool(graphs) and all(
        len(r['publishers'].get(t, [])) == 1 and
        r['publishers'][t][0]['node'] == 'trajectory_tracker_node'
        for r in graphs for t in topics)
    one_bctra = bool(graphs) and all(
        len(r['publishers'].get('/planning/target_prediction', [])) == 1
        and r['publishers']['/planning/target_prediction'][0]['node'] == 'target_predictor_node'
        for r in graphs)
    retired = [r for r in selected if (r['kind'] == 'follow_ack' and r['state'] == 'ACTIVE')
               or r['kind'] == 'intercept_proposal']
    valid_interface = bool(references) and all(
        all(math.isfinite(v) for v in r['v'])
        and not any(math.isfinite(v) for v in (*r['p'], *r['a'])) for r in references)
    targets = sorted((r for r in rows if r['kind'] == 'ideal' and r['valid']),
                     key=lambda r: r['stamp'])
    tt = np.array([r['stamp'] for r in targets])
    tp = np.array([r['p'][:2] for r in targets])
    physical = [r for r in native if tt[0] <= r['stamp'] <= tt[-1]]
    t = np.array([r['stamp'] for r in physical])
    p = np.array([r['p'] for r in physical])
    target = np.column_stack([np.interp(t, tt, tp[:, i]) for i in (0, 1)])
    distance = np.linalg.norm(p[:, :2]-target, axis=1)
    altitude_error = np.abs(p[:, 2]+5.)
    speeds = [np.linalg.norm(r['v'][:2]) for r in native]
    accelerations = [np.linalg.norm(r['a'][:2]) for r in native]
    emitted = [r for r in rows if r['kind'] == 'reference' and r['stamp'] <= stop
               and all(math.isfinite(v) for v in r['v'])]
    steps = [(b['stamp'], float(np.linalg.norm(np.subtract(b['v'], a['v'])[:2])),
              b['stamp']-a['stamp']) for a, b in zip(emitted, emitted[1:])
             if start <= b['stamp'] and 0 < b['stamp']-a['stamp'] <= .2]
    y_step = steps[0][1] if steps else None
    active_fraction = (sum(r['status'] == 'P7_GUIDANCE' for r in diagnostics)
                       / max(1, len(diagnostics)))
    visible_fraction = sum(r['visible'] for r in diagnostics)/max(1, len(diagnostics))
    final_distance = float(np.median(distance[t >= t[-1]-2.]))
    complete = reason == 'P7_GUIDANCE_OBSERVATION_COMPLETE'
    healthy = bool(complete and not malformed and one_tracker and one_bctra
                   and not retired and valid_interface and len(guidance) >= 100
                   and active_fraction >= .9 and visible_fraction >= .95
                   and max(altitude_error) <= 1. and max(speeds) <= 7.
                   and max(accelerations) <= 4. and 4. <= final_distance <= 5.3
                   and min(distance) >= 3.8 and float(distance[0])-final_distance >= .2)
    return dict(completed=complete, healthy=healthy, reason=reason, f3_eligible=False,
                guidance_start=start, guidance_samples=len(guidance),
                guidance_fraction=active_fraction, visible_fraction=visible_fraction,
                initial_distance_m=float(distance[0]), final_distance_m=final_distance,
                minimum_distance_m=float(min(distance)),
                maximum_height_error_m=float(max(altitude_error)),
                actual_speed_max_mps=float(max(speeds)),
                actual_acceleration_max_mps2=float(max(accelerations)),
                single_tracker=one_tracker, single_bctra=one_bctra,
                velocity_only_interface=valid_interface, retired_authority_events=len(retired),
                y_velocity_step_mps=y_step,
                command_acceleration_max_mps2=max((s[1]/s[2] for s in steps), default=None),
                image_error_u_max=max(abs(r['error_u']) for r in guidance),
                image_error_v_max=max(abs(r['error_v']) for r in guidance),
                yaw_rate_max_radps=max(abs(r['yaw_rate']) for r in guidance),
                lateral_acceleration_max_mps2=max(
                    abs(r['lateral_acceleration']) for r in guidance),
                png_samples=sum(r['mode'] == 'PNG' for r in guidance),
                physical_window_end=stop)


def analyze_guidance(run, output):
    summary = json.loads((run/'summary.json').read_text())
    rows, _ = read(run/'offline_evidence.jsonl')
    start, stop = summary['guidance_start'], summary['physical_window_end']
    guidance = [r for r in rows if r['kind'] == 'p7_guidance' and start <= r['stamp'] <= stop]
    native = [r for r in rows if r['kind'] == 'uav' and r['valid']
              and start <= r['stamp'] <= stop]
    t = np.array([r['stamp']-start for r in guidance])
    nt = np.array([r['stamp']-start for r in native])
    headings = np.unwrap([r['yaw'] for r in native])
    earlier = np.searchsorted(nt, nt-.2, side='right')-1
    valid = earlier >= 0
    yaw_times = nt[valid]
    yaw_rates = ((headings[valid]-headings[earlier[valid]])
                 / (nt[valid]-nt[earlier[valid]]))
    before = [r for r in rows if r['kind'] == 'diagnostic' and r['stamp'] < start]
    minco = [r for r in before if r['status'] == 'MINCO_FOLLOW']
    summary['minco_before_y_observed_span_s'] = (
        minco[-1]['stamp']-minco[0]['stamp'] if minco else 0.)
    summary['actual_yaw_rate_0p2s_max_radps'] = float(max(abs(yaw_rates)))
    summary['source_summary_sha256'] = sha(run/'summary.json')
    output.mkdir(exist_ok=False)
    (output/'metrics.json').write_text(json.dumps(summary, indent=2)+'\n')
    fields = ('stamp', 'distance', 'error_u', 'error_v', 'los_rate', 'closing_speed',
              'lateral_acceleration', 'yaw_rate', 'mode')
    with (output/'guidance.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k: r[k] for k in fields} for r in guidance)
    fig, axes = plt.subplots(3, 2, figsize=(11, 9))
    for key, label in (('error_u', 'horizontal'), ('error_v', 'vertical')):
        axes[0, 0].plot(t, [r[key] for r in guidance], label=label)
    axes[0, 0].set(ylabel='Normalized image error')
    axes[0, 1].plot(t, [r['distance'] for r in guidance], label='KF horizontal distance')
    axes[0, 1].axhline(4.5, linestyle='--', color='gray', label='safe standoff')
    axes[0, 1].set(ylabel='Distance (m)')
    axes[1, 0].plot(t, [r['yaw_rate'] for r in guidance], label='IBVS yaw rate')
    axes[1, 0].plot(t, [r['los_rate'] for r in guidance], label='LOS rate')
    axes[1, 0].plot(yaw_times, yaw_rates, linestyle='--', alpha=.7,
                    label='PX4 heading derivative (0.2 s)')
    axes[1, 0].set(ylabel='Angular rate (rad/s)')
    axes[1, 1].plot(t, [r['closing_speed'] for r in guidance], label='measured closure')
    axes[1, 1].plot(t, [r['lateral_acceleration'] for r in guidance], label='command lateral a')
    axes[1, 1].set(ylabel='m/s or m/s²')
    axes[2, 0].plot(nt, [np.linalg.norm(r['v'][:2]) for r in native], label='PX4 actual speed')
    axes[2, 0].plot(t, [np.linalg.norm(r['velocity'][:2]) for r in guidance],
                    label='velocity command')
    axes[2, 0].set(ylabel='Horizontal speed (m/s)')
    axes[2, 1].plot(nt, [-r['p'][2] for r in native], label='PX4 actual height')
    axes[2, 1].axhline(5., linestyle='--', color='gray', label='height reference')
    axes[2, 1].set(ylabel='Height (m)')
    for ax in axes.flat:
        ax.set_xlabel('Time since guidance (s)')
        ax.grid(alpha=.25)
        ax.legend()
    fig.tight_layout()
    fig.savefig(output/'guidance.png', dpi=160)
    plt.close(fig)
    references = [r for r in rows if r['kind'] == 'reference'
                  and start-3. <= r['stamp'] <= start+4.
                  and all(math.isfinite(v) for v in r['v'])]
    measured = [r for r in rows if r['kind'] == 'uav' and r['valid']
                and start-3. <= r['stamp'] <= start+4.]
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for i, (axis, name) in enumerate(zip(axes, ('North', 'East', 'Down'))):
        axis.plot([r['stamp']-start for r in references], [r['v'][i] for r in references],
                  label='final PX4 velocity setpoint')
        axis.plot([r['stamp']-start for r in measured], [r['v'][i] for r in measured],
                  label='actual PX4 velocity', linestyle='--')
        axis.axvline(0., color='black', linestyle=':', label='first P7 guidance output')
        axis.set_ylabel(name+' velocity (m/s)')
        axis.grid(alpha=.25)
        axis.legend()
    axes[-1].set_xlabel('Time across Y handover (s)')
    fig.tight_layout()
    fig.savefig(output/'handover.png', dpi=160)
    plt.close(fig)
    takeoff = next(r['receipt'] for r in rows if r['kind'] == 'mission' and r['name'] == 'TAKEOFF')
    flown = [r for r in rows if r['kind'] == 'uav' and r['valid']
             and takeoff <= r['stamp'] <= stop]
    boat = [r for r in rows if r['kind'] == 'ideal' and r['valid']
            and takeoff <= r['stamp'] <= stop]
    diag = [r for r in rows if r['kind'] == 'diagnostic' and takeoff <= r['stamp'] <= stop]
    codes = {'TAKEOFF': 0, 'FOLLOW': 1, 'MINCO_FOLLOW': 2, 'P7_GUIDANCE': 3}
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    axes[0, 0].plot([r['p'][0] for r in boat], [r['p'][1] for r in boat], label='USV')
    axes[0, 0].plot([r['p'][0] for r in flown], [r['p'][1] for r in flown], label='PX4 UAV')
    axes[0, 0].set(xlabel='North (m)', ylabel='East (m)')
    axes[0, 0].axis('equal')
    ft = [r['stamp']-takeoff for r in flown]
    axes[0, 1].plot(ft, [-r['p'][2] for r in flown], label='actual height')
    axes[0, 1].set(ylabel='Height (m)')
    axes[1, 0].plot(ft, [np.linalg.norm(r['v'][:2]) for r in flown], label='actual UAV speed')
    axes[1, 0].set(ylabel='Horizontal speed (m/s)')
    axes[1, 1].step([r['stamp']-takeoff for r in diag],
                    [codes.get(r['status'], -1) for r in diag], where='post', label='actual owner')
    axes[1, 1].set_yticks([-1, 0, 1, 2, 3], ['Other', 'Takeoff', 'FOLLOW', 'MINCO', 'P7'])
    for axis in (axes[0, 1], axes[1, 0], axes[1, 1]):
        axis.axvline(start-takeoff, color='black', linestyle=':', label='Y guidance')
        axis.set_xlabel('Time after X (s)')
    for axis in axes.flat:
        axis.grid(alpha=.25)
        axis.legend()
    fig.tight_layout()
    fig.savefig(output/'xy_mission.png', dpi=160)
    plt.close(fig)
    (output/'SHA256SUMS.json').write_text(json.dumps(
        {p.name: sha(p) for p in output.iterdir()}, indent=2)+'\n')


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
    parser.add_argument('--guidance', action='store_true')
    args = parser.parse_args()
    (analyze_guidance if args.guidance else analyze)(args.run.resolve(), args.output.resolve())
