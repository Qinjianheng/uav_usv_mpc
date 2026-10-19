#!/usr/bin/env python3
"""Offline radius-capture and frozen-world checks; never part of flight feedback."""

import argparse
import bisect
import json
from pathlib import Path

import numpy as np

from p4_follow_analysis import read
from p5_minco_sitl import sha
from p7_follow_analysis import guidance_metrics


def native_reference_distances(rows, start, end, marker_height):
    """Same native pose epoch, mapped only inside <=50 ms server clock brackets."""
    first_clocks = {}
    for row in rows:
        if row['kind'] == 'clock':
            first_clocks.setdefault(row['sim'], row)
    clocks = sorted(first_clocks.values(), key=lambda r: r['sim'])
    times = [r['sim'] for r in clocks]
    boats = {r['sim']: r for r in rows if r['kind'] == 'rendered_target'}
    result = []
    for uav in (r for r in rows if r['kind'] == 'rendered_uav' and r['sim'] in boats):
        i = bisect.bisect_left(times, uav['sim'])
        if i < len(times) and abs(times[i]-uav['sim']) <= 1e-9:
            stamp = clocks[i]['system']
        elif 0 < i < len(times):
            a, b = clocks[i-1:i+1]
            if not (0 < b['sim']-a['sim'] <= .05+1e-9
                    and 0 < b['system']-a['system'] <= .05+1e-9):
                continue
            stamp = a['system']+(uav['sim']-a['sim'])/(b['sim']-a['sim'])*(b['system']-a['system'])
        else:
            continue
        if not start <= stamp <= end:
            continue
        target = np.array(boats[uav['sim']]['p'])
        target[2] += marker_height
        result.append((stamp, uav['sim'], float(np.linalg.norm(np.array(uav['p'])-target)),
                       -uav['p'][2]))
    return result


def freeze_metrics(rows, result_receipt):
    """Require repeated independent world/target/native-pose observations after capture."""
    selected = [r for r in rows if r['receipt'] >= result_receipt+.5]
    stats = [r for r in selected if r['kind'] == 'gazebo_stats' and r['paused']]
    boats = [r for r in selected if r['kind'] == 'ideal']
    poses = {}
    for r in selected:
        if r['kind'] == 'paused_native_pose':
            poses.setdefault(r['name'], []).append(r)

    def span(records):
        return records[-1]['receipt']-records[0]['receipt'] if len(records) >= 2 else 0.

    def spread(records, field):
        return float(np.max(np.ptp([r[field] for r in records], axis=0))) if records else None

    clock = spread(stats, 'sim')
    boat = spread(boats, 'p')
    zero_velocity = bool(boats) and all(np.linalg.norm(r['v']) <= 1e-9 for r in boats)
    pose_spread = {name: spread(records, 'p') for name, records in poses.items()}
    latest_native = {}
    for r in rows:
        if r['kind'] in ('rendered_uav', 'rendered_target') and r['receipt'] <= result_receipt+.5:
            latest_native[r['name']] = r['p']
    pose_residual = {name: float(np.linalg.norm(np.subtract(records[0]['p'], latest_native[name])))
                     for name, records in poses.items() if name in latest_native}
    native_ok = ('usv_target' in poses and any(n.startswith('x500_mono_cam_') for n in poses)
                 and all(span(p) >= 1. and spread(p, 'p') <= 1e-6 for p in poses.values())
                 and len(pose_residual) == len(poses)
                 and all(v <= .1 for v in pose_residual.values()))
    world_verified = bool(span(stats) >= 1. and clock is not None and clock <= 1e-9
                          and span(boats) >= 1. and boat is not None and boat <= 1e-9
                          and zero_velocity)
    return dict(verified=world_verified and native_ok, world_verified=world_verified,
                native_pose_verified=bool(native_ok),
                pause_samples=len(stats), paused_seconds=span(stats),
                clock_span_s=clock, target_position_span_m=boat,
                target_velocity_zero=zero_velocity, native_pose_span_m=pose_spread,
                native_pose_continuity_error_m=pose_residual)


def capture_metrics(run, reason):
    """Keep ownership/interface checks, substitute the explicitly requested capture envelope."""
    rows, malformed = read(run/'offline_evidence.jsonl')
    results = [r for r in rows if r['kind'] == 'capture_result']
    result = results[-1] if results else None
    summary = guidance_metrics(run, reason, stop=result['stamp'] if result else None)
    frozen = (freeze_metrics(rows, result['receipt']) if result else
              dict(verified=False, world_verified=False, native_pose_verified=False))
    start = summary.get('guidance_start', float('inf'))
    end = result['stamp'] if result else summary.get(
        'physical_window_end', json.loads((run/'bounded_stop.json').read_text())['stamp'])
    native = [r for r in rows if r['kind'] == 'uav' and r['valid']
              and start <= r['stamp'] <= end]
    height_min = min((-r['p'][2] for r in native), default=None)
    maximum_vz = max((abs(r['v'][2]) for r in native), default=None)
    takeoff = next((r['receipt'] for r in rows if r['kind'] == 'mission'
                    and r['name'] == 'TAKEOFF'), start)
    motion = [r for r in rows if r['kind'] == 'ideal' and r['valid']
              and takeoff+8. <= r['stamp'] <= end]
    target_speeds = [float(np.linalg.norm(r['v'][:2])) for r in motion]
    minco = [r['stamp'] for r in rows if r['kind'] == 'diagnostic'
             and r['status'] == 'MINCO_FOLLOW' and r['stamp'] < start]
    complete = bool(reason == 'P7_CAPTURE_COMPLETE' and result and result['success'])
    healthy = bool(complete and 'guidance_start' in summary
                   and not malformed and result['radius'] == .5
                   and result['minimum_distance'] <= .5 and frozen['world_verified']
                   and summary['single_tracker'] and summary['single_bctra']
                   and summary['velocity_only_interface']
                   and not summary['retired_authority_events']
                   and summary['actual_speed_max_mps'] <= 7.
                   and summary['actual_acceleration_max_mps2'] <= 4.
                   and summary['visible_fraction'] >= .95
                   and height_min is not None and height_min >= .23
                   and maximum_vz is not None and maximum_vz <= 4.)
    summary.update(completed=complete, healthy=healthy, capture_result=result, freeze=frozen,
                   minimum_height_m=height_min, maximum_vertical_speed_mps=maximum_vz,
                   minco_before_y_seconds=max(minco)-min(minco) if minco else 0.,
                   observed_target_speed_min_mps=min(target_speeds, default=None),
                   observed_target_speed_max_mps=max(target_speeds, default=None),
                   motion_mode=json.loads((run/'condition.json').read_text())['motion_mode'],
                   capture_window_end=end)
    for name in ('initial_distance_m', 'final_distance_m', 'minimum_distance_m',
                 'final_2s_median_distance_m'):
        if name in summary:
            summary[name.replace('distance', 'horizontal_distance')] = summary.pop(name)
    return summary


def analyze(run, output):
    """Plot measured NED states against interpolated evaluation truth inside its coverage."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import yaml
    recorded = json.loads((run/'summary.json').read_text())
    summary = capture_metrics(run, recorded['reason'])
    summary.update(recorded_run_healthy=recorded['healthy'],
                   analysis_source_sha256=sha(Path(__file__)),
                   source_summary_sha256=sha(run/'summary.json'),
                   source_evidence_sha256=sha(run/'offline_evidence.jsonl'),
                   qualification_scope='Capture, physical limits and paused world/target. '
                   'Repeated native pose verification is reported separately.')
    output.mkdir(exist_ok=False)
    (output/'metrics.json').write_text(json.dumps(summary, indent=2)+'\n')
    if 'guidance_start' not in summary:
        (output/'SHA256SUMS.json').write_text(json.dumps(
            {p.name: sha(p) for p in output.iterdir()}, indent=2)+'\n')
        return
    rows, _ = read(run/'offline_evidence.jsonl')
    start = summary['guidance_start']
    end = summary['capture_window_end']
    targets = [r for r in rows if r['kind'] == 'ideal' and r['valid']]
    tt = np.array([r['stamp'] for r in targets])
    tp = np.array([r['p'] for r in targets])
    native = [r for r in rows if r['kind'] == 'uav' and r['valid']
              and max(start, tt[0]) <= r['stamp'] <= min(end, tt[-1])]
    nt = np.array([r['stamp'] for r in native])
    p = np.array([r['p'] for r in native])
    q = np.column_stack([np.interp(nt, tt, tp[:, i]) for i in range(3)])
    distance = np.linalg.norm(p-q, axis=1)
    guidance = [r for r in rows if r['kind'] == 'p7_guidance' and start <= r['stamp'] <= end]
    config = yaml.safe_load((run/'config/flight.yaml').read_text())
    native_geometry = native_reference_distances(
        rows, start, end,
        config['moving_target']['ros__parameters']['gazebo_visual_height_offset'])
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    axes[0, 0].plot(nt-start, distance, label='PX4 estimate to USV reference')
    if native_geometry:
        actual = np.array(native_geometry)
        axes[0, 0].plot(actual[:, 0]-start, actual[:, 2], label='Gazebo model references')
        axes[0, 1].plot(actual[:, 0]-start, actual[:, 3], label='Gazebo model height')
        np.savetxt(output/'native_distance.csv', actual, delimiter=',',
                   header='ros_epoch,native_sim_seconds,distance_3d_m,model_height_m', comments='')
    axes[0, 0].axhline(.5, color='gray', linestyle='--', label='capture radius')
    axes[0, 0].set(ylabel='Distance (m)')
    axes[0, 1].plot(nt-start, -p[:, 2], label='PX4 estimated height')
    axes[0, 1].axhline(.33, color='gray', linestyle='--', label='terminal clearance request')
    axes[0, 1].set(ylabel='Height (m)')
    axes[1, 0].plot(q[:, 0], q[:, 1], label='USV evaluation truth')
    axes[1, 0].plot(p[:, 0], p[:, 1], label='PX4 UAV')
    axes[1, 0].axis('equal')
    axes[1, 0].set(xlabel='North (m)', ylabel='East (m)')
    axes[1, 1].plot([r['stamp']-start for r in guidance],
                    [r['error_u'] for r in guidance], label='horizontal image error')
    axes[1, 1].plot([r['stamp']-start for r in guidance],
                    [r['error_v'] for r in guidance], label='vertical image error')
    axes[1, 1].set(ylabel='Normalized error')
    for i, ax in enumerate(axes.flat):
        if i != 2:
            ax.set_xlabel('Time after Y (s)')
        ax.grid(alpha=.25)
        ax.legend()
    fig.tight_layout()
    fig.savefig(output/'capture.png', dpi=160)
    plt.close(fig)
    np.savetxt(output/'distance.csv', np.column_stack((nt-start, distance, -p[:, 2])),
               delimiter=',', header='seconds_after_Y,distance_3d_m,height_m', comments='')
    (output/'SHA256SUMS.json').write_text(json.dumps(
        {p.name: sha(p) for p in output.iterdir()}, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    analyze(args.run.resolve(), args.output.resolve())
