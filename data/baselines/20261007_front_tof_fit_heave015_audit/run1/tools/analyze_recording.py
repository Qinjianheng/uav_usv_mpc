"""Recompute task-cutoff accuracy and recovery evidence from archived records."""

import argparse
import collections
import csv
import json
import math
from pathlib import Path

import numpy as np


def seconds(stamp):
    return stamp['sec'] + stamp['nanosec'] * 1e-9


def stats(rows, field):
    values = np.array([float(row[field]) for row in rows
                       if row[field] and math.isfinite(float(row[field]))])
    return {
        'n': len(values),
        'rmse_m': float(np.sqrt(np.mean(values ** 2))) if len(values) else None,
        'p95_abs_m': float(np.quantile(abs(values), .95)) if len(values) else None,
        'max_abs_m': float(max(abs(values))) if len(values) else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('output_dir', type=Path)
    args = parser.parse_args()
    root, output = args.run_dir, args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    result = json.loads((root / 'probe_summary.json').read_text())['result']
    end = seconds(result['stamp'])
    start = end - result['elapsed_time']
    vision = list(csv.DictReader(next((root / 'raw').glob('*vision.csv')).open()))
    pre = [row for row in vision
           if row['receipt_stamp'] and float(row['receipt_stamp']) <= end]
    valid = [row for row in pre if row['valid'] == 'True'
             and row['truth_available'] == 'True'
             and 0 < float(row['measurement_stamp']) <= end
             and math.isfinite(float(row['position_3d_error']))]
    analysis = {
        'result_cutoff_ros_s': end,
        'selection': ('receipt <= result; valid accuracy also requires 0 < '
                      'acquisition <= result, aligned truth and finite error; '
                      'no post-timeout metrics'),
        'raw_rows': len(vision), 'rows_before_result': len(pre),
        'valid_accuracy_samples': len(valid),
        'rejections_before_result': dict(collections.Counter(
            row['rejection_reason'] for row in pre)),
        'by_phase': {},
    }
    for phase in sorted({row['approach_phase'] for row in pre}):
        subset = [row for row in valid if row['approach_phase'] == phase]
        analysis['by_phase'][phase] = {
            field: stats(subset, field) for field in (
                'position_3d_error', 'error_x', 'error_y', 'error_z',
                'camera_center_error_3d')}
    probe = [json.loads(line) for line in (root / 'probe.jsonl').open()]
    phases = [row['data'] for row in probe if row['kind'] == 'phase']
    follow = next(seconds(row['stamp']) for row in phases
                  if row['state_name'] == 'FOLLOW') + .5
    far = next(seconds(row['stamp']) for row in phases
               if row['state_name'] == 'FAR_GUIDANCE')
    subset = [row for row in valid
              if follow <= float(row['measurement_stamp']) < far]
    follow_stats = stats(subset, 'position_3d_error')
    analysis['stable_follow_window'] = {
        'begin_ros_s': follow, 'end_ros_s': far, 'n': follow_stats['n'],
        'rmse_m': follow_stats['rmse_m'], 'p95_m': follow_stats['p95_abs_m'],
        'max_m': follow_stats['max_abs_m'],
    }
    main_rows = list(csv.DictReader(next(
        (root / 'raw').glob('*mission_1.csv')).open()))
    active = [row for row in main_rows
              if row['intercept_started'] == 'True'
              and row['intercept_elapsed_time']
              and 0 <= float(row['intercept_elapsed_time']) <= result['elapsed_time']]
    nearest = min((row for row in active if row['distance']
                   and math.isfinite(float(row['distance']))),
                  key=lambda row: float(row['distance']))
    analysis['nearest_during_task'] = {key: nearest[key] for key in (
        'time', 'intercept_elapsed_time', 'phase', 'distance',
        'horizontal_distance', 'vertical_error', 'target_locked',
        'kf_state_age', 'planner_failure_reason', 'sea_safety_state',
        'body_clearance')}
    (output / 'analysis.json').write_text(json.dumps(analysis, indent=2) + '\n')
    fields = ('intercept_elapsed_time', 'phase', 'controller_status',
              'target_visible', 'target_locked', 'last_valid_observation_age',
              'kf_state_age', 'prediction_age', 'prediction_sample_age',
              'planner_failure_reason', 'remaining_t_go', 'distance',
              'horizontal_distance', 'vertical_error', 'sea_safety_state')
    selected = []
    for row in active:
        t = float(row['intercept_elapsed_time'])
        if 8.5 <= t <= 14.5 or 19.5 <= t <= 24.5:
            record = {key: row[key] for key in fields}
            record['horizontal_speed_mps'] = math.hypot(
                float(row['uav_vx']), float(row['uav_vy']))
            record['vz_ned_mps'] = float(row['uav_vz'])
            selected.append(record)
    with (output / 'deceleration_trace.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected[0]))
        writer.writeheader()
        writer.writerows(selected)
    invalid_near = [row for row in pre
                    if 11.4 <= float(row['receipt_stamp']) - start <= 12.2]
    (output / 'near_depth_trace.json').write_text(json.dumps([
        {key: row[key] for key in (
            'receipt_stamp', 'measurement_stamp', 'valid', 'rejection_reason',
            'valid_depth_ratio', 'depth_min', 'depth_median', 'target_range',
            'center_camera_x', 'center_camera_y', 'center_camera_z')}
        for row in invalid_near], indent=2) + '\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    window = [row for row in active
              if 8.5 <= float(row['intercept_elapsed_time']) <= 14.5]
    times = [float(row['intercept_elapsed_time']) for row in window]
    recovery = next(float(row['intercept_elapsed_time']) for row in window
                    if row['phase'] == 'SAFE_RECOVERY')
    plan = next(float(row['intercept_elapsed_time']) for row in window
                if row['phase'] == 'MINCO_READY')
    fig, axes = plt.subplots(3, 1, figsize=(10, 7.2), sharex=True)
    axes[0].plot(times, [math.hypot(float(row['uav_vx']), float(row['uav_vy']))
                        for row in window], color='#1565c0', lw=2)
    axes[0].set_ylabel('Measured speed (m/s)')
    axes[0].set_title('First approach: preparation slowing, then recovery braking')
    distance_rows = [row for row in window
                     if row['distance'] and math.isfinite(float(row['distance']))]
    distance_times = [float(row['intercept_elapsed_time']) for row in distance_rows]
    axes[1].plot(distance_times, [float(row['distance']) for row in distance_rows],
                 label='3-D distance', color='#1565c0', lw=2)
    axes[1].plot(distance_times, [float(row['vertical_error']) for row in distance_rows],
                 label='Vertical separation', color='#7b1fa2', lw=1.5)
    axes[1].axhline(result['capture_radius'], color='#b71c1c', ls='--',
                   label='Capture radius (0.5 m)')
    axes[1].set_ylabel('Separation (m)')
    axes[1].legend(loc='upper left', fontsize=9)
    axes[2].step(times, [float(row['last_valid_observation_age']) for row in window],
                 where='post', color='#00695c', lw=2)
    axes[2].axhline(.125, color='#b71c1c', ls='--', label='Freshness gate (125 ms)')
    axes[2].set_ylim(0, .8)
    axes[2].set_ylabel('Last observation age (s)')
    axes[2].set_xlabel('Time after interception starts (s)')
    axes[2].legend(loc='upper left', fontsize=9)
    for ax in axes:
        ax.axvspan(8.5, plan, color='#90caf9', alpha=.18)
        ax.axvspan(recovery, 14.5, color='#ffb74d', alpha=.22)
        ax.axvline(recovery, color='#ef6c00', ls=':', lw=1.5)
        ax.grid(alpha=.2)
        ax.set_xlim(8.5, 14.5)
    axes[0].text(8.62, 1.0, 'FAR_GUIDANCE', fontsize=9)
    axes[0].text(12.08, 1.0, 'SAFE_RECOVERY', fontsize=9)
    fig.tight_layout()
    fig.savefig(output / 'first_approach_deceleration.png', dpi=170)
    plt.close(fig)
    print(json.dumps({'valid_accuracy_samples': len(valid),
                      'terminal_samples': analysis['by_phase']['TERMINAL_APPROACH']
                      ['position_3d_error']['n'], 'first_recovery_s': recovery}))


if __name__ == '__main__':
    main()
