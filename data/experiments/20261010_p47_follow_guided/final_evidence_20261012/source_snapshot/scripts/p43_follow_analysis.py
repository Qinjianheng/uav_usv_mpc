#!/usr/bin/env python3
"""Read-only P43 stage/ULog/response analysis; empirical errors never grant holding."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from p4_follow_analysis import analyze, read, distribution


def profile_summary(profiles):
    """Aggregate exclusive stage values without summing nested profiler cumulative totals."""
    stages, counts = defaultdict(list), Counter()
    calls = Counter()
    for report in profiles:
        if not report:
            continue
        for name, row in report['stages'].items():
            stages[name].append(row['seconds'])
            calls[name] += row['calls']
        counts.update(report.get('counts', {}))
    return dict(stages={k: dict(distribution(v), total=sum(v), calls=calls[k],
                                mean_per_call=sum(v)/calls[k]) for k, v in stages.items()},
                counts=dict(counts), reports=len(profiles))


def publication_summary(final):
    """Keep every attempted publication, including crossed deadlines, in explicit populations."""
    published = [r for r in final if r['published']]
    legal = [r for r in published if not r.get('publication_crossed_deadline', False)]
    return dict(proposals=len(published), legal_proposals=len(legal),
                crossed_deadline_proposals=len(published)-len(legal),
                legal_publication_fraction=len(legal)/len(final) if final else None,
                final_all_seconds=distribution([r['whole_cycle_time'] for r in final]),
                final_published_seconds=distribution([r['whole_cycle_time'] for r in published]),
                final_legal_seconds=distribution([r['whole_cycle_time'] for r in legal]),
                remaining_raw_ttl=distribution([r['remaining_raw_ttl'] for r in published]))


def publication_input_ttl(rows):
    """Evaluate the current input guard at recorded pre-publication time, without clock shifts."""
    requests = {r['request']['context']['cycle_id']: r['request']['context'] for r in rows
                if r['event'] == 'completion' and r.get('request')}
    revised = {r['cycle_id']: r['new_prediction'] for r in rows
               if r['event'] == 'prediction_revalidation' and r['report'].get('valid')}
    remaining = []
    for row in rows:
        if row['event'] != 'follow_proposal':
            continue
        cycle = row['identity'][-1]
        c = requests.get(cycle)
        if c is None:
            continue
        prediction = revised.get(cycle, dict(
            observation_stamp=c['observation_stamp'], source_stamp=c['prediction_source_stamp'],
            valid_until=c['prediction_valid_until']))
        deadline = min(c['navigation_stamp']+.125, c['attitude_stamp']+.125,
                       prediction['observation_stamp']+.125, prediction['source_stamp']+.125,
                       prediction['valid_until'], c['execution_start_stamp'])
        remaining.append(deadline-row['stamp'])
    return distribution(remaining)


def ulog_summary(root, native_window):
    """Use raw PX4 boot sample times and empirical quantiles; no timestamp offset fitting."""
    datasets = ('vehicle_local_position', 'vehicle_attitude', 'vehicle_angular_velocity',
                'vehicle_thrust_setpoint', 'actuator_motors', 'vehicle_status')
    output = []
    for path in (root/'ulogs').glob('*.ulg'):
        log = ULog(str(path), message_name_filter_list=datasets)
        report = dict(path=str(path), windows={}, parameters={
            k: v for k, v in log.initial_parameters.items() if k.startswith((
                'MC_YAW', 'MC_ROLLRATE', 'MC_PITCHRATE', 'MPC_ACC', 'MPC_TILT', 'MPC_THR',
                'COM_OF_LOSS', 'COM_OBL_RC', 'COM_RCL', 'CA_ROTOR'))})
        local = next((d.data for d in log.data_list if d.name == 'vehicle_local_position'), None)
        for window, bounds in (('all', None), ('follow_20_to_60_native', native_window)):
            metrics = {}
            for dataset in log.data_list:
                d, name = dataset.data, dataset.name
                t = np.asarray(d.get('timestamp_sample', d['timestamp']))/1e6
                mask = np.ones(len(t), bool) if bounds is None else (
                    (t >= bounds[0]) & (t <= bounds[1]))
                if bounds is None and window != 'all':
                    mask[:] = False
                if name == 'vehicle_angular_velocity':
                    for i, axis in enumerate('pqr'):
                        metrics[axis] = distribution(np.abs(d[f'xyz[{i}]'][mask]))
                elif name == 'vehicle_attitude':
                    q = np.column_stack([d[f'q[{i}]'] for i in range(4)])
                    w, x, y, z = q.T
                    yaw = np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
                    tilt = np.arccos(np.clip(1-2*(x*x+y*y), -1., 1.))
                    metrics['tilt_rad'] = distribution(tilt[mask])
                    if local is not None and all(k in local for k in ('ax', 'ay', 'az')):
                        lt = np.asarray(local.get('timestamp_sample', local['timestamp']))/1e6
                        index = np.searchsorted(lt, t, side='right')-1
                        index = np.clip(index, 0, len(lt)-1)
                        paired = mask & (t >= lt[index]) & (t-lt[index] <= .02)
                        acceleration = np.column_stack(
                            [local[k][index] for k in ('ax', 'ay', 'az')])
                        force = np.array((0., 0., 9.80665))-acceleration
                        norm = np.linalg.norm(force, axis=1)
                        paired &= np.isfinite(norm) & (norm > 1e-6)
                        body_down = np.c_[2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]
                        dot = np.sum(body_down[paired]*force[paired], axis=1)/norm[paired]
                        metrics['empirical_flatness_thrust_axis_error_rad'] = distribution(
                            np.arccos(np.clip(dot, -1., 1.)))

                    dt = np.diff(t)
                    valid = mask[1:] & mask[:-1] & (dt > 0) & (dt < .1)
                    rate = np.abs(np.diff(yaw)[valid]/dt[valid])
                    metrics['measured_yaw_rate'] = distribution(rate)
                elif name == 'vehicle_thrust_setpoint':
                    metrics['normalized_thrust_setpoint_z'] = distribution(
                        np.abs(d['xyz[2]'][mask]))
                elif name == 'actuator_motors':
                    metrics['motor_controls'] = distribution(np.concatenate([
                        d[f'control[{i}]'][mask] for i in range(4)]))
                elif name == 'vehicle_status':
                    metrics['offboard_fraction'] = float(np.mean(d['nav_state'][mask] == 14)) if (
                        mask.any()) else None
            report['windows'][window] = metrics
        output.append(report)
    return output


def response_study(raw):
    """Fit/test a causal first-order velocity response proxy, never a guaranteed bridge model."""
    nav = [r for r in raw if r['kind'] == 'uav' and r['valid']]
    commands = [r for r in raw if r['kind'] == 'reference' and np.isfinite(r['v']).all()]
    if not nav or not commands:
        return dict(qualified=False, reason='NO_VELOCITY_SAMPLES')
    times = np.array([r['receipt'] for r in commands])
    errors, measured, rows = [], [], []
    for r in nav:
        i = np.searchsorted(times, r['receipt'], side='right')-1
        if i < 0 or not 0 <= r['receipt']-times[i] <= .125:
            continue
        command = commands[i]
        delta = np.asarray(command['v'])-r['v']
        errors.append(delta[:2])
        measured.append(r['a'][:2])
        rows.append((r, command))
    if len(errors) < 20:
        return dict(qualified=False, reason='INSUFFICIENT_CAUSAL_SAMPLES')
    e, a = np.asarray(errors), np.asarray(measured)
    split = len(e)//2
    gain = float(np.sum(e[:split]*a[:split])/max(np.sum(e[:split]**2), 1e-12))
    residual = np.linalg.norm(a[split:]-gain*e[split:], axis=1)
    return dict(qualified=False, actual_minco_bridge=False, finite_contingency_proven=False,
                model='a_xy = gain * (latest received v_command_xy - v_measured_xy)',
                source='original FOLLOW only; causal packet receipt matching <=125ms',
                samples=len(rows), fit_count=split, test_count=len(e)-split,
                gain_per_second=gain, tau_seconds=1/gain if gain > 0 else None,
                heldout_acceleration_residual=distribution(residual),
                velocity_command_error=distribution(np.linalg.norm(e, axis=1)),
                evidence_kind='empirical quantiles and sample maxima, not physical bounds')


def analyze_p43(root):
    """Write only this new experiment's derived reports; retain source and shutdown failures."""
    report = analyze(root)
    raw, malformed = read(root/'offline_evidence.jsonl')
    rows = [json.loads(line) for path in (root/'shadow').glob('mpc_seed_shadow*.jsonl')
            for line in path.open()]
    final = [r for r in rows if r['event'] == 'follow_publication_final']
    complete = [r for r in rows if r['event'] == 'completion']
    profiles = [r.get(key) for r in final for key in
                ('preparation_profile', 'worker_profile', 'publication_profile')]
    prediction_profiles = []
    logpath = root/'UAV-USV_experiment.txt'
    if logpath.exists():
        for line in logpath.open(errors='replace'):
            if 'P43_PREDICTOR_PROFILE ' in line:
                text = line.split('P43_PREDICTOR_PROFILE ', 1)[1]
                prediction_profiles.append(json.JSONDecoder().raw_decode(text)[0])
    follow = [r for r in raw if r['kind'] == 'diagnostic' and r['status'] == 'FOLLOW']
    start = follow[0]['stamp'] if follow else None
    stable = [r for r in raw if r['kind'] == 'uav' and start is not None
              and 20 <= r['stamp']-start <= 60]
    native = (min(r['native'] for r in stable)/1e6, max(r['native'] for r in stable)/1e6) if (
        stable) else None
    report.update(
        condition=json.loads((root/'condition.json').read_text()),
        malformed_lines=malformed, cycles=len(complete), final_cycles=len(final),
        monitor_shutdown_clean=report['end']['monitor_returncode'] == 0,
        planner_runtime_error=logpath.exists() and (
            "KeyError: 'metrics'" in logpath.read_text(errors='replace')),
        **publication_summary(final),
        current_input_remaining_at_prepublication=publication_input_ttl(rows),
        exclusive_profile=profile_summary([r for r in profiles if r]),
        producer_profile=profile_summary(prediction_profiles),
        revalidation=distribution([r['report']['elapsed'] for r in rows if (
            r['event'] == 'prediction_revalidation' and 'elapsed' in r['report'])]),
        ack_compute=distribution([r['receiver_compute_seconds'] for r in raw if (
            r['kind'] == 'follow_ack')]),
        rtf=distribution([r['rtf'] for r in raw if r['kind'] == 'gazebo_stats']),
        cycle_statuses=dict(Counter(r['output']['solver_status'] for r in complete)),
        actual_minco_performance='未执行', safety_qualification=False,
        ulog=ulog_summary(root, native), response_model=response_study(raw))
    (root/'p43_analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('flights', type=Path)
    args = parser.parse_args()
    reports = {}
    for root in sorted(args.flights.iterdir()):
        if not (root/'watchdog_end.json').exists() or not (root/'offline_evidence.jsonl').exists():
            continue
        reports[root.name] = analyze_p43(root)
        print(root.name, reports[root.name]['proposals'],
              reports[root.name]['final_published_seconds'], flush=True)
    (args.flights.parent/(args.flights.name+'_comparison.json')).write_text(
        json.dumps(reports, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
