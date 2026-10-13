#!/usr/bin/env python3
"""Read sealed actual F2 evidence without changing its timing, windows or records."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from p4_follow_analysis import distribution
from p5_release import sha, verify_manifest


def full_observation(rows, complete, first):
    """Use the fixed 25s physical window, bounded truth interpolation and full-pose FOV."""
    from p53_saved_request_replay import restore_actual
    from p4_follow_analysis import rendered_marker_visibility
    from uav_control.guidance.camera_visibility import body_frd_to_ned_from_quaternion
    if first is None:
        return None
    _, model = restore_actual(next(iter(complete.values())))
    truth = [r for r in rows if r['kind'] == 'ideal' and r['valid']]
    ts = np.asarray([r['stamp'] for r in truth])
    states = [r for r in rows if r['kind'] == 'uav' and r['valid']
              and first <= r['stamp'] <= first+25. and ts[0] <= r['stamp'] <= ts[-1]]
    xy, vz, altitude = [], [], []
    for row in states:
        target = np.array([np.interp(row['stamp'], ts, [t['p'][i] for t in truth])
                           for i in (0, 1)])
        velocity = np.array([np.interp(row['stamp'], ts, [t['v'][i] for t in truth])
                             for i in (0, 1)])
        desired = target-5*velocity/max(np.linalg.norm(velocity), 1e-9)
        xy.append(np.linalg.norm(np.array(row['p'][:2])-desired))
        vz.append(np.linalg.norm(np.array(row['v'][:2])-velocity))
        altitude.append(row['p'][2]+5.)
    clocks = [r for r in rows if r['kind'] == 'clock']
    times, epochs = np.array([r['sim'] for r in clocks]), [r['system'] for r in clocks]
    boats = {r['sim']: r for r in rows if r['kind'] == 'rendered_target'}
    views = []
    for row in rows:
        if row['kind'] != 'rendered_uav' or row['sim'] not in boats:
            continue
        if not times[0] <= row['sim'] <= times[-1]:
            continue
        epoch = float(np.interp(row['sim'], times, epochs))
        if first <= epoch <= first+25.:
            rotation = (np.array(((0., 1., 0.), (1., 0., 0.), (0., 0., -1.)))
                        @ body_frd_to_ned_from_quaternion(row['q']) @ np.diag((1., -1., -1.)))
            views.append(rendered_marker_visibility(row['p'], rotation,
                                                    boats[row['sim']]['p'], model))
    debug = [r for r in rows if r['kind'] == 'execution_debug' and r['plan']
             and first <= r['stamp'] <= first+25. and r['state_stamp'] >= r['active_start']]
    tracking = [np.linalg.norm(np.array(r['actual_position'])-np.array(r['reference'][:3]))
                for r in debug]

    def rmse(values):
        return float(np.sqrt(np.mean(np.square(values)))) if values else None

    return dict(start=first, end=first+25., samples=len(states), horizontal_rmse=rmse(xy),
                height_rmse=rmse(altitude), relative_velocity_rmse=rmse(vz),
                actual_reference_tracking_3d_rmse=rmse(tracking), reference_samples=len(tracking),
                fov_samples=len(views), full_marker_safe_fraction=(
                    float(np.mean([v.whole_target_safe for v in views])) if views else None),
                horizontal_margin=distribution([v.horizontal_margin_rad for v in views]),
                vertical_margin=distribution([v.vertical_margin_rad for v in views]),
                evaluation_only=True, no_extrapolation=True,
                scope='All owners over full fixed window. Truth interpolated only offline. '
                'FOV uses same-message rendered UAV/marker poses and recorded camera calibration.')


def interval_distribution(epochs):
    """Keep distinct physical event epochs; report how many intervals exist."""
    return distribution(np.diff(sorted(set(epochs))))


def attempt_blocker(completion, final, acknowledgements):
    """Find the first observed failure; missing records never prove an upstream cause."""
    if any(a['state'] == 'ACTIVE' for a in acknowledgements):
        return 'ACTIVE'
    if completion is None:
        return 'NO_RECORDED_REQUEST'
    if not final or not final.get('published'):
        value = completion.get('candidate_output') or completion['output']
        if not value.get('valid'):
            return value.get('solver_status') or completion['output'].get(
                'solver_status', 'UNKNOWN_CANDIDATE_RESULT')
        return (final or {}).get('reason', 'NO_RECORDED_PUBLICATION')
    failed = [a for a in acknowledgements if a['state'] in ('REJECTED', 'REVOKED', 'EXPIRED')]
    if failed:
        return ','.join(failed[-1]['reasons']) or failed[-1]['state']
    return 'NO_RECORDED_ACTIVE' if acknowledgements else 'NO_RECORDED_RECEIVER_ACK'


def analyze(root, output):
    """Derive populations and per-loss supply chains from immutable source records."""
    verify_manifest(root)
    output.mkdir(parents=True, exist_ok=False)
    rows = [json.loads(line) for line in (root/'offline_evidence.jsonl').open()]
    events = [json.loads(line) for path in (root/'shadow').glob('mpc_seed_shadow_*.jsonl')
              for line in path.open()]
    summary = json.loads((root/'p5_run_summary.json').read_text())
    condition = json.loads((root/'condition.json').read_text())
    stop = json.loads((root/'bounded_stop.json').read_text())
    recorded = json.loads((root/'execution_analysis.json').read_text())
    audit = summary['p51_audit']
    complete = {r['request']['context']['cycle_id']: r for r in events
                if r.get('event') == 'completion' and r.get('request')}
    finals = {r['cycle_id']: r for r in events if r.get('event') == 'follow_publication_final'}
    proposals = {r['plan']: r for r in rows if r['kind'] == 'proposal'}
    acks = [r for r in rows if r['kind'] == 'follow_ack']
    receives = [r for r in rows if r['kind'] == 'follow_receive']
    active = {r['plan']: r for r in acks if r['state'] == 'ACTIVE'}
    accepted = {r['plan']: r for r in acks if r['state'] == 'ACCEPTED'}
    timeline = [dict(event='FOLLOW_START', stamp=stop['follow_start']),
                dict(event='FIVE_SECOND_GATE_COMPLETE', stamp=condition['planner_started_wall'])]
    timeline.extend(dict(event=r['state'], stamp=r['stamp'], plan=r['plan'],
                         active=r['active'], pending=r['pending'], owner=r['owner'],
                         reasons=r['reasons']) for r in acks)
    previous = None
    for r in rows:
        if r['kind'] != 'receiver_epoch':
            continue
        state = (r['research_state'], r['entry_allowed'], r['converged'])
        if state != previous:
            timeline.append(dict(event='ADMISSION', stamp=r['stamp'], state=state,
                                 attempts=r['entry_attempts'], probes=r['valid_probe_count']))
            previous = state
    timeline.append(dict(event='STOP', stamp=stop['stamp'], reason=stop['reason']))
    timeline.sort(key=lambda r: r['stamp'])
    losses = []
    for loss in audit['fallbacks']:
        plan = loss['plan']
        parent = proposals[plan]
        attempts = []
        for cycle, event in complete.items():
            prior = event['request'].get('prior_curve') or {}
            if prior.get('plan_id') != plan:
                continue
            observations = [a for a in acks if a['plan'] == cycle]
            final = finals.get(cycle)
            attempts.append(dict(cycle=cycle, dispatch=event['request']['now_stamp'],
                                 raw_input_observation=event['request']['context'][
                                     'observation_stamp'],
                                 worker=event['candidate_output']['metrics'].get(
                                     'worker_timeline'),
                                 result=event['output']['solver_status'],
                                 candidate_result=event['candidate_output'].get('solver_status'),
                                 first_blocker=attempt_blocker(event, final, observations),
                                 final=final, acknowledgements=observations,
                                 retry=event.get('deadline_retry')))
        renewed = next((a for a in acks if a['state'] == 'ACTIVE'
                        and a['stamp'] > loss['stamp']
                        and proposals.get(a['plan'], {}).get('parent') == 0), None)
        losses.append(dict(**loss, original_lease_deadline=min(parent['end'], parent['start']+.45),
                           active_ack=active.get(plan), attempts=attempts,
                           next_initial_active=renewed,
                           recovery_seconds=renewed['stamp']-loss['stamp'] if renewed else None,
                           no_request_causality='Missing dispatch events remain unknown'))
    first = summary['first_minco_receipt']
    end = first+25. if first else None
    successors = [r for r in proposals.values() if r['parent'] and r['start']+.05 < stop['stamp']]
    metrics = dict(
        source=str(root), evidence_sha256=sha(root/'offline_evidence.jsonl'),
        source_manifest_sha256=sha(root/'SHA256SUMS'),
        predefined_observation=dict(start=first, end=end, seconds=summary['observation_seconds'],
                                    minco_fraction=summary['minco_fraction'],
                                    longest_minco_seconds=summary['longest_minco_seconds'],
                                    diagnostic_coverage=summary['diagnostic_coverage'],
                                    recovery_time_included=True, passed=summary['f3_eligible']),
        follow_to_five_second_gate=condition['planner_started_wall']-stop['follow_start'],
        planner_to_first_active=(recorded['lifecycle']['first_active']
                                 - condition['planner_started_wall']),
        actual_active_plans_all=len(active), actual_active_plans_observation=audit['active_plans'],
        published_successor_cohort=dict(
            matured=len(successors),
            activated=sum(r['plan'] in active for r in successors),
            meaning='Published successor cohort only; not all requests or lease opportunities'),
        timing=dict(
            observed_request_intervals=interval_distribution(
                [r['request']['now_stamp'] for r in complete.values()]),
            prediction_generation_intervals=interval_distribution(
                [r['stamp'] for r in rows if r['kind'] == 'prediction' and r['valid']]),
            prediction_observation_intervals=interval_distribution(
                [r['observation'] for r in rows if r['kind'] == 'prediction' and r['valid']]),
            wire_publication_intervals=interval_distribution(
                [r['stamp'] for r in proposals.values()]),
            whole_cycle_final=distribution([r['whole_cycle_time'] for r in finals.values()]),
            solver=recorded['research']['solver'], validation=recorded['research']['validation'],
            receiver_validation=recorded['lifecycle']['receiver_service_seconds'],
            publish_to_accepted=distribution([
                a['stamp']-proposals[p]['stamp']
                for p, a in accepted.items() if p in proposals]),
            publish_to_active=distribution([
                a['stamp']-proposals[p]['stamp']
                for p, a in active.items() if p in proposals]),
            activation_delay=recorded['lifecycle']['activation_deviation_seconds'],
            receiver_finish_raw_margin=distribution([
                proposals[r['plan']]['input_until']-r['stamp']
                for r in receives if r['plan'] in proposals]),
            successor_start_before_parent_expiry=distribution([
                min(proposals[r['parent']]['end'], proposals[r['parent']]['start']+.45)-r['start']
                for r in successors if r['parent'] in proposals])),
        publication_reasons=dict(Counter(r['reason'] or 'PUBLISHED' for r in finals.values())),
        start_missed=sum('START_MISSED' in r['reasons'] for r in acks),
        losses=losses, reference_height=audit['reference_height'],
        tracking=recorded['active_only_decomposition'],
        actual_window_scope='Existing FOLLOW+20 to FOLLOW+60 clipped by stop; mixed actual owners',
        actual_window=recorded['windows']['follow_20_to_60'],
        full_25s_metrics=full_observation(rows, complete, first),
        native_px4_commands=recorded['native_command'], physical_ulog=recorded['physical'],
        gate_violations=audit['violations'], authority=recorded['authority'],
        cleanup=json.loads((root/'p5_cleanup.json').read_text()),
        missing_fields=['Tracker subscription dequeue order', 'every request dispatch event',
                        'first settled health epoch; only completed five-second gate recorded'])
    (output/'analysis.json').write_text(json.dumps(metrics, indent=2, ensure_ascii=False)+'\n')
    (output/'timeline.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in timeline))
    with (output/'timeline.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=['stamp', 'event', 'details'])
        writer.writeheader()
        writer.writerows(dict(stamp=r['stamp'], event=r['event'], details=json.dumps(r))
                         for r in timeline)
    return metrics


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = analyze(args.root.resolve(), args.output.resolve())
    print(json.dumps(result['predefined_observation']))
