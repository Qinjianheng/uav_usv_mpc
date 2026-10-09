"""Audit trajectory handovers from diagnostics; truth is evaluation only."""
import argparse
import json
import gzip
from pathlib import Path


def stamp(value):
    return value['sec'] + value['nanosec'] * 1e-9


def audit(run, output):
    output.mkdir(parents=True, exist_ok=True)
    probe = run / 'probe.jsonl'
    stream = probe.open() if probe.exists() else gzip.open(run / 'probe.jsonl.gz', 'rt')
    with stream:
        rows = [json.loads(line) for line in stream]
    result = next(row['data'] for row in rows if row['kind'] == 'result')
    end = stamp(result['stamp'])
    start = end - result['elapsed_time']
    previous_execution = None
    accepted = []
    rejected = []
    for row in rows:
        if row['kind'] != 'controller':
            continue
        data = row['data']
        if not start <= stamp(data['stamp']) <= end:
            continue
        status = data['status']
        if status == 'PLAN_ACCEPTED' and data['trajectory_replaced']:
            old = previous_execution
            accepted.append(dict(
                elapsed=stamp(data['stamp']) - start,
                plan_id=data['plan_id'],
                previous_execution_status=old['status'] if old else None,
                previous_plan_id=old['plan_id'] if old else None,
                contact_shift=(stamp(data['contact_stamp']) - stamp(old['contact_stamp'])
                               if old else None),
                locked=data['target_locked'],
                observation_age=data['last_valid_observation_age'],
                prediction_age=data['prediction_age'],
                remaining=data['remaining_time'],
                continuity_position=data['handover_position_error'],
                continuity_velocity=data['handover_velocity_error'],
                continuity_acceleration=data['handover_acceleration_error']))
        elif status == 'PLAN_REJECTED':
            rejected.append(dict(elapsed=stamp(data['stamp']) - start,
                                 active_plan_id=data['plan_id'],
                                 candidate_plan_id=data['attempted_plan_id'],
                                 reason=data['rejection_reason'],
                                 trajectory_replaced=data['trajectory_replaced']))
        if status in ('TRACKING', 'TERMINAL_COMMITTED'):
            previous_execution = data
    report = dict(accepted=accepted, rejected=rejected,
                  accepted_during_commitment=[x for x in accepted
                      if x['previous_execution_status'] == 'TERMINAL_COMMITTED'])
    (output / 'replan_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    audit(args.run, args.output)
