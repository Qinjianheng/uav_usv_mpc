#!/usr/bin/env python3
"""Audit immutable MINCO supply records with explicit, separate clock domains."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def statistics(values):
    """Retain missing/nonfinite counts instead of silently reporting a smaller population."""
    values = list(values)
    finite = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    result = dict(attempts=len(values), count=len(finite), missing=len(values)-len(finite),
                  p50=None, p95=None, p99=None, maximum=None, negative_count=0)
    if finite:
        result.update(zip(('p50', 'p95', 'p99', 'maximum'),
                          map(float, np.percentile(finite, (50, 95, 99, 100)))))
        result['negative_count'] = sum(v < 0 for v in finite)
    return result


def difference(a, b):
    """Subtract only explicitly chosen timestamps from the same clock domain."""
    return float(a)-float(b) if a is not None and b is not None else None


def cycle_metrics(record, final, acknowledgements):
    """Expose recorded supply stages; no historical missing timing is reconstructed."""
    request = record['request']
    context = request['context']
    candidate = record.get('candidate_output') or {}
    worker = candidate.get('metrics', {}).get('worker_timeline', {})
    prediction = record.get('input_provenance', {}).get('prediction') or {}
    completion = record.get('completion_timeline') or {}
    preparation = record.get('preparation_profile') or {}
    pose = record.get('input_provenance', {}).get('paired_navigation') or {}
    final = final or {}
    durations = dict(
        worker_wall_monotonic=difference(worker.get('solve_end_monotonic'),
                                         worker.get('solve_start_monotonic')),
        worker_cpu_seconds=worker.get('worker_thread_cpu_seconds'),
        request_to_worker_ros=difference(worker.get('solve_start_ros'), request.get('now_stamp')),
        dispatch_to_worker_monotonic=difference(worker.get('solve_start_monotonic'),
                                                record.get('cycle_started_monotonic')),
        worker_queue_monotonic=difference(worker.get('solve_start_monotonic'),
                                          record.get('submitted_monotonic')),
        preparation_monotonic=preparation.get('total_seconds'),
        future_poll_monotonic=difference(completion.get('future_observed_monotonic'),
                                         worker.get('solve_end_monotonic')),
        completion_payload_monotonic=difference(completion.get('event_prepared_monotonic'),
                                                completion.get('future_observed_monotonic')),
        navigation_transport_ros=difference(pose.get('navigation_receipt_stamp'),
                                            context.get('navigation_stamp')),
        prediction_transport_ros=difference(prediction.get('receipt_ros_stamp'),
                                            prediction.get('generated_stamp')),
        acquisition_to_wire_ros=difference(final.get('wire_finish_ros'),
                                           context.get('observation_stamp')),
        worker_end_to_wire_ros=difference(final.get('wire_finish_ros'),
                                          worker.get('solve_end_ros')),
        critical_cycle_monotonic=final.get('wire_cycle_seconds'),
        whole_audit_cycle_monotonic=final.get('whole_cycle_time'))
    if record.get('critical_outcome') is not None:
        # Typed delivery precedes candidate/provenance snapshots in the new path.
        # Legacy snapshot-first traces retain their original timing semantics.
        wire_mono = (record['cycle_started_monotonic']+final['wire_cycle_seconds']
                     if final.get('wire_cycle_seconds') is not None else None)
        durations['completion_payload_monotonic'] = difference(
            completion.get('event_prepared_monotonic'),
            completion.get('critical_finished_monotonic'))
        durations['future_to_wire_monotonic'] = difference(
            wire_mono, completion.get('future_observed_monotonic'))
        durations['post_wire_critical_callback_monotonic'] = difference(
            completion.get('critical_finished_monotonic'), wire_mono)
    for key, value in (candidate.get('timing') or {}).items():
        durations['solver_'+key+'_monotonic'] = value
    for key, value in (candidate.get('metrics', {}).get('cycle_profile') or {}).get(
            'stages', {}).items():
        durations['exclusive_'+key+'_monotonic'] = value['seconds']
    accepted = next((a for a in acknowledgements if a.get('state') == 'ACCEPTED'), {})
    active = next((a for a in acknowledgements if a.get('state') == 'ACTIVE'), {})
    durations['wire_to_accepted_ros'] = difference(
        accepted.get('stamp'), final.get('wire_finish_ros'))
    durations['activation_lag_ros'] = difference(
        active.get('stamp'), context.get('execution_start_stamp'))
    expiry_values = [context.get('navigation_stamp'), context.get('attitude_stamp'),
                     context.get('observation_stamp'), context.get('prediction_source_stamp')]
    expiry = (min(*(s+.125 for s in expiry_values), context['prediction_valid_until'])
              if all(s is not None for s in expiry_values)
              and context.get('prediction_valid_until') is not None else None)
    rejected = next((a for a in acknowledgements if a.get('state') == 'REJECTED'), {})
    failure = (','.join(rejected.get('reasons', [])) or final.get('reason')
               or (record.get('output') or {}).get('solver_status'))
    if active:
        failure = ''
    return dict(cycle_id=context['cycle_id'], durations=durations,
                clock_domains={key: ('thread_cpu' if key == 'worker_cpu_seconds' else
                                     'ROS' if key.endswith('_ros') else 'monotonic')
                               for key in durations},
                margins=dict(
                    original_request_ttl_at_wire_ros=difference(
                        expiry, final.get('wire_finish_ros')),
                    final_wire_input_margin_ros=final.get('wire_finish_input_margin')),
                failure=failure, published=final.get('published', False),
                accepted=bool(accepted), active=bool(active),
                original_context=context, request_epoch=request.get('now_stamp'),
                worker_timeline=worker, acknowledgements=acknowledgements,
                budget=candidate.get('metrics', {}).get('follow_guided', {}).get('budget'),
                timing_limits=[
                    'request_to_worker_ros includes preparation; worker_queue_monotonic '
                    'is submit-call to worker-start, including executor handoff/OS scheduling',
                    'Worker wall inflation alone cannot identify GIL versus OS scheduling',
                    'Exclusive profile stages and nested solver timing must not be summed',
                    'Prediction transport is the recorded receipt snapshot, which may be newer '
                    'than the prepared request; original request epochs remain in context',
                    'Original-request TTL and acquisition-to-wire describe the prepared request, '
                    'not a newer prediction accepted during revalidation; use final wire input '
                    'margin and the independent identity/owner audit for flight acceptance',
                    'ACK matching here uses plan ID within this single manifest; it is not an '
                    'independent mission/sequence/trajectory-identity ownership validator'])


def sha(path):
    """Hash a recorded file without modifying it."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_jsonl(paths):
    """Reject malformed records without dropping lines from the cohort."""
    return [json.loads(line) for path in paths for line in path.open()]


def analyze(evidence, output, flight=None):
    """Use the remote manifest and optional verified complete local original records."""
    manifest = json.loads((evidence/'manifest.json').read_text())
    for item in manifest['uploaded_files']:
        if sha(evidence/item['name']) != item['sha256']:
            raise ValueError('ORIGINAL_EVIDENCE_CHANGED: '+item['name'])
    if flight:
        for item in manifest['original_sources']:
            path = flight.parent/item['source']
            if sha(path) != item['sha256']:
                raise ValueError('LOCAL_ORIGINAL_EVIDENCE_CHANGED: '+item['source'])
        planner_paths = sorted((flight/'shadow').glob('mpc_seed_shadow_*.jsonl'))
        events = read_jsonl(planner_paths)
        observations = read_jsonl([flight/'offline_evidence.jsonl'])
        acknowledgements = [r for r in observations if r['kind'] == 'follow_ack']
    else:
        planner_paths = sorted(evidence.glob('planner_*.jsonl'))
        events = read_jsonl(planner_paths)
        observations = read_jsonl(sorted(evidence.glob('observation_*.jsonl')))
        acknowledgements = [r for r in observations if r['kind'] == 'follow_ack']
    completions = [r for r in events if r['event'] == 'completion' and r.get('request')]
    finals = {r['cycle_id']: r for r in events if r['event'] == 'follow_publication_final'}
    cycles = []
    for record in completions:
        cycle_id = record['request']['context']['cycle_id']
        acks = [a for a in acknowledgements if a.get('plan') == cycle_id]
        cycles.append(cycle_metrics(record, finals.get(cycle_id), acks))
    fields = sorted({key for r in cycles for key in r['durations']})
    distributions = {key: statistics(r['durations'].get(key) for r in cycles) for key in fields}
    losses = []
    for loss in manifest['fallbacks']:
        attempts = [r for r in completions
                    if (r['request'].get('prior_curve') or {}).get('plan_id') == loss['plan']]
        waiting = [dict(event=r['event'], stamp=r.get('event_stamp'),
                        reason=(r.get('output') or {}).get('reason')) for r in events
                   if r['event'] == 'reject' and loss['stamp']-.7 <= r.get('event_stamp', 0.)
                   <= loss['stamp']+.05]
        losses.append(dict(**loss,
                           attempts=[r['request']['context']['cycle_id'] for r in attempts],
                           cycles=[c for c in cycles if c['cycle_id'] in {
                               r['request']['context']['cycle_id'] for r in attempts}],
                           recorded_wait_states=waiting))
    report = dict(schema='p61-supply-timing-v1', cohort='complete original flight' if flight else
                  'manifest-selected failure windows only', cycles=cycles,
                  distributions=distributions, losses=losses,
                  publication_reasons=dict(Counter(r.get('reason') or 'PUBLISHED'
                                                   for r in finals.values())),
                  observed_counts=dict(completions=len(cycles), finals=len(finals),
                                       acknowledgements=len(acknowledgements)),
                  failed_cycle_fraction=sum(not r['active'] for r in cycles)/max(1, len(cycles)),
                  evidence_manifest_sha256=sha(evidence/'manifest.json'),
                  source_sha256=sha(__file__), clock_mixing=False,
                  limitations=['Historical records lack worker thread CPU time and every DDS '
                               'dequeue/dispatch event. No invented attribution.',
                               'Publication-final audit end is later than critical wire end.',
                               'Failed-cycle fraction includes inhibited/recovery periods; '
                               'actual ownership is reported by the independent flight audit.'])
    output.mkdir(parents=True, exist_ok=False)
    (output/'analysis.json').write_text(json.dumps(report, indent=2)+'\n')
    (output/'SHA256SUMS').write_text(sha(output/'analysis.json')+'  analysis.json\n')
    return report


def main():
    """Create a new external report; never overwrite input evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--flight', type=Path)
    args = parser.parse_args()
    report = analyze(args.evidence.resolve(), args.output.resolve(),
                     args.flight.resolve() if args.flight else None)
    print(json.dumps(dict(counts=report['observed_counts'],
                          reasons=report['publication_reasons'],
                          distributions=report['distributions']), indent=2))


if __name__ == '__main__':
    main()
