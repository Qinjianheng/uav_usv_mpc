#!/usr/bin/env python3
"""P44 all-population provenance, counterexamples and real Tracker arrival epoch audit."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from p43_replay_analysis import matched, read
from p43_follow_analysis import analyze_p43, profile_summary
from p4_follow_analysis import distribution


def candidate_summary(rows):
    """Untried rows remain untried; strict successes and final expiry are distinct."""
    audits = [r['result'].get('metrics', {}).get('p43', {}) for r in rows]
    attempts = [a for d in audits for a in d.get('attempts', [])]
    ranks = Counter(str(next((i + 1 for i, a in enumerate(d.get('attempts', []))
                             if a['valid']), None)) for d in audits)
    generated = float(np.mean([d.get('generated', 0) for d in audits])) if audits else None
    return dict(count=len(rows), first_strict_pass=ranks.get('1', 0), pass_rank=dict(ranks),
                generated_mean=generated,
                strict_mean=len(attempts)/len(rows) if rows else None,
                strict_seconds=distribution([a['strict_seconds'] for a in attempts]),
                strict_statuses=dict(Counter(a['status'] for a in attempts)),
                final_statuses=dict(Counter(r.get('final_status', r['result']['solver_status'])
                                            for r in rows)),
                successful_direction=dict(Counter(str(a['beta']) for a in attempts if a['valid'])),
                successful_scale=dict(Counter(str(a['scale']) for a in attempts if a['valid'])),
                top_depth={str(n): sum(any(a['valid'] for a in d.get('attempts', [])[:n])
                                       for d in audits) for n in (1, 2, 3, 5)},
                unattempted_budget=sum(d.get('budget_stopped', False) for d in audits),
                profile=profile_summary([r['profile'] for r in rows
                                         if r['profile'].get('stages')]))


def offline(root):
    """Same-source comparisons keep every loss, even if aggregate feasible counts are equal."""
    summaries, comparisons, counterexamples = {}, {}, []
    for scene in ('f2', 'f3'):
        for horizon in (1.2, 1.6):
            for kind in ('single', 'rolling'):
                prefix = f'{scene}_h{horizon}_'
                baseline = read(root/f'{prefix}A_none_{kind}.jsonl')
                for engine in 'ABCDEF':
                    name = f'{prefix}{engine}_none_{kind}'
                    rows = read(root/(name+'.jsonl'))
                    summaries[name] = candidate_summary(rows)
                    errors, losses, different_start = [], [], 0
                    for a, b in zip(baseline, rows):
                        assert (a['source_sha256'], a['line']) == (b['source_sha256'], b['line'])
                        same_start = np.allclose(a['request']['state'], b['request']['state'],
                                                 rtol=0., atol=1e-9)
                        different_start += int(not same_start)
                        if a['result']['valid'] and not b['result']['valid']:
                            reason = ('different_own_history' if not same_start else
                                      'budget' if 'DEADLINE' in b['result']['solver_status'] else
                                      'requires_same_request_diagnosis')
                            losses.append(dict(
                                line=a['line'], sha=a['source_sha256'], reason=reason))
                            counterexamples.append(dict(group=name, classification=reason,
                                                        original=a, optimized=b))
                        if a['result']['valid'] and b['result']['valid'] and same_start:
                            errors.append(float(np.max(np.abs(
                                np.asarray(a['result']['metrics']['xyz_coefficients'])
                                - np.asarray(b['result']['metrics']['xyz_coefficients'])))))
                    comparisons[name] = dict(losses=losses, changed_start=different_start,
                                             coefficient_max=max(errors, default=None),
                                             coefficient_compared=len(errors))
    matches = {f'{scene}_h{h}': matched([root/f'{scene}_h{h}_{e}_none_rolling.jsonl'
                                        for e in 'ABCDEF'])
               for scene in ('f2', 'f3') for h in (1.2, 1.6)}
    report = dict(candidates=summaries, comparisons=comparisons, matched_errors=matches)
    (root.parent/'offline_analysis.json').write_text(json.dumps(report, indent=2))
    with (root.parent/'all_counterexamples.jsonl').open('x') as f:
        for row in counterexamples:
            f.write(json.dumps(row)+'\n')
    print('groups', len(summaries), 'losses', len(counterexamples))


def merge_acks(monitor, planner):
    """Union two real subscriptions by complete identity; absent service timing stays unknown."""
    rows = {}
    for row in planner:
        receiver, publisher, mission, generation, plan = row['identity']
        rows[tuple(row['identity'])] = dict(
            plan=plan, mission=mission, generation=generation, receiver_boot=receiver,
            planner_boot=publisher, stamp=row['stamp'], state=row['state'], reasons=row['reasons'],
            receiver_compute_seconds=None, evidence_source='planner_subscription')
    for row in monitor:
        key = (row['receiver_boot'], row['planner_boot'], row['mission'],
               row['generation'], row['plan'])
        if key in rows and (abs(rows[key]['stamp']-row['stamp']) > 1e-7
                            or rows[key]['state'] != row['state']):
            raise ValueError('ACK_EVIDENCE_MISMATCH')
        rows[key] = dict(row, evidence_source='monitor_subscription')
    return list(rows.values())


def tracker_arrival(rows, acknowledgements):
    """ACK stamp is real receiver callback context.now, before receiver computation."""
    requests = {r['request']['context']['cycle_id']: r['request']['context'] for r in rows
                if r['event'] == 'completion' and r.get('request')}
    revised = {r['cycle_id']: r['new_prediction'] for r in rows
               if r['event'] == 'prediction_revalidation' and r['report'].get('valid')}
    records = []
    for ack in acknowledgements:
        c = requests.get(ack['plan'])
        if c is None:
            continue
        old_forecast = dict(observation_stamp=c['observation_stamp'],
                            source_stamp=c['prediction_source_stamp'])
        forecast = revised.get(ack['plan'], old_forecast)
        stamps = [c['navigation_stamp'], c['attitude_stamp'], forecast['observation_stamp'],
                  forecast['source_stamp']]
        age = max(ack['stamp']-s for s in stamps)
        service = ack.get('receiver_compute_seconds')
        after = None if service is None else .125-age-service
        records.append(dict(plan=ack['plan'], callback_stamp=ack['stamp'], maximum_input_age=age,
                            remaining_input_ttl=.125-age,
                            remaining_after_compute=after,
                            state=ack['state'], reasons=ack['reasons']))
    return dict(records=records,
                maximum_input_age=distribution([r['maximum_input_age'] for r in records]),
                ttl=distribution([r['remaining_input_ttl'] for r in records]),
                after_compute_ttl=distribution([r['remaining_after_compute'] for r in records]))


def online(root):
    """Analyze only new flights; source experiments are never rewritten."""
    reports = {}
    for case in sorted(root.iterdir()):
        if not (case/'offline_evidence.jsonl').exists():
            continue
        report = analyze_p43(case)
        rows = [json.loads(s) for path in (case/'shadow').glob('mpc_seed_shadow*.jsonl')
                for s in path.open()]
        raw = read(case/'offline_evidence.jsonl')
        complete = [dict(result=(r.get('candidate_output') or r['output']), profile={},
                         final_status=r['output']['solver_status']) for r in rows
                    if r['event'] == 'completion']
        report['candidate_search'] = candidate_summary(complete)
        monitor_acks = [r for r in raw if r['kind'] == 'follow_ack']
        planner_acks = [json.loads(s) for p in (case/'shadow').glob('follow_ack*.jsonl')
                        for s in p.open()]
        acknowledgements = merge_acks(monitor_acks, planner_acks)
        report['ack_evidence'] = dict(
            planner_count=len(planner_acks), monitor_count=len(monitor_acks),
            union_count=len(acknowledgements),
            missing_compute=sum(a['receiver_compute_seconds'] is None for a in acknowledgements),
            states=dict(Counter(a['state'] for a in acknowledgements)))
        report['tracker_arrival'] = tracker_arrival(rows, acknowledgements)
        reports[case.name] = report
        (case/'p44_analysis.json').write_text(json.dumps(report, indent=2))
        print(case.name, report['final_legal_seconds'], flush=True)
    (root.parent/'online_analysis.json').write_text(json.dumps(reports, indent=2))


def main():
    """Select frozen or real-isolated evidence, both explicit populations."""
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=('offline', 'online'))
    p.add_argument('root', type=Path)
    args = p.parse_args()
    (offline if args.mode == 'offline' else online)(args.root)


if __name__ == '__main__':
    main()
