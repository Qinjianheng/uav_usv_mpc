#!/usr/bin/env python3
"""Recorded Receiver event replay. Prefix observations are evidence, not a new FOV proof."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np

from p5_release import sha, verify_manifest
from uav_control.guidance.follow_contract import FollowCurve, ReceiverContext
from uav_control.guidance.follow_minco_execution import NominalFollowReceiver

FAULTS = ('none', 'one_solve_failure', 'two_solve_failures', 'prediction_delay',
          'input_expired', 'solve_spike', 'callback_delay_50ms', 'parent_expired',
          'pending_parent_loss', 'infeasible', 'lease_natural', 'offboard_loss',
          'height_reference')


def dataset(root):
    """Restore omitted wire metadata from the recorded producer request, never new epochs."""
    verify_manifest(root)
    raw = [json.loads(line) for line in (root/'offline_evidence.jsonl').open()]
    completed = {r['request']['context']['cycle_id']: r
                 for path in (root/'shadow').glob('mpc_seed_shadow_*.jsonl')
                 for line in path.open() for r in [json.loads(line)]
                 if r.get('event') == 'completion' and r.get('request')}
    curves = {}
    for row in raw:
        if row['kind'] != 'proposal':
            continue
        event = completed[row['plan']]
        c, metrics = event['request']['context'], event['candidate_output']['metrics']
        curve = FollowCurve(
            row['plan'], c['mission_id'], row['generation'], row['prediction'], row['parent'],
            row['navigation'], row['attitude'], row['observation'], row['source'],
            row['input_until'], row['source']+event['request']['prediction_times'][-1],
            row['start'], row['end'], row['holding'], tuple(row['durations']),
            tuple(map(tuple, np.asarray(row['xyz']).reshape(-1, 3))),
            tuple(map(tuple, np.asarray(row['yaw']).reshape(-1, 4))),
            frame=c['frame_id'], receiver_boot_id=row['receiver_boot'],
            planner_boot_id=row['planner_boot'],
            constraint_snapshot=metrics['constraint_snapshot'],
            constraint_fingerprint=metrics['constraint_fingerprint'])
        curves[str(curve.plan_id)] = asdict(curve)
    events, expected = [], []
    upstream = []
    for cycle, event in completed.items():
        worker = event['candidate_output']['metrics'].get('worker_timeline', {})
        upstream.append(dict(kind='worker', cycle=cycle, request_now=event['request']['now_stamp'],
                             observation=event['request']['context']['observation_stamp'],
                             timeline=worker, result=event['output']['solver_status'],
                             prediction_receipt=event['input_provenance']['prediction']))
    for row in raw:
        if row['kind'] == 'follow_receive':
            events.append(dict(kind='receive', stamp=row['stamp'], plan=row['plan'],
                               state=row['state'], reasons=row['reasons']))
        elif row['kind'] == 'execution_debug':
            events.append(dict(kind='control', stamp=row['stamp'], plan=row['plan'],
                               prediction=row['prediction'], context=row['execution_context'],
                               checks=row['prefix_checks']))
        elif row['kind'] == 'follow_ack' and row['state'] in ('ACTIVE', 'EXPIRED', 'REVOKED'):
            expected.append(dict(stamp=row['stamp'], plan=row['plan'], state=row['state']))
        elif row['kind'] == 'follow_ack' and 'START_MISSED' in row['reasons']:
            expected.append(dict(stamp=row['stamp'], plan=row['plan'], state=row['state']))
    return dict(source=str(root), source_sha256=sha(root/'offline_evidence.jsonl'),
                source_manifest_sha256=sha(root/'SHA256SUMS'), curves=curves, events=events,
                upstream=upstream,
                prediction_arrivals=[r for r in raw if r['kind'] == 'prediction'],
                ack_deliveries=[r for r in raw if r['kind'] == 'follow_ack'],
                receiver_heartbeats=[r for r in raw if r['kind'] == 'receiver_epoch'],
                wire_proposals=[r for r in raw if r['kind'] == 'proposal'],
                expected=expected, scope='Recorded validation/control completion epochs; '
                'metadata restored by producer rules; recorded prefix checks supplied; '
                'no new FOV proof. Initial-entry qualification is a separate recorded audit.')


def replay_ack_delivery(data, late_duplicates=False):
    """Use actual ACK payload/delivery epochs with the production Planner consumer."""
    from io import StringIO
    from types import SimpleNamespace
    from uav_usv_interfaces.msg import FollowPlanAck
    from uav_control.controllers.follow_transport import stamp
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    node = object.__new__(P4FollowPlannerNode)
    curves = [FollowCurve(**v) for v in data['curves'].values()]
    node.follow_execution_enabled, node.planner_boot_id = True, curves[0].planner_boot_id
    node.runner = SimpleNamespace(mission={'mission_id': curves[0].mission_id})
    node.receiver_epoch, node.accepted_curve = None, None
    node.ack_file, node.expected_plans, node.execution_curves = StringIO(), {}, {}
    for row in data['wire_proposals']:
        curve = next(c for c in curves if c.plan_id == row['plan'])
        key = (curve.receiver_boot_id, curve.planner_boot_id, curve.mission_id,
               curve.generation, curve.plan_id)
        node.expected_plans[key], node.execution_curves[key] = row['stamp'], curve
    node.get_publishers_info_by_topic = lambda _: [SimpleNamespace(
        node_name='trajectory_tracker_node')]
    events = [(r['receipt'], 'heartbeat', r) for r in data['receiver_heartbeats']]
    events += [(r['receipt'], 'ack', r) for r in data['ack_deliveries']]
    injected = []
    if late_duplicates:
        active = {r['plan']: r for r in data['ack_deliveries'] if r['state'] == 'ACTIVE'}
        for row in data['ack_deliveries']:
            if row['state'] == 'ACCEPTED' and row['plan'] in active:
                injected.append((active[row['plan']]['receipt']+.001, 'ack', row))
        last = max(t for t, _, _ in events)
        injected.extend((last+.01+i*.001, 'ack', r)
                        for i, r in enumerate(list(active.values())[:3]))
        events += injected
    for now, kind, row in sorted(events, key=lambda e: e[0]):
        if kind == 'heartbeat':
            identity = (row['boot'], row['generation'], row['mission'], 0)
            node.receiver_epoch = (identity, row['stamp'])
            continue
        node._ros_seconds = lambda: now
        message = FollowPlanAck()
        message.stamp, message.plan_id = stamp(row['stamp']), row['plan']
        message.mission_id, message.clock_generation = row['mission'], row['generation']
        message.receiver_boot_id = row['receiver_boot']
        message.planner_boot_id = row['planner_boot']
        message.receiver, message.state = row['receiver'], row['state']
        message.reasons, message.control_owner = row['reasons'], row['owner']
        message.active_plan_id, message.pending_plan_id = row['active'], row['pending']
        message.replaced = row['replaced']
        node.ack_callback(message)
    return dict(processed=len(data['ack_deliveries']), injected_count=len(injected),
                final_active=node.accepted_curve.plan_id if node.accepted_curve else None,
                source_sha256=data['source_sha256'],
                scope='Real ACK payloads and delivery epochs; graph authority established '
                'by original audit. Reordered duplicates explicitly simulated.')


def replay(data, fault='none'):
    if fault not in FAULTS:
        raise ValueError('UNKNOWN_FAULT')
    curves = {int(k): FollowCurve(**v) for k, v in data['curves'].items()}
    events = [dict(r) for r in data['events']]
    # Choose the actual long second ownership segment, preserving its recorded durations.
    entries = [r for r in data['expected'] if r['state'] == 'ACTIVE'
               and not curves[r['plan']].parent_id]
    injection = entries[1]['stamp']+.6 if len(entries) > 1 else entries[0]['stamp']+.6
    selected = sorted(c.plan_id for c in curves.values() if c.parent_id and c.start >= injection)
    dropped = set(selected[:2 if fault == 'two_solve_failures' else 1])
    if fault in ('input_expired', 'solve_spike', 'parent_expired') and selected:
        for row in events:
            if row['kind'] == 'receive' and row['plan'] == selected[0]:
                row['stamp'] += {'input_expired': .13, 'solve_spike': .06,
                                 'parent_expired': .45}[fault]
    if fault == 'callback_delay_50ms':
        for row in events:
            if row['kind'] == 'control' and injection <= row['stamp'] < injection+.15:
                row['stamp'] += .05
    events.sort(key=lambda r: r['stamp'])
    receiver, result, expired, illegal = NominalFollowReceiver(), [], 0, 0
    changed, old = False, None
    for row in events:
        now = row['stamp']
        if row['kind'] == 'receive':
            curve = curves[row['plan']]
            if fault in ('one_solve_failure', 'two_solve_failures', 'infeasible'):
                if curve.plan_id in dropped:
                    continue
            if fault == 'lease_natural' and curve.start >= injection:
                continue
            if fault == 'height_reference' and curve.plan_id in dropped:
                xyz = np.asarray(curve.xyz).copy()
                xyz[::6, 2] += 2.
                curve = replace(curve, xyz=tuple(map(tuple, xyz)))
            # Rejected physical validation observations do not become a protocol grant.
            probe = 'SHADOW_PROBE_VALID' in row['reasons']
            if row['state'] != 'ACCEPTED' and not probe:
                continue
            context = ReceiverContext(now, curve.mission_id, curve.generation,
                                      curve.prediction_id, True, True, True, True,
                                      receiver_boot_id=curve.receiver_boot_id)
            ack = receiver.propose(curve, context, probe=probe)
            result.append(asdict(ack))
            continue
        current = receiver.active or receiver.pending
        if current is None:
            continue
        flags = row['context']
        visible = flags['visible']
        offboard = flags['offboard']
        checks = {int(k): bool(v.get('visible') and v.get('reason') == 'VALID')
                  for k, v in row['checks'].items()}
        if fault == 'prediction_delay' and injection <= now < injection+.15:
            visible = False  # Explicit simulated absence, not a fresh-input invention.
        if fault == 'offboard_loss' and injection <= now < injection+.1:
            offboard = False
        if (fault == 'pending_parent_loss' and receiver.pending
                and not changed and now >= injection):
            if receiver.active:
                checks[receiver.active.plan_id] = False
                changed = True
        context = ReceiverContext(now, current.mission_id, current.generation,
                                  row['prediction'], flags['following'], offboard,
                                  flags['armed'], visible,
                                  receiver_boot_id=current.receiver_boot_id)
        old = receiver.active
        receiver.tick(context, checks)
        generated = receiver.drain_events()
        for ack in generated:
            if ack.state == 'ACTIVE' and old and ack.plan_id != old.plan_id:
                illegal += curves[ack.plan_id].parent_id != old.plan_id
            result.append(asdict(ack))
        if receiver.active:
            expired += now >= min(receiver.active.end, receiver.active.start+.45)
    observed = [(r['plan_id'], r['state']) for r in result
                if r['state'] in ('ACTIVE', 'EXPIRED', 'REVOKED')
                or 'START_MISSED' in r['reasons']]
    expected = [(r['plan'], r['state']) for r in data['expected']]
    return dict(fault=fault, hypothetical_only=True, prefix_scope=data['scope'],
                actual_source_sha256=data['source_sha256'], events=result,
                observed_sequence_matches=observed == expected if fault == 'none' else None,
                expected_lifecycle=expected, replay_lifecycle=observed,
                successor_activations=sum(r['state'] == 'ACTIVE'
                                          and curves[r['plan_id']].parent_id != 0 for r in result),
                expired_active_count=expired, illegal_parent_activations=illegal,
                simulated_delay=({'input_expired': .13, 'solve_spike': .06,
                                  'callback_delay_50ms': .05, 'parent_expired': .45}.get(fault)),
                injection_scope='Observed replay unchanged. Faults alter labelled event delivery '
                'or absence; recorded prefix signals are not a new geometry validation. '
                'Entry/cooldown, ACK order and alignment use separate production policy tests.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    data = dataset(args.root)
    (args.output/'dataset.json').write_text(json.dumps(data, indent=2)+'\n')
    for fault in FAULTS:
        value = replay(data, fault)
        (args.output/(fault+'.json')).write_text(json.dumps(value, indent=2)+'\n')
        print(fault, value['observed_sequence_matches'], value['expired_active_count'],
              value['illegal_parent_activations'], value['successor_activations'])
    for label, delayed in (('actual_ack_delivery', False), ('reordered_ack', True)):
        (args.output/(label+'.json')).write_text(json.dumps(
            replay_ack_delivery(data, delayed), indent=2)+'\n')
