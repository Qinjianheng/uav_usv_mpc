#!/usr/bin/env python3
"""Old/new DKF callback comparison on one unchanged real causal acquisition/receipt trace."""
import argparse
from collections import defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace as Row

from rosidl_runtime_py.set_message import set_message_fields
from uav_usv_interfaces.msg import (
    NativeAttitude, TargetImageDetection, TargetObservation, UavState,
)

from p763_dkf_replay import fixture_nodes, legacy_classes, CONTRACT
from uav_control.common.p76_contract import load_contract
from uav_control.tracking.spatial_dkf_node import SpatialDkfNode
from uav_control.tracking.image_dkf_node import ImageDkfNode
from p764_diagnostic_collect import stats


def replay(events, classes):
    clock = Row(value=events[0]['receipt'])
    nodes = fixture_nodes(classes, clock, load_contract(CONTRACT))
    timings, valid, active, source_after, illegal, transitions = defaultdict(list), [], [
    ], set(), 0, []
    mapping = {'SpatialDkfNode': nodes[0], 'ImageDkfNode': nodes[1]}
    message_types = {'SpatialDkfNode.receive': TargetObservation,
                     'ImageDkfNode.receive': TargetImageDetection,
                     'ImageDkfNode.pose': NativeAttitude, 'ImageDkfNode.navigation': UavState}
    cpu = time.process_time()
    for event in events:
        clock.value = event['receipt']
        node = mapping[event['node']]
        node.p76_phase, node.p76_epoch, node.p76_y_stamp = (
            event['phase'], event['epoch'], event['y_stamp'])
        if event['method'] == 'mode_changed' and not hasattr(node, 'mode_changed'):
            # Legacy spatial install_phase_gate had no on-change hook. Its next real tick
            # applies the phase gate; adding a new reset hook would alter the old architecture.
            continue
        name = event['node']+'.'+event['method']
        arguments = []
        for fields in event['arguments']:
            message = message_types[name]()
            set_message_fields(message, fields)
            arguments.append(message)
        before = node.last_state
        began = time.perf_counter()
        getattr(node, event['method'])(*arguments)
        timings[name].append(time.perf_counter()-began)
        after = node.last_state
        if event['method'] == 'mode_changed' and event['node'] == 'ImageDkfNode':
            transitions.append(dict(phase=node.p76_phase,
                                    valid_before=bool(before and before.valid),
                                    filter_source=node.filter.filter.source_stamp))
        ack = node.last_ack
        permitted = ((event['node'] == 'SpatialDkfNode') == (node.p76_phase == 'FOLLOW_ACTIVE'))
        illegal += bool(ack.get('control_authorized')) and not permitted
        if after is not None:
            valid.append(bool(after.valid))
            active.append((event['node'], node.p76_phase, bool(after.valid)))
            if (event['node'] == 'SpatialDkfNode' and node.p76_phase != 'FOLLOW_ACTIVE'
                    and after.valid):
                source_after.add((after.source_stamp.sec, after.source_stamp.nanosec))
    return dict(callback_seconds={k: stats(v) for k, v in timings.items()},
                cpu_seconds=time.process_time()-cpu, states_observed=len(valid),
                valid_states=sum(valid), transitions=transitions,
                spatial_distinct_updates_after_y=len(source_after),
                illegal_authorizations=int(illegal))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.session/'actual_callback_inputs.jsonl.gz'
    with gzip.open(source, 'rt') as stream:
        events = [json.loads(line) for line in stream]
    if not events:
        raise ValueError('NO_REAL_CALLBACK_TRACE')
    current = replay(events, (SpatialDkfNode, ImageDkfNode))
    old = replay(events, legacy_classes())
    output = dict(evidence_class='REAL_MATCHED_CALLBACK_REPLAY',
                  source=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  events=len(events), original_receipt_epochs_preserved=True,
                  original_acquisition_epochs_preserved=True, parallel=current, legacy=old,
                  flight_qualified=False,
                  scope='Exact real callback inputs with mock transport; executor timing separate')
    args.output.write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps(dict(events=len(events), spatial_after_y=current[
        'spatial_distinct_updates_after_y'], illegal=current['illegal_authorizations'])))


if __name__ == '__main__':
    main()
