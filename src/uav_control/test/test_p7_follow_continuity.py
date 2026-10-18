"""Completed successors keep original epochs; revocations retain receiver evidence."""
from dataclasses import replace
import json
from types import SimpleNamespace
from pathlib import Path
import sys

import pytest

from test_p6_input_aligned_dispatch import runner, receive, finish
from test_p52_execution_chain import tracker_fixture
from test_follow_sitl_execution import ctx

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


@pytest.mark.parametrize('fault,want', [
    ('original_expired', 'STALE_INPUT'),
    ('latest_expired', 'STALE_INPUT'),
    ('prediction_lost', 'PREDICTION_UNAVAILABLE'),
    ('mission_changed', 'MISSION_CHANGED'),
    ('generation_changed', 'CLOCK_GENERATION_CHANGED'),
    ('parent_changed', 'ACTIVE_PARENT_CHANGED'),
])
def test_periodic_completion_rejects_real_input_or_identity_loss(fault, want):
    r, pool, events = runner()
    r.tick(100.04, 10.04)
    now = 100.09
    if fault == 'original_expired':
        now = 100.13
        receive(r, 100.08, 100.115, 10.115, 2)
    elif fault == 'latest_expired':
        r.prediction = replace(r.prediction, observation_stamp=99.9,
                               source_stamp=99.93, valid_until=100.025)
    elif fault == 'prediction_lost':
        r.prediction = None
    elif fault == 'mission_changed':
        r.mission['mission_id'] += 1
    elif fault == 'generation_changed':
        r.adapter.clock_generation += 1
    else:
        def changed_parent(_request, _now):
            raise ValueError('ACTIVE_PARENT_CHANGED')
        r.completion_admission = changed_parent
    finish(r, pool, now, now-90.)
    assert r.pending is None
    assert events[-1]['candidate_output']['valid']
    assert not events[-1]['output']['valid']
    assert events[-1]['output']['reason'] == want


def test_revocation_logs_consumed_input_context_without_preserving_authority():
    n = tracker_fixture()
    records, acks = [], []
    n.follow_debug_pub = SimpleNamespace(publish=lambda m: records.append(json.loads(m.data)))
    n.follow_ack_pub = SimpleNamespace(publish=acks.append)
    n.follow_input_status = dict(now=100.06, target_source_stamp=99.9,
                                 callback_receipts=dict(target=99.94))
    n._follow_context = lambda: replace(ctx(100.06), target_fresh=False)
    n._revoke_nominal_follow('TARGET_STATE_STALE')
    assert n.follow_receiver.active is None
    assert acks[-1].state == 'REVOKED'
    assert list(acks[-1].reasons) == ['TARGET_STATE_STALE']
    assert records[-1]['event'] == 'execution_revoked'
    assert records[-1]['plan'] == 1
    assert records[-1]['reason'] == 'TARGET_STATE_STALE'
    assert records[-1]['input_freshness']['target_source_stamp'] == 99.9
    assert records[-1]['input_freshness']['callback_receipts']['target'] == 99.94
    assert records[-1]['execution_context']['target_fresh'] is False


def test_revocation_diagnostic_cannot_masquerade_as_an_active_reference():
    from p4_shadow_monitor import execution_debug_kind
    from p51_execution_audit import audit_rows
    from test_p51_execution_audit import rows
    evidence = rows()
    evidence.extend([
        dict(kind='follow_ack', receipt=100.29, stamp=100.29, plan=1, state='REVOKED',
             owner='ORIGINAL_FOLLOW', active=0, reasons=['TARGET_STATE_STALE']),
        dict(kind='diagnostic', receipt=100.29, stamp=100.29, plan=0, status='FOLLOW'),
    ])
    record = dict(event='execution_revoked', receipt=100.29, stamp=100.29, plan=1,
                  reason='TARGET_STATE_STALE', input_freshness=dict(target_source_stamp=100.1))
    evidence.append(dict(record, kind=execution_debug_kind(record)))
    report = audit_rows(evidence, 100.04, 'f2')
    assert report['passed'], report['violations']
    assert report['fallbacks'][0]['reason'] == 'TARGET_STATE_STALE'
    assert report['reference_samples'] == 5
