"""Control delivery precedes expensive evidence snapshots without losing full audit records."""
from dataclasses import replace
import pytest

from test_p6_input_aligned_dispatch import runner, receive
from uav_control.controllers import follow_mpc_shadow_node as shadow
from uav_control.controllers.follow_mpc_seed import MpcSeedResult


def test_typed_critical_delivery_precedes_recursive_candidate_snapshot(monkeypatch):
    r, pool, events = runner()
    receive(r, 100.2, 100.235, 10.235, 2)
    r.tick(100.24, 10.24)
    req = r.pending['request']
    result = MpcSeedResult(req.context, valid=True, solver_status='SAMPLED_FEASIBLE')
    order = []
    snapshot = shadow.record_snapshot

    def observed_snapshot(value):
        if isinstance(value, MpcSeedResult):
            order.append('snapshot')
        return snapshot(value)

    def critical(request, solved, record):
        assert request is req and solved is result
        order.append('wire')
        return dict(published=True, reason='', wire_finish_ros=100.25)

    r.critical_completion = critical
    monkeypatch.setattr(shadow, 'record_snapshot', observed_snapshot)
    pool.future.set_result(result)
    r.tick(100.25, 10.25)
    assert order[:2] == ['wire', 'snapshot']
    assert events[-1]['candidate_output']['context']['cycle_id'] == req.context.cycle_id
    assert events[-1]['critical_outcome']['published'] is True


def test_invalid_context_never_reaches_critical_publisher():
    r, pool, events = runner()
    receive(r, 100.2, 100.235, 10.235, 2)
    r.tick(100.24, 10.24)
    req = r.pending['request']
    r.critical_completion = lambda *_: (_ for _ in ()).throw(AssertionError('Invalid publication'))
    pool.future.set_result(MpcSeedResult(replace(req.context, mission_id=99), valid=True))
    r.tick(100.25, 10.25)
    assert not events[-1]['output']['valid']
    assert 'critical_outcome' not in events[-1]


def test_audit_failure_cannot_repeat_an_already_attempted_wire_delivery():
    r, pool, _ = runner()
    receive(r, 100.2, 100.235, 10.235, 2)
    r.tick(100.24, 10.24)
    count = []
    r.critical_completion = lambda *_: (count.append(1), dict(published=True, reason=''))[1]
    r.emit = lambda _: (_ for _ in ()).throw(OSError('audit storage failure'))
    pool.future.set_result(MpcSeedResult(r.pending['request'].context, valid=True))
    for now in (100.25, 100.26):
        with pytest.raises(OSError, match='audit storage failure'):
            r.tick(now, now-90.)
    assert count == [1]
