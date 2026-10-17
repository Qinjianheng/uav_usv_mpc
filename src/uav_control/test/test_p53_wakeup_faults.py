"""Explicit clock/parent/load faults on the production accepted-start wakeup."""
from dataclasses import replace
from test_p52_activation_wakeup import fixture
from test_follow_sitl_execution import candidate, ctx


def test_backward_clock_wakeup_rearms_without_early_execution():
    node, curve = fixture()
    node._schedule_follow_activation()
    node.epoch = curve.start-.04
    node.test_wakeups[-1].callback()
    assert not node.calls and node.follow_receiver.active is None
    assert abs(node.test_wakeups[-1].period-.04) < 1e-9


def test_forward_clock_jump_beyond_lease_never_activates():
    node, curve = fixture()
    node._schedule_follow_activation()
    node.epoch = curve.start+.5
    node.test_wakeups[-1].callback()
    assert node.follow_receiver.active is None and node.follow_receiver.pending is None


def test_fifty_ms_executor_delay_plus_validation_keeps_original_start_window():
    node, curve = fixture()
    node._schedule_follow_activation()
    node.epoch = curve.start+.05
    node.test_wakeups[-1].callback()
    assert node.follow_receiver.active is None
    assert any('START_MISSED' in ack.reasons for ack in node.follow_receiver.drain_events())


def test_parent_loss_clears_child_before_its_wakeup_can_execute():
    node, _ = fixture()
    old = candidate(9, 100.04)
    node.follow_receiver.active, node.follow_receiver.deadline = old, 100.49
    node.follow_receiver.pending = None
    child = replace(candidate(2, 100.15, 9), xyz=((.11, 0., -5.), *old.xyz[1:]))
    assert node.follow_receiver.propose(child, ctx(100.06)).state == 'ACCEPTED'
    node._schedule_follow_activation()
    node.follow_receiver.tick(ctx(100.1), {9: False, 2: True})
    node.epoch = 100.15
    node.test_wakeups[-1].callback()
    assert not node.calls and node.follow_receiver.active is None
