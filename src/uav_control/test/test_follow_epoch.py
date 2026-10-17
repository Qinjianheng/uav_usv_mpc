"""Receiver-owned epoch and independent planner restarts cannot be inferred from proposals."""
from dataclasses import replace

from test_follow_contract import curve, context
from uav_control.guidance.follow_contract import FollowReceiver


def test_receiver_boot_stable_but_reset_generation_advances_and_boot_restart_changes_identity():
    from uav_control.guidance.follow_epoch import ReceiverClock
    clock = ReceiverClock('receiver-a')
    assert clock.observe(100., 1_000_000) == 0
    assert clock.observe(100.1, 1_020_000) == 0
    assert clock.observe(100.2, 20_000) == 1
    assert clock.observe(99., 21_000) == 2
    assert clock.boot_id == 'receiver-a'
    assert ReceiverClock('receiver-b').boot_id != clock.boot_id


def test_proposal_never_sets_receiver_generation_or_boot():
    r = FollowReceiver()
    candidate = replace(curve(), generation=13, receiver_boot_id='untrusted')
    ctx = replace(context(), receiver_boot_id='receiver-a')
    ack = r.propose(candidate, ctx)
    assert 'CLOCK_GENERATION_CHANGED' in ack.reasons
    assert 'RECEIVER_RESTARTED' in ack.reasons
    assert ack.receiver_boot_id == 'receiver-a'


def test_new_planner_boot_revokes_old_reference_and_old_boot_cannot_return():
    r = FollowReceiver()
    ctx = replace(context(), receiver_boot_id='receiver-a')
    a = replace(curve(), receiver_boot_id='receiver-a', planner_boot_id='planner-a')
    r.propose(a, ctx)
    r.active = a  # Lifecycle fault injection only, not synthetic flight approval.
    b = replace(a, plan_id=2, planner_boot_id='planner-b')
    assert 'PLANNER_RESTARTED' in r.propose(b, ctx).reasons
    assert r.active is None
    assert 'PLANNER_BOOT_REPLAY' in r.propose(replace(a, plan_id=3), ctx).reasons


def test_invalid_clock_or_mission_proposal_cannot_retire_the_current_planner():
    r = FollowReceiver()
    ctx = replace(context(), receiver_boot_id='receiver-a')
    a = replace(curve(), receiver_boot_id='receiver-a', planner_boot_id='planner-a')
    r.propose(a, ctx)
    r.active = a
    forged = replace(a, mission_id=999, planner_boot_id='other')
    r.propose(forged, ctx)
    assert r.active is a and r.planner_boot == 'planner-a'


def test_malformed_new_planner_cannot_revoke_existing_reference():
    r = FollowReceiver()
    ctx = replace(context(), receiver_boot_id='receiver-a')
    a = replace(curve(), receiver_boot_id='receiver-a', planner_boot_id='planner-a')
    r.propose(a, ctx)
    r.active = a
    forged = replace(a, xyz=(), planner_boot_id='broken-boot')
    r.propose(forged, ctx)
    assert r.active is a and r.planner_boot == 'planner-a'
