"""Test complete FOLLOW curves and receiver-owned acknowledgements."""
from dataclasses import replace

import numpy as np
import pytest

from uav_control.guidance.follow_contract import (
    FollowCurve, FollowReceiver, ReceiverContext, SafetyApproval,
)


def curve(plan=1, start=100.04, parent=0):
    xyz = np.zeros((1, 6, 3))
    xyz[0, 0] = (0., 0., -5.)
    xyz[0, 1, 0] = 1.
    return FollowCurve(plan, 1, 0, 7, parent, 100., 100., 100., 100., 100.125,
                       104., start, start+1.2, start+1.2, (1.2,),
                       tuple(map(tuple, xyz.reshape(-1, 3))), ((0., 0., 0., 0.),),
                       'local_ned', 'constraints-p4-v1', 'camera-p1-v1', 'test-model')


def context(now=100.01, **kwargs):
    return replace(ReceiverContext(now, 1, 0, 7, True, True, True, True,
                                   (0., 0., -5., 1., 0., 0., 0., 0., 0., 0., 0.)), **kwargs)


def test_default_receiver_cannot_accept_planner_claimed_holding_or_first_cva():
    receiver = FollowReceiver()
    ack = receiver.propose(curve(), context())
    assert ack.state == 'REJECTED'
    assert 'NO_QUALIFIED_HOLDING_MODEL' in ack.reasons
    assert 'INITIAL_BRIDGE_UNAVAILABLE' in ack.reasons
    assert receiver.active is None and receiver.pending is None
    assert ack.receiver == 'trajectory_tracker_node'


@pytest.mark.parametrize('change,reason', [
    ({'now': 100.13}, 'INPUT_EXPIRED'), ({'now': 100.05}, 'START_MISSED'),
    ({'generation': 2}, 'CLOCK_GENERATION_CHANGED'), ({'mission_id': 9}, 'MISSION_CHANGED'),
    ({'prediction_id': 8}, 'PREDICTION_CHANGED'), ({'following': False}, 'NOT_FOLLOW'),
    ({'offboard': False}, 'OFFBOARD_UNAVAILABLE'), ({'armed': False}, 'NOT_ARMED'),
    ({'visible': False}, 'TARGET_NOT_LOCKED'),
])
def test_faults_never_grant_execution(change, reason):
    assert reason in FollowReceiver().propose(curve(), context(**change)).reasons


@pytest.mark.parametrize('change,reason', [
    ({'input_until': 104.}, 'TTL_EXTENSION'),
    ({'holding_until': 100.1}, 'HOLDING_DOES_NOT_COVER'),
    ({'coverage_until': 100.2}, 'PREDICTION_COVERAGE'),
    ({'frame': 'enu'}, 'FRAME_OR_VERSION'),
    ({'xyz': ((float('nan'), 0., 0.),)*6}, 'INVALID_COEFFICIENTS'),
    ({'end': 102.}, 'INVALID_INTERVAL'),
])
def test_corrupt_complete_contract_rejected(change, reason):
    assert reason in FollowReceiver().propose(replace(curve(), **change), context()).reasons


def proof(c, ctx):
    # Independent injected test verifier
    # NEVER installed by the ROS receiver.
    return SafetyApproval(c.fingerprint(), c.holding_until, 'unit-test-only')


def test_receiver_lifecycle_pending_replacement_and_nonrenewable_expiry():
    receiver = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    first = curve()
    assert receiver.propose(first, context()).state == 'ACCEPTED'
    assert receiver.active is None
    assert receiver.tick(context(now=100.04)).state == 'ACTIVE'
    old_until = receiver.active.holding_until
    stale = receiver.propose(curve(2, 100.05, 1), context(now=100.14))
    assert stale.state == 'REJECTED'
    assert receiver.active.holding_until == old_until
    assert receiver.tick(context(now=old_until)).state == 'EXPIRED'
    assert receiver.active is None


def test_true_old_curve_is_sampled_at_handover_not_endpoint_or_measurement():
    receiver = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    receiver.propose(curve(), context())
    receiver.tick(context(now=100.04))
    xyz = ((.06, 0., -5.), (1., 0., 0.), (0., 0., 0.),
           (0., 0., 0.), (0., 0., 0.), (0., 0., 0.))
    new = replace(curve(2, 100.1, 1), xyz=xyz)
    assert receiver.propose(new, context(now=100.06)).state == 'ACCEPTED'
    assert receiver.active.plan_id == 1
    bad = replace(new, plan_id=3, xyz=((1.2, 0., -5.), *new.xyz[1:]))
    assert 'PVA_YAW_DISCONTINUITY' in receiver.propose(bad, context(now=100.07)).reasons
    assert receiver.pending.plan_id == 2
    assert receiver.tick(context(now=100.1)).plan_id == 2


def test_pred_change_and_reset_revoke_pending_and_active():
    r = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    r.propose(curve(), context())
    assert r.tick(context(now=100.04, prediction_id=9)).state == 'REVOKED'
    assert r.pending is None
    r = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    r.propose(curve(2), context())
    r.tick(context(now=100.04))
    assert r.tick(context(now=100.05, generation=2)).state == 'REVOKED'
    assert r.active is None


def test_duplicate_and_bad_safety_ack_are_rejected():
    r = FollowReceiver(safety_validator=lambda c, x: SafetyApproval('wrong', c.end, 'unit'),
                       bridge=lambda ctx, epoch: ctx.boundary)
    assert 'SAFETY_APPROVAL_MISMATCH' in r.propose(curve(), context()).reasons
    assert 'PLAN_ID_REPLAY' in r.propose(curve(), context()).reasons


def test_circle_yaw_and_yaw_rate_are_both_checked():
    r = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    good = replace(curve(), yaw=((2*np.pi, 0., 0., 0.),))
    assert r.propose(good, context()).state == 'ACCEPTED'
    r = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    bad = replace(curve(), yaw=((0., .1, 0., 0.),))
    assert 'PVA_YAW_DISCONTINUITY' in r.propose(bad, context()).reasons


def test_future_acquisition_and_late_activation_are_not_authorized():
    r = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    future = replace(curve(), navigation_stamp=100.02, attitude_stamp=100.02)
    assert 'FUTURE_INPUT' in r.propose(future, context(now=100.01)).reasons
    r = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    assert r.propose(curve(), context()).state == 'ACCEPTED'
    assert r.tick(context(now=100.06)).state == 'REJECTED'
    assert r.active is None


def test_holding_cannot_authorize_sampling_after_curve_end():
    r = FollowReceiver(safety_validator=proof, bridge=lambda ctx, epoch: ctx.boundary)
    c = replace(curve(), holding_until=103.)
    r.propose(c, context())
    r.tick(context(now=c.start))
    assert r.tick(context(now=c.end)).state == 'EXPIRED'


@pytest.mark.parametrize('expiry', [float('nan'), float('inf')])
def test_nonfinite_independent_approval_is_never_accepted(expiry):
    r = FollowReceiver(
        safety_validator=lambda c, x: SafetyApproval(c.fingerprint(), expiry, 'unit'),
        bridge=lambda ctx, epoch: ctx.boundary)
    assert 'SAFETY_APPROVAL_MISMATCH' in r.propose(curve(), context()).reasons
