"""Independent receiver must re-sample local legality after expensive validation."""
from dataclasses import replace

from test_follow_sitl_execution import candidate, ctx
from uav_control.guidance.follow_minco_execution import NominalFollowReceiver


def test_input_expiring_during_receiver_math_is_rejected_without_new_pending():
    receiver = NominalFollowReceiver()
    curve = candidate()
    outcome = receiver.propose(
        curve, ctx(),
        context_provider=lambda: ctx(curve.input_until+.001))
    assert outcome.state == 'REJECTED'
    assert 'INPUT_EXPIRED' in outcome.reasons
    assert receiver.pending is None
    assert outcome.stamp == curve.input_until+.001


def test_clock_reversal_during_receiver_math_is_rejected():
    receiver = NominalFollowReceiver()
    outcome = receiver.propose(candidate(), ctx(), context_provider=lambda: ctx(99.99))
    assert 'CLOCK_REVERSED' in outcome.reasons
    assert receiver.pending is None


def test_context_refresh_cannot_replace_or_renew_an_active_parent():
    receiver = NominalFollowReceiver()
    parent = candidate()
    assert receiver.propose(parent, ctx()).state == 'ACCEPTED'
    receiver.tick(ctx(parent.start), nominal_valid=True)
    deadline = receiver.deadline
    child = replace(candidate(2, parent.start+.1, parent.plan_id),
                    xyz=((.1, 0., -5.), *parent.xyz[1:]))
    outcome = receiver.propose(
        child, ctx(parent.start+.02),
        context_provider=lambda: ctx(child.input_until+.001))
    assert outcome.state == 'REJECTED'
    assert receiver.active is parent and receiver.deadline == deadline
