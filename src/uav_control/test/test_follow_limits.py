"""Research envelopes are explicit and receiver-owned; wire values cannot widen them."""
from dataclasses import replace
import pytest

from test_follow_contract import curve, context
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
from uav_control.guidance.follow_contract import FollowReceiver, curve_errors
from uav_control.guidance import follow_limits as limits


@pytest.mark.parametrize('acceleration', (3., 3.5, 4., 4.5))
@pytest.mark.parametrize('rate', (1., 1.25, 1.5))
def test_twelve_research_envelopes_have_distinct_complete_fingerprints(acceleration, rate):
    config = limits.ResearchConfig(maximum_horizontal_acceleration=acceleration,
                                   maximum_yaw_rate=rate)
    config.validate()
    model = FollowMpcSeed(config)
    snapshot = limits.constraint_snapshot(model)
    assert snapshot.fingerprint == limits.constraint_snapshot(model).fingerprint
    assert snapshot.values['mpc']['maximum_horizontal_acceleration'] == acceleration
    assert 'extrinsics' in snapshot.values and 'attitude' in snapshot.values
    with pytest.raises(ValueError):
        MpcConfig(maximum_horizontal_acceleration=4.).validate()


@pytest.mark.parametrize('kwargs', [
    dict(maximum_horizontal_acceleration=4.6), dict(maximum_yaw_rate=1.6),
    dict(maximum_horizontal_jerk=6.1), dict(maximum_input_age=.126),
    dict(maximum_horizontal_acceleration=float('nan'))])
def test_invalid_research_parameters_do_not_relax_other_gates(kwargs):
    with pytest.raises(ValueError):
        limits.ResearchConfig(**kwargs).validate()


def test_receiver_checks_local_snapshot_and_config_change_revokes_approval():
    local = limits.constraint_snapshot(FollowMpcSeed())
    c = replace(curve(), constraint_snapshot=local.payload,
                constraint_fingerprint=local.fingerprint)
    assert 'CONSTRAINT_MISMATCH' not in curve_errors(c, local)
    wrong = replace(c, constraint_fingerprint='0'*64)
    assert 'CONSTRAINT_MISMATCH' in curve_errors(wrong, local)
    receiver = FollowReceiver(limits=local)
    ack = receiver.propose(c, context())
    assert ack.state == 'REJECTED' and 'NO_QUALIFIED_HOLDING_MODEL' in ack.reasons
    # Synthetic config invalidation only; never ROS qualification.
    receiver.active = c
    changed = FollowMpcSeed(limits.ResearchConfig(maximum_yaw_rate=1.25))
    new = limits.constraint_snapshot(changed)
    receiver.set_limits(new)
    assert receiver.active is None and receiver.pending is None
    assert 'CONSTRAINT_MISMATCH' in receiver.propose(c, context()).reasons
