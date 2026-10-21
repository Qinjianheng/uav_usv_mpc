"""Historical safe refusal never substitutes for an executable identification protocol."""
from types import SimpleNamespace

import numpy as np
import pytest

from test_p761_calibration import ROOT, script
from uav_control.control import p763_calibration_control as control


SESSION = ROOT/'data/experiments/20261021_p761_01/session_A1_03'


def physical_inputs(age=.125):
    return dict(velocity=[0., 0., 0.], acceleration=[0., 0., 0.], sample_age=age,
                mass=2., maximum_thrust=58.8, gravity=9.8, upward_room=4.,
                clearance=8., upward_braking=4., downward_braking=4.)


def test_historical_refusal_without_a_new_protocol_cannot_start_a1():
    replay, session = script('p763_exit_replay'), script('p761_session')
    historical = replay.replay(SESSION)
    assert historical['passed'] and not historical['guarded']['admission']['admitted']
    with pytest.raises(ValueError, match='NEW_PROTOCOL_REQUIRED'):
        session.validate_e0_evidence(historical, None)


def test_maximum_pulse_has_no_safe_window_at_allowed_native_age():
    protocol = control.analyze_pulse_window(**physical_inputs(), sample_period=.024002)
    assert protocol['status'] == 'A1_IDENTIFICATION_WINDOW_INFEASIBLE'
    assert protocol['safe_duration_interval'] is None
    assert protocol['selected_duration'] is None
    assert not protocol['admissible']
    assert protocol['zero_duration_bound']['predicted_peak_speed'] == pytest.approx(7.154)


def test_fresh_short_pulse_does_not_invent_three_effective_force_samples():
    protocol = control.analyze_pulse_window(**physical_inputs(0.), sample_period=.024002)
    assert protocol['safe_duration_interval'] is not None
    assert protocol['safe_duration_interval'][1] < .08
    assert protocol['maximum_original_window_samples'] == 0
    assert protocol['required_effective_samples'] == 3
    assert protocol['original_identification_duration'] == pytest.approx(.128004)
    assert protocol['selected_duration'] is None


def test_safe_virtual_long_window_can_supply_independent_effective_samples():
    inputs = dict(physical_inputs(0.), maximum_thrust=30., upward_room=20., clearance=20.)
    result = control.analyze_pulse_window(**inputs, sample_period=.024002, maximum_duration=.5)
    assert result['admissible']
    assert result['selected_duration'] == pytest.approx(.315)
    assert result['selected_bound']['admitted']
    assert result['selected_duration'] > result['response_aware_identification_duration']


def test_tilt_cannot_claim_downward_braking_without_vertical_force_authority():
    from scipy.spatial.transform import Rotation
    result = control.pulse_admission(
        **physical_inputs(0.), duration=.005,
        rotation=Rotation.from_euler('y', np.deg2rad(80.)).as_matrix())
    assert not result['admitted']
    assert 'DOWNWARD_BRAKING_THRUST_UNAVAILABLE' in result['reasons']


@pytest.mark.parametrize('context,reason', [
    ({'control_mode_confirmed': False}, 'BODYRATE_MODE_UNCONFIRMED'),
    ({'outer_saturated': True}, 'OUTER_LOOP_SATURATED'),
    ({'rotation': np.diag([1., -1., -1.])}, 'THRUST_ATTITUDE_UNSAFE'),
])
def test_admission_rejects_loss_of_control_or_safe_thrust_direction(context, reason):
    result = control.pulse_admission(**physical_inputs(0.), duration=.005, **context)
    assert not result['admitted'] and reason in result['reasons']


def test_calibration_cannot_fall_back_to_unanalyzed_022_second_schedule():
    calibration = script('p76_bodyrate_calibration')
    node = SimpleNamespace(stage='A1', steps=[], protocol=None)
    with pytest.raises(ValueError, match='A1_PROTOCOL_BLOCKED'):
        calibration.Calibration.make_steps(node)
    assert node.steps == []


def test_real_proposed_protocol_is_independent_and_blocked():
    replay, session = script('p763_exit_replay'), script('p761_session')
    historical = replay.replay(SESSION)
    proposed = replay.new_protocol(SESSION)
    assert proposed['new_protocol_defined']
    assert not proposed['new_protocol_admissible']
    assert proposed['outcome'] == 'SAFE_REFUSAL / A1_PROTOCOL_BLOCKED'
    assert proposed['reason'] == 'A1_IDENTIFICATION_WINDOW_INFEASIBLE'
    assert len(proposed['new_protocol_source_sha256']) == 64
    assert proposed['definition']['mass'] == pytest.approx(2.1948611353641296)
    assert proposed['definition']['static_model_twr'] == 3.
    assert proposed['physical_identity']['mass_recomputed_from_sdf'] == pytest.approx(
        2.1948611353641296)
    assert proposed['physical_identity']['static_twr_recomputed_from_sdf'] == pytest.approx(3.)
    assert proposed['physical_identity']['model_files_verified']
    assert proposed['executable_steps'] == []
    with pytest.raises(ValueError, match='A1_PROTOCOL_BLOCKED'):
        session.validate_e0_evidence(historical, proposed)


def test_explicit_historical_model_copy_preserves_physical_identity_and_refusal():
    replay = script('p763_exit_replay')
    copy = SESSION.parent/'frozen/models'
    proposed = replay.new_protocol(SESSION, historical_model_root=copy)
    assert proposed['physical_identity']['actual_model_root'] == str(copy)
    assert proposed['physical_identity']['original_model_root'] == '/tmp/uav_usv_lab.MnVnZL/models'
    assert proposed['physical_identity']['manifest_sha256'] == (
        '3c64ac0b5761c25dbb51832de490c7a679e360619fa20dcf266d793fb66682cf')
    assert not proposed['new_protocol_admissible']
