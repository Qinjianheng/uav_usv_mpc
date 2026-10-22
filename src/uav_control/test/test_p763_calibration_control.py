"""Calibration protects the original motion envelope before a high pulse is issued."""
import json

import numpy as np
import pytest

from test_p761_calibration import script


def control():
    from uav_control.control import p763_calibration_control
    return p763_calibration_control


def test_static_maximum_and_measured_240ms_delay_refuse_original_full_pulse():
    module = control()
    result = module.pulse_admission(
        velocity=[0., 0., 0.], acceleration=[0., 0., 0.], sample_age=.006,
        duration=.22, mass=2., maximum_thrust=58.8, gravity=9.8,
        upward_room=4., clearance=8., upward_braking=4., downward_braking=4.)
    assert not result['admitted']
    assert result['worst_response_delay'] == .24
    assert result['predicted_peak_speed'] > 7.
    assert result['speed_limit'] == 7.


def test_guard_rejects_stale_data_and_both_altitude_boundaries():
    args = dict(velocity=[0., 0., 0.], acceleration=[0., 0., 0.],
                sample_age=.01, duration=.01, mass=2., maximum_thrust=30., gravity=9.8,
                upward_room=20., clearance=20., upward_braking=4., downward_braking=4.)
    assert control().pulse_admission(**args)['admitted']
    assert not control().pulse_admission(**dict(args, sample_age=.126))['admitted']
    assert not control().pulse_admission(**dict(args, upward_room=.01))['admitted']
    assert not control().pulse_admission(**dict(args, clearance=1.99))['admitted']


def test_unqualified_bootstrap_never_claims_qualified_or_accepts_other_px4(tmp_path):
    old = dict(model={'total_mass': 2., 'gravity': 9.8}, px4={'binary_sha256': 'a'})
    (tmp_path/'identity.json').write_text(json.dumps(old))
    (tmp_path/'analysis.json').write_text(json.dumps(dict(
        hover_normalized=.503, measured_curve=[dict(force=f, normalized=u)
                                               for f, u in ((8., .3), (20., .5), (58., 1.))])))
    physical = control().load_bootstrap(tmp_path, old)
    assert physical.qualified is False
    assert physical.to_body_z(20.) == pytest.approx(-.5)
    with pytest.raises(ValueError, match='BOOTSTRAP_IDENTITY'):
        control().load_bootstrap(tmp_path, dict(old, px4={'binary_sha256': 'b'}))


def test_body_brake_explicitly_overrides_full_thrust_with_bounded_rate_request():
    mapping = control().BootstrapThrustMap([8., 20., 58.], [.3, .503, 1.])
    hold = control().CalibrationBodyHold(2., 9.8, mapping)
    hold.initialize([.2, -.1, 0.], 1.)
    result = hold.step([0., 0., -8.], [0., 0., -9.], [0., 0., -3.83],
                       np.eye(3), .005, 0., brake=True)
    assert result.thrust < .503
    assert result.explicit_high_pulse_override
    assert np.max(np.abs(result.rates)) <= 1.5
    assert result.acceleration[2] > 0.


def test_old_velocity_recovery_labels_cannot_qualify_schema2_a2():
    analysis = script('p76_calibration_analysis')
    rows = [dict(kind='fault_recovered', phase='FAULT_RECOVERY',
                 initial_velocity=[0., 0., 1.], reliable_deceleration=4.)]
    result = analysis.analyze_rows(rows, 2.)
    assert not result['checks']['bodyrate_recovery']
    assert not result['checks']['native_land_takeover']
    assert result['schema_version'] == 2


def test_old_stage_assessment_is_preserved_when_version2_is_written(tmp_path, monkeypatch):
    analysis = script('p76_calibration_analysis')
    old = {'stage': 'A1', 'completed': True}
    (tmp_path/'run_result.json').write_text(json.dumps(old))
    identity = dict(model={'total_mass': 2., 'gravity': 9.8})
    (tmp_path/'identity.json').write_text(json.dumps(identity))
    (tmp_path/'analysis.json').write_text(json.dumps({}))
    (tmp_path/'calibration.jsonl').write_text('{"kind": "command", "phase": "OLD"}\n')
    (tmp_path/'stage_assessment.json').write_text('HISTORICAL\n')
    monkeypatch.setattr(analysis, 'analyze_rows', lambda *args: dict(checks={}, A1_passed=False))
    result = analysis.assess_stage(tmp_path)
    assert not result['qualified']
    assert (tmp_path/'stage_assessment.json').read_text() == 'HISTORICAL\n'
    assert (tmp_path/'stage_assessment_v2.json').is_file()
    assert json.loads((tmp_path/'run_result.json').read_text()) == old


def formal_rows():
    """Synthetic native/input fixture proves replay integrity, not actual dynamics."""
    from test_p761_calibration import ROOT
    from uav_control.common.p76_contract import load_contract
    from uav_control.common.p76_readiness import sha256
    analysis = script('p76_calibration_analysis')
    path = ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'
    contract = load_contract(path)
    curve = [dict(force=f, normalized=u) for f, u in ((8., .3), (20., .5), (58., 1.))]
    identity = analysis.formal_controller_identity(
        contract, 2., 58., curve, [[0., 0., 0.]], 4., .24, {str(path): sha256(path)})
    outer, recovery, barrier = analysis.make_formal_controller(identity)
    outer.initialize(np.zeros(3), -.5)
    digest = analysis.formal_identity_digest(identity)
    rows = [dict(kind='formal_controller_identity', identity=identity, identity_sha256=digest)]
    for i, speed in enumerate((1., .8, .6, .4, .2, .05)):
        stamp, phase = 1.+.04*i, 'FAULT_RECOVERY'
        p, v = np.array([0., 0., -8.+.02*i]), np.array([0., 0., speed])
        now = stamp+.01
        output = recovery.step(outer, p, v, np.eye(3), np.zeros(3), now, .04, .01, barrier)
        rows += [dict(kind='pose', stamp=stamp, q=[1., 0., 0., 0.], omega=[0., 0., 0.]),
                 dict(kind='state', phase=phase, stamp=stamp, age=.01, p=p.tolist(),
                      v=v.tolist()),
                 dict(kind='command', phase=phase, stamp=now+.001, interface='body_rate',
                      rates=output['rates'].tolist(), thrust=-output['thrust']),
                 dict(kind='formal_control', phase=phase, controller_identity_sha256=digest,
                      phase_begin=1., recovery=True,
                      seed=dict(rates=[0., 0., 0.], thrust=-.5) if i == 0 else None,
                      inputs=dict(native_stamp=stamp, position=p.tolist(), velocity=v.tolist(),
                                  rotation=np.eye(3).tolist(), omega=[0., 0., 0.],
                                  now=now, dt=.04, age=.01, sea=0.), command_stamp=now+.001,
                      reference=output['reference'].tolist(),
                      acceleration=output['acceleration'].tolist())]
    rows += [dict(kind='mode', phase='FAULT_RECOVERY', flags=dict(offboard=True, rates=True))]
    return rows


def test_recovery_requires_native_bounded_response_and_body_only_commands():
    analysis = script('p76_calibration_analysis')
    rows = formal_rows()
    assert analysis.bounded_response(rows, 'FAULT_RECOVERY')['passed']
    rows.append(dict(kind='command', phase='FAULT_RECOVERY', interface='velocity'))
    assert not analysis.bounded_response(rows, 'FAULT_RECOVERY')['passed']


def test_formal_recovery_replay_detects_an_unrelated_a1_brake_output():
    analysis = script('p76_calibration_analysis')
    rows = formal_rows()
    assert analysis.formal_recovery_evidence(rows)['passed']
    commands = [r for r in rows if r['kind'] == 'command']
    commands[0]['thrust'] += .1
    assert not analysis.formal_recovery_evidence(rows)['passed']


def test_land_requires_one_request_native_takeover_motors_and_actual_landing():
    analysis = script('p76_calibration_analysis')
    rows = [dict(kind='vehicle_command', command=21, phase='LAND', stamp=1.),
            dict(kind='wire', topic='vehicle_status', phase='LAND', stamp=1.2,
                 values=dict(nav_state=18, arming_state=2)),
            dict(kind='wire', topic='vehicle_status', phase='LAND', stamp=4.,
                 values=dict(nav_state=18, arming_state=1)),
            dict(kind='wire', topic='actuator_motors', phase='LAND',
                 values=dict(control=[.5, .5, .5, .5])),
            dict(kind='wire', topic='actuator_motors', phase='LAND',
                 values=dict(control=[0., 0., 0., 0.])),
            dict(kind='wire', topic='vehicle_land_detected', phase='LAND',
                 values=dict(landed=True))]
    rows += [dict(kind='executed_collective', phase='LAND', xyz=[0., 0., u])
             for u in (-.5, -.3, 0.)]
    rows += [dict(kind='state', phase='LAND', age=.01, v=[0., 0., v])
             for v in (1., .8, .6, .3, 0.)]
    assert analysis.native_land_takeover(rows)['passed']
    assert not analysis.native_land_takeover(rows+[rows[0]])['passed']
    assert not analysis.native_land_takeover(rows[:-3])['passed']


def test_schema1_raw_cannot_be_relabelled_as_a_current_qualification(tmp_path):
    manifest = script('p76_calibration_manifest')
    for name, value in (('identity.json', {}), ('run_result.json', {'completed': True})):
        (tmp_path/name).write_text(json.dumps(value))
    (tmp_path/'calibration.jsonl').write_text('{}\n')
    with pytest.raises(ValueError, match='OLD_CALIBRATION_SCHEMA'):
        manifest.validate_stage_evidence(tmp_path, {})


def test_session_requires_separate_stage_authorization_before_simulator_or_budget(tmp_path):
    session = script('p761_session')
    with pytest.raises(ValueError, match='STAGE_AUTHORIZATION'):
        session.run(tmp_path/'session_A1_04', 'A1')
    assert not (tmp_path/'session_A1_04').exists()


def test_sequence_keeps_all_actions_and_resets_on_the_body_interface():
    import hashlib
    calibration = script('p76_bodyrate_calibration')
    node = calibration.Calibration.__new__(calibration.Calibration)
    node.stage, node.steps = 'A1', []
    # Virtual low-TWR fixture tests sequence construction only. It is never used
    # as real identification evidence or inherited by the default A1 constructor.
    admitted = control().pulse_admission(
        velocity=[0., 0., 0.], acceleration=[0., 0., 0.], sample_age=0., duration=.22,
        mass=2., maximum_thrust=30., gravity=9.8, upward_room=4., clearance=8.,
        upward_braking=4., downward_braking=4.)
    assert admitted['admitted']
    virtual_steps = []
    for value in (.3, .4, .5, .6, .7, .85, 1.):
        virtual_steps += [('BODY_PREP', .65), (f'THRUST_{value:.3f}', .22),
                          ('BODY_BRAKE', 3.), ('BODY_HOLD', 3.)]
    node.protocol = dict(
        new_protocol_admissible=True, qualified=False,
        scope='UNIT TEST virtual low-TWR safety admission; no identification qualification',
        new_protocol_source_sha256=hashlib.sha256(
            json.dumps(admitted, sort_keys=True).encode()).hexdigest(),
        executable_steps=virtual_steps)
    node.make_steps()
    assert [name for name, _ in node.steps if name.startswith('THRUST_')] == [
        'THRUST_0.300', 'THRUST_0.400', 'THRUST_0.500', 'THRUST_0.600',
        'THRUST_0.700', 'THRUST_0.850', 'THRUST_1.000']
    assert all(duration == .22 for phase, duration in node.steps if phase.startswith('THRUST_'))
    assert 'RESET' not in [phase for phase, _ in node.steps]
    for index, (phase, _) in enumerate(node.steps):
        if phase.startswith('THRUST_'):
            assert node.steps[index+1][0] == 'BODY_BRAKE'
            assert node.steps[index+2][0] == 'BODY_HOLD'
    node.body_entered = True
    with pytest.raises(ValueError, match='VELOCITY_AFTER_BODY_ENTRY'):
        node.emit_velocity([0., 0., 0.])


def test_land_is_latched_and_does_not_request_modes_repeatedly():
    calibration = script('p76_bodyrate_calibration')
    node = calibration.Calibration.__new__(calibration.Calibration)
    changes, requests = [], []
    node.land_requested = False
    node.change, node.command = changes.append, requests.append
    node.land()
    node.land()
    assert changes == ['LAND']
    assert requests == [21]


def test_e0_replays_actual_native_failure_and_requires_refusal():
    from test_p761_calibration import ROOT
    replay = script('p763_exit_replay')
    session = ROOT/'data/experiments/20261021_p761_01/session_A1_03'
    result = replay.replay(session)
    assert result['old']['exit_native_upward_speed'] == pytest.approx(3.83200145)
    assert result['old']['measured_collective_response_delay'] == pytest.approx(.24011755)
    assert result['old']['first_exceed_speed'] > 7.
    assert result['guarded']['admission']['predicted_peak_speed'] > 9.
    assert result['guarded']['explicit_body_brake_thrust'] < .503
    assert result['outcome'] == 'SAFE_REFUSAL' and result['passed']
    assert not result['qualified']
    assert not result['guarded']['maximum_pulse_executed']


def test_land_announcement_without_actual_native_motor_response_is_insufficient():
    analysis = script('p76_calibration_analysis')
    rows = [dict(kind='vehicle_command', command=21, phase='LAND', stamp=1.),
            dict(kind='wire', topic='vehicle_status', phase='LAND', stamp=1.2,
                 values=dict(nav_state=18, arming_state=2)),
            dict(kind='wire', topic='vehicle_status', phase='LAND', stamp=4.,
                 values=dict(nav_state=18, arming_state=1)),
            dict(kind='wire', topic='actuator_motors', phase='LAND', values={}),
            dict(kind='wire', topic='vehicle_land_detected', phase='LAND',
                 values=dict(landed=True))]
    rows += [dict(kind='executed_collective', phase='LAND', xyz=[0., 0., u])
             for u in (-.5, -.3, 0.)]
    assert not analysis.native_land_takeover(rows)['passed']


def test_recovery_bodyrate_labels_without_formal_core_replay_cannot_qualify():
    analysis = script('p76_calibration_analysis')
    rows = [dict(kind='state', phase='FAULT_RECOVERY', stamp=1.+.04*i, age=.01,
                 p=[0., 0., .02*i], v=[0., 0., speed])
            for i, speed in enumerate((1., .8, .6, .4, .2, .05))]
    rows += [dict(kind='command', phase='FAULT_RECOVERY', interface='body_rate'),
             dict(kind='mode', phase='FAULT_RECOVERY', flags=dict(offboard=True, rates=True)),
             dict(kind='formal_control', phase='FAULT_RECOVERY',
                  controller='P763_BODYRATE_RECOVERY')]
    assert not analysis.bounded_response(rows, 'FAULT_RECOVERY')['passed']


def test_a2_emitter_uses_real_formal_jerk_and_thrust_slew_core():
    from types import SimpleNamespace
    analysis, calibration = script('p76_calibration_analysis'), script('p76_bodyrate_calibration')
    rows = formal_rows()
    identity = rows[0]['identity']
    node = calibration.Calibration.__new__(calibration.Calibration)
    node.formal_identity = identity
    node.formal_identity_sha256 = analysis.formal_identity_digest(identity)
    node.formal_outer, node.formal_recovery, node.formal_barrier = (
        analysis.make_formal_controller(identity))
    node.formal_last_ros, node.formal_recovery_phase = None, None
    node.phase_ros, node.current_sea, node.stage = 1., 0., 'A2'
    node.last_native, node.thrust = (1.,), (1., .5)
    node.control_mode = SimpleNamespace(flag_control_rates_enabled=True,
                                        flag_control_velocity_enabled=False)
    node.rows = []
    node.now = lambda: 1.01
    node.publish_body = lambda rates, thrust: node.rows.append(dict(
        kind='command', stamp=1.01, rates=rates.tolist(), thrust=thrust))
    node.log = lambda kind, **data: node.rows.append(dict(kind=kind, **data))
    pose = SimpleNamespace(rotation=np.eye(3), omega=np.zeros(3))
    node.emit_formal(pose, np.array([0., 0., -8.]), np.array([0., 0., 2.]), brake=True)
    command, trace = node.rows
    assert trace['recovery']
    assert np.linalg.norm(trace['acceleration']) <= 3.*.005+1e-9
    assert abs(command['thrust']-.5) <= 2.*.005+1e-9
    assert trace['controller_identity_sha256'] == analysis.formal_identity_digest(identity)
