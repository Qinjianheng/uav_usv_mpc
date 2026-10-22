"""Measured calibration cannot be replaced by ideal points or successful DDS commands."""
import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]


def script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    previous = list(sys.path)
    sys.path.insert(0, str(ROOT/'scripts'))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path[:] = previous
    return module


def test_measured_force_curve_does_not_require_an_invented_zero_force_anchor():
    from uav_control.control.p76_paper_thrust import CalibratedThrustMap
    curve = CalibratedThrustMap([4., 20., 60.], [.25, .57, 1.], qualified=True)
    assert curve.to_body_z(20.) == pytest.approx(-.57)
    with pytest.raises(ValueError, match='BELOW_CALIBRATED'):
        curve.to_body_z(0.)


def test_empty_or_command_only_log_cannot_qualify_measured_dynamics():
    analysis = script('p76_calibration_analysis')
    result = analysis.analyze_rows([{'kind': 'command', 'phase': 'THRUST', 'thrust': .6}],
                                   mass=2., gravity=9.8)
    assert not result['A1_passed']
    assert 'NO_NATIVE_FORCE_SAMPLES' in result['reasons']


def test_manifest_rejects_a_stage_without_real_raw_evidence(tmp_path):
    manifest = script('p76_calibration_manifest')
    with pytest.raises(ValueError, match='RAW_EVIDENCE'):
        manifest.validate_stage_evidence(tmp_path, {'A1_passed': True})


def test_follow_qualification_stage_cannot_enable_y_even_with_old_follow_file(monkeypatch):
    from types import SimpleNamespace
    from uav_control.control import p76_tracker_runtime as module
    from uav_control.control.p76_mode_coordinator import ModeCoordinator
    monkeypatch.setenv('UAV_USV_P761_STAGE', 'B')
    monkeypatch.setattr(module, 'validate_follow_qualification', lambda *args: None)
    runtime = module.P76TrackerRuntime.__new__(module.P76TrackerRuntime)
    runtime.workspace, runtime.contract = str(ROOT), {}
    runtime.node = SimpleNamespace(_revoke_nominal_follow=lambda reason: None,
                                   tracker=SimpleNamespace(active_trajectory=None))
    runtime.mode = ModeCoordinator()
    runtime.publish_phase = lambda: None
    runtime.accept_y(10.)
    assert runtime.mode.phase == 'SAFE_RECOVERY'


def test_native_lift_uses_physical_acceleration_and_bounded_pose_bracket():
    analysis = script('p76_calibration_analysis')
    poses = [dict(kind='pose', stamp=t, q=[1., 0., 0., 0.]) for t in (1., 1.04)]
    state = dict(kind='state', phase='THRUST_0.6', phase_elapsed=.1, stamp=1.02,
                 a=[0., 0., -4.], v=[0., 0., 0.], age=.02)
    samples = analysis.native_force_samples(poses+[state], 2., 9.8)
    assert samples[0]['force'] == pytest.approx(27.6)
    poses[-1]['stamp'] = 1.08
    assert analysis.native_force_samples(poses+[state], 2., 9.8) == []


def test_stopped_diagnostics_cannot_qualify_live_minco_follow():
    from types import SimpleNamespace as Row
    validation = script('p761_validation')

    def stamp(t):
        return Row(sec=int(t), nanosec=round((t-int(t))*1e9))
    control = Row(stamp=stamp(10.), status='MINCO_FOLLOW', plan_id=3,
                  remaining_time=.2, source_age=.02, last_valid_observation_age=.02)
    target = Row(valid=True, stamp=stamp(10.), source_stamp=stamp(9.98))
    acks = {name: dict(schema_version=2, active=True, epoch=0, phase='FOLLOW_ACTIVE', stamp=10.,
                       ready=True, source_stamp=9.98, control_authorized=authorized)
            for name, authorized in (('spatial', True), ('image', False))}
    mode = dict(phase='FOLLOW_ACTIVE', epoch=0)
    runtime = dict(schema_version=2, phase='FOLLOW_ACTIVE', guidance_owner='MINCO_FOLLOW',
                   px4_interface='BODY_RATE', stamp=10.,
                   output=dict(stamp=10., rates=[0., 0., 0.], thrust=-.5))
    native = dict(stamp=10., flags=dict(
        offboard=True, rates=True, position=False, velocity=False, attitude=False,
        altitude=False, climb_rate=False))
    assert validation.follow_authority_fresh(
        10.02, control, target, acks, [10.], mode, runtime, native)
    assert not validation.follow_authority_fresh(
        10.2, control, target, acks, [10.], mode, runtime, native)
    acks['spatial']['epoch'] = 1
    assert not validation.follow_authority_fresh(
        10.02, control, target, acks, [10.], mode, runtime, native)


def test_real_px4_float32_thrust_relay_preserves_sample_epoch():
    import json
    from types import SimpleNamespace as Row
    from px4_msgs.msg import VehicleThrustSetpoint
    from uav_control.common.p76_native import install_odometry_relay
    callbacks, emitted = {}, []
    node = Row(create_publisher=lambda *args: Row(publish=emitted.append),
               create_subscription=lambda cls, topic, callback, qos:
               callbacks.setdefault(topic, callback),
               _ros_seconds=lambda: 10.02,
               _map_px4_pose_stamp=lambda wire, now: (wire, wire, 'NATIVE'))
    install_odometry_relay(node)
    message = VehicleThrustSetpoint(timestamp=10000000, timestamp_sample=10000000,
                                    xyz=[0., 0., -.5])
    callbacks['/fmu/out/vehicle_thrust_setpoint'](message)
    packet = json.loads(emitted[-1].data)
    assert packet['xyz'] == [0., 0., -.5]
    assert packet['native_timestamp'] == packet['px4_timestamp'] == 10000000
    assert packet['stamp'] == 10.


def test_calibration_pairs_native_state_causally_without_extrapolating_latest_pose():
    import numpy as np
    from uav_control.common.p76_native import NativePoseHistory, NativeReferenceHistory
    calibration = script('p76_bodyrate_calibration')
    poses, refs = NativePoseHistory(), NativeReferenceHistory()
    poses.add(10., [1., 0., 0., 0.], [0., 0., 0.], [0., 0., 0.])
    refs.add(10., 0.)
    states = [(t, np.zeros(3), np.zeros(3), np.zeros(3)) for t in (10., 10.008)]
    state, _, _ = calibration.paired_native_state(states, poses, refs, 10.02)
    assert state[0] == 10.
    poses.add(10.008, [1., 0., 0., 0.], [0., 0., 0.], [0., 0., 0.])
    refs.add(10.008, 0.)
    state, _, _ = calibration.paired_native_state(states, poses, refs, 10.02)
    assert state[0] == 10.008
    with pytest.raises(ValueError, match='PAIRED_NATIVE'):
        calibration.paired_native_state(states, poses, refs, 10.2)
