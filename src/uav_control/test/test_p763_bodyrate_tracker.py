"""Physical reference tracking and same-interface Y authority boundaries."""
from pathlib import Path

import numpy as np
import pytest

from uav_control.common.p76_contract import load_contract

ROOT = Path(__file__).resolve().parents[3]


def test_follow_feedback_sign_and_hover_thrust_direction():
    from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker
    from uav_control.control.p76_paper_thrust import CalibratedThrustMap
    c = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    control = BodyrateTracker(c, 2., 60.,
                              CalibratedThrustMap([0., 20., 60.], [0., .5, 1.], qualified=True))
    control.initialize([0., 0., 0.], -.49)
    ref = np.zeros(11)
    ref[0] = 1.
    out = control.step(ref, np.zeros(3), np.zeros(3), np.eye(3), .005, np.zeros(3))
    assert out['acceleration'][0] > 0
    assert out['rates'][1] < 0  # FRD nose down accelerates NED +x.
    assert -1 <= out['thrust'] < 0
    assert np.linalg.norm(out['acceleration']) <= 3.*.005+1e-9


def test_reference_sampler_rejects_expired_lease_instead_of_repeating_20hz_velocity():
    from types import SimpleNamespace
    from uav_control.control.p76_minco_bodyrate_tracker import sample_reference
    called = []
    curve = SimpleNamespace(start=10., end=11., sample=lambda t: called.append(t) or np.zeros(11))
    sample_reference(curve, 10.025, 10.1)
    sample_reference(curve, 10.030, 10.1)
    assert called == [10.025, 10.030]
    with pytest.raises(ValueError, match='LEASE'):
        sample_reference(curve, 10.1, 10.1)


def test_first_y_limiter_holds_seed_then_bounds_rates_and_thrust():
    from uav_control.control.p76_minco_bodyrate_tracker import OutputLimiter
    c = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    limiter = OutputLimiter(c)
    limiter.initialize([.2, -.1, .05], -.503)
    rates, thrust = limiter.apply([1.5, 1.5, 1.2], -.9, .005)
    assert np.linalg.norm(rates-np.array([.2, -.1, .05])) <= .015+1e-9
    assert abs(thrust+.503) <= .01+1e-9


def test_waypoint_reference_has_consistent_position_velocity_acceleration():
    from uav_control.control.p76_minco_bodyrate_tracker import WaypointReference
    curve = WaypointReference(10., [0., 0., 0.], [0., 0., 0.], [1., 2., -4.], 8., .3)
    center, left, right = (curve.sample(t) for t in (14., 14.-1e-4, 14.+1e-4))
    assert (right[:3]-left[:3])/(2e-4) == pytest.approx(center[3:6], abs=1e-7)
    assert (right[3:6]-left[3:6])/(2e-4) == pytest.approx(center[6:9], abs=1e-7)
    assert curve.sample(18.)[:9] == pytest.approx([1., 2., -4., 0., 0., 0., 0., 0., 0.])


def test_warm_y_handover_uses_no_target3d_and_preserves_command_slew():
    from types import SimpleNamespace as Row
    from uav_control.control.p76_tracker_runtime import P76TrackerRuntime
    from uav_control.control.p76_mode_coordinator import ModeCoordinator
    from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker
    from uav_control.control.p76_paper_thrust import CalibratedThrustMap
    from uav_control.common.p76_native import NativePoseHistory, NativeReferenceHistory
    from uav_usv_interfaces.msg import TargetImageState
    from builtin_interfaces.msg import Time
    c = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    runtime = P76TrackerRuntime.__new__(P76TrackerRuntime)
    runtime.contract = c
    runtime.node = Row(follow_navigation_history=[dict(stamp=10., p=[0, 0, -4], v=[4, 0, 0])],
                       follow_receiver=Row(active=None, pending=None))
    runtime.mode = ModeCoordinator()
    runtime.mode.accept_y(10., True)
    runtime.follow_revoked, runtime.initialized = True, False
    runtime.image = TargetImageState(valid=True, mode_epoch=1,
                                     model_sha256=c['detector']['model_sha256'])
    runtime.image.stamp = runtime.image.pose_stamp = Time(sec=10)
    runtime.image.source_stamp = Time(sec=9, nanosec=990000000)
    runtime.image_ack = dict(schema_version=2, epoch=1, active=True, ready=True,
                             control_authorized=True, stamp=10.005)
    runtime.executed_thrust = (10., np.array([0., 0., -.51]))
    runtime.history, runtime.references = NativePoseHistory(), NativeReferenceHistory()
    runtime.history.add(10., [1., 0., 0., 0.], [0., 0., 0.], [0., 0., 0.])
    runtime.references.add(10., 0.)
    runtime.outer = BodyrateTracker(c, 2., 60.,
                                    CalibratedThrustMap([0., 20., 60.], [0., .5, 1.],
                                                        qualified=True))
    runtime.outer.initialize([.2, 0., 0.], -.5)
    initialized = []
    runtime.controller = Row(barrier=Row(
        sea=0.), initialize=lambda *args: initialized.append(args))
    runtime._bodyrate_confirmed = lambda now: True
    runtime.px4_mode = (9.99, {})
    output = runtime._paper_output(10.01, .005)
    assert len(initialized) == 1
    assert runtime.mode.guidance_owner == 'PAPER'
    assert np.linalg.norm(output['rates']-[.2, 0., 0.]) <= .015+1e-9
    assert abs(output['thrust']+.5) <= .01+1e-9


def test_legacy_supervisor_cannot_publish_velocity_heartbeat_in_p763():
    from types import SimpleNamespace as Row
    from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
    calls = []
    node = Row(p76=object(), offboard_pub=Row(publish=calls.append))
    TrajectoryTrackerNode._publish_offboard_mode(node, 100, velocity_control=True)
    assert calls == []


def test_rgbd_observations_continue_after_y_for_the_spatial_estimator():
    from types import SimpleNamespace as Row
    from uav_control.perception.rgbd_target_localizer import RgbdTargetLocalizer
    calls = []
    node = Row(p76_phase='PAPER_INTERCEPT_ACTIVE', pending_image_pairs=[],
               waiting_image_pair=None, localize=lambda: calls.append('observation'),
               compute_time_pub=Row(publish=lambda message: None))
    RgbdTargetLocalizer.timed_localize(node)
    assert calls == ['observation']


def test_contract_rejects_unbounded_or_unstable_outer_loop_candidates():
    from copy import deepcopy
    from uav_control.common.p76_contract import _validate
    c = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    for key, value in (('kp', -1.), ('kv', 0.), ('maximum_tilt', 1.6),
                       ('maximum_acceleration', 9.8), ('maximum_thrust_slew', 0.)):
        bad = deepcopy(c)
        bad['bodyrate_follow'][key] = value
        with pytest.raises(ValueError, match='P763_FOLLOW'):
            _validate(bad)


def test_body_recovery_holds_stop_position_and_can_reset_for_new_safety_entry():
    from types import SimpleNamespace as Row
    from uav_control.control.p76_recovery import BodyrateRecovery
    recovery = BodyrateRecovery(2., .24)
    references = []
    outer = Row(settings={'maximum_acceleration': 5.},
                step=lambda reference, *args: references.append(reference) or reference)
    barrier = Row(check=lambda *args: 'CLEAR')
    recovery.check_margin = lambda *args: 'CLEAR'
    recovery.step(outer, [0., 0., -4.], [2., 0., 0.], np.eye(3), np.zeros(3),
                  10., .005, .02, barrier)
    recovery.step(outer, [1., 0., -4.], [0., 0., 0.], np.eye(3), np.zeros(3),
                  11., .005, .02, barrier)
    assert references[-1][:3] == pytest.approx([1., 0., -4.])
    recovery.reset()
    recovery.step(outer, [2., 0., -4.], [1., 0., 0.], np.eye(3), np.zeros(3),
                  20., .005, .02, barrier)
    assert recovery.started == 20.


def test_delayed_native_feedback_does_not_create_error_on_exact_constant_speed_track():
    from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker
    from uav_control.control.p76_paper_thrust import CalibratedThrustMap
    c = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    outer = BodyrateTracker(c, 2., 60.,
                            CalibratedThrustMap([0., 20., 60.], [0., .5, 1.], qualified=True))
    outer.initialize([0., 0., 0.], -.49)
    execution, feedback = np.zeros(11), np.zeros(11)
    execution[0], execution[3] = 4., 4.
    feedback[0], feedback[3] = 3.6, 4.  # Native sample is 100 ms older.
    output = outer.step(execution, [3.6, 0., 0.], [4., 0., 0.], np.eye(3), .005,
                        np.zeros(3), feedback_reference=feedback)
    assert output['acceleration'] == pytest.approx(np.zeros(3))


def test_safety_margin_does_not_credit_instantaneous_jerk_or_collective_braking():
    from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker
    from uav_control.control.p76_paper_thrust import CalibratedThrustMap
    from uav_control.control.p76_recovery import BodyrateRecovery
    from uav_control.control.p76_bodyrate_safety import BodyrateSeaBarrier
    c = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    outer = BodyrateTracker(c, 2., 60.,
                            CalibratedThrustMap([0., 20., 60.], [0., .5, 1.], qualified=True))
    outer.previous_acceleration = np.array([0., 0., 2.])
    recovery = BodyrateRecovery(3., .24)
    barrier = BodyrateSeaBarrier([[0., 0., 0.]], .02, .24, 3.)
    p, v, a = np.array([0., 0., -2.5]), np.array([0., 0., 2.]), np.array([0., 0., 2.])
    assert barrier.check(p, v, np.eye(3), a, .02, .005) == 'CLEAR'
    assert recovery.check_margin(outer, p, v, np.eye(3), a, .02, .005,
                                 barrier) == 'UNRECOVERABLE'


def test_disarmed_startup_waits_for_native_callbacks_without_latching_recovery():
    from types import SimpleNamespace as Row
    from uav_control.control.p76_tracker_runtime import P76TrackerRuntime
    from uav_control.control.p76_mode_coordinator import ModeCoordinator
    runtime = P76TrackerRuntime.__new__(P76TrackerRuntime)
    runtime.node = Row(_ros_seconds=lambda: 10., vehicle_armed=False, navigation_frame_fault='')
    runtime.contract = {'paper': {'paper_controller_target_rate_hz': 200.}}
    runtime.mode, runtime.fatal_dispatched = ModeCoordinator(), False
    runtime.last_tick, runtime.last_report = None, 10.
    runtime.outer = Row(initialize=lambda *args: None)
    runtime._snapshot = lambda now: (_ for _ in ()).throw(ValueError('NOT_YET_DELIVERED'))
    outputs = []
    runtime._emit_rates = lambda *args: outputs.append(args)
    runtime.tick()
    assert runtime.mode.phase == 'FOLLOW_ACTIVE'
    assert len(outputs) == 1 and outputs[0][1] == 0.


def test_forged_positive_gate_booleans_with_untyped_hashes_cannot_admit_a1(tmp_path):
    import json
    from test_p761_calibration import script
    from uav_control.common.p76_readiness import source_fingerprint, sha256
    session = script('p761_session')
    evidence = tmp_path/'unrelated.txt'
    evidence.write_text('This is not E0/E1/E2 evidence.\n')
    manifest = tmp_path/'gates.json'
    manifest.write_text(json.dumps(dict(
        schema_version=2, architecture='parallel_dkf_unified_bodyrate',
        sources=source_fingerprint(ROOT),
        contract_sha256=sha256(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'),
        gates=dict(E0=True, E1=True, E2=True), high_pulse_admissible=True,
        raw_evidence={str(evidence): sha256(evidence)})))
    with pytest.raises(ValueError, match='TYPED_OFFLINE_EVIDENCE_REQUIRED'):
        session.validate_offline_gates(manifest)
