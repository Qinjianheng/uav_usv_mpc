"""Native armed edges must qualify a real, physically safe Body-rate output seed."""
import json
from pathlib import Path
from types import SimpleNamespace as Row

import numpy as np
import pytest

from uav_control.common.p76_contract import load_contract
from uav_control.control.p76_bodyrate_safety import BodyrateSeaBarrier
from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker
from uav_control.control.p76_mode_coordinator import ModeCoordinator
from uav_control.control.p76_paper_thrust import CalibratedThrustMap
from uav_control.control.p76_recovery import BodyrateRecovery
from uav_control.control.p76_tracker_runtime import P76TrackerRuntime

ROOT = Path(__file__).resolve().parents[3]


def runtime_fixture():
    """Exercise the actual tick and limiter; ROS transport alone is replaced by list sinks."""
    runtime = P76TrackerRuntime.__new__(P76TrackerRuntime)
    runtime.contract = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    runtime.now, runtime.outputs, runtime.lands = 10., [], []
    runtime.node = Row(
        _ros_seconds=lambda: runtime.now, navigation_frame_fault='', vehicle_armed=False,
        vehicle_status_stamp=10., mission_state_name='GROUND_HOLD', current_heading=0.,
        _revoke_nominal_follow=lambda reason: None,
        _publish_vehicle_command=lambda *args: runtime.lands.append(args))
    runtime.mode, runtime.fatal_dispatched = ModeCoordinator(), False
    runtime.follow_revoked, runtime.initialized = False, False
    runtime.last_tick, runtime.last_report = None, 1000.
    runtime.executed_thrust, runtime.px4_mode = None, None
    runtime.land_observation = dict(status_receipt=10., status_native_wire=100,
                                    disarmed=True, landed=True)
    runtime.outer = BodyrateTracker(
        runtime.contract, 2., 60.,
        CalibratedThrustMap([0., 20., 60.], [0., .5, 1.], qualified=True))
    runtime.controller = Row(barrier=BodyrateSeaBarrier([[0., 0., 0.]], .02, .24, 3.))
    runtime.recovery = BodyrateRecovery(3., .24)
    runtime.recovery_phase, runtime.mode_wait_started = None, None
    runtime.waypoint, runtime.waypoint_state = None, None
    runtime.position, runtime.velocity = np.array([0., 0., -4.]), np.zeros(3)
    runtime.pose = Row(rotation=np.eye(3), omega=np.array([.1, 0., 0.]))
    runtime._snapshot = lambda now: (
        now, runtime.position.copy(), runtime.velocity.copy(), runtime.pose, 0.)
    runtime._emit_rates = lambda rates, thrust, now: runtime.outputs.append(
        dict(rates=np.asarray(rates).copy(), thrust=thrust, stamp=now))
    return runtime


def advance(runtime, *, armed=None, thrust=None, mode=True, seconds=.005):
    runtime.now += seconds
    if armed is not None:
        if armed and not runtime.node.vehicle_armed:
            runtime.fixture_arm_native = round(runtime.now*1e6)-2000
        runtime.node.vehicle_armed = armed
    runtime.node.vehicle_status_stamp = runtime.now
    runtime.land_observation.update(status_receipt=runtime.now, status_native_wire=101,
                                    armed_time_native=getattr(runtime, 'fixture_arm_native', 0),
                                    disarmed=not runtime.node.vehicle_armed)
    if thrust is not None:
        runtime.receive_thrust(Row(data=json.dumps(dict(
            stamp=runtime.now, native_timestamp=round(runtime.now*1e6),
            px4_timestamp=round(runtime.now*1e6), xyz=[0., 0., thrust]))))
    if mode is not None:
        flags = dict(offboard=mode, rates=mode, position=False, velocity=False,
                     attitude=False, altitude=False, climb_rate=False)
        runtime.px4_mode = runtime.now, flags
    runtime.tick()


def test_100_disarmed_zero_packets_never_seed_the_flight_limiter():
    runtime = runtime_fixture()
    for _ in range(100):
        advance(runtime)
    assert len(runtime.outputs) == 100
    assert all(row['thrust'] == 0 for row in runtime.outputs)
    assert runtime.outer.limiter.thrust is None
    assert runtime.control_state == 'DISARMED_PRESTREAM'
    assert runtime.initialization_count == 0


def test_arm_edge_seeds_once_from_native_feedback_and_first_packet_matches_it():
    runtime = runtime_fixture()
    advance(runtime)
    advance(runtime, armed=True, thrust=-.49)
    assert runtime.control_state == 'BODYRATE_READY'
    assert runtime.initialization_count == 1
    assert runtime.outputs[-1]['thrust'] == -.49
    assert runtime.outputs[-1]['rates'] == pytest.approx(runtime.pose.omega)
    advance(runtime, armed=True, thrust=-.6)
    assert runtime.control_state == 'FLIGHT_ACTIVE'
    assert runtime.initialization_count == 1
    assert abs(runtime.outputs[-1]['thrust']+.49) <= .01+1e-9
    assert runtime.lands == []


def test_delayed_mode_and_thrust_feedback_cannot_reauthorize_after_safe_land():
    runtime = runtime_fixture()
    advance(runtime)
    advance(runtime, armed=True, mode=False)
    assert runtime.control_state == 'LAND_REQUESTED'
    assert len(runtime.outputs) == 1 and runtime.initialization_count == 0
    advance(runtime, thrust=-.49, mode=True)
    assert runtime.control_state == 'LAND_REQUESTED'
    assert len(runtime.outputs) == 1 and runtime.initialization_count == 0
    assert len(runtime.lands) == 1


def test_bodyrate_owns_control_without_thrust_feedback_requests_land_immediately():
    runtime = runtime_fixture()
    advance(runtime)
    advance(runtime, armed=True)
    assert runtime.control_state == 'LAND_REQUESTED'
    assert len(runtime.outputs) == 1 and len(runtime.lands) == 1
    advance(runtime, thrust=-.49)
    assert len(runtime.lands) == 1 and runtime.initialization_count == 0


@pytest.mark.parametrize('mode', ['missing', 'old'])
def test_missing_or_expired_mode_immediately_latches_land_without_armed_output(mode):
    runtime = runtime_fixture()
    advance(runtime)
    runtime.px4_mode = None if mode == 'missing' else (runtime.now-1., {})
    advance(runtime, armed=True, thrust=-.49, mode=None)
    assert runtime.control_state == 'LAND_REQUESTED'
    assert len(runtime.lands) == 1
    for _ in range(132):
        advance(runtime, thrust=-.49, mode=None)
    assert runtime.control_state == 'LAND_REQUESTED'
    assert len(runtime.lands) == 1 and len(runtime.outputs) == 1


def test_armed_wait_is_an_explicit_unqualified_transition_with_no_output():
    runtime = runtime_fixture()
    runtime._reset_initialization()
    runtime.node.vehicle_armed = True
    assert runtime._observe_armed_edge(runtime.now)
    assert runtime.control_state == 'ARMED_WAIT_NATIVE_FEEDBACK'
    assert runtime.outer.limiter.thrust is None
    assert runtime.outputs == [] and runtime.initialization_count == 0


def test_hover_build_up_respects_slew_and_a_physical_descent_margin():
    runtime = runtime_fixture()
    advance(runtime)
    advance(runtime, armed=True, thrust=-.45)
    assert runtime.control_state == 'BODYRATE_READY'
    assert runtime.initialization_margin['ramp_seconds'] == pytest.approx(.02)
    for _ in range(30):
        advance(runtime, thrust=-.49)
    thrusts = [row['thrust'] for row in runtime.outputs[1:]]
    assert all(abs(b-a) <= .01+1e-9 for a, b in zip(thrusts, thrusts[1:]))
    assert thrusts[-1] == pytest.approx(-.49)
    assert min(thrusts) > -.5 and runtime.lands == []


def test_ground_hover_ramp_is_rejected_when_physical_clearance_cannot_cover_it():
    runtime = runtime_fixture()
    runtime.position[2] = -.03
    advance(runtime)
    advance(runtime, armed=True, thrust=-.01)
    assert runtime.control_state == 'LAND_REQUESTED'
    assert 'HOVER_RAMP_INFEASIBLE' in runtime.mode.reason
    assert runtime.initialization_count == 0 and len(runtime.outputs) == 1


def test_disarm_and_rearm_clear_old_seed_but_duplicate_armed_messages_do_not():
    runtime = runtime_fixture()
    advance(runtime)
    advance(runtime, armed=True, thrust=-.49)
    advance(runtime, armed=True, thrust=-.6)
    assert runtime.initialization_count == 1
    advance(runtime, armed=False)
    assert runtime.outer.limiter.thrust is None
    assert runtime.control_state == 'DISARMED_PRESTREAM'
    advance(runtime, armed=True, thrust=-.46)
    assert runtime.control_state == 'BODYRATE_READY'
    assert runtime.outputs[-1]['thrust'] == -.46
    assert runtime.initialization_count == 2


def test_armed_clock_rollback_invalidates_seed_and_latches_native_land():
    runtime = runtime_fixture()
    advance(runtime)
    advance(runtime, armed=True, thrust=-.49)
    count = len(runtime.outputs)
    advance(runtime, seconds=-.01, thrust=-.49)
    assert runtime.control_state == 'LAND_REQUESTED'
    assert runtime.outer.limiter.thrust is None
    assert len(runtime.outputs) == count and len(runtime.lands) == 1


def test_restart_while_armed_requires_a_new_native_seed_and_never_reuses_zero():
    runtime = runtime_fixture()
    advance(runtime, armed=True, thrust=-.45)
    assert runtime.control_state == 'BODYRATE_READY'
    assert runtime.initialization_count == 1
    assert runtime.outputs[0]['thrust'] == -.45


def test_thrust_feedback_requires_native_wire_provenance_and_rejects_zero():
    runtime = runtime_fixture()
    for values in (dict(stamp=runtime.now, xyz=[0., 0., -.49]),
                   dict(stamp=runtime.now, xyz=[0., 0., 0.],
                        px4_timestamp=1, native_timestamp=1)):
        runtime.receive_thrust(Row(data=json.dumps(values)))
        assert runtime.executed_thrust is None


@pytest.mark.parametrize('phase', ['FOLLOW_ACTIVE', 'PAPER_INTERCEPT_ACTIVE'])
@pytest.mark.parametrize('recoverable', [True, False])
def test_actual_native_overspeed_revokes_nominal_before_any_target_calculation(
        phase, recoverable):
    runtime = runtime_fixture()
    advance(runtime)
    advance(runtime, armed=True, thrust=-.49)
    runtime.mode.phase = phase
    runtime.velocity[0] = 7.01
    runtime.position[2] = -4. if recoverable else -.01
    revocations = []
    runtime.node._revoke_nominal_follow = revocations.append

    def nominal_must_not_run(*args):
        raise AssertionError('Nominal controller consumed an overspeed native state')

    runtime._follow_output = runtime._paper_output = nominal_must_not_run
    before = len(runtime.outputs)
    advance(runtime, thrust=-.49)
    assert runtime.mode.phase == 'SAFE_RECOVERY'
    assert runtime.follow_revoked
    assert revocations[0] == 'P764_NATIVE_SPEED_LIMIT_EXCEEDED'
    if recoverable:
        assert runtime.control_state == 'SAFE_RECOVERY'
        assert len(runtime.outputs) == before+1 and runtime.lands == []
    else:
        assert runtime.control_state == 'LAND_REQUESTED'
        assert len(runtime.outputs) == before and len(runtime.lands) == 1
