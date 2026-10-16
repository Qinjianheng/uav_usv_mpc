"""Use fresh visual/KF authority for bounded takeoff without enabling low blind search."""
from types import SimpleNamespace

import pytest

from test_strict_visual_control import control_node, target
from uav_control.control.flight_guidance import FlightKinematicState
from uav_control.mission.target_visibility import VisibilityDecision
from uav_usv_interfaces.msg import MissionState


def node_for_takeoff(height=1.):
    n, s = control_node(height=height)
    n.mission_state, n.mission_state_name = MissionState.TAKEOFF, 'TAKEOFF'
    n.takeoff_visual_tracking = True
    n.vehicle_armed, n.offboard_active = True, True
    n.vehicle_status_stamp, n.vehicle_status_timeout = s.stamp, 2.
    n.flight_guidance.command('GROUND_HOLD', FlightKinematicState(
        (1., 2., 0.), (0., 0., 0.)), None, .05)
    n.visibility_decision = VisibilityDecision('TRACKING', True, True, .4, 0)
    n.visibility.last_valid_image_bearing = .4
    n.latest_kf_message = target(now=s.stamp, source=s.stamp-.03)
    n.latest_kf_message.position = SimpleNamespace(x=1., y=12., z=0.)
    n.latest_kf_message.velocity = SimpleNamespace(x=0., y=4., z=0.)
    return n, s


def command(n, s):
    method = getattr(n, '_flight_guidance_command', None)
    assert callable(method), 'Tracker must route takeoff through fresh target admission'
    return method(FlightKinematicState(s.position, s.velocity), s.stamp, .05)


def test_takeoff_routes_fresh_locked_kf_into_existing_height_blend():
    n, s = node_for_takeoff()
    c = command(n, s)
    assert c.velocity[1] > 0.
    assert abs(c.velocity[1]) <= 1.5*.05+1e-9
    assert c.velocity[2] < 0.


@pytest.mark.parametrize('source,valid,locked', [
    (9., True, True), (10.21, True, True),
    (10.17, False, True), (10.17, True, False),
])
def test_takeoff_never_follows_stale_future_invalid_or_unlocked_target(source, valid, locked):
    n, s = node_for_takeoff()
    n.latest_kf_message = target(now=s.stamp, source=source)
    n.latest_kf_message.valid = valid
    n.visibility_decision = VisibilityDecision('TRACKING', locked, True, .4, 0)
    c = command(n, s)
    assert c.velocity[:2] == (0., 0.)
    assert c.velocity[2] < 0.


def test_takeoff_keeps_horizontal_zero_below_existing_half_metre_gate():
    n, s = node_for_takeoff(.4)
    assert command(n, s).velocity[:2] == (0., 0.)


def test_disabled_takeoff_feature_preserves_previous_stationary_climb():
    n, s = node_for_takeoff()
    n.takeoff_visual_tracking = False
    assert command(n, s).velocity[:2] == (0., 0.)


def test_visible_takeoff_can_center_camera_above_horizontal_start_height():
    n, s = node_for_takeoff(.6)
    c = SimpleNamespace()
    n._final_yaw(c, s, .05)
    assert c.yawspeed == pytest.approx(.4)
    assert n.yaw_owner == 'VISION'


@pytest.mark.parametrize('height,visible,phase', [
    (.6, False, MissionState.TAKEOFF),
    (.6, True, MissionState.GROUND_HOLD),
])
def test_takeoff_visual_exception_never_enables_ground_or_blind_low_search(height, visible, phase):
    n, s = node_for_takeoff(height)
    n.mission_state = phase
    n.visibility_decision = VisibilityDecision('SEARCH', False, visible, .4, 1)
    c = SimpleNamespace()
    n._final_yaw(c, s, .05)
    assert c.yawspeed == 0.
    assert n.yaw_owner == 'HOLD'


def test_unlocked_visible_takeoff_yaw_retains_existing_search_rate_limit():
    n, s = node_for_takeoff(.6)
    n.visibility_decision = VisibilityDecision('SEARCH', False, True, .4, 1)
    n.visibility.last_valid_image_bearing = 1.2
    c = SimpleNamespace()
    n._final_yaw(c, s, .05)
    assert c.yawspeed == pytest.approx(n.visibility.config.maximum_search_yaw_rate)


@pytest.mark.parametrize('stamp', [10.06, 10.21, 0.])
def test_takeoff_visible_flag_cannot_hide_stale_future_or_missing_bearing(stamp):
    n, s = node_for_takeoff(.6)
    n.visibility.last_valid_bearing_stamp = stamp
    c = SimpleNamespace()
    n._final_yaw(c, s, .05)
    assert c.yawspeed == 0.


def test_armed_takeoff_visual_yaw_starts_before_target_exits_low_height_fov():
    n, s = node_for_takeoff(.05)
    c = SimpleNamespace()
    n._final_yaw(c, s, .05)
    assert c.yawspeed == pytest.approx(.4)
    assert n.yaw_owner == 'VISION'


@pytest.mark.parametrize('armed,offboard,status_stamp', [
    (False, True, 10.2), (True, False, 10.2),
    (True, True, 0.), (True, True, 7.), (True, True, 10.3),
])
def test_low_takeoff_visual_yaw_requires_current_armed_offboard_authority(
        armed, offboard, status_stamp):
    n, s = node_for_takeoff(.4)
    n.vehicle_armed, n.offboard_active = armed, offboard
    n.vehicle_status_stamp = status_stamp
    c = SimpleNamespace()
    n._final_yaw(c, s, .05)
    assert c.yawspeed == 0.
    assert n.yaw_owner == 'HOLD'
