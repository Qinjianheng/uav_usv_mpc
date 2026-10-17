"""Visual authority changes must not create position pullback or speed steps."""

from dataclasses import replace
import math

import pytest

from test_bearing_approach import approach_node
from test_strict_visual_control import target
from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState
from uav_control.evaluation.run_metrics import RunMetricAccumulator
from uav_control.mission.target_visibility import TargetVisibilityState
from uav_usv_interfaces.msg import MissionState


@pytest.mark.parametrize('phase', [MissionState.TARGET_ACQUIRE, MissionState.TARGET_LOCK])
def test_first_fresh_depth_sample_keeps_rgb_motion_until_follow_handover(phase):
    node, _ = approach_node()
    node.timer_callback()
    previous = node.diagnostics[-1][0].velocity
    node.latest_kf_message = target(now=10.2, source=10.2)
    node.visibility.mark_observation(10.2, True, 10.2)
    node.mission_state = phase
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'BEARING_APPROACH'
    assert node.bearing_approach_active
    assert command.mode == 'VELOCITY'
    assert node.offboard_modes == [True, True]
    assert math.dist(command.velocity[:2], previous[:2]) == pytest.approx(.15)
    assert math.atan2(command.velocity[1], command.velocity[0]) == pytest.approx(.05)


def test_expired_kf_after_follow_lock_brakes_with_independently_fresh_rgb_yaw():
    node, state = approach_node()
    node.mission_state = MissionState.FOLLOW
    node.mission_state_name = 'FOLLOW'
    node.latest_kf_message = target(now=10.2, source=10.2)
    node.visibility.mark_observation(10.2, True, 10.2)
    node.flight_guidance.previous_velocity = (6., 0., 0.)
    node.timer_callback()
    previous = node.diagnostics[-1][0].velocity
    node._ros_seconds = lambda: 10.33
    node.latest_state = replace(state, stamp=10.33, velocity=(5.8, 0., 0.))
    node.visibility.observe(10.32, .05, True, 10.33)
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'FOLLOW'
    assert not node.bearing_approach_active
    assert node.visibility_decision.state == 'INPUT_WAIT'
    assert node.visibility_decision.visual_locked
    assert node.yaw_owner == 'VISION'
    assert node.latest_target_state is None
    assert not node.visibility_decision.locked
    assert command.mode == 'VELOCITY'
    assert all(node.offboard_modes)
    assert math.dist(command.velocity[:2], previous[:2]) <= 3. * .13 + 1e-9
    assert math.hypot(*command.velocity[:2]) < math.hypot(*previous[:2])


def test_moving_visual_loss_brakes_without_capturing_old_position_anchor():
    node, state = approach_node(bearing=0.)
    node.flight_guidance.previous_velocity = (6., 0., 0.)
    node.latest_state = replace(state, velocity=(6., 0., 0.))
    node.timer_callback()
    node.visibility.observe(10.25, 0., False, 10.25)
    node._ros_seconds = lambda: 10.25
    node.latest_state = replace(state, stamp=10.25, position=(4., 2., -5.),
                                velocity=(5.8, 0., 0.))
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'VISUAL_BRAKING'
    assert command.mode == 'VELOCITY'
    assert command.velocity == pytest.approx((5.85, 0., 0.))
    assert node.search_hold_position is None
    assert node.offboard_modes == [True, True]
    assert all(math.isnan(v) for v in node.setpoint_pub.messages[-1].position)


def test_search_anchor_is_created_at_stop_and_then_remains_fixed():
    node, state = approach_node()
    node.visibility.observe(10.201, 0., False, 10.201)
    node.flight_guidance.previous_velocity = (6., 0., 0.)
    position = [1., 2., -5.]
    velocity = (6., 0., 0.)
    for index in range(42):
        now = 10.25 + .05 * index
        node._ros_seconds = lambda now=now: now
        node.latest_state = replace(state, stamp=now, position=tuple(position),
                                    velocity=velocity)
        node.timer_callback()
        command = node.diagnostics[-1][0]
        assert command.mode == 'VELOCITY'
        assert 0. <= command.velocity[0] <= velocity[0]
        assert velocity[0] - command.velocity[0] <= .15 + 1e-8
        if velocity[0] > .1:
            assert node.search_hold_position is None
        velocity = command.velocity
        position[0] += velocity[0] * .05
    anchor = node.search_hold_position
    assert anchor == pytest.approx((6.85, 2., -5.))
    node._ros_seconds = lambda: 12.35
    node.latest_state = replace(state, stamp=12.35, position=(7.05, 2., -5.))
    node.timer_callback()
    assert node.search_hold_position == anchor
    assert node.diagnostics[-1][0].velocity[0] == pytest.approx(-.15)
    assert all(node.offboard_modes)


def test_follow_first_command_is_shaped_from_actual_velocity():
    guidance = FlightGuidanceCore()
    command = guidance.command(
        'FOLLOW', FlightKinematicState((0., 0., -5.), (0., 0., 0.)),
        FlightKinematicState((100., 0., 0.), (4., 0., 0.)), .05,
    )
    assert command.mode == 'VELOCITY'
    assert command.velocity == pytest.approx((.15, 0., 0.))


def test_rgb_resumption_preserves_last_braking_command_instead_of_resetting():
    node, state = approach_node(bearing=0.)
    node.flight_guidance.previous_velocity = (5.85, 0., 0.)
    # The physical vehicle can lag its last command. Reinitializing each RGB
    # segment from measured velocity would introduce an unbounded command jump.
    node.latest_state = replace(state, velocity=(4., 0., 0.))
    node.timer_callback()
    assert node.diagnostics[-1][0].velocity[0] == pytest.approx(6.)


def test_braking_travel_is_separate_from_drift_after_search_anchor():
    metrics = RunMetricAccumulator(1.)
    metrics.observe(1., 'REACQUIRE', position=(0., 0., -5.),
                    control_status='VISUAL_BRAKING')
    metrics.observe(2., 'REACQUIRE', position=(4.5, 0., -5.),
                    control_status='VISUAL_BRAKING')
    metrics.observe(3., 'REACQUIRE', position=(6., 0., -5.),
                    control_status='REACQUIRE')
    metrics.observe(4., 'REACQUIRE', position=(6.2, 0., -5.),
                    control_status='REACQUIRE')
    summary = metrics.summary(5.)
    assert summary['visual_braking_duration'] == 2.
    assert summary['search_xy_drift_max'] == pytest.approx(.2)


def test_recovery_handover_reinitializes_follow_from_actual_stopped_velocity():
    node, state = approach_node()
    node.flight_guidance.previous_velocity = (6., 0., 0.)
    node.latest_state = replace(state, velocity=(6., 0., 0.))
    node.timer_callback()
    node.safe_recovery_latched = True
    node._ros_seconds = lambda: 10.25
    node.latest_state = replace(state, stamp=10.25, position=(1., 2., -.2))
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'SAFE_RECOVERY'
    node._ros_seconds = lambda: 10.3
    node.latest_state = replace(state, stamp=10.3)
    node.latest_kf_message = target(now=10.3, source=10.3)
    node.visibility.mark_observation(10.3, True, 10.3)
    node.mission_state = MissionState.FOLLOW
    node.mission_state_name = 'FOLLOW'
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'FOLLOW'
    assert math.hypot(*node.diagnostics[-1][0].velocity[:2]) == pytest.approx(.15)


def test_safe_wait_relock_before_mission_callback_keeps_mode_and_setpoint_consistent():
    node, state = approach_node()
    node.mission_state = MissionState.SAFE_WAIT
    node.mission_state_name = 'SAFE_WAIT'
    node.latest_kf_message = target(now=10.2, source=10.2)
    node.visibility.mark_observation(10.2, True, 10.2)
    node.flight_guidance.previous_velocity = (6., 0., 0.)
    node.latest_state = replace(state, velocity=(6., 0., 0.))
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert node.visibility_decision.locked  # Mission callback has not advanced yet.
    assert status == 'NO_VALID_PLAN'
    assert command.mode == 'VELOCITY'
    assert command.velocity == pytest.approx((5.85, 0., 0.))
    assert node.offboard_modes[-1]
    assert all(math.isnan(v) for v in node.setpoint_pub.messages[-1].position)


def test_three_fresh_offcenter_frames_grant_direction_motion_but_not_position_lock():
    node, _ = approach_node()
    node.visibility = TargetVisibilityState()
    for value in (10., 10.1, 10.2):
        node.visibility.observe(value, .35, True, value)
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'BEARING_APPROACH'
    assert command.velocity[0] > 0.
    assert command.velocity[1] > 0.
    assert not node.visibility_decision.locked
    assert node.visibility.consecutive_valid_frames == 0  # Strict centered lock remains separate.
    assert not command.far_guidance_available


def test_crossing_lock_center_threshold_changes_motion_continuously():
    node, state = approach_node()
    node.flight_guidance.previous_velocity = (5.9, .8, 0.)
    node.visibility.observe(10.201, .149, True, 10.201)
    node._ros_seconds = lambda: 10.201
    node.latest_state = replace(state, stamp=10.201, velocity=(5.9, .8, 0.))
    node.timer_callback()
    before = node.diagnostics[-1][0]
    node.visibility.observe(10.251, .151, True, 10.251)
    node._ros_seconds = lambda: 10.251
    node.latest_state = replace(state, stamp=10.251, velocity=before.velocity)
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'BEARING_APPROACH'
    assert command.velocity[0] > 5.8
    assert math.dist(command.velocity[:2], before.velocity[:2]) <= .15 + 1e-8
    assert node.search_hold_position is None


@pytest.mark.parametrize('bearing', [-.35, .35])
def test_direction_velocity_follows_rgb_ray_with_continuous_speed_reduction(bearing):
    core = FlightGuidanceCore()
    state = FlightKinematicState((0., 0., -5.), (0., 0., 0.))
    for _ in range(80):
        command = core.bearing_approach(state, .4, 6., .05, bearing=bearing)
    assert math.hypot(*command.velocity[:2]) == pytest.approx(5.636236277)
    assert math.atan2(command.velocity[1], command.velocity[0]) == pytest.approx(.4 + bearing)
    assert command.velocity[2] == 0.


def test_direction_frame_history_cannot_survive_loss_duplicate_or_stale_acquisition():
    core = TargetVisibilityState()
    for stamp in (10., 10.1, 10.2):
        core.observe(stamp, .35, True, stamp)
    assert core.bearing_approach_ready(10.2, .125)
    assert not core.bearing_approach_ready(10.326, .125)
    core.observe(10.21, 0., False, 10.21)
    core.observe(10.22, .35, True, 10.22)
    for _ in range(3):
        core.observe(10.22, .35, True, 10.22)
    assert not core.bearing_approach_ready(10.22, .125)
    core.observe(10.27, .35, True, 10.27)
    assert not core.bearing_approach_ready(10.27, .125)
    core.observe(10.32, .35, True, 10.32)
    assert core.bearing_approach_ready(10.32, .125)


def test_offcenter_direction_frames_cannot_lock_even_with_fresh_kf():
    core = TargetVisibilityState()
    for stamp in (10., 10.1, 10.2):
        core.observe(stamp, .35, True, stamp, position_valid=True)
    assert core.bearing_approach_ready(10.2, .125)
    assert not core.update(10.2, 0., 5., .05, True).locked
