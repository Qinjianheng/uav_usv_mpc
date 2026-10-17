"""Mission follow restoration and consistent safe yaw wait regressions."""

from dataclasses import replace
from types import SimpleNamespace

import pytest

from uav_control.mission.mission_manager import MissionManagerCore
from uav_control.mission.mission_manager import MissionPhase
from uav_control.mission.target_visibility import TargetVisibilityState
from uav_control.mission.target_visibility import VisibilityDecision
from uav_usv_interfaces.msg import MissionState

from test_strict_visual_control import control_node, target


def acquired_mission():
    core = MissionManagerCore()
    core.set_flight_ready(True)
    assert core.handle_command('X', 1.)
    assert core.mark_takeoff_complete(2.)
    core.observe_visibility(core.mission_id, 'TARGET_LOCK', True, 2.1)
    assert core.target_locked
    return core


def test_follow_does_not_wait_for_a_separate_lock_phase():
    core = acquired_mission()
    assert core.tick(2.11) == MissionPhase.FOLLOW
    assert core.tick(2.16) == MissionPhase.FOLLOW
    assert core.target_locked
    assert not core.intercept_requested


def test_unlocked_follow_queues_y_without_granting_intercept_authority():
    core = acquired_mission()
    core.observe_visibility(core.mission_id, 'TARGET_ACQUIRE', False, 2.12)
    assert core.tick(2.2) == MissionPhase.FOLLOW
    assert core.handle_command('Y', 2.3)
    assert core.phase == MissionPhase.FOLLOW


def test_old_plan_acceptance_cannot_skip_relock_confirmation_publication():
    core = acquired_mission()
    core.tick(2.2)
    assert core.handle_command('Y', 2.3)
    core.observe_visibility(core.mission_id, 'REACQUIRE', False, 3.)
    core.observe_visibility(core.mission_id, 'TARGET_LOCK', True, 3.2)
    assert not core.observe_tracker(
        core.mission_id, 1, 'TRACKING', .01, 1.5, 3.21,
    )
    assert core.phase == MissionPhase.TARGET_LOCK
    assert core.tick(3.3) == MissionPhase.FAR_GUIDANCE


def test_safe_wait_geometry_alone_cannot_start_intercept_before_y():
    core = acquired_mission()
    core.phase = MissionPhase.SAFE_WAIT
    assert core.tick(2.2, far_guidance_available=True) == (
        MissionPhase.SAFE_WAIT)


def test_y_in_follow_requires_current_visual_lock():
    core = acquired_mission()
    core.tick(2.2)
    assert core.phase == MissionPhase.FOLLOW
    core.target_locked = False
    assert core.handle_command('Y', 2.3)
    assert core.phase == MissionPhase.FOLLOW
    core.target_locked = True
    core.observe_visibility(core.mission_id, 'TARGET_LOCK', True, 2.4)
    assert core.phase == MissionPhase.FAR_GUIDANCE


@pytest.mark.parametrize('intercept_requested,destination', [
    (False, MissionPhase.FOLLOW), (True, MissionPhase.FAR_GUIDANCE),
])
def test_follow_loss_and_relock_preserve_intercept_intent(
    intercept_requested, destination,
):
    core = acquired_mission()
    core.tick(2.2)
    if intercept_requested:
        assert core.handle_command('Y', 2.3)
    core.observe_visibility(core.mission_id, 'REACQUIRE', False, 3.)
    if intercept_requested:
        assert core.phase == MissionPhase.REACQUIRE
        assert not core.handle_command('Y', 3.1)
    else:
        assert core.phase == MissionPhase.FOLLOW
    core.observe_visibility(core.mission_id, 'TARGET_LOCK', True, 3.2)
    if intercept_requested:
        assert core.phase == MissionPhase.TARGET_LOCK
        assert core.tick(3.21) == MissionPhase.TARGET_LOCK
    else:
        assert core.phase == MissionPhase.FOLLOW
    assert core.tick(3.3) == destination


def test_reacquire_timeout_stops_yaw_and_direction_until_new_rgb():
    core = TargetVisibilityState()
    for stamp in (1., 1.02, 1.04):
        core.observe(stamp, .05, True, stamp, position_valid=True)
    assert core.update(1.04, 0., 5., .02, True).locked
    for stamp in (1.06, 1.08, 1.10):
        core.observe(stamp, 0., False, stamp)
    assert core.update(1.10, 0., 5., .02, True).yaw_rate > 0.
    decision = core.update(3.11, .4, 5., .02, False)
    assert decision == VisibilityDecision('SAFE_WAIT', False, False, 0., 0)
    assert core.update(4., .4, 5., .02, False) == decision
    core.observe(4.02, .4, True, 4.02)
    decision = core.update(4.02, .4, 5., .02, False)
    assert decision.state == 'REACQUIRE'
    assert decision.visible
    assert not decision.locked
    assert decision.yaw_rate == .4


def test_safe_wait_final_arbiter_inhibits_even_inconsistent_scan_rate():
    node, state = control_node()
    node.mission_state = MissionState.SAFE_WAIT
    node.visibility_decision = VisibilityDecision(
        'SAFE_WAIT', False, False, .35, 1,
    )
    setpoint = SimpleNamespace()
    node._final_yaw(setpoint, state, .05)
    assert setpoint.yawspeed == 0.
    assert node.search_yaw_rate_command == 0.
    assert node.yaw_owner == 'HOLD'
    assert setpoint.yaw == node.current_heading


class CapturedPublisher:
    """Capture the emitted PX4 command instead of contacting DDS."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def runnable_follow_node():
    node, state = control_node()
    node.mission_state = MissionState.FOLLOW
    node.mission_state_name = 'FOLLOW'
    node._update_visibility(state, state.stamp, .05)
    node.latest_state = state
    node.latest_prediction = None
    node.last_timer_stamp = None
    node.control_counter = 0
    node.takeoff_complete_sent = True
    node._timestamp_us = lambda: 10_200_000
    node._process_pending_trajectory = lambda: None
    node._request_flight_mode = lambda: None
    node._publish_bool = lambda publisher, value: None
    node._publish_offboard_mode = lambda timestamp, velocity_control: None
    node.flight_ready_pub = node.takeoff_complete_pub = object()
    node.far_guidance_pub = object()
    node.setpoint_pub = CapturedPublisher()
    node.reference_pub = CapturedPublisher()
    node.diagnostics = []
    node._publish_diagnostic = lambda now, command, status, elapsed: (
        node.diagnostics.append((command, status)))
    return node, state


def test_follow_timer_uses_fresh_kf_for_xyz_and_rgb_for_yaw():
    node, _ = runnable_follow_node()
    # The KF target lies right; independent RGB bearing lies left.
    node.latest_kf_message.position.x = 11.
    node.latest_kf_message.position.y = 4.
    node.visibility.last_valid_image_bearing = -.1
    node.timer_callback()
    command, status = node.diagnostics[-1]
    setpoint = node.setpoint_pub.messages[-1]
    assert status == 'FOLLOW'
    assert command.velocity[0] > 0.
    assert command.velocity[1] > 0.
    assert setpoint.yawspeed == -.1
    assert node.yaw_owner == 'VISION'
    # A fresh subsequent KF sample moves the target left; XYZ must react.
    node.latest_kf_message = target(now=10.22, source=10.22)
    node.latest_kf_message.position.x = -9.
    node.latest_kf_message.position.y = -2.
    node.latest_state = replace(node.latest_state, stamp=10.22)
    node._ros_seconds = lambda: 10.22
    node.visibility.mark_observation(10.22, True, now=10.22)
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'FOLLOW'
    assert node.diagnostics[-1][0].velocity[0] < command.velocity[0]


def test_follow_timer_expired_image_preempts_translation_and_anchors_xyz():
    node, state = runnable_follow_node()
    node.latest_kf_message = target(now=10.2, source=9.9)
    node.visibility.observe(10.2, 0., False, 10.2)
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'FOLLOW'
    assert node.visibility_decision.state == 'INPUT_WAIT'
    assert command.position == state.position
    assert command.velocity == (0., 0., 0.)
    assert node.latest_target_state is None
    node.latest_state = replace(state, stamp=10.22, position=(2., 3., -5.))
    node._ros_seconds = lambda: 10.22
    node.timer_callback()
    assert node.diagnostics[-1][0].position == state.position


def test_follow_one_or_two_invalid_rgb_frames_continue_only_with_fresh_kf():
    node, _ = runnable_follow_node()
    node.timer_callback()
    for now in (10.22, 10.24):
        node.visibility.observe(now, 0., False, now)
        node.latest_state = replace(node.latest_state, stamp=now)
        node.latest_kf_message = target(now=now, source=10.2)
        node._ros_seconds = lambda now=now: now
        node.timer_callback()
        assert node.diagnostics[-1][1] == 'FOLLOW'
    node.visibility.observe(10.26, 0., False, 10.26)
    node.latest_state = replace(node.latest_state, stamp=10.26)
    node._ros_seconds = lambda: 10.26
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'FOLLOW'
    assert node.visibility_decision.state == 'REACQUIRE'
    assert not node.visibility_decision.locked
    assert not node.diagnostics[-1][0].far_guidance_available
    assert node.diagnostics[-1][0].mode == 'VELOCITY'
