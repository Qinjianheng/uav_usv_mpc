"""Terminal execution owns an accepted plan without relabeling stale input."""

from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from test_follow_wait import runnable_follow_node
from test_strict_visual_control import stamp, target
from test_trajectory_tracking import linear_trajectory
from uav_control.control.trajectory_tracking import PolynomialSegmentData
from uav_control.control.trajectory_tracking import TrajectoryRejectReason
from uav_control.mission.mission_manager import MissionManagerCore, MissionPhase
from uav_usv_interfaces.msg import MissionState


def terminal_node(end=11.):
    node, state = runnable_follow_node()
    now = 10.4
    node.mission_state = MissionState.TERMINAL_MINCO
    node.mission_state_name = 'TERMINAL_MINCO'
    node.terminal_mode_latched = True
    node.intercept_requested = True
    node.use_velocity_control = True
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.terminal_cruise_enabled = True
    node.latest_state = replace(state, stamp=now, position=(2.6, 0., -.5),
                                velocity=(6.5, 0., 0.))
    node._ros_seconds = lambda: now
    node.latest_kf_message = target(now, now)
    node.latest_kf_message.position = SimpleNamespace(x=3.8, y=0., z=0.)
    node.latest_kf_message.velocity = SimpleNamespace(x=4., y=0., z=0.)
    for t in (10.25, 10.3, now):
        node.visibility.observe(t, 0., True, t, position_valid=True)
    node.latest_prediction = SimpleNamespace(
        valid=True, mission_id=1, observation_stamp=stamp(now),
        source_stamp=stamp(now), source='tracking', frame_id='local_ned',
        samples=[SimpleNamespace(relative_time=stamp(t), position=SimpleNamespace(
            x=3.8 + 4. * t, y=0., z=0.)) for t in (0., 4.)],
    )
    coefficients = (0., 6.5, 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0.,
                    -.5, 0., 0., 0., 0., 0.)
    node.tracker.active_trajectory = replace(
        linear_trajectory(1), source_stamp=10., generated_stamp=10.,
        valid_until=end, contact_stamp=end, terminal_mode=True,
        observation_stamp=now, target_state_source='tracking',
        segments=(PolynomialSegmentData(end - 10., coefficients),),
    )
    node.tracker.previous_command_velocity = (6.5, 0., 0.)
    node.tracker.previous_command_stamp = now - .05
    return node


def advance(node, now):
    node._ros_seconds = lambda: now
    node.latest_state = replace(node.latest_state, stamp=now,
                                position=(6.5 * (now - 10.), 0., -.5))


def test_terminal_depth_loss_preserves_speed_but_not_visual_lock():
    node = terminal_node()
    node.timer_callback()
    node.latest_prediction = None
    node.latest_kf_message = None
    advance(node, 10.6)
    node.visibility.mark_observation(10.6, False, now=10.6)
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'TERMINAL_COMMITTED'
    assert math.hypot(*command.velocity[:2]) == pytest.approx(6.5)
    assert not node.visibility_decision.locked
    assert node.latest_target_state is None
    assert not node.safe_recovery_latched
    assert node.search_state == 'TERMINAL_COMMITTED'
    assert node.tracker.active_trajectory is not None


def test_plan_accepted_before_terminal_phase_can_enter_commitment():
    node = terminal_node()
    original = replace(node.tracker.active_trajectory, terminal_mode=False)
    node.tracker.active_trajectory = original
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'TERMINAL_COMMITTED'
    assert node._terminal_execution_active(10.4)
    assert node.tracker.active_trajectory.terminal_mode
    assert node.tracker.active_trajectory.valid_until == original.valid_until
    assert node.tracker.active_trajectory.segments == original.segments
    assert not original.terminal_mode


def test_execution_cannot_outlive_original_trajectory_or_rearm_from_stale_input():
    node = terminal_node()
    node.timer_callback()
    original = node.tracker.active_trajectory
    node.latest_prediction = node.latest_kf_message = None
    advance(node, 11.)
    node.timer_callback()
    assert node.safe_recovery_latched
    assert node.tracker.active_trajectory is None
    assert original.valid_until == 11.
    advance(node, 11.05)
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'SAFE_RECOVERY'


def test_kf_loss_before_prediction_expiry_cannot_reduce_committed_speed():
    node = terminal_node()
    node.timer_callback()
    node.latest_kf_message = None
    advance(node, 10.45)
    node.latest_state = replace(node.latest_state, position=(3.425, 0., -.5))
    assert node._prediction_fresh(10.45)
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'TERMINAL_COMMITTED'
    assert math.hypot(*node.diagnostics[-1][0].velocity[:2]) == pytest.approx(6.5)


def test_camera_epoch_reset_immediately_cancels_old_execution_and_plan():
    node = terminal_node()
    node.timer_callback()
    node.bearing_callback(SimpleNamespace(
        stamp=stamp(0.), raw_stamp=stamp(0.), valid=False))
    assert not node._terminal_execution_active(10.45)
    assert node.tracker.active_trajectory is None
    assert node.pending_trajectory is None
    assert node.safe_recovery_latched


def test_prediction_dropout_does_not_discard_an_accepted_plan_with_fresh_lock():
    node = terminal_node(end=11.6)  # Not in the last 0.7 s.
    node.latest_prediction = None
    original = node.tracker.active_trajectory
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'TRACKING'
    assert node.tracker.active_trajectory is original
    assert not node.safe_recovery_latched


def test_stale_inputs_cannot_enter_terminal_commitment():
    node = terminal_node()
    node.latest_kf_message = node.latest_prediction = None
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'SAFE_RECOVERY'
    assert node.tracker.active_trajectory is None


def test_stale_commitment_does_not_accept_a_replacement_or_extend_its_deadline():
    node = terminal_node()
    node.timer_callback()
    original = node.tracker.active_trajectory
    node.latest_prediction = node.latest_kf_message = None
    node._candidate_endpoint = lambda trajectory: trajectory.terminal_position
    node.terminal_replacement_position_error = .15
    candidate = replace(original, source_stamp=10.4, valid_until=11.4,
                        contact_stamp=11.4)
    reason = node._evaluate_trajectory(candidate)
    assert reason.value == 'TERMINAL_COMMITTED'
    assert node.tracker.active_trajectory is original


@pytest.mark.parametrize('phase', [MissionState.ABORTED, MissionState.SAFE_RECOVERY])
def test_explicit_mission_exit_revokes_execution(phase):
    node = terminal_node()
    node.timer_callback()
    node.mission_state = phase
    assert not node._terminal_execution_active(10.5)


def test_clock_reversal_revokes_execution():
    node = terminal_node()
    node.timer_callback()
    assert not node._terminal_execution_active(10.3)


def test_navigation_expiry_revokes_execution_and_emits_no_new_setpoint():
    node = terminal_node()
    node.timer_callback()
    count = len(node.setpoint_pub.messages)
    node._ros_seconds = lambda: 10.6
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'STATE_STALE'
    assert len(node.setpoint_pub.messages) == count
    assert not node._terminal_execution_active(10.6)


def test_core_terminal_hold_preserves_speed_and_keeps_vertical_guard():
    node = terminal_node()
    measured = replace(node.latest_state, position=(2.6, 0., -.075),
                       velocity=(6.5, 0., 2.))
    command = node.tracker.command(measured, 1, control_stamp=10.4,
                                   hold_terminal_velocity=True)
    assert math.hypot(*command.velocity[:2]) == pytest.approx(6.5)
    assert command.velocity[2] <= 0.
    assert command.safety_state != 'SAFE'


def committed_mission():
    core = MissionManagerCore()
    core.phase = MissionPhase.TERMINAL_MINCO
    core.mission_id = 1
    core.intercept_requested = True
    core.active_plan_id = 7
    return core


def test_mission_keeps_execution_without_claiming_fresh_target_lock():
    core = committed_mission()
    core.observe_visibility(1, 'TERMINAL_COMMITTED', False, 10.4,
                            execution_deadline=11., execution_plan_id=7)
    assert core.phase == MissionPhase.TERMINAL_MINCO
    assert not core.target_locked
    core.observe_visibility(1, 'TERMINAL_COMMITTED', False, 10.6,
                            execution_deadline=11.2, execution_plan_id=7)
    assert core.tick(10.99) == MissionPhase.TERMINAL_MINCO
    assert core.tick(11.) == MissionPhase.SAFE_RECOVERY


def test_mission_rejects_commitment_without_the_accepted_plan():
    core = committed_mission()
    core.observe_visibility(1, 'TERMINAL_COMMITTED', False, 10.4,
                            execution_deadline=11., execution_plan_id=8)
    assert core.tick(10.5) == MissionPhase.SAFE_RECOVERY


def test_uncommitted_recovery_still_preempts_terminal_motion():
    core = committed_mission()
    core.observe_visibility(1, 'SAFE_RECOVERY', False, 10.4)
    assert core.phase == MissionPhase.SAFE_RECOVERY


def test_expired_core_trajectory_cannot_be_rescued_by_terminal_hold():
    node = terminal_node()
    assert node.tracker.command(node.latest_state, 1, control_stamp=11.,
                                hold_terminal_velocity=True) is None
    assert node.tracker.status == 'NO_VALID_PLAN'
    assert node.tracker.active_trajectory is None


def test_new_plan_admission_still_rejects_stale_observation():
    node = terminal_node()
    node.latest_prediction = node.latest_kf_message = None
    assert node._evaluate_trajectory(node.tracker.active_trajectory) == (
        TrajectoryRejectReason.SOURCE_STALE)


def test_fresh_accepted_minco_keeps_speed_before_the_commitment_window():
    node = terminal_node(end=12.)
    node.mission_state = MissionState.MINCO_TRACKING
    node.mission_state_name = 'MINCO_TRACKING'
    node.terminal_mode_latched = False
    coefficients = list(node.tracker.active_trajectory.segments[0].coefficients)
    coefficients[1] = 5.5
    node.tracker.active_trajectory = replace(
        node.tracker.active_trajectory, terminal_mode=False,
        segments=(PolynomialSegmentData(2., tuple(coefficients)),))
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'TRACKING'
    assert math.hypot(*command.velocity[:2]) == pytest.approx(6.5)
    assert node.terminal_execution is None


def test_guidance_to_minco_handover_uses_the_last_emitted_velocity():
    node = terminal_node(end=12.)
    accepted = node.tracker.active_trajectory
    node.tracker.active_trajectory = None
    node.tracker.previous_command_velocity = (0., 0., 0.)
    node.tracker.previous_command_stamp = 2.
    node.flight_guidance.previous_velocity = (4.1, 0., 0.)
    node.mission_state = MissionState.FAR_GUIDANCE
    node.mission_state_name = 'FAR_GUIDANCE'
    node.terminal_mode_latched = False
    node.timer_callback()
    emitted = tuple(node.setpoint_pub.messages[-1].velocity)
    assert math.hypot(*emitted[:2]) > 3.5
    node.tracker.active_trajectory = replace(accepted, terminal_mode=False)
    node.mission_state = MissionState.MINCO_TRACKING
    node.mission_state_name = 'MINCO_TRACKING'
    advance(node, 10.45)
    node.timer_callback()
    commanded = tuple(node.setpoint_pub.messages[-1].velocity)
    assert math.hypot(*commanded[:2]) >= math.hypot(*emitted[:2]) - 1e-6
    assert math.dist(commanded[:2], emitted[:2]) <= 3. * .05 + 1e-6


def test_depth_loss_before_last_point_seven_still_revokes_execution():
    node = terminal_node(end=12.)
    node.timer_callback()
    node.latest_kf_message = node.latest_prediction = None
    advance(node, 10.6)
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'SAFE_RECOVERY'
    assert node.tracker.active_trajectory is None
    assert node.terminal_execution is None


def fresh_replacement(node):
    coefficients = (2.6, 6.5, -1., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0.,
                    -.5, 0., .1 / .49, 0., 0., 0.)
    return replace(node.tracker.active_trajectory, plan_id=8,
                   source_stamp=10.4, generated_stamp=10.4,
                   observation_stamp=10.4, valid_until=11.1, contact_stamp=11.1,
                   terminal_position=(6.66, 0., -.4),
                   terminal_velocity=(5.1, 0., .2 / .7),
                   segments=(PolynomialSegmentData(.7, coefficients),))


def test_fresh_valid_replacement_rearms_only_the_new_accepted_plan():
    node = terminal_node()
    node.timer_callback()
    original = node.tracker.active_trajectory
    candidate = fresh_replacement(node)
    node.terminal_replacement_position_error = .15
    assert node._evaluate_trajectory(candidate) == TrajectoryRejectReason.NONE
    assert node.tracker.active_trajectory is candidate
    assert node.terminal_execution is None
    assert node.search_state != 'TERMINAL_COMMITTED'
    assert original.valid_until == 11.
    node.timer_callback()
    assert node.terminal_execution.trajectory == candidate
    assert node.terminal_execution.trajectory is node.tracker.active_trajectory
    assert node.terminal_execution.deadline == pytest.approx(11.1)
    assert math.hypot(*node.diagnostics[-1][0].velocity[:2]) == pytest.approx(6.5)


def test_rejected_fresh_replacement_cannot_change_committed_deadline():
    node = terminal_node()
    node.timer_callback()
    execution = node.terminal_execution
    node.terminal_replacement_position_error = .15
    candidate = replace(fresh_replacement(node), terminal_position=(100., 0., -.4))
    assert node._evaluate_trajectory(candidate) == (
        TrajectoryRejectReason.TARGET_ENDPOINT_MISMATCH)
    assert node.terminal_execution is execution
    assert node.tracker.active_trajectory is execution.trajectory


def test_new_tracker_acceptance_clears_only_the_previous_execution_deadline():
    core = committed_mission()
    core.observe_visibility(1, 'TERMINAL_COMMITTED', True, 10.4,
                            execution_deadline=11., execution_plan_id=7)
    assert core.observe_tracker(1, 8, 'PLAN_ACCEPTED', .05, .7, 10.5)
    assert core.terminal_execution_deadline is None
    core.observe_visibility(1, 'TERMINAL_COMMITTED', True, 10.5,
                            execution_deadline=11.2, execution_plan_id=8)
    assert core.tick(11.01) == MissionPhase.TERMINAL_MINCO
    assert core.tick(11.2) == MissionPhase.SAFE_RECOVERY


@pytest.mark.parametrize('reason', ['stale', 'same_plan', 'no_lock'])
def test_unaccepted_deadline_updates_cannot_renew_old_execution(reason):
    core = committed_mission()
    core.observe_visibility(1, 'TERMINAL_COMMITTED', True, 10.4,
                            execution_deadline=11., execution_plan_id=7)
    core.target_locked = reason != 'no_lock'
    core.observe_tracker(1, 7 if reason == 'same_plan' else 8,
                         'PLAN_ACCEPTED', .2 if reason == 'stale' else .05, .7, 10.5)
    assert core.terminal_execution_deadline == 11.


def test_committed_replacement_rejects_stale_navigation_without_renewal():
    node = terminal_node()
    node.timer_callback()
    execution = node.terminal_execution
    node.latest_state = replace(node.latest_state, stamp=10.2)
    assert node._evaluate_trajectory(fresh_replacement(node)) == (
        TrajectoryRejectReason.TERMINAL_COMMITTED)
    assert node.terminal_execution is execution


@pytest.mark.parametrize('end', [11., 12.])
def test_coincident_xy_without_capture_preserves_speed(end):
    node = terminal_node(end=end)
    coefficients = list(node.tracker.active_trajectory.segments[0].coefficients)
    coefficients[1] = 5.5
    node.tracker.active_trajectory = replace(
        node.tracker.active_trajectory,
        segments=(PolynomialSegmentData(end - 10., tuple(coefficients)),))
    node.latest_kf_message.position.x = node.latest_state.position[0]
    node.latest_kf_message.position.z = .06  # 0.56 m vertical separation.
    for sample in node.latest_prediction.samples:
        sample.position.x = node.latest_state.position[0]
    node.timer_callback()
    assert math.hypot(*node.diagnostics[-1][0].velocity[:2]) == pytest.approx(6.5)
    if end == 11.:
        assert node.terminal_execution is not None
        node.latest_kf_message = node.latest_prediction = None
        advance(node, 10.6)
        node.timer_callback()
        assert math.hypot(*node.diagnostics[-1][0].velocity[:2]) == pytest.approx(6.5)


def test_delayed_navigation_cannot_admit_a_plan_too_short_at_control_epoch():
    node = terminal_node()
    node.timer_callback()
    execution = node.terminal_execution
    node.latest_state = replace(node.latest_state, stamp=10.285,
                                position=(1.8525, 0., -.5))
    node.latest_kf_message.position.x = 2.9725
    for t, sample in zip((0., 4.), node.latest_prediction.samples):
        sample.position.x = 2.9725 + 4. * t
    coefficients = (1.8525, 6.5, -1., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0.,
                    -.5, 0., .03 / .09, 0., 0., 0.)
    candidate = replace(fresh_replacement(node), source_stamp=10.285,
                        valid_until=10.585, contact_stamp=10.585,
                        terminal_position=(3.7125, 0., -.47),
                        segments=(PolynomialSegmentData(.3, coefficients),))
    node.terminal_replacement_position_error = .15
    assert node._evaluate_trajectory(candidate) == (
        TrajectoryRejectReason.INSUFFICIENT_REMAINING_TIME)
    assert node.terminal_execution is execution
    assert node.tracker.active_trajectory is execution.trajectory
