"""P7 Y revokes FOLLOW, rejects unsafe requests and cannot activate legacy interception."""

from types import SimpleNamespace
from dataclasses import replace

import numpy as np

from uav_control.mission.mission_manager import MissionManagerCore, MissionPhase
from uav_usv_interfaces.msg import MissionState


def test_p7_y_requires_lock_and_navigation_instead_of_queuing():
    core = MissionManagerCore(p7_guidance_enabled=True)
    core.set_flight_ready(True)
    core.handle_command('X', 1.)
    core.mark_takeoff_complete(2.)
    assert not core.handle_command('Y', 2.1, navigation_ready=True)
    assert not core.intercept_requested
    core.observe_visibility(1, 'TARGET_LOCK', True, 2.2)
    assert not core.handle_command('Y', 2.3, navigation_ready=False)
    assert core.handle_command('Y', 2.4, navigation_ready=True)
    assert core.phase == MissionPhase.FAR_GUIDANCE
    assert not core.observe_tracker(1, 42, 'PLAN_ACCEPTED', .01, 1., 2.5)
    assert core.tick(2.6) == MissionPhase.FAR_GUIDANCE
    assert core.active_plan_id == 0
    core.observe_visibility(1, 'REACQUIRE', False, 3.)
    assert core.phase == MissionPhase.REACQUIRE
    core.observe_visibility(1, 'TARGET_LOCK', True, 3.1)
    assert core.tick(3.2) == MissionPhase.FAR_GUIDANCE
    assert core.handle_command('Q', 3.3)
    assert core.phase == MissionPhase.ABORTED
    assert core.handle_command('R', 3.4)
    assert not core.intercept_requested


def test_y_immediately_revokes_actual_follow_and_preserves_emitted_velocity():
    from test_follow_wait import runnable_follow_node, CapturedPublisher
    from test_follow_sitl_execution import candidate, ctx, receiver
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    node, _ = runnable_follow_node()
    node.p7_guidance_enabled = True
    node.p7_controller = P7VisualGuidance()
    node.p7_image = None
    node.follow_minco_requested = True
    node.follow_receiver = receiver()
    node.follow_receiver.active = candidate()
    node.follow_receiver.pending = candidate(2)
    node.follow_ack_pub = CapturedPublisher()
    node._follow_context = lambda: ctx(100.1)
    node.tracker.previous_command_velocity = (2., .2, .01)
    message = MissionState(mission_id=1, state=MissionState.FAR_GUIDANCE,
                           state_name='FAR_GUIDANCE', intercept_requested=True)
    node.mission_callback(message)
    assert node.follow_receiver.active is None and node.follow_receiver.pending is None
    assert len(node.follow_ack_pub.messages) == 2
    assert all(m.state == 'REVOKED' for m in node.follow_ack_pub.messages)
    np.testing.assert_allclose(node.p7_controller.previous, (2., .2))
    assert node.tracker.active_trajectory is None and node.pending_trajectory is None


def test_p7_timer_publishes_one_velocity_setpoint_and_ibvs_owns_yaw():
    from test_follow_wait import runnable_follow_node, CapturedPublisher
    from test_strict_visual_control import target
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    node, _ = runnable_follow_node()
    node.p7_guidance_enabled = True
    node.p7_controller = P7VisualGuidance()
    node.p7_image = (10.19, .1, .3)
    node.p7_result = None
    node.p7_diagnostic_pub = CapturedPublisher()
    node.p7_actual_dt = .05
    node.mission_state = MissionState.FAR_GUIDANCE
    node.mission_state_name = 'FAR_GUIDANCE'
    node.intercept_requested = True
    node.offboard_active = node.vehicle_armed = True
    node.vehicle_status_stamp = 10.19
    node.vehicle_status_timeout = 2.
    node.latest_kf_message = target(now=10.2, source=10.19)
    node.latest_kf_message.position.x = 13.
    node.latest_kf_message.position.y = 2.
    node.tracker.previous_command_velocity = (2., 0., 0.)
    node.timer_callback()
    assert len(node.setpoint_pub.messages) == 1
    message = node.setpoint_pub.messages[-1]
    assert np.isnan(message.position).all() and np.isnan(message.acceleration).all()
    assert np.isfinite(message.velocity).all()
    assert node.yaw_owner == 'IBVS'
    assert message.yawspeed > 0.
    assert node.diagnostics[-1][1] == 'P7_GUIDANCE'
    node._ros_seconds = lambda: 10.4
    node.latest_state = replace(node.latest_state, stamp=10.4)
    node.timer_callback()
    assert node.p7_result is None
    assert node.setpoint_pub.messages[-1].yawspeed == 0.


def test_p7_legacy_planner_cannot_submit_or_publish_even_a_queued_result():
    from uav_control.guidance.intercept_planner_node import InterceptPlannerNode
    node = SimpleNamespace(p7_guidance_enabled=True)
    assert InterceptPlannerNode._planning_tick(node, False) is None
    assert InterceptPlannerNode._publish_job(node, object()) is None


def test_p7_follow_planner_stops_generating_on_y():
    from uav_control.controllers.follow_mpc_shadow_node import ShadowResearchRunner
    node = SimpleNamespace(mission=dict(mission_id=1, state=MissionState.FOLLOW, completed=False))
    assert ShadowResearchRunner._follow(node)
    node.mission['state'] = MissionState.FAR_GUIDANCE
    assert not ShadowResearchRunner._follow(node)
