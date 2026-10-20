"""Safety and truth-boundary regressions for strict visual control."""

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from uav_control.control.trajectory_tracker_node import fresh_flight_target
from uav_control.control.trajectory_tracker_node import anchored_search_hold
from uav_control.mission.mission_manager import MissionManagerCore, MissionPhase


def stamp(value):
    return SimpleNamespace(sec=int(value), nanosec=round((value % 1) * 1e9))


def target(now=10.0, source=9.95):
    return SimpleNamespace(
        valid=True, stamp=stamp(now), source_stamp=stamp(source),
        frame_id='local_ned', position=SimpleNamespace(x=2., y=3., z=0.),
        velocity=SimpleNamespace(x=1., y=0., z=0.),
    )


def test_projected_kf_cannot_hide_stale_image():
    assert fresh_flight_target(target(), 10., .125) is not None
    assert fresh_flight_target(target(source=9.), 10., .125) is None
    assert fresh_flight_target(target(source=10.01), 10., .125) is None
    assert fresh_flight_target(target(source=0.), 10., .125) is None
    assert fresh_flight_target(target(now=9.98, source=9.99), 10., .125) is None


def test_hold_captures_fixed_xyz_and_preserves_search_height():
    anchor = anchored_search_hold(None, (1., 2., -5.), 0., 1.5)
    assert anchor == (1., 2., -5.)
    assert anchored_search_hold(anchor, (4., 3., -4.), 0., 1.5) == anchor
    assert anchored_search_hold(None, (1., 2., -.3), 0., 1.5)[2] == -1.5


def test_takeoff_allows_follow_and_queued_y_but_requires_lock_to_intercept():
    core = MissionManagerCore()
    core.set_flight_ready(True)
    core.handle_command('X', 1.)
    core.mark_takeoff_complete(2.)
    assert core.phase == MissionPhase.FOLLOW
    assert core.handle_command('Y', 2.1)
    assert core.phase == MissionPhase.FOLLOW
    core.observe_visibility(core.mission_id, 'TARGET_LOCK', True, 2.2)
    assert core.phase == MissionPhase.FAR_GUIDANCE


def test_terminal_loss_recovers_before_reacquisition():
    core = MissionManagerCore()
    core.phase = MissionPhase.TERMINAL_MINCO
    core.intercept_requested = True
    core.observe_visibility(core.mission_id, 'SAFE_RECOVERY', False, 5.)
    assert core.phase == MissionPhase.SAFE_RECOVERY
    core.observe_visibility(core.mission_id, 'REACQUIRE', False, 5.5)
    assert core.phase == MissionPhase.REACQUIRE
    core.observe_visibility(core.mission_id, 'TARGET_LOCK', True, 6.)
    core.tick(6.05)
    assert core.phase == MissionPhase.FAR_GUIDANCE


def test_mission_has_no_evaluator_result_subscription():
    path = Path(__file__).parents[1] / 'uav_control/mission/mission_manager_node.py'
    tree = ast.parse(path.read_text())
    topics = [n.args[1].value for n in ast.walk(tree)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr == 'create_subscription' and len(n.args) >= 2
              and isinstance(n.args[1], ast.Constant)]
    assert '/simulation/impact/result' not in topics
    assert '/simulation/impact/hit' not in topics


def test_optional_evaluator_and_shadow_nodes_cannot_disable_online_inputs():
    workspace_src = Path(__file__).parents[2]
    path = workspace_src / 'uav_usv_bringup/launch/modular_intercept.launch.py'
    tree = ast.parse(path.read_text())
    nodes = {}
    for call in ast.walk(tree):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == 'Node'):
            keywords = {item.arg: item.value for item in call.keywords}
            nodes[keywords['name'].value] = keywords
    for name in (
        'trajectory_tracker_node', 'mission_manager_node',
        'dual_tof_image_bridge',
        'target_bearing_node', 'rgbd_target_localizer',
    ):
        assert 'condition' not in nodes[name]
    assert ast.unparse(nodes['target_kalman_filter']['condition']) == (
        "UnlessCondition(LaunchConfiguration('p76_enabled'))")
    assert ast.unparse(nodes['spatial_dkf_node']['condition']) == (
        "IfCondition(LaunchConfiguration('p76_enabled'))")
    # P7 omits the unused legacy planner through its own switch. Evaluator
    # and perception diagnostics must never gate that switch or online inputs.
    assert ast.unparse(nodes['intercept_planner_node']['condition']) == (
        "IfCondition(LaunchConfiguration('enable_intercept_planner'))")
    for name in ('target_predictor_node', 'imm_target_predictor_node'):
        assert 'prediction_model' in ast.unparse(nodes[name]['condition'])
    assert 'shadow_target_predictor_node' not in nodes
    assert '/planning/target_prediction' in ast.unparse(
        nodes['imm_target_predictor_node']['parameters'])
    condition = nodes['intercept_evaluator_node']['condition']
    assert condition.func.id == 'IfCondition'
    assert condition.args[0].id == 'enable_evaluator'
    px4_publishers = [
        name for name, keywords in nodes.items()
        if keywords['executable'].value in (
            'trajectory_tracker_node', 'pure_pursuit',
            'predictive_intercept', 'trajectory_impact_sim',
        )
    ]
    assert px4_publishers == ['trajectory_tracker_node']


@pytest.mark.parametrize('module_name', (
    'pure_pursuit', 'predictive_intercept', 'trajectory_impact_sim',
))
def test_retired_truth_controllers_are_unimportable(module_name):
    import importlib

    with pytest.raises(ModuleNotFoundError, match='uav_control.' + module_name):
        importlib.import_module('uav_control.' + module_name)


def control_node(now=10.2, height=5.0):
    from uav_control.control import trajectory_tracker_node as module
    from uav_control.control.flight_guidance import FlightGuidanceCore
    from uav_control.control.trajectory_tracking import TrajectoryTrackerCore
    from uav_control.control.trajectory_tracking import TrackerKinematicState
    from uav_control.mission.target_visibility import TargetVisibilityState
    from uav_usv_interfaces.msg import MissionState

    node = object.__new__(module.TrajectoryTrackerNode)
    node._ros_seconds = lambda: now
    node.expected_frame_id = 'local_ned'
    node.maximum_state_age = .125
    node.tracker = TrajectoryTrackerCore(recovery_clearance=.5)
    node.flight_guidance = FlightGuidanceCore()
    node.visibility = TargetVisibilityState()
    node.latest_kf_message = target(now=now, source=now)
    node.latest_target_state = None
    node.visibility_decision = None
    node.search_hold_position = None
    node.safe_recovery_latched = False
    node.terminal_mode_latched = False
    node.mission_state = MissionState.TARGET_ACQUIRE
    node.current_heading = 0.
    node.last_target_yaw = None
    node.max_observation_yaw_rate = 1.
    node.pending_trajectory = None
    node.mission_id = 1
    state = TrackerKinematicState(now, (1., 2., -height), (0., 0., 0.))
    for value in (now - .2, now - .1, now):
        node.visibility.observe(value, .05, True, value, position_valid=True)
    return node, state


def test_invalid_kf_preempts_candidate_before_next_timer():
    from uav_control.control.trajectory_tracking import TrajectoryRejectReason
    node, state = control_node()
    node._update_visibility(state, state.stamp, .05)
    assert node.visibility_decision.locked
    invalid = target(now=10.2, source=10.2)
    invalid.valid = False
    node.target_state_callback(invalid)
    candidate = SimpleNamespace(
        source_stamp=10.2, observation_stamp=10.2,
        target_state_source='tracking',
    )
    assert node._evaluate_trajectory(candidate) == (
        TrajectoryRejectReason.PREDICTION_MISMATCH
    )
    assert node.tracker.active_trajectory is None
    assert not node.terminal_mode_latched


def test_stale_reordered_kf_does_not_revoke_new_valid_cache():
    node, _ = control_node()
    newer = node.latest_kf_message
    node.target_state_callback(target(now=9.0, source=8.9))
    assert node.latest_kf_message is newer
    invalid = target(now=9.0, source=8.9)
    invalid.valid = False
    node.target_state_callback(invalid)
    assert node.latest_kf_message is None


def test_terminal_prediction_loss_climbs_to_larger_recovery_height():
    from uav_usv_interfaces.msg import MissionState
    node, state = control_node(height=1.7)
    node.tracker.recovery_clearance = 2.0
    node.mission_state = MissionState.TERMINAL_MINCO
    node.terminal_mode_latched = True
    node._update_visibility(state, state.stamp, .05)
    command = node._search_or_recovery_command(state)
    assert node.safe_recovery_latched
    assert command.velocity[2] < 0.
    assert command.position[2] < state.position[2]
    assert node.search_state == 'SAFE_RECOVERY'


def test_search_hold_xyz_does_not_follow_measured_drift():
    from dataclasses import replace
    node, state = control_node()
    first = node._search_or_recovery_command(state)
    second = node._search_or_recovery_command(replace(
        state, position=(2., 3., -4.),
    ))
    assert first.position == second.position == (1., 2., -5.)
    assert second.mode == 'VELOCITY'
    assert second.velocity == pytest.approx((-.106066017, -.106066017, -.15))
    assert not second.far_guidance_available


def test_single_yaw_arbiter_limits_search_then_vision_and_recovery():
    from dataclasses import replace
    from uav_control.mission.target_visibility import VisibilityDecision
    from uav_usv_interfaces.msg import MissionState
    node, state = control_node()
    node.visibility.last_valid_image_bearing = .8
    node.visibility_decision = VisibilityDecision(
        'TARGET_ACQUIRE', False, True, .6, 0,
    )
    setpoint = SimpleNamespace()
    node._final_yaw(setpoint, state, .05)
    assert setpoint.yawspeed == .6
    assert node.yaw_owner == 'SEARCH'
    node.visibility_decision = replace(node.visibility_decision, locked=True)
    node.mission_state = MissionState.FAR_GUIDANCE
    node._final_yaw(setpoint, state, .05)
    assert setpoint.yawspeed == .8
    assert node.yaw_owner == 'VISION'
    node.safe_recovery_latched = True
    node._final_yaw(setpoint, state, .05)
    assert setpoint.yawspeed == 0.
    assert node.yaw_owner == 'HOLD'


def test_camera_clock_failure_does_not_restart_scan_every_frame():
    node, _ = control_node()
    node.visibility.last_valid_bearing_sign = -1
    node.bearing_callback(SimpleNamespace(
        stamp=stamp(0.), raw_stamp=stamp(4.), valid=False,
    ))
    assert node.visibility.last_valid_bearing_sign == -1
    node.bearing_callback(SimpleNamespace(
        stamp=stamp(0.), raw_stamp=stamp(0.), valid=False,
    ))
    assert node.visibility.last_valid_bearing_stamp is None
    assert node.latest_kf_message is None


def test_search_yaw_inhibited_before_safe_height():
    from uav_control.mission.target_visibility import VisibilityDecision
    node, state = control_node(height=.4)
    node.visibility_decision = VisibilityDecision(
        'TARGET_ACQUIRE', False, False, .25, 1,
    )
    setpoint = SimpleNamespace()
    node._final_yaw(setpoint, state, .05)
    assert setpoint.yawspeed == 0.
