"""Completion remains authoritative despite queued status and mission messages."""

import pytest

from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
from uav_control.control.trajectory_tracking import TrajectoryTrackerCore
from uav_control.mission.mission_manager import MissionManagerCore, MissionPhase
from uav_control.mission.mission_manager_node import mission_state_message


def intercept():
    core = MissionManagerCore()
    core.set_flight_ready(True)
    core.handle_command('X', 1.)
    core.mark_takeoff_complete(3.)
    core.observe_visibility(core.mission_id, 'TARGET_LOCK', True, 3.1)
    assert core.handle_command('Y', 4.)
    return core


@pytest.mark.parametrize('terminal', ['abort', 'capture', 'failure'])
def test_terminal_core_ignores_queued_acceptance_and_tick_until_reset(terminal):
    core = intercept()
    if terminal == 'abort':
        core.handle_command('Q', 4.01)
    else:
        getattr(core, 'mark_' + terminal)(4.01)
    phase = core.phase
    assert not core.observe_tracker(core.mission_id, 2, 'PLAN_ACCEPTED', .02, 1.5, 4.02)
    core.tick(4.03, far_guidance_available=True)
    assert core.phase == phase and core.completed
    assert not core.intercept_requested and core.active_plan_id == 0
    assert not core.handle_command('Y', 4.04)
    assert core.handle_command('R', 4.05)
    assert not core.completed and core.phase == MissionPhase.GROUND_HOLD


@pytest.mark.parametrize('completed_flag', [False, True])
def test_tracker_latches_terminal_for_same_mission_even_if_old_state_arrives(completed_flag):
    core = intercept()
    old_message = mission_state_message(core, 4.)
    node = object.__new__(TrajectoryTrackerNode)
    node.mission_id = core.mission_id
    node.tracker = TrajectoryTrackerCore()
    node.latest_state = None
    core.handle_command('Q', 4.01)
    message = mission_state_message(core, 4.01)
    if completed_flag:
        # Independent wire defense when a sender emits a nonterminal state
        # together with completed=true.
        message.state, message.state_name = old_message.state, old_message.state_name
    node.mission_callback(message)
    assert node.mission_state in node.TERMINAL_STATES
    node.mission_callback(old_message)
    assert node.mission_state in node.TERMINAL_STATES
    assert not node.intercept_requested
