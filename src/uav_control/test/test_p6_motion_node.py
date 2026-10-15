"""Exercise the real motion node without starting PX4 or Gazebo."""

import json
import math
import os
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from uav_control.moving_target import MovingTarget


def test_node_exposes_independent_motion_parameter_selector():
    assert hasattr(MovingTarget, 'mode_parameters_callback'), (
        'P6 motion parameter ACK interface is missing')


@pytest.fixture
def node():
    if not hasattr(MovingTarget, 'mode_parameters_callback'):
        pytest.skip('motion parameter ACK interface is not implemented yet')
    rclpy.init()
    target = None
    try:
        target = MovingTarget(parameter_overrides=[
            Parameter('enable_gazebo_visualization', value=False)])
        yield target
    finally:
        if target is not None:
            target.destroy_node()
        rclpy.shutdown()


def test_parameter_service_ack_applies_mode_and_locks_after_x(node):
    result = node.set_parameters([
        Parameter('trajectory_type', value='variable_speed_s')])[0]
    assert result.successful
    acknowledgement = json.loads(result.reason)
    assert acknowledgement['mode'] == 'variable_speed_s'
    assert acknowledgement['mode_id'] == 4
    assert acknowledgement['started'] is False
    assert node.motion_profile.mode == 'variable_speed_s'
    node.flight_ready = True
    node.command_callback(SimpleNamespace(data='X'))
    assert node.started
    result = node.set_parameters([
        Parameter('trajectory_type', value='linear')])[0]
    assert not result.successful
    assert node.get_parameter('trajectory_type').value == 'variable_speed_s'
    assert node.motion_profile.mode == 'variable_speed_s'
    result = node.set_parameters([
        Parameter('trajectory_type', value='variable_speed_s')])[0]
    assert result.successful
    assert json.loads(result.reason)['started'] is True


def test_invalid_and_mixed_parameter_requests_do_not_partially_change_mode(node):
    result = node.set_parameters([
        Parameter('trajectory_type', value='invalid')])[0]
    assert not result.successful
    assert json.loads(result.reason)['mode'] == 'figure_eight'
    result = node.set_parameters_atomically([
        Parameter('trajectory_type', value='variable_speed_z'),
        Parameter('initial_x', value=500.)])
    assert not result.successful
    assert node.motion_profile.mode == 'figure_eight'
    assert node.get_parameter('trajectory_type').value == 'figure_eight'
    assert node.x == pytest.approx(8.)


def test_later_ros_validation_rejection_does_not_commit_prepared_profile(node):
    # Public callback registration order ensures the profile is prepared,
    # followed by another validator rejecting the atomic transaction.
    node.remove_on_set_parameters_callback(node.mode_parameters_callback)
    node.add_on_set_parameters_callback(
        lambda parameters: SetParametersResult(successful=False, reason='external rejection'))
    node.add_on_set_parameters_callback(node.mode_parameters_callback)
    profile = node.motion_profile
    result = node.set_parameters_atomically([
        Parameter('trajectory_type', value='variable_speed_z')])
    assert not result.successful
    assert node.motion_profile is profile
    assert node.trajectory_type == 'figure_eight'
    assert node.get_parameter('trajectory_type').value == 'figure_eight'
    assert json.loads(result.reason)['reason'] == 'external rejection'


def test_status_payload_has_applied_mode_and_started_state(node):
    messages = []
    node.motion_mode_pub = SimpleNamespace(publish=messages.append)
    node.publish_motion_mode()
    status = json.loads(messages[-1].data)
    assert status['mode'] == 'figure_eight' and status['id'] == 2
    node.set_parameters([Parameter('trajectory_type', value='variable_speed_z')])
    status = json.loads(messages[-1].data)
    assert status['mode'] == 'variable_speed_z' and status['id'] == 5
    node.flight_ready = True
    node.command_callback(SimpleNamespace(data='X'))
    assert json.loads(messages[-1].data)['started'] is True


@pytest.mark.parametrize('mode', [
    'linear', 'figure_eight', 'variable_speed_eight',
    'variable_speed_s', 'variable_speed_z',
])
def test_state_velocity_acceleration_and_visualizer_share_current_sample(node, mode):
    node.set_parameters([Parameter('trajectory_type', value=mode)])
    assert node.motion_profile.mode == mode
    clock = [1_000_000_000]
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=clock[0]))
    node.flight_ready = True
    node.command_callback(SimpleNamespace(data='X'))
    positions, velocities, states, visual = [], [], [], []
    node.position_pub = SimpleNamespace(publish=positions.append)
    node.velocity_pub = SimpleNamespace(publish=velocities.append)
    node.state_pub = SimpleNamespace(publish=states.append)
    node.gazebo_visualizer = SimpleNamespace(
        world_paused=False,
        update=lambda *position: visual.append(position) or True)
    clock[0] += 50_000_000
    node.timer_callback()
    sample = node.motion_profile.state()
    state = states[-1]
    assert (state.stamp.sec, state.stamp.nanosec) == (1, 50_000_000)
    assert (positions[-1].x, positions[-1].y, positions[-1].z) == sample.position
    assert (velocities[-1].x, velocities[-1].y, velocities[-1].z) == sample.velocity
    assert (state.acceleration.x, state.acceleration.y, state.acceleration.z) == (
        sample.acceleration)
    assert visual[-1] == (sample.position[0], sample.position[1],
                          sample.position[2] - node.gazebo_visual_height_offset)


def test_clock_pause_rollback_and_large_jump_freeze_actual_node(node):
    clock = [1_000_000_000]
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=clock[0]))
    node.flight_ready = True
    node.command_callback(SimpleNamespace(data='X'))
    clock[0] += 50_000_000
    node.timer_callback()
    before = node.motion_profile.state()
    for stamp in (clock[0], 500_000_000, 5_000_000_000):
        clock[0] = stamp
        node.timer_callback()
        assert node.motion_profile.state() == before
    clock[0] += 50_000_000
    node.timer_callback()
    assert node.elapsed_time == pytest.approx(.1)
    assert math.hypot(node.x - before.position[0], node.y - before.position[1]) > 0.


def test_y_r_q_keep_existing_motion_command_behavior(node):
    for command in ('Y', 'R', 'Q'):
        node.command_callback(SimpleNamespace(data=command))
        assert not node.started
        assert node.trajectory_type == 'figure_eight'
    node.command_callback(SimpleNamespace(data='X'))
    assert not node.started
    node.flight_ready = True
    node.command_callback(SimpleNamespace(data='X'))
    sample = node.motion_profile.state()
    for command in ('X', 'Y', 'R', 'Q'):
        node.command_callback(SimpleNamespace(data=command))
        assert node.started
        assert node.motion_profile.state() == sample


@pytest.mark.skipif(os.environ.get('P6_ROS_TRANSPORT_TESTS') != '1',
                    reason='explicit isolated DDS transport smoke test')
def test_real_parameter_service_and_late_join_transient_status(node):
    observer = Node('p6_motion_test_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)
    statuses = []
    try:
        observer.create_subscription(
            String, '/simulation/impact/motion_mode',
            lambda message: statuses.append(json.loads(message.data)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + 3.
        while not statuses and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert statuses[-1]['id'] == 2  # Published before observer existed.
        getter = observer.create_client(GetParameters, '/moving_target/get_parameters')
        setter = observer.create_client(SetParameters, '/moving_target/set_parameters')
        assert getter.wait_for_service(timeout_sec=3.)
        assert setter.wait_for_service(timeout_sec=3.)
        future = getter.call_async(GetParameters.Request(names=['trajectory_type']))
        executor.spin_until_future_complete(future, timeout_sec=3.)
        assert future.result().values[0].string_value == 'figure_eight'
        future = setter.call_async(SetParameters.Request(parameters=[
            Parameter('trajectory_type', value='variable_speed_z').to_parameter_msg()]))
        executor.spin_until_future_complete(future, timeout_sec=3.)
        result = future.result().results[0]
        assert result.successful
        assert json.loads(result.reason)['mode_id'] == 5
        assert node.motion_profile.mode == 'variable_speed_z'
        profile = node.motion_profile
        future = setter.call_async(SetParameters.Request(parameters=[
            Parameter('trajectory_type', value='variable_speed_z').to_parameter_msg()]))
        executor.spin_until_future_complete(future, timeout_sec=3.)
        assert future.result().results[0].successful
        assert node.motion_profile is profile
        node.flight_ready = True
        node.command_callback(SimpleNamespace(data='X'))
        future = setter.call_async(SetParameters.Request(parameters=[
            Parameter('trajectory_type', value='linear').to_parameter_msg()]))
        executor.spin_until_future_complete(future, timeout_sec=3.)
        result = future.result().results[0]
        assert not result.successful
        assert json.loads(result.reason)['started'] is True
        assert node.motion_profile.mode == 'variable_speed_z'
    finally:
        executor.remove_node(node)
        executor.remove_node(observer)
        executor.shutdown()
        observer.destroy_node()
