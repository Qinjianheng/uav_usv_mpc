"""A spatial discontinuity fails closed across the existing online processes."""

from types import SimpleNamespace

import pytest
from rclpy.time import Time
from std_msgs.msg import String

from test_rgbd_target_localizer import _make_synthetic_localizer
from test_visual_delivery_latency import filter_node, observation


def localizer():
    node, observations = _make_synthetic_localizer([100.1])
    node.navigation_state_pub = None
    node._map_px4_pose_stamp = lambda stamp, receipt: (stamp, stamp, stamp)
    faults = []
    node.frame_fault_pub = SimpleNamespace(publish=faults.append)
    node.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    return node, observations, faults


@pytest.mark.parametrize('stream', ['position', 'attitude'])
def test_reset_cuts_both_histories_and_latches_until_process_restart(stream):
    node, _, faults = localizer()
    node.position_history.add(100., (0., 0., -5.))
    node.attitude_history.add(100., (1., 0., 0., 0.))
    for stamp, counter in ((100., 5), (100.04, 6), (100.08, 6)):
        if stream == 'position':
            message = SimpleNamespace(x=10., y=0., z=-5., xy_valid=True, z_valid=True,
                                      timestamp_sample=round(stamp*1e6),
                                      xy_reset_counter=counter, z_reset_counter=0)
            node.position_callback(message)
        else:
            message = SimpleNamespace(q=(1., 0., 0., 0.), quat_reset_counter=counter,
                                      timestamp_sample=round(stamp*1e6))
            node.attitude_callback(message)
        if stamp == 100.:
            assert not faults  # First nonzero counter is a seed, not a reset.
    assert len(faults) == 1
    assert node.navigation_frame_fault
    assert node.position_history.value_at(100.02) is None
    assert node.position_history.latest_value is None
    assert node.attitude_history.latest_value is None


def test_old_counter_delivery_does_not_fabricate_a_reset():
    node, _, faults = localizer()
    for stamp, counter in ((100.04, 5), (100., 4), (100.08, 5)):
        node.position_callback(SimpleNamespace(
            x=0., y=0., z=-5., timestamp_sample=round(stamp*1e6),
            xy_valid=True, z_valid=True, xy_reset_counter=counter))
    assert not faults
    assert node.position_history.time_range == pytest.approx((100.04, 100.08))


def test_wire_timesync_offset_change_keeps_native_counter_order():
    node, _, faults = localizer()
    mapping = {100.: (100., 10., 100.), 99.95: (100.04, 10.04, 99.95)}
    node._map_px4_pose_stamp = lambda stamp, receipt: mapping[round(stamp, 2)]
    for stamp in (100., 99.95):
        node.position_callback(SimpleNamespace(
            x=0., y=0., z=-5., timestamp_sample=round(stamp*1e6),
            xy_valid=True, z_valid=True, xy_reset_counter=5))
    assert not faults
    assert node.position_history.time_range == pytest.approx((100., 100.04))


def test_kf_fault_discards_old_state_and_rejects_later_images():
    node, states = filter_node([10.1])
    node.measurement_callback(observation(10.))
    node.navigation_frame_fault = 'PX4_POSITION_RESET'
    node.navigation_frame_failed(node.navigation_frame_fault)
    node.measurement_callback(observation(10.1))
    node.timer_callback()
    assert not node.filter.initialized
    assert node.last_measurement_time_ns is None
    assert not states[-1].valid


def test_tracker_fault_revokes_and_never_resumes_from_queued_navigation():
    from test_follow_wait import runnable_follow_node
    from uav_usv_interfaces.msg import UavState
    node, _ = runnable_follow_node()
    node.navigation_frame_fault = 'PX4_ATTITUDE_RESET'
    node.navigation_frame_failed(node.navigation_frame_fault)
    message = UavState()
    message.valid = True
    message.frame_id = 'local_ned'
    message.stamp = Time(nanoseconds=10_200_000_000).to_msg()
    node.uav_callback(message)
    node.timer_callback()
    assert node.latest_state is None
    assert node.tracker.active_trajectory is None
    assert node.pending_trajectory is None
    assert not node.setpoint_pub.messages
    assert node.diagnostics[-1][1] == 'NAVIGATION_FRAME_FAULT'


def test_guard_is_transient_local_and_cannot_be_cleared_by_an_empty_event():
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from uav_control.common.navigation_frame import install_navigation_frame_guard
    callbacks, seen = [], []
    node = SimpleNamespace(create_subscription=lambda cls, topic, cb, qos:
                           callbacks.append((topic, cb, qos)))
    install_navigation_frame_guard(node, seen.append)
    topic, callback, qos = callbacks[0]
    assert topic == '/navigation/frame_fault'
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    callback(String(data=''))
    callback(String(data='PX4_POSITION_RESET'))
    callback(String(data=''))
    callback(String(data='PX4_ATTITUDE_RESET'))
    assert seen == ['PX4_POSITION_RESET']
    assert node.navigation_frame_fault == 'PX4_POSITION_RESET'


def test_mission_fault_is_not_cleared_by_repeated_commands():
    from uav_control.mission.mission_manager import MissionManagerCore, MissionPhase
    from uav_control.mission.mission_manager_node import MissionManagerNode
    node = object.__new__(MissionManagerNode)
    node.core = MissionManagerCore()
    node.core.set_flight_ready(True)
    node._now = lambda: 10.
    node._publish = lambda: None
    node.navigation_frame_fault = 'PX4_POSITION_RESET'
    node.navigation_frame_failed(node.navigation_frame_fault)
    for command in ('R', 'X', 'Y'):
        node.command_callback(String(data=command))
    assert node.core.completed and node.core.phase == MissionPhase.FAILURE


def test_late_subscriber_receives_latched_fault_over_local_dds():
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from uav_control.common.navigation_frame import (
        FRAME_FAULT_TOPIC, install_navigation_frame_guard, navigation_frame_fault_qos)
    context = Context()
    rclpy.init(context=context)
    producer = observer = executor = None
    try:
        producer = Node('frame_fault_producer_test', context=context)
        pub = producer.create_publisher(String, FRAME_FAULT_TOPIC, navigation_frame_fault_qos())
        pub.publish(String(data='PX4_POSITION_RESET'))
        observer = Node('frame_fault_observer_test', context=context)
        seen = []
        install_navigation_frame_guard(observer, seen.append)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(observer)
        deadline = time.monotonic() + 3.
        while not seen and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert seen == ['PX4_POSITION_RESET']
    finally:
        if executor:
            executor.shutdown()
        if observer:
            observer.destroy_node()
        if producer:
            producer.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('node_type,method,args', [
    ('bctra', 'state_callback', (None,)),
    ('imm', 'observation_callback', (None,)),
    ('follow', 'tick', ()),
    ('follow', '_publish_proposal', ({},)),
    ('follow', '_publish_wire_proposal', (None,)),
    ('legacy', '_planning_tick', (False,)),
    ('legacy', '_publish_job', (None,)),
])
def test_fault_blocks_online_input_and_planner_publication(node_type, method, args):
    from uav_control.tracking.target_predictor_node import TargetPredictorNode
    from uav_control.tracking.imm_target_predictor_node import ImmTargetPredictorNode
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    from uav_control.guidance.intercept_planner_node import InterceptPlannerNode
    cls = dict(bctra=TargetPredictorNode, imm=ImmTargetPredictorNode,
               follow=P4FollowPlannerNode, legacy=InterceptPlannerNode)[node_type]
    node = object.__new__(cls)
    node.navigation_frame_fault = 'PX4_POSITION_RESET'
    # No other setup: touching cached input, a solver or a publisher would
    # fail here, rather than silently accepting work in the old frame.
    getattr(node, method)(*args)
