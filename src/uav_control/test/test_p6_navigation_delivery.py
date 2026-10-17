"""Native interpolation requires intermediate DDS samples during a short callback burst."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import pytest
from uav_usv_interfaces.msg import UavState

from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
from uav_control.guidance.follow_admission import follow_settling


def test_tracker_retains_native_bracket_when_executor_resumes_after_delivery_burst():
    rclpy.init(args=['--ros-args', '-p', 'follow_minco_enabled:=false'])
    tracker = sender = executor = None
    try:
        tracker = TrajectoryTrackerNode()
        for timer in tracker.timers:
            timer.cancel()  # Exercise transport and state input only; no flight commands.
        tracker._ros_seconds = lambda: 100.1
        sender = Node('p6_navigation_delivery_test')
        publisher = sender.create_publisher(
            UavState, '/navigation/uav_state', qos_profile_sensor_data)
        executor = SingleThreadedExecutor()
        executor.add_node(tracker)
        deadline = time.monotonic()+3.
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(.01)
        assert publisher.get_subscription_count() == 1

        def publish(seconds):
            message = UavState()
            message.valid, message.frame_id = True, 'local_ned'
            message.stamp.sec = int(seconds)
            message.stamp.nanosec = round((seconds-int(seconds))*1e9)
            message.native_timestamp_sample = round(seconds*1e6)
            message.position.z, message.velocity.x = -5., 2.
            publisher.publish(message)

        publish(100.)
        while not tracker.follow_navigation_history and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert tracker.follow_navigation_history[-1]['stamp'] == 100.
        # No executor work during this burst, as with validation/JSON callback load.
        # The native source still supplies all original, closely spaced samples.
        for stamp in (100.02, 100.04, 100.06, 100.08):
            publish(stamp)
            time.sleep(.005)
        time.sleep(.02)
        deadline = time.monotonic()+.25
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        history = tracker.follow_navigation_history
        target = [dict(valid=True, stamp=100.07, source=100.04,
                       p=[5., 0., 0.], v=[2., 0., 0.])]
        metrics = follow_settling(history, target, 100.1)
        assert metrics['settled'], (list(history), metrics)
        assert metrics['navigation_bracket'] == pytest.approx([100.06, 100.08])
        assert tracker.latest_state.stamp == 100.08
        assert not follow_settling(history, target, 100.2)['settled']
    finally:
        if executor is not None:
            executor.shutdown()
        if tracker is not None:
            tracker.destroy_node()
        if sender is not None:
            sender.destroy_node()
        rclpy.shutdown()
