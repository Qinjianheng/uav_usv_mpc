"""Ready observations must reach strict control without two timer waits."""

from types import SimpleNamespace
from dataclasses import replace

import pytest
from rclpy.time import Time

from test_rgbd_target_localizer import _make_synthetic_localizer, _rgbd_messages
from uav_control.control.trajectory_tracker_node import fresh_flight_target
from uav_control.tracking.constant_velocity_kalman import ConstantVelocityKalmanFilter
from uav_control.tracking.target_kalman_filter import TargetKalmanFilterNode
from uav_control.mission.mission_manager import MissionManagerCore, MissionPhase
from uav_control.mission.target_visibility import TargetVisibilityState
from uav_usv_interfaces.msg import TargetObservation
from test_follow_wait import runnable_follow_node


def filter_node(now):
    node = object.__new__(TargetKalmanFilterNode)
    node.filter = ConstantVelocityKalmanFilter(.5, .02, 5.)
    node.filter_time_ns = node.last_measurement_time_ns = None
    node.frame_id = 'local_ned'
    node.measurement_timeout = .5
    node.prediction_horizon = .5
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=round(now[0] * 1e9)))
    states = []
    node.state_pub = SimpleNamespace(publish=states.append)
    node.prediction_pub = SimpleNamespace(publish=lambda value: None)
    return node, states


def observation(stamp):
    message = TargetObservation()
    message.stamp = Time(nanoseconds=round(stamp * 1e9)).to_msg()
    message.frame_id = 'local_ned'
    message.position.x = 8.
    message.covariance = [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.01]
    message.valid = True
    return message


def test_control_images_request_retransmission_without_unbounded_backlog():
    from rclpy.qos import HistoryPolicy, ReliabilityPolicy
    from uav_control.perception import rgbd_target_localizer as localizer
    from uav_control.perception import target_bearing_node as bearing

    factory = getattr(localizer, 'aligned_camera_qos', None)
    assert callable(factory), 'Control images need reliable delivery from image_bridge'
    assert getattr(bearing, 'aligned_camera_qos', None) is factory
    qos = factory()
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert 2 <= qos.depth <= 8


def test_kf_publishes_new_measurement_without_waiting_for_timer():
    now = [10.025]
    node, states = filter_node(now)
    node.measurement_callback(observation(10.))
    assert len(states) == 1
    assert states[0].source_stamp == Time(nanoseconds=10_000_000_000).to_msg()
    assert states[0].stamp == Time(nanoseconds=10_025_000_000).to_msg()
    assert states[0].position.x == pytest.approx(8.)
    assert fresh_flight_target(states[0], 10.1, .125) is not None
    assert fresh_flight_target(states[0], 10.126, .125) is None
    node.measurement_callback(observation(10.))
    old = observation(9.9)
    node.measurement_callback(old)
    old.valid = False
    node.measurement_callback(old)
    assert len(states) == 1


def test_ready_rgbd_runs_in_executor_before_periodic_localization_timer():
    now = [100.02]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100., (0., 0., -5.))
    node.position_history.add(100.04, (0., 0., -5.))
    node.attitude_history.add(100., (1., 0., 0., 0.))
    node.attitude_history.add(100.04, (1., 0., 0., 0.))
    ready = []
    node.localization_guard = SimpleNamespace(trigger=lambda: ready.append(node.localize))
    color, depth = _rgbd_messages(100.)
    node.color_callback(color)
    node.depth_callback(depth)
    # DDS callbacks only queue work. Run the queued executor event at receipt.
    assert not observations
    for callback in ready:
        callback()
    assert len(observations) == 1
    assert observations[0].valid
    assert observations[0].stamp == Time(nanoseconds=100_000_000_000).to_msg()
    node.localize()  # Later periodic timer cannot duplicate the frame.
    assert len(observations) == 1


def test_pose_arrival_wakes_waiting_frame_without_extrapolation():
    now = [100.12]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100., (0., 0., -5.))
    node.attitude_history.add(100., (1., 0., 0., 0.))
    ready = []
    node.localization_guard = SimpleNamespace(trigger=lambda: ready.append(node.localize))
    color, depth = _rgbd_messages(100.1)
    node.color_callback(color)
    node.depth_callback(depth)
    while ready:
        ready.pop(0)()
    assert not observations
    assert node.waiting_image_pair is not None
    # Mapping is held fixed here; native causal mapping has its own regression suite.
    node._map_px4_pose_stamp = lambda stamp, receipt: (stamp, stamp, stamp)
    node.position_callback(SimpleNamespace(
        x=0., y=0., z=-5., timestamp_sample=100_140_000))
    while ready:
        ready.pop(0)()
    assert not observations  # An attitude right bracket is still required.
    node.attitude_callback(SimpleNamespace(
        q=(1., 0., 0., 0.), timestamp_sample=100_140_000))
    now[0] = 100.145
    while ready:
        ready.pop(0)()
    assert len(observations) == 1
    assert observations[0].valid
    assert observations[0].stamp == Time(nanoseconds=100_100_000_000).to_msg()


def test_delivery_reaches_follow_and_translation_with_original_age_gate():
    now = [10.]
    kf, states = filter_node(now)
    visibility = TargetVisibilityState()
    mission = MissionManagerCore()
    mission.set_flight_ready(True)
    mission.handle_command('X', 9.)
    mission.mark_takeoff_complete(9.9)
    for stamp in (10., 10.1, 10.2):
        now[0] = stamp + .025
        kf.measurement_callback(observation(stamp))
        assert states, 'new KF measurement must be published before its timer'
        visibility.observe(stamp, .05, True, now[0], position_valid=True)
        control_time = stamp + .05
        decision = visibility.update(control_time, 0., 5., .05,
                                     fresh_flight_target(states[-1], control_time, .125)
                                     is not None)
        mission.observe_visibility(mission.mission_id, decision.state,
                                   decision.locked, control_time)
    assert mission.phase == MissionPhase.FOLLOW
    assert mission.target_locked
    now[0] = 10.30
    decision = visibility.update(now[0], 0., 5., .05,
                                 fresh_flight_target(states[-1], now[0], .125) is not None)
    mission.observe_visibility(mission.mission_id, decision.state, decision.locked, now[0])
    assert mission.tick(now[0]) == MissionPhase.FOLLOW
    controller, state = runnable_follow_node()
    controller.visibility = visibility
    controller.latest_kf_message = states[-1]
    controller.latest_state = replace(state, stamp=now[0])
    controller._ros_seconds = lambda: now[0]
    controller.timer_callback()
    assert controller.diagnostics[-1][1] == 'FOLLOW'
    assert controller.diagnostics[-1][0].velocity[0] > 0.
    assert controller.yaw_owner == 'VISION'
    now[0] = 10.326  # No new acquisition: publishing a projection cannot rescue it.
    kf.timer_callback()
    controller.latest_kf_message = states[-1]
    controller.latest_state = replace(state, stamp=now[0])
    controller.timer_callback()
    assert controller.diagnostics[-1][1] == 'FOLLOW'
    assert controller.visibility_decision.state == 'INPUT_WAIT'
    assert controller.visibility_decision.visual_locked
    assert not controller.visibility_decision.locked
    assert controller.latest_target_state is None
    assert not controller.diagnostics[-1][0].far_guidance_available
    assert controller.diagnostics[-1][0].velocity == (0., 0., 0.)
