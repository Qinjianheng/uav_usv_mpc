"""Opt-in 3-D approach; radius completion remains solely in the evaluator."""

import numpy as np
import pytest
from pathlib import Path
import sys
import math

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def test_image_corridor_contains_capture_geometry_inside_real_camera_frame():
    from uav_control.control.p7_visual_guidance import GuidanceConfig
    # Independent level-body geometry from the current SDF/YAML mount:
    # camera forward/up = 0.18/0.39 m, sphere center 0.42 m above USV,
    # camera points down 28 degrees. This endpoint is inside the 0.50 m sphere.
    assert math.hypot(.35, .33) < .5
    depression = math.atan2(.33+.39-.42, .35-.18)
    endpoint_error = math.tan(depression-math.radians(28.))
    frame_edge = math.tan(1.74/2.)*480./640.
    assert endpoint_error < GuidanceConfig().vertical_alignment_limit < frame_edge


def test_capture_coordinates_closure_with_descent_and_retains_sea_clearance():
    from uav_control.control.p7_visual_guidance import capture_reference
    distance, vz = capture_reference((0., 0., -5.), (5., 0., 0.), (2., 0., 0.))
    assert distance == pytest.approx(4.5)
    assert vz == .6
    distance, vz = capture_reference((0., 0., -.33), (.2, 0., 0.), (2., 0., 0.))
    assert distance == pytest.approx(.3) and vz == pytest.approx(0.)
    # A target below the sea cannot command a lower UAV floor.
    _, vz = capture_reference((0., 0., -.33), (.2, 0., .2), (2., 0., 0.))
    assert vz == pytest.approx(0.)
    with pytest.raises(ValueError):
        capture_reference((0., 0., float('nan')), (0., 0., 0.), (0., 0., 0.))
    # Measured terminal state from failed linear_02: 0.62 m vertical
    # separation cannot permit closing to 0.16 m with this fixed front camera.
    distance, _ = capture_reference((0., 0., -.52), (0.16, 0., .10), (2., 0., 0.))
    assert distance >= .65


@pytest.mark.parametrize('distance', (0., .01, .25, .99))
def test_capture_near_range_matches_velocity_without_png_division(distance):
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    controller = P7VisualGuidance()
    controller.reset((2., 0.))
    out = controller.step(10., .05, (0., 0.), (2., 0.), (distance, 0.), (2., 0.),
                          9.98, 0., .3, approach_distance=0.)
    assert out.valid and out.mode == 'CAPTURE_MATCH'
    assert np.isfinite(out.velocity).all()
    assert np.linalg.norm(np.array(out.velocity)-(2., 0.)) <= .1+1e-10
    default = P7VisualGuidance()
    default.reset((2., 0.))
    assert not default.step(10., .05, (0., 0.), (2., 0.), (distance, 0.), (2., 0.),
                            9.98, 0., .3).valid


def test_capture_stands_back_when_fresh_target_approaches_camera_lower_edge():
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    controller = P7VisualGuidance()
    controller.reset((2., 0.))
    out = controller.step(10., .05, (0., 0.), (2., 0.), (.6, 0.), (2., 0.),
                          9.98, 0., .8, approach_distance=.3)
    assert out.valid and out.velocity[0] < 2.


def test_coordinated_approach_reaches_original_3d_radius_in_ideal_kinematics():
    from uav_control.control.p7_visual_guidance import P7VisualGuidance, capture_reference
    controller = P7VisualGuidance()
    p, v, q, w = map(np.array, (
        (0., 0., -5.), (2., 0., 0.), (5., 0., 0.), (2., 0., 0.)))
    controller.reset(v)
    minimum_height = 5.
    for i in range(800):
        now = 10.+i*.05
        distance, vz = capture_reference(p, q, w)
        out = controller.step(now, .05, p[:2], v[:2], q[:2], w[:2], now-.02,
                              0., .3, approach_distance=distance)
        assert out.valid
        v = np.array((*out.velocity, vz))
        p += v*.05
        q += w*.05
        minimum_height = min(minimum_height, -p[2])
        if np.linalg.norm(q-p) <= .5:
            break
    assert np.linalg.norm(q-p) <= .5
    assert minimum_height >= .33


def test_pause_evidence_requires_repeated_clock_target_and_native_pose_samples():
    from p7_capture_analysis import freeze_metrics
    rows = [dict(kind='gazebo_stats', receipt=t, paused=True, sim=50.) for t in (11., 12., 13.)]
    for name in ('usv_target', 'x500_mono_cam_0'):
        rows.append(dict(kind='rendered_target' if name == 'usv_target' else 'rendered_uav',
                         receipt=10., name=name, p=[1., 2., 3.]))
        rows += [dict(kind='paused_native_pose', receipt=t, name=name, p=[1., 2., 3.])
                 for t in (11., 12., 13.)]
    rows += [dict(kind='ideal', receipt=t, p=[1., 2., 0.], v=[0., 0., 0.])
             for t in (11., 12., 13.)]
    assert freeze_metrics(rows, 10.)['verified']
    rows[-1]['p'][0] += .1
    assert not freeze_metrics(rows, 10.)['verified']
    assert not freeze_metrics([], 10.)['verified']


def test_freeze_rejects_repeated_initial_scene_poses_instead_of_current_native_poses():
    from p7_capture_analysis import freeze_metrics
    rows = [dict(kind='gazebo_stats', receipt=t, paused=True, sim=50.) for t in (11., 12., 13.)]
    for name in ('usv_target', 'x500_mono_cam_0'):
        rows.append(dict(kind='rendered_target' if name == 'usv_target' else 'rendered_uav',
                         receipt=10., name=name, p=[100., 0., -.4]))
        rows += [dict(kind='paused_native_pose', receipt=t, name=name, p=[8., 0., -.42])
                 for t in (11., 12., 13.)]
    rows += [dict(kind='ideal', receipt=t, p=[100., 0., 0.], v=[0., 0., 0.])
             for t in (11., 12., 13.)]
    assert not freeze_metrics(rows, 10.)['verified']


def test_world_pause_proof_is_separate_from_optional_repeated_native_pose_proof():
    from p7_capture_analysis import freeze_metrics
    rows = [dict(kind='gazebo_stats', receipt=t, paused=True, sim=50.) for t in (11., 12., 13.)]
    rows += [dict(kind='ideal', receipt=t, p=[100., 0., 0.], v=[0., 0., 0.])
             for t in (11., 12., 13.)]
    result = freeze_metrics(rows, 10.)
    assert result['world_verified']
    assert not result['verified']


def test_ecs_current_pose_decoder_uses_signed_protobuf_component_keys():
    from gz.msgs10.serialized_map_pb2 import SerializedStepMap
    from p4_shadow_monitor import component_id, state_model_poses
    assert component_id('Name') == -998690179357215250
    assert component_id('Pose') == -7527930132038368260
    state = SerializedStepMap()
    components = state.state.entities[42].components
    components[component_id('Name')].component = b'usv_target'
    components[component_id('Pose')].component = b'2 100 0.4 0 0 0'
    assert state_model_poses(state)['usv_target']['p'] == [100., 2., -.4]


def test_tracker_capture_descent_stops_and_recovers_when_visual_input_expires():
    from test_follow_wait import runnable_follow_node, CapturedPublisher
    from test_strict_visual_control import target
    from dataclasses import replace
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    from uav_usv_interfaces.msg import MissionState
    node, _ = runnable_follow_node()
    node.p7_guidance_enabled = node.p7_capture_enabled = True
    node.p7_controller = P7VisualGuidance()
    node.p7_diagnostic_pub = CapturedPublisher()
    node.p7_image = (10.19, 0., .3)
    node.mission_state = MissionState.FAR_GUIDANCE
    node.mission_state_name = 'FAR_GUIDANCE'
    node.offboard_active = node.vehicle_armed = True
    node.vehicle_status_stamp, node.vehicle_status_timeout = 10.19, 2.
    node.latest_kf_message = target(now=10.2, source=10.19)
    node.latest_kf_message.position.x = 3.
    node.latest_state = replace(node.latest_state, position=(0., 0., -1.))
    node.tracker.previous_command_velocity = (2., 0., 0.)
    node.timer_callback()
    assert node.setpoint_pub.messages[-1].velocity[2] > 0.
    node._ros_seconds = lambda: 10.4
    node.latest_state = replace(node.latest_state, stamp=10.4)
    node.timer_callback()
    assert node.p7_result is None
    assert node.setpoint_pub.messages[-1].velocity[2] <= 0.
