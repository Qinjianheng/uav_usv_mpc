"""P7 camera/profile consistency and the actual sole-Tracker spatial path."""

import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from uav_control.guidance.camera_visibility import (
    CameraExtrinsics, CameraIntrinsics, TargetBoundingSphere, VisibilityConstraints,
    evaluate_visibility,
)

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
CANDIDATE = ROOT/'src/uav_usv_bringup/config/p7_spatial_candidate.yaml'


def test_effective_p7_geometry_rejects_radius_or_height_mismatch(tmp_path):
    from p7_follow_sitl import prepare, validate_p7_geometry
    config = yaml.safe_load(CANDIDATE.read_text())
    validate_p7_geometry(config)
    prepare(tmp_path/'run', 'f2', 'figure_eight', capture=True, config_file=CANDIDATE)
    flight = yaml.safe_load((tmp_path/'run/config/flight.yaml').read_text())
    research = yaml.safe_load((tmp_path/'run/config/research.yaml').read_text())
    assert flight['trajectory_tracker_node']['ros__parameters']['flight_altitude'] == -5.
    assert research['p4_follow_planner_node']['ros__parameters']['flight_altitude'] == -5.
    assert flight['trajectory_tracker_node']['ros__parameters']['p7_capture_enabled']
    config['intercept_evaluator_node']['ros__parameters']['evaluation_capture_radius'] = .25
    with pytest.raises(ValueError, match='RADIUS_CONFIG_MISMATCH'):
        validate_p7_geometry(config)


def test_session_camera_uses_config_pitch_and_preserves_all_translation_and_body(tmp_path):
    spec = importlib.util.spec_from_file_location(
        'p73_prepare', ROOT/'scripts/prepare_gz_camera_model.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = ROOT/'src/uav_usv_bringup/models/x500_mono_cam'
    original = ET.parse(model/'model.sdf')
    pose = original.find(".//link[@name='front_camera_link']/pose")
    source = [float(v) for v in pose.text.split()]
    assert source[4] == pytest.approx(math.radians(28.))
    # An explicit angle override stays in a session copy, never the source default.
    module.prepare(model, tmp_path/'candidate', front_camera_pitch_down=math.radians(38.))
    prepared = ET.parse(tmp_path/'candidate/model.sdf')
    actual_pose = prepared.find(".//link[@name='front_camera_link']/pose")
    actual = [float(v) for v in actual_pose.text.split()]
    assert actual[:4] == source[:4] and actual[5] == source[5]
    assert actual[4] == pytest.approx(math.radians(38.))


def test_failed_candidate_cannot_replace_default_follow_or_capture_criterion(tmp_path):
    from p7_follow_sitl import prepare
    prepare(tmp_path/'default', 'f2', capture=True)
    prepare(tmp_path/'candidate', 'f2', capture=True, config_file=CANDIDATE)
    for name, pitch, altitude, spatial in (
            ('default', 28., -5., False), ('candidate', 28., -5., True)):
        config = yaml.safe_load((tmp_path/name/'config/flight.yaml').read_text())
        tracker = config['trajectory_tracker_node']['ros__parameters']
        assert tracker.get('p7_spatial_enabled', False) is spatial
        assert tracker.get('p7_follow_geometry_enabled', False) is spatial
        assert tracker['flight_altitude'] == altitude
        assert config['rgbd_target_localizer']['ros__parameters'][
            'camera_pitch_down'] == pytest.approx(math.radians(pitch))
        evaluator = config['intercept_evaluator_node']['ros__parameters']
        assert evaluator['evaluation_capture_radius'] == evaluator['planned_capture_radius'] == .5


def test_f2_qualification_cannot_cross_between_default_and_candidate_profiles(tmp_path):
    import p7_follow_sitl as entry
    summary = tmp_path/'summary.json'
    summary.write_text(json.dumps(dict(
        healthy=True, completed=True, f3_eligible=True,
        source_fingerprint=entry.source_fingerprint())))
    (tmp_path/'condition.json').write_text(json.dumps(dict(
        config_sources={str(entry.CONFIG): entry.harness.sha(entry.CONFIG)})))
    entry.require_f2(summary)
    with pytest.raises(ValueError, match='CURRENT_HEALTHY_F2_REQUIRED'):
        entry.require_f2(summary, config_file=CANDIDATE)


def test_automated_spatial_capture_requires_qualified_a_before_creating_run(tmp_path, monkeypatch):
    import p7_follow_sitl as entry
    monkeypatch.setattr(entry, 'WORKSPACE', tmp_path)
    output = tmp_path/'data/experiments/no_flight'
    with pytest.raises(ValueError, match='CURRENT_HEALTHY_F2_REQUIRED'):
        entry.run(output, 'f2', guidance='d', capture=True, config_file=CANDIDATE)
    assert not output.exists()


@pytest.mark.parametrize('pitch,roll,heave', [(0., 0., 0.), (-12., 12., -.15), (12., -12., .15)])
def test_three_metre_follow_whole_sphere_visibility_uses_full_attitude(pitch, roll, heave):
    result = evaluate_visibility(
        (0., 0., -3.), Rotation.from_euler('xyz', (roll, pitch, 8.), degrees=True).as_matrix(),
        (5., 0., -heave-.42), CameraIntrinsics.from_horizontal_fov(640, 480, 1.74),
        CameraExtrinsics.from_sdf_pose((.18, 0., .39), 0., math.radians(42.), 0.),
        TargetBoundingSphere(.25), VisibilityConstraints(.05, 25.))
    assert result.center_visible and result.whole_target_safe


def test_partial_near_field_rgbd_fit_is_valid_but_not_whole_sphere_visibility():
    from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd
    intrinsic = CameraIntrinsics.from_horizontal_fov(640, 480, 1.74)
    view = evaluate_visibility(
        (0., 0., 0.), np.eye(3), (0., 0., .24-.42), intrinsic,
        CameraExtrinsics.from_sdf_pose((.18, 0., .39), 0., math.radians(42.), 0.),
        TargetBoundingSphere(.25), VisibilityConstraints(.05, 25.))
    center = np.array(view.target_camera_flu)
    yy, xx = np.mgrid[0:480, 0:640]
    rays = np.stack((np.ones_like(xx), (intrinsic.cx-xx)/intrinsic.fx,
                     (intrinsic.cy-yy)/intrinsic.fy), axis=-1)
    denominator = np.sum(rays*rays, axis=-1)
    dot = rays@center
    disc = dot*dot-denominator*(center@center-.25**2)
    depth = (dot-np.sqrt(np.maximum(0., disc)))/denominator
    mask = (disc >= 0) & (depth >= .05) & (depth <= 25.)
    fit = target_geometry_from_rgbd(mask, np.where(mask, depth, np.nan), 1.74,
                                    .05, 25., .25, require_valid_sphere=True)
    assert fit is not None and fit.center_camera == pytest.approx(center, abs=1e-7)
    assert not view.center_in_image and not view.whole_target_safe


def test_spatial_capture_retains_one_velocity_publisher_and_final_sea_barrier():
    from dataclasses import replace
    from test_follow_wait import runnable_follow_node, CapturedPublisher
    from test_strict_visual_control import target
    from uav_control.control.p7_spatial_guidance import P7SpatialGuidance, ImageSample
    from uav_usv_interfaces.msg import MissionState
    node, _ = runnable_follow_node()
    node.p7_guidance_enabled = node.p7_capture_enabled = node.p7_spatial_enabled = True
    node.p7_common_epoch_enabled = True
    node.p7_controller = P7SpatialGuidance()
    node.p7_controller.reset((2., 0., 0.))
    node.p7_image = (10.19, .1, .3)
    node.p7_spatial_image = ImageSample(
        10.19, (5., -.5, -1.5), (1., 0., 0., 0.),
        tuple(node.latest_state.position), (13., 2., 0.))
    node.p7_result = None
    node.p7_diagnostic_pub = CapturedPublisher()
    node.p7_actual_dt = .05
    node.mission_state = MissionState.FAR_GUIDANCE
    node.mission_state_name = 'FAR_GUIDANCE'
    node.intercept_requested = node.offboard_active = node.vehicle_armed = True
    node.vehicle_status_stamp, node.vehicle_status_timeout = 10.19, 2.
    node.latest_kf_message = target(now=10.2, source=10.19)
    node.latest_kf_message.position.x, node.latest_kf_message.position.y = 13., 2.
    node.follow_navigation_history = [dict(
        valid=True, stamp=10.2, p=node.latest_state.position, v=node.latest_state.velocity)]
    node.follow_target_history = [dict(
        valid=True, stamp=10.2, source=10.19, p=(13., 2., 0.), v=(0., 0., 0.))]
    node.tracker.previous_command_velocity = (2., 0., 0.)
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'P7_GUIDANCE'
    assert len(node.setpoint_pub.messages) == 1
    command = node.setpoint_pub.messages[-1]
    assert np.isnan(command.position).all() and np.isnan(command.acceleration).all()
    assert np.isfinite(command.velocity).all() and command.velocity[2] <= .6
    assert node.yaw_owner == 'IBVS'
    record = json.loads(node.p7_diagnostic_pub.messages[-1].data)
    assert record['feedback_epoch'] == 10.2
    assert record['feedback_navigation_bracket'] == [10.2, 10.2]
    node._ros_seconds = lambda: 10.25
    node.latest_state = replace(node.latest_state, stamp=10.25, position=(0., 0., -.19),
                                velocity=(2., 0., .6))
    node.timer_callback()
    # Existing final guard must still stop descent below the reserve; it is not bypassed by PNG.
    assert node.setpoint_pub.messages[-1].velocity[2] < .6
    node.p7_spatial_image = None
    node.timer_callback()
    assert node.p7_result is None


def test_p7_follow_sender_receiver_use_same_new_height_and_mount_contract():
    from types import SimpleNamespace
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_limits import ResearchConfig, constraint_snapshot
    from uav_control.control.p7_spatial_guidance import configure_p7_follow_geometry
    values = dict(p7_follow_geometry_enabled=True, flight_altitude=-3.,
                  p7_camera_pitch_down=math.radians(42.))
    node = SimpleNamespace(has_parameter=lambda name: name in values,
                           declare_parameter=lambda name, value: values.setdefault(name, value),
                           get_parameter=lambda name: SimpleNamespace(value=values[name]))
    receiver = FollowMpcSeed(ResearchConfig())
    planner = FollowMpcSeed(ResearchConfig(flight_altitude=-3.))
    configure_p7_follow_geometry(node, receiver)
    configure_p7_follow_geometry(node, planner)
    assert receiver.config.flight_altitude == -3.
    assert constraint_snapshot(receiver).fingerprint == constraint_snapshot(planner).fingerprint
    assert receiver.extrinsics == planner.extrinsics


def test_p7_height_health_uses_explicit_goal_without_changing_measured_position():
    from p7_follow_sitl import P7FollowHealthGuard
    from test_p53_sitl import healthy
    health = healthy(10., owner='MINCO_FOLLOW')
    health['navigation']['p'][2] = -3.
    guard = P7FollowHealthGuard(flight_altitude=-3.)
    assert guard(health, 10.) is None
    assert health['navigation']['p'][2] == -3.
    health['navigation']['p'][2] = -1.5
    assert guard(health, 10.) == 'HEIGHT_ERROR_OVER_1M'


def test_p7_reference_audit_checks_configured_altitude_and_keeps_old_default():
    from p51_execution_audit import audit_rows
    from test_p51_execution_audit import rows
    records = rows()
    for r in records:
        if r['kind'] == 'proposal':
            r['xyz'][2*(len(r['xyz'])//3)] += 2.
        elif r['kind'] == 'execution_debug':
            r['reference'][2] += 2.
    assert audit_rows(records, 100.04, 'f2', flight_altitude=-3.)['passed']
    assert not audit_rows(records, 100.04, 'f2')['passed']
