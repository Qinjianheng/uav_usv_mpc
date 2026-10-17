# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""Independent geometry, boundary, and existing-localizer compatibility checks."""

from dataclasses import replace
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from uav_control.guidance.camera_visibility import (
    CameraExtrinsics,
    CameraIntrinsics,
    TargetBoundingSphere,
    VisibilityConstraints,
    body_frd_to_ned_from_quaternion,
    evaluate_visibility,
)


INTRINSICS = CameraIntrinsics(640, 480, 320.0, 240.0, 320.0, 240.0)
MOUNT = CameraExtrinsics.from_sdf_pose((0.0, 0.0, 0.0), 0.0, 0.0, 0.0)
LIMITS = VisibilityConstraints(0.05, 25.0)


def _view(position, rotation=None, camera=INTRINSICS, mount=MOUNT,
          target=TargetBoundingSphere(), limits=LIMITS, uav=(0.0, 0.0, 0.0)):
    if rotation is None:
        rotation = np.eye(3)
    return evaluate_visibility(uav, rotation, position, camera, mount, target, limits)


def test_level_vehicle_axis_hits_center_with_explicit_horizontal_test_mount():
    result = _view((10.0, 0.0, 0.0))
    assert result.valid_input and result.whole_target_safe
    assert result.target_camera_flu == pytest.approx((10.0, 0.0, 0.0))
    assert result.target_camera_optical == pytest.approx((0.0, 0.0, 10.0))
    assert result.image_center_uv == pytest.approx((320.0, 240.0))
    assert result.horizontal_margin_rad == pytest.approx(math.pi / 4)
    assert result.vertical_margin_rad == pytest.approx(math.pi / 4)
    assert result.reasons == ()


@pytest.mark.parametrize('position,uv,axis', [
    ((10.0, -10.0, 0.0), (0.0, 240.0), 'horizontal'),
    ((10.0, 10.0, 0.0), (640.0, 240.0), 'horizontal'),
    ((10.0, 0.0, -10.0), (320.0, 0.0), 'vertical'),
    ((10.0, 0.0, 10.0), (320.0, 480.0), 'vertical'),
])
def test_known_sensor_edge_rays_and_safety_inset(position, uv, axis):
    result = _view(position)
    assert result.image_center_uv == pytest.approx(uv)
    assert result.center_in_image and result.whole_target_safe
    assert getattr(result, axis + '_margin_rad') == pytest.approx(0.0, abs=1e-12)
    inset = _view(position, limits=replace(
        LIMITS, horizontal_margin_rad=0.01, vertical_margin_rad=0.01))
    assert inset.center_visible and not inset.whole_target_safe
    assert getattr(inset, axis + '_margin_rad') == pytest.approx(-0.01)
    assert axis.upper() + '_FOV' in inset.reasons


def test_yaw_quarter_turn_moves_northeast_target_from_right_to_left():
    q = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    rotation = body_frd_to_ned_from_quaternion(q)
    assert _view((10.0, 10.0, 0.0)).image_center_uv == pytest.approx((640.0, 240.0))
    result = _view((10.0, 10.0, 0.0), rotation)
    assert result.target_camera_optical == pytest.approx((-10.0, 0.0, 10.0))
    assert result.image_center_uv == pytest.approx((0.0, 240.0), abs=1e-10)


def test_positive_px4_pitch_raises_nose_and_moves_level_target_down():
    q = (math.cos(math.pi / 12), 0.0, math.sin(math.pi / 12), 0.0)
    result = _view((10.0, 0.0, 0.0), body_frd_to_ned_from_quaternion(q))
    assert result.target_camera_optical == pytest.approx((0.0, 5.0, 5 * math.sqrt(3)))
    assert result.image_center_uv == pytest.approx((320.0, 240.0 + 240 / math.sqrt(3)))


def test_positive_px4_roll_rotates_image_right_target_up():
    q = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
    result = _view((10.0, 2.0, 0.0), body_frd_to_ned_from_quaternion(q))
    assert result.target_camera_optical == pytest.approx((0.0, -2.0, 10.0), abs=1e-12)
    assert result.image_center_uv == pytest.approx((320.0, 192.0))


def test_sdf_positive_pitch_points_camera_down():
    mount = CameraExtrinsics.from_sdf_pose((0.0, 0.0, 0.0), 0.0, math.pi / 2, 0.0)
    result = _view((0.0, 0.0, 10.0), mount=mount)
    assert result.target_camera_optical == pytest.approx((0.0, 0.0, 10.0), abs=1e-12)
    assert result.image_center_uv == pytest.approx((320.0, 240.0))


def test_actual_28_degree_mount_and_navigation_origin_offset():
    pitch = math.radians(28.0)
    mount = CameraExtrinsics.from_sdf_pose((0.18, 0.0, 0.39), 0.0, pitch, 0.0)
    # Measured installation: axis descends 4.6947 m over 8.8295 m north.
    result = _view((9.00947592858927, 0.0, 4.304715627858908), mount=mount)
    assert result.target_camera_optical == pytest.approx((0.0, 0.0, 10.0), abs=1e-12)
    assert result.image_center_uv == pytest.approx((320.0, 240.0))
    # A level horizontal ray sits above the tilted camera's image center.
    assert _view((10.18, 0.0, -0.39), mount=mount).image_center_uv[1] < 240


def test_mount_translation_rotates_with_vehicle_and_is_not_world_fixed():
    mount = CameraExtrinsics.from_sdf_pose((1.0, 2.0, 3.0), 0.0, 0.0, 0.0)
    yaw90 = np.array(((0, -1, 0), (1, 0, 0), (0, 0, 1)))
    # At yaw +90, the mount's world/NED center is (2,1,-3).
    result = _view((2.0, 11.0, -3.0), yaw90, mount=mount)
    assert result.target_camera_optical == pytest.approx((0.0, 0.0, 10.0))


def test_center_visible_but_sphere_crosses_safe_right_edge():
    result = _view((10.0, 8.0, 0.0), target=TargetBoundingSphere(2.0))
    assert result.center_visible and not result.whole_target_safe
    assert result.image_center_uv == pytest.approx((576.0, 240.0))
    assert result.image_bounds_uv[2] > 640.0
    assert result.horizontal_margin_rad < 0.0
    assert 'HORIZONTAL_FOV' in result.reasons


def test_sphere_tangent_on_45_degree_edge_has_zero_margin():
    # Distance of (forward=10,right=8) to plane forward-right=0 is sqrt(2).
    result = _view((10.0, 8.0, 0.0), target=TargetBoundingSphere(math.sqrt(2)))
    assert result.whole_target_safe
    assert result.horizontal_margin_rad == pytest.approx(0.0, abs=1e-12)
    assert result.image_bounds_uv[2] == pytest.approx(640.0)


def test_sphere_silhouette_matches_independent_ray_tangency():
    result = _view((10.0, 3.0, 4.0), target=TargetBoundingSphere(1.5))
    u0, v0, u1, v1 = result.image_bounds_uv
    # A bounding ray line is exactly radius 1.5 from the 2D circle center.
    for pixel in (u0, u1):
        slope = (pixel - 320.0) / 320.0
        assert abs(3.0 - 10.0 * slope) / math.hypot(1.0, slope) == pytest.approx(1.5)
    for pixel in (v0, v1):
        slope = (pixel - 240.0) / 240.0
        assert abs(4.0 - 10.0 * slope) / math.hypot(1.0, slope) == pytest.approx(1.5)


@pytest.mark.parametrize('forward,reason', [
    (-10.0, 'TARGET_BEHIND_CAMERA'), (0.0, 'ZERO_DEPTH'), (1e-10, 'ZERO_DEPTH'),
])
def test_no_projection_at_or_behind_camera_plane(forward, reason):
    result = _view((forward, 0.0, 0.0))
    assert result.valid_input and not result.center_in_front
    assert not result.whole_target_safe and result.image_center_uv is None
    assert result.horizontal_margin_rad is None
    assert reason in result.reasons


def test_center_in_front_but_sphere_intersects_camera_plane():
    result = _view((0.5, 0.0, 0.0), target=TargetBoundingSphere(0.5))
    assert result.center_visible and not result.whole_in_front
    assert result.image_center_uv == pytest.approx((320.0, 240.0))
    assert result.image_bounds_uv is None and not result.whole_target_safe
    assert 'TARGET_CROSSES_CAMERA_PLANE' in result.reasons


@pytest.mark.parametrize('position,reason', [
    ((0.01, 0, 0), 'BELOW_MINIMUM_DISTANCE'), ((26, 0, 0), 'ABOVE_MAXIMUM_DISTANCE'),
])
def test_rgb_projection_survives_invalid_depth_range(position, reason):
    result = _view(position)
    assert result.center_in_image and not result.center_in_range
    assert result.image_center_uv == pytest.approx((320.0, 240.0))
    assert not result.center_visible and not result.whole_target_safe
    assert reason in result.reasons


def test_whole_sphere_range_is_stricter_than_center_range():
    result = _view((24.9, 0.0, 0.0), target=TargetBoundingSphere(0.25))
    assert result.center_visible and not result.whole_in_range
    assert result.distance_interval_m == pytest.approx((24.65, 25.15))
    assert 'ABOVE_MAXIMUM_DISTANCE' in result.reasons


def test_axial_and_radial_ranges_are_explicit_and_different():
    axial = _view((20.0, 18.0, 0.0))
    radial = _view((20.0, 18.0, 0.0), limits=replace(LIMITS, distance_mode='radial'))
    assert axial.whole_target_safe and not radial.whole_target_safe
    assert radial.distance_interval_m == pytest.approx((math.sqrt(724),) * 2)
    assert 'ABOVE_MAXIMUM_DISTANCE' in radial.reasons


def test_radial_distance_cannot_be_negative_when_camera_is_inside_sphere():
    limits = replace(LIMITS, minimum_distance=0, distance_mode='radial')
    result = _view((0.5, 0, 0), target=TargetBoundingSphere(1), limits=limits)
    assert result.distance_interval_m == pytest.approx((0, 1.5))
    assert result.whole_in_range and not result.whole_target_safe
    assert result.reasons == ('TARGET_CROSSES_CAMERA_PLANE',)


def test_exact_near_far_sphere_boundaries_are_inclusive():
    near = _view((1.0, 0, 0), target=TargetBoundingSphere(0.95))
    far = _view((24.0, 0, 0), target=TargetBoundingSphere(1.0))
    assert near.whole_in_range and far.whole_in_range


def test_target_reference_offset_is_explicit_and_world_fixed():
    result = _view((10.0, 0.0, 0.42), target=TargetBoundingSphere(0.25, (0, 0, -0.42)))
    assert result.image_center_uv == pytest.approx((320.0, 240.0))
    assert TargetBoundingSphere.from_box_dimensions((2, 4, 4)).radius == pytest.approx(3)


def test_off_center_intrinsics_use_asymmetric_frustum():
    camera = replace(INTRINSICS, cx=100.0, cy=150.0)
    result = _view((10, 0, 0), camera=camera)
    assert result.image_center_uv == pytest.approx((100.0, 150.0))
    assert result.horizontal_margin_rad == pytest.approx(math.atan(100 / 320))
    outside = _view((10, -4, 0), camera=camera)
    assert not outside.center_in_image and not outside.whole_target_safe


@pytest.mark.parametrize('kwargs,reason', [
    ({'uav': (math.nan, 0, 0)}, 'INVALID_UAV_POSITION'),
    ({'uav': ('bad', 0, 0)}, 'INVALID_UAV_POSITION'),
    ({'rotation': np.diag((1, 1, -1))}, 'INVALID_BODY_ROTATION'),
    ({'rotation': np.diag((1, 1, 2))}, 'INVALID_BODY_ROTATION'),
    ({'rotation': np.full((3, 3), math.inf)}, 'INVALID_BODY_ROTATION'),
    ({'rotation': np.eye(2)}, 'INVALID_BODY_ROTATION'),
    ({'rotation': 'bad'}, 'INVALID_BODY_ROTATION'),
    ({'rotation': np.diag((1, 1, 1e308))}, 'INVALID_BODY_ROTATION'),
    ({'camera': replace(INTRINSICS, fx=0)}, 'INVALID_INTRINSICS'),
    ({'camera': replace(INTRINSICS, fx=None)}, 'INVALID_INTRINSICS'),
    ({'camera': replace(INTRINSICS, fy=math.inf)}, 'INVALID_INTRINSICS'),
    ({'camera': replace(INTRINSICS, cx=641)}, 'INVALID_INTRINSICS'),
    ({'camera': replace(INTRINSICS, width=True)}, 'INVALID_INTRINSICS'),
    ({'camera': replace(INTRINSICS, height=2.5)}, 'INVALID_INTRINSICS'),
    ({'mount': CameraExtrinsics((0, 0, math.nan), np.eye(3))}, 'INVALID_EXTRINSICS'),
    ({'mount': CameraExtrinsics((0, 0, 0), -np.eye(3))}, 'INVALID_EXTRINSICS'),
    ({'target': TargetBoundingSphere(-1)}, 'INVALID_TARGET_GEOMETRY'),
    ({'target': TargetBoundingSphere(None)}, 'INVALID_TARGET_GEOMETRY'),
    ({'target': TargetBoundingSphere(math.inf)}, 'INVALID_TARGET_GEOMETRY'),
    ({'target': TargetBoundingSphere(1, (0, math.nan, 0))}, 'INVALID_TARGET_GEOMETRY'),
    ({'limits': replace(LIMITS, maximum_distance=0)}, 'INVALID_CONSTRAINTS'),
    ({'limits': replace(LIMITS, minimum_distance=None)}, 'INVALID_CONSTRAINTS'),
    ({'limits': replace(LIMITS, minimum_distance=-1)}, 'INVALID_CONSTRAINTS'),
    ({'limits': replace(LIMITS, horizontal_margin_rad=-0.1)}, 'INVALID_CONSTRAINTS'),
    ({'limits': replace(LIMITS, vertical_margin_rad=math.pi)}, 'INVALID_CONSTRAINTS'),
    ({'limits': replace(LIMITS, distance_mode='guess')}, 'INVALID_CONSTRAINTS'),
    ({'limits': replace(LIMITS, depth_epsilon=0)}, 'INVALID_CONSTRAINTS'),
])
def test_invalid_inputs_fail_closed_with_specific_reason(kwargs, reason):
    result = _view((10, 0, 0), **kwargs)
    assert not result.valid_input and not result.whole_target_safe
    assert not result.center_visible and result.image_center_uv is None
    assert result.reasons == (reason,)


@pytest.mark.parametrize('position', [(math.inf, 0, 0), (10, math.nan, 0), (1, 2)])
def test_invalid_target_position(position):
    assert _view(position).reasons == ('INVALID_TARGET_POSITION',)


@pytest.mark.parametrize('q', [(0, 0, 0, 0), (1, 0, 0), (math.nan, 0, 0, 0)])
def test_quaternion_adapter_rejects_invalid_attitude(q):
    with pytest.raises(ValueError):
        body_frd_to_ned_from_quaternion(q)


def test_quaternion_sign_and_scale_do_not_change_rotation():
    q = np.array((0.5, 0.5, 0.5, 0.5))
    assert body_frd_to_ned_from_quaternion(-2 * q) == pytest.approx(
        body_frd_to_ned_from_quaternion(q))


@pytest.mark.parametrize('width,height,fov', [
    (0, 480, 1.74), (640, 480, 0), (640, 480, math.nan), (640, 480, math.pi),
    ('bad', 480, 1.74), (640, 480, None),
])
def test_intrinsics_factory_rejects_invalid_calibration(width, height, fov):
    with pytest.raises(ValueError):
        CameraIntrinsics.from_horizontal_fov(width, height, fov)


def test_factories_reject_invalid_mount_and_box_dimensions():
    with pytest.raises(ValueError):
        CameraExtrinsics.from_sdf_pose((0, 0, 0), 0, math.nan, 0)
    with pytest.raises(ValueError):
        TargetBoundingSphere.from_box_dimensions((1, -2, 3))


def test_extreme_finite_coordinates_report_numerical_failure():
    result = _view((1e308, 0, 0), uav=(-1e308, 0, 0))
    assert not result.valid_input and result.reasons == ('NUMERICAL_ERROR',)


@pytest.mark.parametrize('rpy', [(0, 0, 0), (0.2, -0.3, 1.1), (-0.3, 0.1, -0.5)])
def test_existing_rgbd_forward_inverse_and_intrinsics_agree(rpy):
    # Compatibility only; independent known-axis/tangent tests above are the oracle.
    from uav_control.perception import rgbd_target_localizer as localizer
    q_xyzw = Rotation.from_euler('xyz', rpy).as_quat()
    q = (q_xyzw[3], *q_xyzw[:3])
    uav = (3.0, 4.0, -5.0)
    mount = CameraExtrinsics.from_sdf_pose((0.18, 0, 0.39), 0, math.radians(28), 0)
    camera = CameraIntrinsics.from_horizontal_fov(640, 480, 1.74)
    ray = (8.0, -1.0, -0.5)
    position = localizer.camera_target_to_local_ned(
        ray, uav, q, mount.translation_flu, math.radians(28), 0.42)
    result = _view(position, body_frd_to_ned_from_quaternion(q), camera, mount,
                   TargetBoundingSphere(0.25, (0, 0, -0.42)), uav=uav)
    assert result.target_camera_flu == pytest.approx(ray)
    assert result.target_camera_optical == pytest.approx((1.0, 0.5, 8.0))
    assert body_frd_to_ned_from_quaternion(q) == pytest.approx(
        localizer.body_frd_to_ned_rotation(q))
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == pytest.approx(
        localizer.camera_intrinsics(640, 480, 1.74))
    assert result.target_camera_flu == pytest.approx(localizer.local_ned_target_to_camera_flu(
        position - np.array((0, 0, 0.42)), uav, q, mount.translation_flu, math.radians(28)))


def test_enu_flu_and_ned_frd_transform_directions_match_known_axes():
    # Gazebo +90 yaw: body forward points north, body left points west.
    enu_to_ned = np.array(((0, 1, 0), (1, 0, 0), (0, 0, -1)))
    gazebo_rotation = np.array(((0, -1, 0), (1, 0, 0), (0, 0, 1)))
    ned_rotation = enu_to_ned @ gazebo_rotation @ np.diag((1, -1, -1))
    assert ned_rotation == pytest.approx(np.eye(3))
    position_enu = np.array((2.0, 10.0, -1.0))
    result = _view(enu_to_ned @ position_enu, ned_rotation)
    assert result.target_camera_flu == pytest.approx((10.0, -2.0, -1.0))
    assert result.image_center_uv == pytest.approx((384.0, 264.0))


def test_current_sdf_and_config_extrinsics_are_consistent():
    root = Path(__file__).resolve().parents[3]
    bringup = root / 'src/uav_usv_bringup'
    model = ET.parse(bringup / 'models/x500_mono_cam/model.sdf').getroot()
    pose = [float(x) for x in model.find('.//link[@name="front_camera_link"]/pose').text.split()]
    config = yaml.safe_load((bringup / 'config/baseline.yaml').read_text())
    params = config['rgbd_target_localizer']['ros__parameters']
    translation = tuple(params['camera_translation_' + axis] for axis in 'xyz')
    assert translation == pytest.approx((pose[0], pose[1], pose[2] + 0.24))
    assert params['camera_pitch_down'] == pytest.approx(pose[4])
    assert pose[3] == 0 and pose[5] == 0


def test_full_sdf_mount_roll_and_yaw_are_applied_in_order():
    # Rz(+90) Rx(+90): camera forward->body left, left->up, up->forward.
    mount = CameraExtrinsics.from_sdf_pose((0, 0, 0), math.pi / 2, 0, math.pi / 2)
    assert np.asarray(mount.rotation_camera_to_body_flu) == pytest.approx(
        np.array(((0, 0, 1), (1, 0, 0), (0, 1, 0))), abs=1e-12)
    result = _view((0, -10, 0), mount=mount)
    assert result.image_center_uv == pytest.approx((320, 240))
    assert result.target_camera_optical == pytest.approx((0, 0, 10), abs=1e-12)
