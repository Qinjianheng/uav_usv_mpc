"""Equivalent complete-sphere geometry, including independent boundary cases."""
from dataclasses import asdict
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_control.guidance import camera_visibility_batch as batch
from uav_control.guidance.camera_visibility import (
    CameraExtrinsics, CameraIntrinsics, TargetBoundingSphere, VisibilityConstraints,
    _validated_inputs, evaluate_visibility,
)


def geometry(positions, rotations, targets, camera, mount, sphere, limits):
    values = _validated_inputs(np.zeros(3), np.eye(3), np.zeros(3),
                               camera, mount, sphere, limits)
    return batch._geometry_batch(np.asarray(positions), np.asarray(rotations),
                                 np.asarray(targets), *values[3:], camera, sphere, limits)


def assert_same(actual, expected):
    for key, value in asdict(expected).items():
        got = getattr(actual, key)
        if (value is None or isinstance(value, bool)
                or isinstance(value, tuple) and (not value or isinstance(value[0], str))):
            assert got == value, key
        else:
            assert np.allclose(got, value, atol=1e-10, rtol=1e-12), key


def test_vector_geometry_known_full_sphere_and_rejection_cases():
    camera = CameraIntrinsics.from_horizontal_fov(640, 480, 1.74)
    mount = CameraExtrinsics.from_sdf_pose((0., 0., 0.), 0., 0., 0.)
    sphere = TargetBoundingSphere(.25)
    limits = VisibilityConstraints(.05, 20., .07, .07)
    targets = np.array(((5., 0., 0.), (-2., 0., 0.), (.2, 0., 0.),
                        (25., 0., 0.), (5., 9., 0.), (5., 0., 9.)))
    values = geometry(np.zeros((6, 3)), np.tile(np.eye(3), (6, 1, 1)), targets,
                      camera, mount, sphere, limits)
    assert [r.whole_target_safe for r in values] == [True, False, False, False, False, False]
    assert values[0].target_camera_optical == (0., 0., 5.)
    assert values[0].image_center_uv == (320., 240.)
    assert values[0].distance_interval_m == (4.75, 5.25)
    assert values[0].horizontal_margin_rad == pytest.approx(.87-math.asin(.05)-.07)
    assert values[0].vertical_margin_rad == pytest.approx(
        math.atan2(240., camera.fy)-math.asin(.05)-.07)
    for result, reason in zip(values[1:], (
            'TARGET_BEHIND_CAMERA', 'TARGET_CROSSES_CAMERA_PLANE',
            'ABOVE_MAXIMUM_DISTANCE', 'HORIZONTAL_FOV', 'VERTICAL_FOV')):
        assert reason in result.reasons


@pytest.mark.parametrize('distance_mode', ('axial', 'radial'))
def test_batch_matches_original_scalar_for_full_poses_and_offset_calibration(distance_mode):
    rng = np.random.default_rng(61015)
    camera = CameraIntrinsics(640, 480, 270., 281., 307., 251.)
    mount = CameraExtrinsics.from_sdf_pose((.18, .03, .39), .06, .48, -.08)
    sphere = TargetBoundingSphere(.25, (.02, -.01, -.42))
    limits = VisibilityConstraints(.05, 25., .07, .07, distance_mode)
    p, t = rng.normal(size=(2, 400, 3))*8.
    r = Rotation.from_rotvec(rng.normal(size=(400, 3))).as_matrix()
    actual = geometry(p, r, t, camera, mount, sphere, limits)
    for i, result in enumerate(actual):
        assert_same(result, evaluate_visibility(p[i], r[i], t[i], camera, mount, sphere, limits))


def test_visibility_batch_preserves_scalar_boundaries_and_invalid_rows():
    camera = CameraIntrinsics.from_horizontal_fov(640, 480, 1.74)
    mount = CameraExtrinsics.from_sdf_pose((0., 0., 0.), 0., 0., 0.)
    sphere = TargetBoundingSphere(.25)
    limits = VisibilityConstraints(.05, 25., .07, .07)
    # Independent tangent ray: h_angle+asin(r/rho)=right inset boundary.
    angle = .87-.07-math.asin(.25/5.)
    targets = np.array([(5.*math.cos(angle+d), 5.*math.sin(angle+d), 0.)
                        for d in (-1e-10, 0., 1e-10)] +
                       [(25.-.25+d, 0., 0.) for d in (-1e-10, 0., 1e-10)] +
                       [(.25+1e-9, 0., 0.), (np.nan, 0., 0.), (1e308, 1e308, 0.)])
    p = np.zeros_like(targets)
    r = np.tile(np.eye(3), (len(p), 1, 1))
    r[-1, 0, 0] = 2.
    actual = batch.visibility_batch(p, r, targets, camera, mount, sphere, limits)
    for i, result in enumerate(actual):
        assert_same(result, evaluate_visibility(p[i], r[i], targets[i],
                                                camera, mount, sphere, limits))


@pytest.mark.parametrize('radius', (0., .25))
def test_valid_rotation_numeric_fallback_and_zero_depth_match_scalar(radius):
    camera = CameraIntrinsics.from_horizontal_fov(640, 480, 1.74)
    mount = CameraExtrinsics.from_sdf_pose((0., 0., 0.), 0., 0., 0.)
    sphere = TargetBoundingSphere(radius)
    limits = VisibilityConstraints(.05, 25., .07, .07)
    # All inputs are finite and every rotation is valid SO(3). The first row
    # overflows during relative-position subtraction, the second at projection.
    p = np.array(((-1e308, 0., 0.), (0., 0., 0.), (0., 0., 0.), (0., 0., 0.)))
    targets = np.array(((1e308, 0., 0.), (1., 1e308, 0.), (0., 0., 0.), (1., 0., 0.)))
    rotations = np.tile(np.eye(3), (len(p), 1, 1))
    actual = batch.visibility_batch(p, rotations, targets, camera, mount, sphere, limits)
    assert actual[0].reasons == actual[1].reasons == ('NUMERICAL_ERROR',)
    assert 'ZERO_DEPTH' in actual[2].reasons
    assert actual[3].whole_target_safe
    for i, result in enumerate(actual):
        assert_same(result, evaluate_visibility(p[i], rotations[i], targets[i],
                                                camera, mount, sphere, limits))
