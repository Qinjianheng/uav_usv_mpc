"""Independent half-plane and uncertainty cases, with no synthetic flight qualification."""
from dataclasses import replace
import math

import numpy as np
import pytest

from uav_control.guidance.camera_visibility import (
    CameraIntrinsics, CameraExtrinsics, TargetBoundingSphere, VisibilityConstraints,
)
from uav_control.guidance.follow_safety_bounds import ErrorBounds, interval_visibility


def evaluate(target=(10., 0., 0.), bounds=ErrorBounds(0., 0., 0., 0., 0.)):
    return interval_visibility((0., 0., 0.), np.eye(3), target,
                               CameraIntrinsics.from_horizontal_fov(640, 480, math.pi/2),
                               CameraExtrinsics.from_sdf_pose((0., 0., 0.), 0., 0., 0.),
                               TargetBoundingSphere(.1), VisibilityConstraints(.2, 20.), bounds)


def test_axis_center_has_known_plane_distance_and_never_grants_authority():
    result = evaluate()
    # vertical half-angle atan(0.75), sine=0.6, plane distance=6m minus radius.
    assert result['robust_nominal']
    assert result['minimum_plane_margin'] == pytest.approx(5.9)
    assert not result['holding_qualified']


def test_boundary_center_can_be_visible_while_uncertain_sphere_is_not():
    assert not evaluate((10., -10., 0.))['robust_nominal']
    assert not evaluate(bounds=ErrorBounds(3., 3., 0., 0., 0.))['robust_nominal']
    assert not evaluate(bounds=ErrorBounds(0., 0., math.pi/2, 0., 0.))['robust_nominal']


def test_interval_motion_and_mount_translation_are_explicit():
    base = ErrorBounds(0., 0., 0., 0., 0.)
    assert not evaluate(bounds=replace(base, relative_motion=6.))['robust_nominal']
    assert not evaluate(bounds=replace(base, extrinsic_translation=6.))['robust_nominal']
    moved = evaluate(bounds=replace(base, relative_motion=.1))
    assert moved['inflated_radius'] == pytest.approx(.2)


def test_missing_or_nonfinite_bounds_fail_closed():
    assert evaluate(bounds=None)['reason'] == 'UNTRUSTED_ERROR_BOUNDS'
    assert not evaluate(bounds=ErrorBounds(0., 0., math.nan, 0., 0.))['robust_nominal']
    assert not evaluate(bounds=ErrorBounds(-1., 0., 0., 0., 0.))['robust_nominal']


def test_finite_extreme_bounds_cannot_produce_nonfinite_report():
    result = evaluate(bounds=ErrorBounds(1e308, 1e308, 0., 0., 0.))
    assert result['reason'] == 'INVALID_ERROR_BOUNDS'
