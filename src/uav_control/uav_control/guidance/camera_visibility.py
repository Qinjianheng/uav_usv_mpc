# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""
Transport-free pinhole visibility in PX4 NED/FRD and Gazebo camera FLU.

Rotations are active: R_AB maps coordinates in B into A. Inputs must share
one epoch and NED origin; callers own time alignment and source validation.
The sphere model bounds target extent, but does not test scene occlusion.
"""

from dataclasses import dataclass
import math
from numbers import Real
from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation


_FLU_TO_FRD = np.diag((1.0, -1.0, -1.0))
_OPTICAL_TO_CAMERA_FLU = np.array(((0.0, 0.0, 1.0),
                                  (-1.0, 0.0, 0.0),
                                  (0.0, -1.0, 0.0)))
_BOUNDARY_TOLERANCE = 1e-12


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _positive_dimension(value):
    return isinstance(value, (int, np.integer)) and not isinstance(value, bool) and value > 0


def _array(value, reason):
    try:
        return np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(reason) from error


def _vector(value, reason):
    array = _array(value, reason)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(reason)
    return array


def _rotation(value, reason):
    array = _array(value, reason)
    if (array.shape != (3, 3) or not np.all(np.isfinite(array))
            or np.any(np.abs(array) > 1.0 + 1e-7)
            or not np.allclose(array.T @ array, np.eye(3), atol=1e-7, rtol=0.0)
            or not math.isclose(float(np.linalg.det(array)), 1.0, abs_tol=1e-7)):
        raise ValueError(reason)
    return array


def body_frd_to_ned_from_quaternion(quaternion):
    """Adapt PX4 Hamilton wxyz, normalizing as the existing localizer does."""
    q = _array(quaternion, 'INVALID_BODY_QUATERNION')
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('INVALID_BODY_QUATERNION')
    norm = math.hypot(*q)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('INVALID_BODY_QUATERNION')
    w, x, y, z = q / norm
    return Rotation.from_quat((x, y, z, w)).as_matrix()


@dataclass(frozen=True)
class CameraIntrinsics:
    """Rectified pinhole calibration; continuous sensor edges are 0..width/height."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_horizontal_fov(cls, width, height, horizontal_fov):
        """Use the current RGB-D square-pixel focal/principal-point convention."""
        if (not _finite(horizontal_fov) or not 0.0 < horizontal_fov < math.pi
                or not _positive_dimension(width) or not _positive_dimension(height)):
            raise ValueError('INVALID_INTRINSICS')
        fov = float(horizontal_fov)
        focal = width / (2.0 * math.tan(fov / 2.0))
        result = cls(width, height, focal, focal, width / 2.0, height / 2.0)
        result.validate()
        return result

    def validate(self):
        """Reject uncalibrated/nonfinite cameras rather than inventing intrinsics."""
        if (not all(_positive_dimension(v) for v in (self.width, self.height))
                or not all(_finite(v) for v in (self.fx, self.fy, self.cx, self.cy))
                or self.fx <= 0.0 or self.fy <= 0.0
                or not 0.0 < self.cx < self.width or not 0.0 < self.cy < self.height):
            raise ValueError('INVALID_INTRINSICS')

    def angle_bounds(self):
        """Return left/right/up/down optical angles, including off-center principal points."""
        return (math.atan2(-self.cx, self.fx), math.atan2(self.width - self.cx, self.fx),
                math.atan2(-self.cy, self.fy), math.atan2(self.height - self.cy, self.fy))


@dataclass(frozen=True)
class CameraExtrinsics:
    """Mount relative to navigation reference: FLU translation and camera-FLU->body-FLU."""

    translation_flu: tuple
    rotation_camera_to_body_flu: tuple

    @classmethod
    def from_sdf_pose(cls, translation_flu, roll, pitch, yaw):
        """Apply SDF Rz(yaw) Ry(pitch) Rx(roll); positive pitch points +X down."""
        translation = _vector(translation_flu, 'INVALID_EXTRINSICS')
        angles = _vector((roll, pitch, yaw), 'INVALID_EXTRINSICS')
        matrix = Rotation.from_euler('xyz', angles).as_matrix()
        return cls(tuple(translation), tuple(tuple(row) for row in matrix))


@dataclass(frozen=True)
class TargetBoundingSphere:
    """Conservative target extent with an explicit world/NED center offset."""

    radius: float = 0.0
    center_offset_ned: tuple = (0.0, 0.0, 0.0)

    @classmethod
    def from_box_dimensions(cls, dimensions, center_offset_ned=(0.0, 0.0, 0.0)):
        """Enclose a box of positive full lengths without assuming its orientation."""
        dimensions = _vector(dimensions, 'INVALID_TARGET_GEOMETRY')
        offset = _vector(center_offset_ned, 'INVALID_TARGET_GEOMETRY')
        if np.any(dimensions <= 0.0):
            raise ValueError('INVALID_TARGET_GEOMETRY')
        radius = math.hypot(*dimensions) / 2.0
        if not math.isfinite(radius):
            raise ValueError('INVALID_TARGET_GEOMETRY')
        return cls(radius, tuple(offset))


@dataclass(frozen=True)
class VisibilityConstraints:
    """Angular safety insets and explicit axial-depth or radial-distance limits."""

    minimum_distance: float
    maximum_distance: float
    horizontal_margin_rad: float = 0.0
    vertical_margin_rad: float = 0.0
    distance_mode: str = 'axial'
    depth_epsilon: float = 1e-9

    def validate(self, bounds):
        """Reject invalid ranges and safety insets which empty the camera frustum."""
        values = (self.minimum_distance, self.maximum_distance, self.horizontal_margin_rad,
                  self.vertical_margin_rad, self.depth_epsilon)
        if (not all(_finite(v) for v in values)
                or not 0.0 <= self.minimum_distance < self.maximum_distance
                or self.depth_epsilon <= 0.0
                or self.distance_mode not in ('axial', 'radial')
                or not 0.0 <= self.horizontal_margin_rad < (bounds[1] - bounds[0]) / 2
                or not 0.0 <= self.vertical_margin_rad < (bounds[3] - bounds[2]) / 2):
            raise ValueError('INVALID_CONSTRAINTS')


@dataclass(frozen=True)
class CameraVisibilityResult:
    """Center diagnostics and whole-sphere safety; undefined projections/margins are None."""

    valid_input: bool = False
    target_camera_optical: Optional[Tuple[float, float, float]] = None
    target_camera_flu: Optional[Tuple[float, float, float]] = None
    image_center_uv: Optional[Tuple[float, float]] = None
    image_bounds_uv: Optional[Tuple[float, float, float, float]] = None
    center_in_front: bool = False
    whole_in_front: bool = False
    center_in_image: bool = False
    center_in_range: bool = False
    center_visible: bool = False
    whole_in_range: bool = False
    distance_interval_m: Optional[Tuple[float, float]] = None
    horizontal_margin_rad: Optional[float] = None
    vertical_margin_rad: Optional[float] = None
    whole_target_safe: bool = False
    reasons: tuple = ()


def _validated_inputs(uav_position_ned, body_frd_to_ned, target_position_ned,
                      intrinsics, extrinsics, target, constraints):
    position = _vector(uav_position_ned, 'INVALID_UAV_POSITION')
    rotation = _rotation(body_frd_to_ned, 'INVALID_BODY_ROTATION')
    target_position = _vector(target_position_ned, 'INVALID_TARGET_POSITION')
    if not isinstance(intrinsics, CameraIntrinsics):
        raise ValueError('INVALID_INTRINSICS')
    intrinsics.validate()
    if not isinstance(extrinsics, CameraExtrinsics):
        raise ValueError('INVALID_EXTRINSICS')
    translation = _vector(extrinsics.translation_flu, 'INVALID_EXTRINSICS')
    mount = _rotation(extrinsics.rotation_camera_to_body_flu, 'INVALID_EXTRINSICS')
    if (not isinstance(target, TargetBoundingSphere) or not _finite(target.radius)
            or target.radius < 0.0):
        raise ValueError('INVALID_TARGET_GEOMETRY')
    offset = _vector(target.center_offset_ned, 'INVALID_TARGET_GEOMETRY')
    if not isinstance(constraints, VisibilityConstraints):
        raise ValueError('INVALID_CONSTRAINTS')
    constraints.validate(intrinsics.angle_bounds())
    return position, rotation, target_position, translation, mount, offset


def _evaluate_geometry(position, rotation, target_position, translation, mount, offset,
                       intrinsics, target, constraints):
    camera_origin = position + rotation @ _FLU_TO_FRD @ translation
    optical_to_ned = rotation @ _FLU_TO_FRD @ mount @ _OPTICAL_TO_CAMERA_FLU
    optical = optical_to_ned.T @ (target_position + offset - camera_origin)
    if not np.all(np.isfinite(optical)):
        raise FloatingPointError('nonfinite transformed position')
    x, y, z = (float(v) for v in optical)
    radius = float(target.radius)
    center_front = z > constraints.depth_epsilon
    whole_front = z - radius > constraints.depth_epsilon
    center_distance = z if constraints.distance_mode == 'axial' else math.hypot(x, y, z)
    if not math.isfinite(center_distance):
        raise FloatingPointError('nonfinite distance')
    near, far = center_distance - radius, center_distance + radius
    if constraints.distance_mode == 'radial':
        near = max(0.0, near)
    center_range = (constraints.minimum_distance - _BOUNDARY_TOLERANCE <= center_distance
                    <= constraints.maximum_distance + _BOUNDARY_TOLERANCE)
    whole_range = (near >= constraints.minimum_distance - _BOUNDARY_TOLERANCE
                   and far <= constraints.maximum_distance + _BOUNDARY_TOLERANCE)
    if not all(math.isfinite(v) for v in (near, far)):
        raise FloatingPointError('nonfinite target interval')
    reasons = []
    uv = bbox = horizontal_margin = vertical_margin = None
    center_image = False
    if not center_front:
        reasons.append('TARGET_BEHIND_CAMERA' if z < -constraints.depth_epsilon else 'ZERO_DEPTH')
    else:
        uv = (intrinsics.fx * (x / z) + intrinsics.cx,
              intrinsics.fy * (y / z) + intrinsics.cy)
        center_image = (-_BOUNDARY_TOLERANCE <= uv[0] <= intrinsics.width + _BOUNDARY_TOLERANCE
                        and -_BOUNDARY_TOLERANCE <= uv[1]
                        <= intrinsics.height + _BOUNDARY_TOLERANCE)
        if not whole_front:
            reasons.append('TARGET_CROSSES_CAMERA_PLANE')
        else:
            h_angle, v_angle = math.atan2(x, z), math.atan2(y, z)
            h_extent = math.asin(min(1.0, radius / math.hypot(x, z)))
            v_extent = math.asin(min(1.0, radius / math.hypot(y, z)))
            h0, h1 = h_angle - h_extent, h_angle + h_extent
            v0, v1 = v_angle - v_extent, v_angle + v_extent
            left, right, up, down = intrinsics.angle_bounds()
            horizontal_margin = min(h0 - left, right - h1) - constraints.horizontal_margin_rad
            vertical_margin = min(v0 - up, down - v1) - constraints.vertical_margin_rad
            bbox = (intrinsics.fx * math.tan(h0) + intrinsics.cx,
                    intrinsics.fy * math.tan(v0) + intrinsics.cy,
                    intrinsics.fx * math.tan(h1) + intrinsics.cx,
                    intrinsics.fy * math.tan(v1) + intrinsics.cy)
            if horizontal_margin < -_BOUNDARY_TOLERANCE:
                reasons.append('HORIZONTAL_FOV')
            if vertical_margin < -_BOUNDARY_TOLERANCE:
                reasons.append('VERTICAL_FOV')
    if near < constraints.minimum_distance - _BOUNDARY_TOLERANCE:
        reasons.append('BELOW_MINIMUM_DISTANCE')
    if far > constraints.maximum_distance + _BOUNDARY_TOLERANCE:
        reasons.append('ABOVE_MAXIMUM_DISTANCE')
    for projection in (uv, bbox):
        if projection is not None and not all(math.isfinite(v) for v in projection):
            raise FloatingPointError('nonfinite projection')
    return CameraVisibilityResult(
        valid_input=True, target_camera_optical=(x, y, z),
        target_camera_flu=tuple(_OPTICAL_TO_CAMERA_FLU @ optical),
        image_center_uv=uv, image_bounds_uv=bbox, center_in_front=center_front,
        whole_in_front=whole_front, center_in_image=center_image, center_in_range=center_range,
        center_visible=center_front and center_image and center_range, whole_in_range=whole_range,
        distance_interval_m=(near, far), horizontal_margin_rad=horizontal_margin,
        vertical_margin_rad=vertical_margin, whole_target_safe=not reasons, reasons=tuple(reasons),
    )


def evaluate_visibility(uav_position_ned, body_frd_to_ned, target_position_ned,
                        intrinsics, extrinsics, target, constraints):
    """
    Evaluate one explicit same-epoch pose/target snapshot without ROS or data access.

    Invalid numeric inputs return valid_input=False and a specific reason.
    A valid pose behind/outside the camera returns valid_input=True but unsafe.
    No default mount, vehicle attitude, source, clock, or target reference is inferred.
    """
    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            values = _validated_inputs(uav_position_ned, body_frd_to_ned, target_position_ned,
                                       intrinsics, extrinsics, target, constraints)
            return _evaluate_geometry(*values, intrinsics, target, constraints)
    except (FloatingPointError, OverflowError):
        return CameraVisibilityResult(reasons=('NUMERICAL_ERROR',))
    except ValueError as error:
        return CameraVisibilityResult(reasons=(str(error),))
