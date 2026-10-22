"""Exact finite-sphere pixel observability; no ROS state, truth input or control owner."""
from dataclasses import dataclass
from functools import lru_cache
import math

import numpy as np

from .camera_visibility import CameraIntrinsics


@dataclass(frozen=True)
class RgbdVisibility:
    """Separate projected RGB support, clipped depth, fit quality and surface safety."""

    valid: bool
    reason: str
    projected_pixels: int = 0
    rgb_pixels: int = 0
    depth_pixels: int = 0
    depth_ratio: float = 0.
    surface_gap: float = 0.
    center: tuple = ()


@lru_cache(maxsize=4)
def _rays(intrinsics):
    rows, columns = np.indices((intrinsics.height, intrinsics.width))
    rays = np.stack((np.ones_like(rows), (intrinsics.cx-columns)/intrinsics.fx,
                     (intrinsics.cy-rows)/intrinsics.fy), axis=-1)
    return rays, np.sum(rays*rays, axis=-1)


def evaluate_rgbd_sphere(center, *, intrinsics=None, radius=.25,
                         rgb_near=.05, rgb_far=60., depth_near=.05, depth_far=25.,
                         minimum_red_pixels=3, minimum_depth_ratio=.5, surface_clearance=.02):
    """
    Check actual pixel rays, axial clipping and the existing ideal sphere-fit gate.

    Counts cover the full rectangular sensor. Sampling the already valid pixels
    follows the localizer's exact row-major 512-point fit stride. Scene occlusion,
    model-mesh/rasterization error and sensor noise remain outside this ideal model.
    """
    intrinsics = intrinsics or CameraIntrinsics.from_horizontal_fov(640, 480, 1.74)
    intrinsics.validate()
    center = np.asarray(center, dtype=float)
    if (center.shape != (3,) or not np.isfinite(center).all()
            or not all(math.isfinite(v) for v in (radius, rgb_near, rgb_far, depth_near,
                                                  depth_far, surface_clearance))
            or not radius > 0 or not 0 < rgb_near < rgb_far
            or not 0 < depth_near < depth_far or surface_clearance < .02
            or not 0 <= minimum_depth_ratio <= 1 or minimum_red_pixels < 3):
        return RgbdVisibility(False, 'INVALID_GEOMETRY')
    gap = float(np.linalg.norm(center))-radius
    rays, norm2 = _rays(intrinsics)
    dot = rays@center
    disc = dot*dot-norm2*(center@center-radius*radius)
    depth = (dot-np.sqrt(np.maximum(0., disc)))/norm2
    projected = (disc >= 0.) & (dot > 0.) & (depth > 0.)
    rgb = projected & (depth >= rgb_near) & (depth <= rgb_far)
    valid = rgb & (depth >= depth_near) & (depth <= depth_far)
    projected_count, rgb_count, valid_count = map(int, (
        projected.sum(), rgb.sum(), valid.sum()))
    ratio = valid_count/max(1, rgb_count)
    reason, fitted = 'VALID', None
    if rgb_count < minimum_red_pixels:
        reason = ('RGB_NEAR_CLIP' if projected_count >= minimum_red_pixels
                  and float(center[0]) < rgb_far else 'RGB_NOT_VISIBLE')
    elif valid_count < 12 or ratio < minimum_depth_ratio:
        reason = 'DEPTH_UNUSABLE'
    elif gap < surface_clearance-1e-9:
        reason = 'CAMERA_CLEARANCE'
    else:
        from ..perception.rgbd_target_localizer import validation_sphere_center
        indices = np.flatnonzero(valid)
        indices = indices[::max(1, math.ceil(len(indices)/512))]
        points = rays.reshape(-1, 3)[indices]*depth.ravel()[indices, None]
        fitted = validation_sphere_center(points, radius)
        if fitted is None:
            reason = 'FIT_QUALITY'
    return RgbdVisibility(reason == 'VALID', reason, projected_count, rgb_count,
                          valid_count, ratio, gap, () if fitted is None else fitted)
