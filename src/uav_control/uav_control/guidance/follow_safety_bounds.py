"""
Conservative geometry over an explicitly bounded interval, without live certification.

Caller bounds must cover the ENTIRE interval, not just measured sample errors.
No calibrated receiver error-tube source currently exists; this module never
constructs SafetyApproval, holding deadlines, or actual bridge permissions.
"""
from dataclasses import dataclass
import math

import numpy as np

from uav_control.guidance.camera_visibility import evaluate_visibility


@dataclass(frozen=True)
class ErrorBounds:
    """Metres except combined body/mount/interval rotation angle in radians."""

    uav_position: float
    target_position: float
    rotation_angle: float
    extrinsic_translation: float
    relative_motion: float


def interval_visibility(position, rotation, target_position, intrinsics, extrinsics,
                        sphere, constraints, bounds):
    """
    Enclose all uncertain optical centres in a ball; test all four unit planes.

    ||R1-R0||2 = 2 sin(theta/2). The lever arm is included when bounding
    the target vector about the body origin. Minkowski addition inflates the
    full target sphere. Axial/radial range uses the same conservative ball.
    """
    result = dict(robust_nominal=False, holding_qualified=False,
                  reason='UNTRUSTED_ERROR_BOUNDS')
    if not isinstance(bounds, ErrorBounds):
        return result
    values = tuple(bounds.__dict__.values())
    if (not all(math.isfinite(v) and v >= 0 for v in values)
            or bounds.rotation_angle > math.pi):
        return dict(result, reason='INVALID_ERROR_BOUNDS')
    nominal = evaluate_visibility(position, rotation, target_position, intrinsics,
                                  extrinsics, sphere, constraints)
    if not nominal.valid_input:
        return dict(result, reason='INVALID_GEOMETRY')
    q = np.asarray(nominal.target_camera_optical)
    translation = bounds.uav_position+bounds.target_position+bounds.extrinsic_translation
    lever = math.hypot(*extrinsics.translation_flu)
    body_distance = math.hypot(*q)+lever+translation+bounds.relative_motion
    if not math.isfinite(body_distance):
        return dict(result, reason='INVALID_ERROR_BOUNDS')
    angular = body_distance*(2*math.sin(bounds.rotation_angle/2))
    radius = sphere.radius+translation+bounds.relative_motion+angular
    if not math.isfinite(radius):
        return dict(result, reason='INVALID_ERROR_BOUNDS')
    left, right, up, down = intrinsics.angle_bounds()
    left += constraints.horizontal_margin_rad
    right -= constraints.horizontal_margin_rad
    up += constraints.vertical_margin_rad
    down -= constraints.vertical_margin_rad
    normals = np.array(((math.cos(left), 0., -math.sin(left)),
                        (-math.cos(right), 0., math.sin(right)),
                        (0., math.cos(up), -math.sin(up)),
                        (0., -math.cos(down), math.sin(down))))
    planes = normals@q-radius
    depth = q[2] if constraints.distance_mode == 'axial' else np.linalg.norm(q)
    ranges = (depth-radius-constraints.minimum_distance,
              constraints.maximum_distance-depth-radius,
              q[2]-radius-constraints.depth_epsilon)
    safe = bool(np.all(planes >= 0) and min(ranges) >= 0)
    return dict(result, robust_nominal=safe, inflated_radius=float(radius),
                minimum_plane_margin=float(planes.min()), range_margins=list(ranges),
                reason='RECEIVER_ERROR_TUBE_UNQUALIFIED' if safe else 'ROBUST_FOV_OR_RANGE_FAILED')
