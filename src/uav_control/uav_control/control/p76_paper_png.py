"""Yan2025 equations3/5/7–10: pixel LOS and dual-plane proportional navigation."""
import math

import numpy as np


def unit(vector):
    """Normalize a finite three-vector or reject a singular direction."""
    result = np.asarray(vector, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all() or np.linalg.norm(result) < 1e-10:
        raise ValueError('P76_DIRECTION')
    return result/np.linalg.norm(result)


def camera_origin(position_ned, body_frd_to_ned, translation_flu):
    """Locate the camera origin; translation never silently changes a unit bearing."""
    return np.asarray(position_ned)+np.asarray(body_frd_to_ned)@(
        np.asarray(translation_flu)*np.array((1., -1., -1.)))


def camera_los(center_uv, calibration, body_frd_to_ned, pitch_down):
    """Rotate a camera-origin optical ray into NED using the actual calibrated mount."""
    cx, cy, fx, fy = calibration
    u, v = center_uv
    rotation = np.asarray(body_frd_to_ned, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6)
            or not all(math.isfinite(x) for x in (*calibration, u, v, pitch_down))
            or fx <= 0 or fy <= 0):
        raise ValueError('P76_LOS_CALIBRATION')
    # Optical(right,down,forward) -> camera FLU(forward,left,up).
    ray_flu = np.array((1., -(u-cx)/fx, -(v-cy)/fy))
    c, s = math.cos(pitch_down), math.sin(pitch_down)
    mount = np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))
    return unit(rotation@np.diag((1., -1., -1.))@mount@ray_flu)


def paper_angles(direction):
    """Equations7/8, with atan2 explicitly extending the original forward hemisphere."""
    x, y, z = unit(direction)
    return math.atan2(z, math.hypot(x, y)), math.atan2(x, y)


def direction_from_angles(vertical, horizontal):
    """Equation10 unit direction; horizontal angle is measured from the positive Y axis."""
    return np.array((math.cos(vertical)*math.sin(horizontal),
                     math.cos(vertical)*math.cos(horizontal), math.sin(vertical)))


def wrapped_difference(current, previous):
    """Return the continuous shortest angular increment across the branch cut."""
    return math.atan2(math.sin(current-previous), math.cos(current-previous))


class DualPlanePng:
    """Equation9 state tied to distinct estimate epochs, not repeated detector callbacks."""

    def __init__(self, ky=3., kz=3.):
        self.ky, self.kz = float(ky), float(kz)
        if not all(math.isfinite(x) and x > 0 for x in (self.ky, self.kz)):
            raise ValueError('PNG_GAIN')
        self.stamp = None

    def initialize(self, los, velocity, stamp):
        """Seed both LOS and actual velocity angles before granting steering authority."""
        self.los_angles = paper_angles(los)
        velocity = np.asarray(velocity)
        fallback = unit(velocity) if np.linalg.norm(velocity) >= .05 else unit(los)
        self.velocity_angles = paper_angles(fallback)
        self.direction = fallback
        self.stamp = float(stamp)
        if not math.isfinite(self.stamp):
            raise ValueError('PNG_EPOCH')

    def update(self, los, velocity, stamp):
        """Use previous measured velocity plus the unique current LOS increment."""
        stamp = float(stamp)
        if self.stamp is None:
            self.initialize(los, velocity, stamp)
            return self.direction.copy()
        if not math.isfinite(stamp) or stamp < self.stamp:
            raise ValueError('PNG_EPOCH')
        if stamp == self.stamp:
            return self.direction.copy()
        angles = paper_angles(los)
        increments = [wrapped_difference(a, b) for a, b in zip(angles, self.los_angles)]
        self.direction = direction_from_angles(self.velocity_angles[0]+self.ky*increments[0],
                                               self.velocity_angles[1]+self.kz*increments[1])
        velocity = np.asarray(velocity)
        if velocity.shape != (3,) or not np.isfinite(velocity).all():
            raise ValueError('PNG_VELOCITY')
        if np.linalg.norm(velocity) >= .05:
            self.velocity_angles = paper_angles(velocity)
        self.los_angles, self.stamp = angles, stamp
        return self.direction.copy()
