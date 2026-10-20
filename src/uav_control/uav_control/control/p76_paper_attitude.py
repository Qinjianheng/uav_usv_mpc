"""Yan2025 equations17–22 with explicit SO(3)/FRD print corrections."""
import math

import numpy as np

from .p76_paper_png import unit


def skew(vector):
    """Cross-product matrix in the stated right-handed frame."""
    x, y, z = vector
    return np.array(((0., -z, y), (z, 0., -x), (-y, x, 0.)))


def rotation_between(current, desired):
    """Unit-axis Rodrigues rotation, including deterministic antipodal directions."""
    a, b = unit(current), unit(desired)
    cross = np.cross(a, b)
    sine, cosine = float(np.linalg.norm(cross)), float(np.clip(a@b, -1., 1.))
    if sine < 1e-10:
        if cosine > 0:
            return np.eye(3)
        axis = unit(np.cross(a, np.eye(3)[int(np.argmin(np.abs(a)))]))
        return 2*np.outer(axis, axis)-np.eye(3)
    axis = cross/sine
    matrix = skew(axis)
    angle = math.atan2(sine, cosine)
    return np.eye(3)+math.sin(angle)*matrix+(1.-math.cos(angle))*(matrix@matrix)


def attitude_rates(rotation_frd_to_ned, desired_acceleration_ned, image_yaw, gravity):
    """Equations18–22 tilt correction plus physical FRD-z image yaw from equation13."""
    rotation = np.asarray(rotation_frd_to_ned, dtype=float)
    lift = -rotation[:, 2]
    desired_lift = unit(np.asarray(desired_acceleration_ned)-(0., 0., gravity))
    desired = rotation_between(lift, desired_lift)@rotation
    error = desired.T@rotation-rotation.T@desired
    rates = -np.array((error[2, 1], error[0, 2], error[1, 0]))
    rates[2] += image_yaw
    return rates
