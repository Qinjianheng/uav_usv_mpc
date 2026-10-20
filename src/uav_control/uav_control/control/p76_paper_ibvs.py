"""Yan2025 pixel PD and explicitly adapted acceleration-based FOV holding14–16."""
import math

import numpy as np

from .p76_paper_png import unit


def image_yaw_rate(error_pixels, derivative_pixels, kp=.03, kd=.01):
    """Equation13 in pixel units; positive image right maps to positive FRD yaw."""
    values = (error_pixels, derivative_pixels, kp, kd)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('P76_PIXEL_PD')
    return kp*error_pixels+kd*derivative_pixels


def fov_vertical_excursion_bound(los_angle_change, acceleration, gravity, focal_y):
    """Equation16 bound with dynamics-induced tilt atan(|a|/g), in actual pixels."""
    angle = abs(los_angle_change)+math.atan2(abs(acceleration), gravity)
    return math.inf if angle >= math.pi/2 else focal_y*math.tan(angle)


def fov_holding_velocity(current_velocity, direction, dt, ka, maximum_speed, maximum_vertical):
    """Interpret equation14 as an acceleration ka integrated with the actual dt."""
    velocity = np.asarray(current_velocity, dtype=float)
    if (velocity.shape != (3,) or not np.isfinite(velocity).all()
            or not all(math.isfinite(v) and v > 0 for v in (dt, ka, maximum_speed,
                                                            maximum_vertical))
            or dt > .125):
        raise ValueError('P76_FOV_INPUT')
    desired = min(maximum_speed, float(np.linalg.norm(velocity))+ka*dt)*unit(direction)
    delta = desired-velocity
    norm = float(np.linalg.norm(delta))
    result = velocity+delta*min(1., ka*dt/max(norm, 1e-12))
    # Candidate physical speed limits are distinct from the paper's gains.
    xy = float(np.linalg.norm(result[:2]))
    result[:2] *= min(1., maximum_speed/max(xy, 1e-12))
    result[2] = float(np.clip(result[2], -maximum_vertical, maximum_vertical))
    return tuple(result)
