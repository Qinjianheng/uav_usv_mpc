"""Validate a physical UAV sample without replacing its acquisition epoch."""

import math


def navigation_components(message, now):
    """Return mapped sample time and P/V/A; reject invalid or future samples."""
    stamp = float(message.stamp.sec) + float(message.stamp.nanosec) * 1e-9
    if (not message.valid or message.frame_id != 'local_ned'
            or not math.isfinite(stamp) or not 0.0 < stamp <= float(now)):
        raise ValueError('invalid navigation sample time or frame')
    position = tuple(float(getattr(message.position, k)) for k in 'xyz')
    velocity = tuple(float(getattr(message.velocity, k)) for k in 'xyz')
    acceleration = tuple(float(getattr(message.acceleration, k)) for k in 'xyz')
    if not all(math.isfinite(v) for v in (*position, *velocity, *acceleration)):
        raise ValueError('nonfinite navigation state')
    return stamp, position, velocity, acceleration
