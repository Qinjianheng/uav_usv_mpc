"""Calibrated stopping envelope for body-rate control above the sea surface."""
import math

import numpy as np

from uav_control.common.p76_body_geometry import lowest_world_z


class BodyrateSeaBarrier:
    """Compare physical body clearance, including attitude, against calibrated braking."""

    def __init__(self, collision_points_frd, minimum_body_clearance, response_seconds,
                 braking_acceleration, sea_surface_z=0.):
        self.points = np.asarray(collision_points_frd, dtype=float)
        if (self.points.ndim != 2 or self.points.shape[1] != 3 or not len(self.points)
                or not np.isfinite(self.points).all() or not math.isfinite(minimum_body_clearance)
                or minimum_body_clearance < 0
                or not math.isfinite(response_seconds) or response_seconds <= 0
                or not math.isfinite(braking_acceleration) or braking_acceleration <= 0):
            raise ValueError('P76_SEA_CALIBRATION_REQUIRED')
        self.minimum_body_clearance = float(minimum_body_clearance)
        self.delay, self.braking = float(response_seconds), float(braking_acceleration)
        self.sea = float(sea_surface_z)

    def check(self, position, velocity, rotation_frd_to_ned, desired_acceleration=(0., 0., 0.),
              sample_age=0., control_interval=.02):
        """Include worst calibrated actuation/tilt delay before applying measured braking."""
        position, velocity, rotation = map(np.asarray, (position, velocity, rotation_frd_to_ned))
        if (position.shape != (3,) or velocity.shape != (3,) or rotation.shape != (3, 3)
                or not all(np.isfinite(v).all() for v in (position, velocity, rotation))
                or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6)):
            return 'UNRECOVERABLE'
        try:
            lower = lowest_world_z(self.points, position, rotation)
        except ValueError:
            return 'UNRECOVERABLE'
        margin = self.sea-lower-self.minimum_body_clearance
        if margin <= 0:
            return 'UNRECOVERABLE'
        down = max(0., float(velocity[2]))
        acceleration = np.asarray(desired_acceleration)
        if (acceleration.shape != (3,) or not np.isfinite(acceleration).all()
                or not 0 <= sample_age <= .125 or not 0 < control_interval <= .125):
            return 'UNRECOVERABLE'
        delay = self.delay+sample_age
        downward = max(0., float(acceleration[2]))
        delayed_speed = down+downward*delay
        distance = down*delay+.5*downward*delay**2+delayed_speed**2/(2*self.braking)
        if margin < distance:
            return 'UNRECOVERABLE'
        guard = delayed_speed*control_interval+.5*self.braking*control_interval**2
        return 'BRAKE' if margin <= distance+guard else 'CLEAR'
