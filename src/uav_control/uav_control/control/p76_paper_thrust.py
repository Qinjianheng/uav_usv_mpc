"""Physical force and independently calibrated PX4 normalized collective thrust."""
import math

import numpy as np


def physical_thrust(mass, rotation_frd_to_ned, desired_acceleration_ned, gravity, maximum):
    """Dimensionally consistent equation23: m times lift-direction specific force."""
    if not all(math.isfinite(v) and v > 0 for v in (mass, gravity, maximum)):
        raise ValueError('P76_PHYSICAL_THRUST')
    lift = -np.asarray(rotation_frd_to_ned)[:, 2]
    force = mass*float(lift@(np.asarray(desired_acceleration_ned)-(0., 0., gravity)))
    return float(np.clip(force, 0., maximum))


class CalibratedThrustMap:
    """Monotone force/normalized-output samples; model-derived guesses stay unqualified."""

    def __init__(self, force_newtons, normalized, *, qualified=False):
        self.force, self.normalized = np.asarray(force_newtons), np.asarray(normalized)
        if (self.force.ndim != 1 or len(self.force) < 3
                or self.force.shape != self.normalized.shape
                or not np.isfinite(self.force).all() or not np.isfinite(self.normalized).all()
                or np.any(np.diff(self.force) <= 0) or np.any(np.diff(self.normalized) <= 0)
                or self.force[0] < 0 or self.normalized[0] < 0
                or self.normalized[-1] > 1):
            raise ValueError('THRUST_CALIBRATION')
        self.qualified = bool(qualified)

    def to_body_z(self, force_newtons, require_qualified=True):
        """Return negative FRD-z normalized collective; reject absent actual qualification."""
        if not math.isfinite(force_newtons) or require_qualified and not self.qualified:
            raise ValueError('THRUST_MAPPING_UNQUALIFIED')
        if force_newtons < self.force[0]:
            raise ValueError('THRUST_BELOW_CALIBRATED_RANGE')
        return -float(np.interp(force_newtons, self.force, self.normalized))
