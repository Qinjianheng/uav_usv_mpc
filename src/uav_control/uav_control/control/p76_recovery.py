"""Latched entry-velocity braking after control authority has been withdrawn."""
import math
from types import SimpleNamespace

import numpy as np


class VelocityRecovery:
    """Advance a bounded stopping request instead of chasing each new measured velocity."""

    def __init__(self):
        self.started, self.initial = None, None

    def command(self, velocity, now, braking):
        """Return a continuous velocity request; navigation freshness is checked by its caller."""
        velocity = np.asarray(velocity, dtype=float)
        if (velocity.shape != (3,) or not np.isfinite(velocity).all()
                or not math.isfinite(now) or not math.isfinite(braking) or braking <= 0):
            raise ValueError('P76_RECOVERY_INPUT')
        if self.started is None:
            self.started, self.initial = now, velocity.copy()
        if now < self.started:
            raise ValueError('P76_RECOVERY_CLOCK')
        fraction = min(1., braking*(now-self.started)/max(np.linalg.norm(self.initial), 1e-9))
        return self.initial*(1-fraction)

    def setpoint(self, position, velocity, now, braking):
        """Share the bounded trajectory derivative and PX4 mapping with calibration."""
        desired = self.command(velocity, now, braking)
        speed = np.linalg.norm(self.initial)
        acceleration = (-braking*self.initial/speed
                        if speed > 1e-9 and now-self.started < speed/braking else np.zeros(3))
        return SimpleNamespace(position=position, velocity=desired, acceleration=acceleration)
