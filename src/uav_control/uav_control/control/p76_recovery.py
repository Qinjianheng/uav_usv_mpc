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


class BodyrateRecovery:
    """Calibrated independent braking without invoking a target controller or Velocity mode."""

    def __init__(self, braking, delay):
        if not math.isfinite(braking) or braking <= 0 or not math.isfinite(delay) or delay <= 0:
            raise ValueError('P763_RECOVERY_CALIBRATION_REQUIRED')
        self.braking, self.delay = braking, delay
        self.reset()

    def reset(self):
        """Start a new independent recovery episode without inheriting handover timeouts."""
        self.started, self.hold_position, self.stopped = None, None, False

    def check_margin(self, outer, position, velocity, rotation, acceleration, age, dt, barrier):
        """Reserve ramp distance before crediting measured braking; A2 must verify this core."""
        from .p76_bodyrate_safety import BodyrateSeaBarrier
        s, c = outer.settings, outer.c['candidate']
        initial_downward = max(0., float(outer.previous_acceleration[2]),
                               float(np.asarray(acceleration)[2]))
        tilt = math.acos(float(np.clip(rotation[2, 2], -1., 1.)))
        # Do not credit instantaneous jerk, normalized collective or attitude changes.
        ramp = ((initial_downward+self.braking)/s['maximum_jerk']
                + 1./s['maximum_thrust_slew']
                + (tilt+s['maximum_tilt'])/c['maximum_body_roll_pitch_rate']
                + c['maximum_body_roll_pitch_rate']/c['maximum_body_angular_acceleration'])
        guard = BodyrateSeaBarrier(barrier.points, barrier.minimum_body_clearance,
                                   max(self.delay, barrier.delay)+ramp,
                                   min(self.braking, barrier.braking), barrier.sea)
        return guard.check(position, velocity, rotation, [0., 0., initial_downward], age, dt)

    def step(self, outer, position, velocity, rotation, omega, now, dt, age, barrier):
        """Use measured stopping margin before a bounded body-rate request, otherwise LAND."""
        p, v = np.asarray(position), np.asarray(velocity)
        if not 0 <= age <= .125:
            raise ValueError('P763_RECOVERY_NATIVE_STALE')
        if self.started is None:
            self.started, self.hold_position = now, p.copy()
        acceleration = -min(self.braking, outer.settings['maximum_acceleration'])*v/max(
            np.linalg.norm(v), .25)
        if self.check_margin(outer, p, v, rotation, acceleration, age, dt,
                             barrier) == 'UNRECOVERABLE':
            raise ValueError('P763_RECOVERY_STOPPING_MARGIN')
        if now-self.started > 5. and np.linalg.norm(v) > .25:
            raise ValueError('P763_RECOVERY_NOT_STOPPED')
        if np.linalg.norm(v) <= .25 and not self.stopped:
            self.hold_position, self.stopped = p.copy(), True
        # During deceleration no position pull-back competes with braking. Hold only after stop.
        reference = np.r_[p if np.linalg.norm(v) > .25 else self.hold_position,
                          v if np.linalg.norm(v) > .25 else np.zeros(3), acceleration,
                          math.atan2(rotation[1, 0], rotation[0, 0]), 0.]
        return outer.step(reference, p, v, rotation, dt, omega)
