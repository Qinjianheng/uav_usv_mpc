"""Continuous short-horizon reference generation for interception."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ReferenceState:
    """Position, velocity and acceleration of a guidance reference."""

    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    ax: float
    ay: float
    az: float


class RollingReferenceFilter:
    """
    Convert successive predicted target points into a continuous trajectory.

    A position-error feedback term makes the internal reference converge on
    the newest prediction.  Velocity and acceleration limits prevent a noisy
    prediction from instantaneously moving the reference across the scene.
    """

    def __init__(
        self,
        position_gain,
        max_horizontal_speed,
        max_vertical_speed,
        max_horizontal_acceleration,
        max_vertical_acceleration,
    ):
        """Configure reference convergence and kinematic limits."""
        self.position_gain = self._positive(
            position_gain,
            'position gain',
        )
        self.max_horizontal_speed = self._positive(
            max_horizontal_speed,
            'maximum horizontal speed',
        )
        self.max_vertical_speed = self._positive(
            max_vertical_speed,
            'maximum vertical speed',
        )
        self.max_horizontal_acceleration = self._positive(
            max_horizontal_acceleration,
            'maximum horizontal acceleration',
        )
        self.max_vertical_acceleration = self._positive(
            max_vertical_acceleration,
            'maximum vertical acceleration',
        )
        self.state = None

    @staticmethod
    def _positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _finite_vector(values, name):
        vector = tuple(float(value) for value in values)
        if len(vector) != 3 or not all(
            math.isfinite(value) for value in vector
        ):
            raise ValueError(f'{name} must contain three finite values')
        return vector

    @staticmethod
    def _limit_horizontal(x, y, limit):
        magnitude = math.hypot(x, y)
        if magnitude <= limit or magnitude <= 1e-12:
            return x, y
        scale = limit / magnitude
        return x * scale, y * scale

    def reset(self):
        """Discard the previous reference state."""
        self.state = None

    def initialize(self, position, velocity):
        """Initialize the continuous reference from one prediction."""
        x, y, z = self._finite_vector(position, 'position')
        vx, vy, vz = self._finite_vector(velocity, 'velocity')
        vx, vy = self._limit_horizontal(
            vx,
            vy,
            self.max_horizontal_speed,
        )
        vz = max(
            min(vz, self.max_vertical_speed),
            -self.max_vertical_speed,
        )
        self.state = ReferenceState(
            x,
            y,
            z,
            vx,
            vy,
            vz,
            0.0,
            0.0,
            0.0,
        )
        return self.state

    def update(self, position, velocity, dt):
        """Advance and return the acceleration-limited reference state."""
        target_x, target_y, target_z = self._finite_vector(
            position,
            'position',
        )
        target_vx, target_vy, target_vz = self._finite_vector(
            velocity,
            'velocity',
        )
        dt = self._positive(dt, 'time step')

        if self.state is None:
            return self.initialize(position, velocity)

        old = self.state
        propagated_x = old.x + old.vx * dt
        propagated_y = old.y + old.vy * dt
        propagated_z = old.z + old.vz * dt
        desired_vx = target_vx + self.position_gain * (
            target_x - propagated_x
        )
        desired_vy = target_vy + self.position_gain * (
            target_y - propagated_y
        )
        desired_vz = target_vz + self.position_gain * (
            target_z - propagated_z
        )
        desired_vx, desired_vy = self._limit_horizontal(
            desired_vx,
            desired_vy,
            self.max_horizontal_speed,
        )
        desired_vz = max(
            min(desired_vz, self.max_vertical_speed),
            -self.max_vertical_speed,
        )

        delta_vx = desired_vx - old.vx
        delta_vy = desired_vy - old.vy
        max_horizontal_delta = self.max_horizontal_acceleration * dt
        delta_vx, delta_vy = self._limit_horizontal(
            delta_vx,
            delta_vy,
            max_horizontal_delta,
        )
        delta_vz = max(
            min(
                desired_vz - old.vz,
                self.max_vertical_acceleration * dt,
            ),
            -self.max_vertical_acceleration * dt,
        )

        new_vx = old.vx + delta_vx
        new_vy = old.vy + delta_vy
        new_vz = old.vz + delta_vz
        new_x = old.x + 0.5 * (old.vx + new_vx) * dt
        new_y = old.y + 0.5 * (old.vy + new_vy) * dt
        new_z = old.z + 0.5 * (old.vz + new_vz) * dt
        self.state = ReferenceState(
            new_x,
            new_y,
            new_z,
            new_vx,
            new_vy,
            new_vz,
            delta_vx / dt,
            delta_vy / dt,
            delta_vz / dt,
        )
        return self.state
