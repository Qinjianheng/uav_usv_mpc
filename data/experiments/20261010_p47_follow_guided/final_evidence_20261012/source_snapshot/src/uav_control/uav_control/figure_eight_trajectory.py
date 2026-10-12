"""Constant-speed planar figure-eight trajectory generation."""

import math


class FigureEightTrajectory:
    """Gerono figure eight anchored at its left-most point."""

    def __init__(
        self,
        initial_x,
        initial_y,
        x_amplitude,
        y_amplitude,
        speed,
    ):
        self.initial_x = float(initial_x)
        self.initial_y = float(initial_y)
        self.x_amplitude = self._positive_finite(
            x_amplitude,
            'x amplitude',
        )
        self.y_amplitude = self._positive_finite(
            y_amplitude,
            'y amplitude',
        )
        self.speed = self._positive_finite(speed, 'speed')

        # Starting at phase pi places the target at (initial_x, initial_y)
        # with its velocity pointing along the positive Y direction.
        self.phase = math.pi
        self.center_x = self.initial_x + self.x_amplitude
        self.center_y = self.initial_y

    @staticmethod
    def _positive_finite(value, description):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(
                f'Figure-eight {description} must be positive.'
            )
        return value

    @staticmethod
    def _nonnegative_finite(value, description):
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                f'Figure-eight {description} must be non-negative.'
            )
        return value

    def set_speed(self, speed):
        """Set the instantaneous path speed, allowing a stationary start."""
        self.speed = self._nonnegative_finite(speed, 'speed')

    def maximum_curvature(self, sample_count=4096):
        """Numerically bound curvature over one complete figure eight."""
        sample_count = max(int(sample_count), 32)
        maximum = 0.0
        for index in range(sample_count):
            phase = 2.0 * math.pi * index / sample_count
            derivative_x, derivative_y = self._path_derivative(phase)
            second_x = -self.x_amplitude * math.cos(phase)
            second_y = -4.0 * self.y_amplitude * math.sin(2.0 * phase)
            speed_parameter = math.hypot(derivative_x, derivative_y)
            curvature = abs(
                derivative_x * second_y
                - derivative_y * second_x
            ) / (speed_parameter ** 3)
            maximum = max(maximum, curvature)
        return maximum

    def kinematic_envelope(self, speed=None):
        """Return peak turn rate and lateral acceleration at path speed."""
        if speed is None:
            speed = self.speed
        speed = self._nonnegative_finite(speed, 'speed')
        curvature = self.maximum_curvature()
        return speed * curvature, speed * speed * curvature

    @property
    def x_limits(self):
        return (
            self.center_x - self.x_amplitude,
            self.center_x + self.x_amplitude,
        )

    @property
    def y_limits(self):
        return (
            self.center_y - self.y_amplitude,
            self.center_y + self.y_amplitude,
        )

    def _path_derivative(self, phase):
        return (
            -self.x_amplitude * math.sin(phase),
            2.0 * self.y_amplitude * math.cos(2.0 * phase),
        )

    def _phase_rate(self, phase):
        derivative_x, derivative_y = self._path_derivative(phase)
        distance_per_radian = math.hypot(
            derivative_x,
            derivative_y,
        )
        return self.speed / distance_per_radian

    def state(self):
        """Return position and constant-magnitude horizontal velocity."""
        position_x = (
            self.center_x
            + self.x_amplitude * math.cos(self.phase)
        )
        position_y = (
            self.center_y
            + self.y_amplitude * math.sin(2.0 * self.phase)
        )
        derivative_x, derivative_y = self._path_derivative(self.phase)
        phase_rate = self._phase_rate(self.phase)
        velocity_x = derivative_x * phase_rate
        velocity_y = derivative_y * phase_rate
        return position_x, position_y, velocity_x, velocity_y

    def advance(self, dt):
        """Advance phase using RK4 arc-length parameterization."""
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('Figure-eight time step must be positive.')

        phase = self.phase
        k1 = self._phase_rate(phase)
        k2 = self._phase_rate(phase + 0.5 * dt * k1)
        k3 = self._phase_rate(phase + 0.5 * dt * k2)
        k4 = self._phase_rate(phase + dt * k3)
        self.phase = (
            phase
            + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        ) % (2.0 * math.pi)
        return self.state()
