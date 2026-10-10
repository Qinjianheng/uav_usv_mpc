"""Small dependency-free PID component for future outer-loop controllers."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PidGains:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float


class AxisPid:
    """One-axis PID with derivative-on-measurement and anti-windup."""

    def __init__(self, gains: PidGains):
        self.gains = gains
        self.integral = 0.0
        self.previous_measurement = None

    def reset(self):
        self.integral = 0.0
        self.previous_measurement = None

    def update(self, setpoint, measurement, dt, feedforward=0.0):
        if dt <= 0.0:
            raise ValueError('dt must be positive')

        error = float(setpoint) - float(measurement)
        candidate_integral = self.integral + error * dt
        integral_limit = max(float(self.gains.integral_limit), 0.0)
        candidate_integral = max(
            min(candidate_integral, integral_limit),
            -integral_limit,
        )

        derivative = 0.0
        if self.previous_measurement is not None:
            derivative = -(
                float(measurement) - self.previous_measurement
            ) / dt

        unclamped = (
            float(feedforward)
            + self.gains.kp * error
            + self.gains.ki * candidate_integral
            + self.gains.kd * derivative
        )
        output_limit = max(float(self.gains.output_limit), 0.0)
        output = max(min(unclamped, output_limit), -output_limit)

        if output == unclamped or error * unclamped < 0.0:
            self.integral = candidate_integral
        self.previous_measurement = float(measurement)
        return output
