"""
Causal, evaluation-only USV motion profiles; no ROS or future-path output.

Legacy modes retain the original semi-implicit speed ramp and Gerono RK4
arc-length integration. New modes use smooth analytic speed/heading laws
and integrate only the elapsed simulation time supplied by the caller.
"""

from dataclasses import dataclass, fields
import math

from uav_control.figure_eight_trajectory import FigureEightTrajectory


MOTION_MODES = {
    1: 'linear',
    2: 'figure_eight',
    3: 'variable_speed_eight',
    4: 'variable_speed_s',
    5: 'variable_speed_z',
}


def normalize_motion_mode(mode):
    """Accept a canonical name or a console digit, reject all other values."""
    value = str(mode).strip().lower()
    if value in MOTION_MODES.values():
        return value
    if value in {str(index) for index in MOTION_MODES}:
        return MOTION_MODES[int(value)]
    raise ValueError('unknown motion mode; expected 1..5 or a canonical name')


@dataclass(frozen=True)
class MotionAcknowledgement:
    successful: bool
    mode: str
    mode_id: int
    started: bool
    reason: str

    def as_dict(self):
        """JSON fields used by the transient-local selector status topic."""
        return {
            'mode': self.mode, 'id': self.mode_id, 'mode_id': self.mode_id,
            'started': self.started, 'ack': self.successful,
            'reason': self.reason, 'successful': self.successful,
        }


class MotionModeSelector:
    """Independent mode selection state, never a mission command machine."""

    def __init__(self, mode='figure_eight'):
        self.mode = normalize_motion_mode(mode)
        self.started = False

    def status(self, successful=True, reason='ready'):
        index = next(key for key, value in MOTION_MODES.items()
                     if value == self.mode)
        return MotionAcknowledgement(
            successful, self.mode, index, self.started, reason)

    def select(self, mode):
        try:
            requested = normalize_motion_mode(mode)
        except ValueError as exc:
            return self.status(False, str(exc))
        if requested == self.mode:
            return self.status(reason='already selected')
        if self.started:
            return self.status(False, 'motion already started; restart session')
        self.mode = requested
        return self.status(reason='selection applied')

    def start(self):
        self.started = True
        return self.status(reason='motion started')


def elapsed_motion_step(previous_time_ns, current_time_ns, maximum_step=1.0):
    """
    Freeze for first sample, stationary/backward clock or discontinuity.

    The caller reanchors its previous epoch after rejection. No nominal time
    is fabricated, and a large clock jump is never integrated as motion.
    """
    if previous_time_ns is None:
        return 0.0
    elapsed = (int(current_time_ns) - int(previous_time_ns)) * 1e-9
    return elapsed if 0.0 < elapsed <= float(maximum_step) else 0.0


@dataclass(frozen=True)
class MotionParameters:
    initial_x: float = 8.0
    initial_y: float = 0.0
    initial_z: float = 0.0
    linear_vx: float = 0.0
    linear_vy: float = 4.0
    velocity_z: float = 0.0
    horizontal_speed: float = 4.0
    horizontal_acceleration_limit: float = 1.0
    maximum_turn_rate: float = 0.65
    maximum_lateral_acceleration: float = 3.2
    maximum_horizontal_acceleration: float = 3.2
    maximum_angular_acceleration: float = 0.35
    figure_eight_x_amplitude: float = 40.0
    figure_eight_y_amplitude: float = 20.0
    vertical_oscillation_amplitude: float = 0.15
    vertical_oscillation_frequency: float = 0.25
    integration_step: float = 0.05
    variable_speed_min: float = 2.0
    variable_speed_max: float = 4.0
    variable_speed_period: float = 24.0
    variable_startup_duration: float = 8.0
    s_heading_amplitude: float = 0.6
    s_period: float = 24.0
    z_heading_change: float = 0.8
    z_straight_duration: float = 7.0
    z_turn_duration: float = 5.0
    z_diagonal_duration: float = 8.0


@dataclass(frozen=True)
class MotionSample:
    position: tuple
    velocity: tuple
    acceleration: tuple
    yaw: float
    turn_rate: float
    curvature: float
    phase: float
    stage: str
    longitudinal_acceleration: float
    lateral_acceleration: float
    angular_acceleration: float


def _smoothstep(time, duration):
    """Quintic blend and its first two time derivatives (C2 joins)."""
    if time <= 0.:
        return 0., 0., 0.
    if time >= duration:
        return 1., 0., 0.
    u = time / duration
    return (
        u ** 3 * (10. - 15. * u + 6. * u ** 2),
        30. * u ** 2 * (1. - u) ** 2 / duration,
        60. * u * (1. - u) * (1. - 2. * u) / duration ** 2,
    )


def _eight_geometry(trajectory, phase):
    """Signed curvature and d(curvature)/d(arc length), analytic Gerono."""
    x, y = trajectory.x_amplitude, trajectory.y_amplitude
    dx, dy = -x * math.sin(phase), 2. * y * math.cos(2. * phase)
    ddx, ddy = -x * math.cos(phase), -4. * y * math.sin(2. * phase)
    dddx, dddy = x * math.sin(phase), -8. * y * math.cos(2. * phase)
    q = math.hypot(dx, dy)
    cross = dx * ddy - dy * ddx
    curvature = cross / q ** 3
    derivative = ((dx * dddy - dy * dddx) / q ** 4
                  - 3. * cross * (dx * ddx + dy * ddy) / q ** 6)
    return curvature, derivative


class UsvMotionProfile:
    """
    One deterministic causal profile. ``advance(dt)`` returns current truth.

    Infeasible new-profile configurations are rejected before motion, rather
    than silently clipping a path and breaking its analytic derivatives.
    """

    def __init__(self, mode='figure_eight', parameters=None):
        self.mode = normalize_motion_mode(mode)
        self.parameters = parameters or MotionParameters()
        self.elapsed_time = 0.
        self.speed = 0.
        self.longitudinal_acceleration = 0.
        self.x = float(self.parameters.initial_x)
        self.y = float(self.parameters.initial_y)
        self.figure_eight = None
        p = self.parameters
        self.initial_yaw = (math.atan2(p.linear_vy, p.linear_vx)
                            if math.hypot(p.linear_vx, p.linear_vy) > 1e-9
                            else math.pi / 2.)
        if self.mode in ('figure_eight', 'variable_speed_eight'):
            self.initial_yaw = math.pi / 2.
            self.figure_eight = FigureEightTrajectory(
                p.initial_x, p.initial_y, p.figure_eight_x_amplitude,
                p.figure_eight_y_amplitude, p.horizontal_speed)
            self.figure_eight.set_speed(0.)
            self.x, self.y, _, _ = self.figure_eight.state()
        self.yaw = self.initial_yaw
        self._validate()

    def _validate(self):
        p = self.parameters
        for field in fields(p):
            if not math.isfinite(float(getattr(p, field.name))):
                raise ValueError(f'{field.name} must be finite')
        positive = (
            'horizontal_speed', 'horizontal_acceleration_limit',
            'maximum_turn_rate', 'maximum_lateral_acceleration',
            'maximum_horizontal_acceleration', 'maximum_angular_acceleration',
            'integration_step',
        )
        for name in positive:
            if getattr(p, name) <= 0.:
                raise ValueError(f'{name} must be positive')
        if p.vertical_oscillation_amplitude < 0.:
            raise ValueError('vertical_oscillation_amplitude must be nonnegative')
        if p.vertical_oscillation_frequency < 0.:
            raise ValueError('vertical_oscillation_frequency must be nonnegative')
        if self.mode == 'figure_eight':
            omega, lateral = self.figure_eight.kinematic_envelope(
                p.horizontal_speed)
            self._check_bound('maximum_turn_rate', omega)
            self._check_bound('maximum_lateral_acceleration', lateral)
        if not self.mode.startswith('variable_speed_'):
            return
        for name in (
            'variable_speed_min', 'variable_speed_max', 'variable_speed_period',
            'variable_startup_duration', 's_period', 's_heading_amplitude',
            'z_heading_change', 'z_straight_duration', 'z_turn_duration',
            'z_diagonal_duration',
        ):
            if getattr(p, name) <= 0.:
                raise ValueError(f'{name} must be positive')
        if p.variable_speed_min > p.variable_speed_max:
            raise ValueError('variable_speed_min exceeds variable_speed_max')
        speed = p.variable_speed_max
        acceleration = max(
            1.875 * speed / p.variable_startup_duration,
            (p.variable_speed_max - p.variable_speed_min)
            * math.pi / p.variable_speed_period)
        self._check_bound('horizontal_acceleration_limit', acceleration)
        if self.mode == 'variable_speed_eight':
            # Dense full-path geometry envelope with a 0.1% sampling margin.
            geometry = [_eight_geometry(self.figure_eight, 2. * math.pi * i / 8192)
                        for i in range(8192)]
            curvature = max(abs(k) for k, _ in geometry) * 1.001
            derivative = max(abs(dk) for _, dk in geometry) * 1.001
            omega = speed * curvature
            alpha = acceleration * curvature + speed ** 2 * derivative
        elif self.mode == 'variable_speed_s':
            frequency = 2. * math.pi / p.s_period
            ramp_rate = 1.875 / p.variable_startup_duration
            ramp_acceleration = (10. / math.sqrt(3.)
                                 / p.variable_startup_duration ** 2)
            omega = p.s_heading_amplitude * (frequency + ramp_rate)
            alpha = p.s_heading_amplitude * (
                frequency ** 2 + 2. * frequency * ramp_rate + ramp_acceleration)
        else:
            omega = 1.875 * p.z_heading_change / p.z_turn_duration
            alpha = (10. / math.sqrt(3.) * p.z_heading_change
                     / p.z_turn_duration ** 2)
        lateral = speed * omega
        self._check_bound('maximum_turn_rate', omega)
        self._check_bound('maximum_angular_acceleration', alpha)
        self._check_bound('maximum_lateral_acceleration', lateral)
        self._check_bound('maximum_horizontal_acceleration',
                          math.hypot(acceleration, lateral))

    def _check_bound(self, name, bound):
        if bound > getattr(self.parameters, name) + 1e-12:
            raise ValueError(
                f'{self.mode} envelope {bound:.6g} exceeds {name}='
                f'{getattr(self.parameters, name):.6g}')

    def _speed_law(self, time):
        p = self.parameters
        if time < p.variable_startup_duration:
            fraction, rate, _ = _smoothstep(time, p.variable_startup_duration)
            return p.variable_speed_max * fraction, p.variable_speed_max * rate
        phase = (2. * math.pi * (time - p.variable_startup_duration)
                 / p.variable_speed_period)
        amplitude = .5 * (p.variable_speed_max - p.variable_speed_min)
        mean = .5 * (p.variable_speed_max + p.variable_speed_min)
        return (mean + amplitude * math.cos(phase),
                -amplitude * 2. * math.pi / p.variable_speed_period
                * math.sin(phase))

    def _heading_law(self, time):
        p = self.parameters
        if self.mode == 'variable_speed_s':
            frequency = 2. * math.pi / p.s_period
            phase = frequency * time
            ramp, rate, acceleration = _smoothstep(
                time, p.variable_startup_duration)
            sine, cosine = math.sin(phase), math.cos(phase)
            # NED yaw is positive clockwise. Negative yaw initially turns
            # left from +Y toward +X, preserving the original forward axis.
            heading = -p.s_heading_amplitude * sine * ramp
            omega = -p.s_heading_amplitude * (frequency * cosine * ramp
                                              + sine * rate)
            alpha = -p.s_heading_amplitude * (
                -frequency ** 2 * sine * ramp
                + 2. * frequency * cosine * rate + sine * acceleration)
            stage = 'turn_left' if omega < 0. else 'turn_right'
            return self.initial_yaw + heading, omega, alpha, phase, stage
        straight, turn, diagonal = (
            p.z_straight_duration, p.z_turn_duration, p.z_diagonal_duration)
        cycle = 2. * straight + 2. * turn + diagonal
        phase = time % cycle
        if phase < straight:
            return self.initial_yaw, 0., 0., phase, 'straight'
        if phase < straight + turn:
            blend, rate, acceleration = _smoothstep(phase - straight, turn)
            return (self.initial_yaw - p.z_heading_change * blend,
                    -p.z_heading_change * rate,
                    -p.z_heading_change * acceleration, phase, 'turn_left')
        if phase < straight + turn + diagonal:
            return (self.initial_yaw - p.z_heading_change,
                    0., 0., phase, 'diagonal')
        if phase < straight + 2. * turn + diagonal:
            blend, rate, acceleration = _smoothstep(
                phase - straight - turn - diagonal, turn)
            return (self.initial_yaw - p.z_heading_change * (1. - blend),
                    p.z_heading_change * rate,
                    p.z_heading_change * acceleration, phase, 'turn_right')
        return self.initial_yaw, 0., 0., phase, 'straight'

    def _planar_velocity(self, time):
        speed, _ = self._speed_law(time)
        heading = self._heading_law(time)[0]
        return speed * math.cos(heading), speed * math.sin(heading)

    def advance(self, dt):
        dt = float(dt)
        if not math.isfinite(dt) or dt < 0.:
            raise ValueError('motion time step must be finite and nonnegative')
        if dt == 0.:
            return self.state()
        p = self.parameters
        remaining = dt
        while remaining > 1e-12:
            step = min(remaining, p.integration_step)
            if self.mode in ('linear', 'figure_eight'):
                previous_speed = self.speed
                self.speed = min(self.speed + p.horizontal_acceleration_limit
                                 * step, p.horizontal_speed)
                self.longitudinal_acceleration = (
                    (self.speed - previous_speed) / step)
                if self.figure_eight is None:
                    requested = math.hypot(p.linear_vx, p.linear_vy)
                    scale = self.speed / requested if requested > 1e-9 else 0.
                    self.x += p.linear_vx * scale * step
                    self.y += p.linear_vy * scale * step
                else:
                    self.figure_eight.set_speed(self.speed)
                    self.x, self.y, _, _ = self.figure_eight.advance(step)
            elif self.figure_eight is not None:
                # Simpson arc distance, retaining the existing spatial RK4.
                t = self.elapsed_time
                speed = (self._speed_law(t)[0]
                         + 4. * self._speed_law(t + .5 * step)[0]
                         + self._speed_law(t + step)[0]) / 6.
                self.figure_eight.set_speed(speed)
                self.x, self.y, _, _ = self.figure_eight.advance(step)
            else:
                t = self.elapsed_time
                v1 = self._planar_velocity(t)
                v2 = self._planar_velocity(t + .5 * step)
                v3 = self._planar_velocity(t + step)
                self.x += step * (v1[0] + 4. * v2[0] + v3[0]) / 6.
                self.y += step * (v1[1] + 4. * v2[1] + v3[1]) / 6.
            self.elapsed_time += step
            remaining -= step
        if self.mode.startswith('variable_speed_'):
            self.speed, self.longitudinal_acceleration = self._speed_law(
                self.elapsed_time)
            if self.figure_eight is not None:
                self.figure_eight.set_speed(self.speed)
        if self.figure_eight is not None:
            _, _, vx, vy = self.figure_eight.state()
            if self.speed > 1e-12:
                raw_yaw = math.atan2(vy, vx)
                self.yaw += math.atan2(math.sin(raw_yaw - self.yaw),
                                       math.cos(raw_yaw - self.yaw))
        return self.state()

    def state(self):
        p, time = self.parameters, self.elapsed_time
        speed, acceleration = self.speed, self.longitudinal_acceleration
        if self.figure_eight is not None:
            _, _, vx, vy = self.figure_eight.state()
            curvature, derivative = _eight_geometry(
                self.figure_eight, self.figure_eight.phase)
            yaw, omega = self.yaw, speed * curvature
            alpha = acceleration * curvature + speed ** 2 * derivative
            phase = self.figure_eight.phase
            stage = ('eight_accelerating' if acceleration > 1e-9 else
                     'eight_decelerating' if acceleration < -1e-9 else 'eight')
        elif self.mode == 'linear':
            requested = math.hypot(p.linear_vx, p.linear_vy)
            scale = speed / requested if requested > 1e-9 else 0.
            vx, vy = p.linear_vx * scale, p.linear_vy * scale
            if requested <= 1e-9:
                speed, acceleration = 0., 0.
            yaw = self.initial_yaw
            omega, alpha, curvature, phase = 0., 0., 0., time
            stage = 'accelerating' if acceleration > 0. else 'straight'
        else:
            yaw, omega, alpha, phase, stage = self._heading_law(time)
            vx, vy = speed * math.cos(yaw), speed * math.sin(yaw)
            curvature = omega / speed if speed > 1e-12 else 0.
        lateral = speed * omega
        ax = acceleration * math.cos(yaw) - lateral * math.sin(yaw)
        ay = acceleration * math.sin(yaw) + lateral * math.cos(yaw)
        frequency = 2. * math.pi * p.vertical_oscillation_frequency
        z = (p.initial_z + p.velocity_z * time
             + p.vertical_oscillation_amplitude * math.sin(frequency * time))
        vz = (p.velocity_z + p.vertical_oscillation_amplitude * frequency
              * math.cos(frequency * time))
        az = -p.vertical_oscillation_amplitude * frequency ** 2 * math.sin(
            frequency * time)
        if self.mode.startswith('variable_speed_') and time < p.variable_startup_duration:
            stage = 'startup'
        return MotionSample(
            (self.x, self.y, z), (vx, vy, vz), (ax, ay, az), yaw, omega,
            curvature, phase, stage, acceleration, lateral, alpha)
