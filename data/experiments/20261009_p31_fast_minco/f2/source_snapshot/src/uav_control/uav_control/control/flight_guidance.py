"""Non-MINCO ground, takeoff, follow, and far-guidance control."""

from dataclasses import dataclass
import math

from uav_control.common.sea_safety import apply_sea_safety_guard
from uav_control.guidance.intercept_reachability import minimum_time_1d


@dataclass(frozen=True)
class FlightKinematicState:
    """Position and velocity in the PX4 local NED frame."""

    position: tuple
    velocity: tuple


@dataclass(frozen=True)
class FlightGuidanceCommand:
    """One position-hold or velocity-guidance command."""

    mode: str
    position: tuple
    velocity: tuple
    acceleration: tuple
    takeoff_complete: bool
    far_guidance_available: bool
    safety_state: str
    safety_margin: float


@dataclass(frozen=True)
class ApproachTarget:
    """Dynamic horizontal preparation point for a later terminal descent."""

    position: tuple
    vertical_time: float
    standoff: float


class FlightGuidanceCore:
    """Provide bounded flight modes outside terminal MINCO tracking."""

    def __init__(
        self,
        flight_altitude=-5.0,
        takeoff_tolerance=0.5,
        takeoff_settle_time=1.0,
        takeoff_maximum_vertical_speed=1.5,
        takeoff_maximum_vertical_acceleration=1.0,
        takeoff_maximum_horizontal_acceleration=1.5,
        takeoff_horizontal_start_height=0.5,
        takeoff_horizontal_full_height=1.5,
        follow_distance=5.0,
        follow_position_gain=0.8,
        altitude_velocity_gain=1.0,
        maximum_horizontal_speed=6.2,
        maximum_vertical_speed=4.0,
        maximum_horizontal_acceleration=3.0,
        maximum_vertical_acceleration=3.0,
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        safety_response_delay=0.15,
        vertical_braking_acceleration=2.5,
        approach_contact_clearance=0.33,
        approach_closing_speed=1.5,
        approach_horizon=4.0,
        approach_response_delay=0.15,
        control_dt=0.05,
        approach_preparation_clearance=None,
    ):
        self.flight_altitude = float(flight_altitude)
        self.takeoff_tolerance = float(takeoff_tolerance)
        self.takeoff_settle_time = float(takeoff_settle_time)
        self.takeoff_maximum_vertical_speed = float(
            takeoff_maximum_vertical_speed
        )
        self.takeoff_maximum_vertical_acceleration = float(
            takeoff_maximum_vertical_acceleration
        )
        self.takeoff_maximum_horizontal_acceleration = float(
            takeoff_maximum_horizontal_acceleration
        )
        self.takeoff_horizontal_start_height = float(
            takeoff_horizontal_start_height
        )
        self.takeoff_horizontal_full_height = max(
            float(takeoff_horizontal_full_height),
            self.takeoff_horizontal_start_height + 1e-3,
        )
        self.follow_distance = float(follow_distance)
        self.follow_position_gain = float(follow_position_gain)
        self.altitude_velocity_gain = float(altitude_velocity_gain)
        self.maximum_horizontal_speed = float(maximum_horizontal_speed)
        self.maximum_vertical_speed = float(maximum_vertical_speed)
        self.maximum_horizontal_acceleration = float(
            maximum_horizontal_acceleration
        )
        self.maximum_vertical_acceleration = float(
            maximum_vertical_acceleration
        )
        self.sea_surface_z = float(sea_surface_z)
        self.reserve_clearance = float(reserve_clearance)
        self.safety_response_delay = float(safety_response_delay)
        self.vertical_braking_acceleration = float(
            vertical_braking_acceleration
        )
        self.approach_contact_clearance = float(approach_contact_clearance)
        self.approach_preparation_clearance = (
            self.sea_surface_z - self.flight_altitude
            if approach_preparation_clearance is None
            else float(approach_preparation_clearance)
        )
        if (
            approach_preparation_clearance is not None
            and (
                not math.isfinite(self.approach_preparation_clearance)
                or self.approach_preparation_clearance
                <= self.approach_contact_clearance
            )
        ):
            raise ValueError('preparation clearance must exceed contact clearance')
        self.approach_closing_speed = float(approach_closing_speed)
        self.approach_horizon = float(approach_horizon)
        self.approach_response_delay = float(approach_response_delay)
        self.control_dt = float(control_dt)
        self.ground_position = None
        self.previous_velocity = None
        self.settled_duration = 0.0

    def reset(self):
        """Clear mission-scoped command shaping and takeoff state."""
        self.ground_position = None
        self.previous_velocity = None
        self.settled_duration = 0.0

    @staticmethod
    def _limit_horizontal(values, maximum):
        magnitude = math.hypot(values[0], values[1])
        if magnitude <= maximum or magnitude <= 1e-12:
            return float(values[0]), float(values[1])
        scale = maximum / magnitude
        return values[0] * scale, values[1] * scale

    def _hold(self, state, takeoff_complete=False):
        self.previous_velocity = None
        return FlightGuidanceCommand(
            mode='POSITION',
            position=tuple(state.position),
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
            takeoff_complete=bool(takeoff_complete),
            far_guidance_available=False,
            safety_state='HOLD',
            safety_margin=math.inf,
        )

    def _shape_velocity(
        self,
        desired,
        dt,
        horizontal_acceleration,
        vertical_acceleration,
    ):
        desired_xy = self._limit_horizontal(
            desired[:2],
            self.maximum_horizontal_speed,
        )
        desired = (
            desired_xy[0],
            desired_xy[1],
            max(
                min(desired[2], self.maximum_vertical_speed),
                -self.maximum_vertical_speed,
            ),
        )
        if self.previous_velocity is None:
            self.previous_velocity = desired
            return desired
        dt = max(float(dt), self.control_dt)
        delta_xy = self._limit_horizontal(
            (
                desired[0] - self.previous_velocity[0],
                desired[1] - self.previous_velocity[1],
            ),
            float(horizontal_acceleration) * dt,
        )
        delta_z = max(
            min(
                desired[2] - self.previous_velocity[2],
                float(vertical_acceleration) * dt,
            ),
            -float(vertical_acceleration) * dt,
        )
        self.previous_velocity = (
            self.previous_velocity[0] + delta_xy[0],
            self.previous_velocity[1] + delta_xy[1],
            self.previous_velocity[2] + delta_z,
        )
        return self.previous_velocity

    def _velocity_command(
        self,
        state,
        desired,
        dt,
        takeoff_complete,
        target_available,
        horizontal_acceleration=None,
        vertical_acceleration=None,
    ):
        horizontal_acceleration = (
            self.maximum_horizontal_acceleration
            if horizontal_acceleration is None
            else float(horizontal_acceleration)
        )
        vertical_acceleration = (
            self.maximum_vertical_acceleration
            if vertical_acceleration is None
            else float(vertical_acceleration)
        )
        velocity = self._shape_velocity(
            desired,
            dt,
            horizontal_acceleration,
            vertical_acceleration,
        )
        safety = apply_sea_safety_guard(
            current_z=state.position[2],
            current_vz=state.velocity[2],
            proposed_vz=velocity[2],
            sea_surface_z=self.sea_surface_z,
            reserve_clearance=self.reserve_clearance,
            response_delay=self.safety_response_delay,
            effective_braking_acceleration=(
                self.vertical_braking_acceleration
            ),
            control_dt=max(float(dt), self.control_dt),
            maximum_vertical_speed=self.maximum_vertical_speed,
        )
        velocity = velocity[0], velocity[1], safety.command_vz
        self.previous_velocity = velocity
        return FlightGuidanceCommand(
            mode='VELOCITY',
            position=tuple(state.position),
            velocity=velocity,
            acceleration=(0.0, 0.0, 0.0),
            takeoff_complete=bool(takeoff_complete),
            far_guidance_available=bool(target_available),
            safety_state=safety.state.value,
            safety_margin=safety.response_margin,
        )

    def _follow_velocity(self, state, target):
        target_speed = math.hypot(target.velocity[0], target.velocity[1])
        if target_speed > 1e-6:
            direction = (
                target.velocity[0] / target_speed,
                target.velocity[1] / target_speed,
            )
        else:
            direction = (1.0, 0.0)
        desired_position = (
            target.position[0] - self.follow_distance * direction[0],
            target.position[1] - self.follow_distance * direction[1],
        )
        return (
            target.velocity[0]
            + self.follow_position_gain
            * (desired_position[0] - state.position[0]),
            target.velocity[1]
            + self.follow_position_gain
            * (desired_position[1] - state.position[1]),
        )

    def bearing_approach(self, state, heading, speed, dt, bearing=0.0):
        """
        Advance along the fresh RGB ray while holding flight altitude.

        The caller grants fresh RGB direction authority. No target position,
        depth, route, prediction, or truth is consumed here.
        """
        if (not all(math.isfinite(value) for value in (heading, speed, dt, bearing))
                or abs(bearing) >= math.pi / 2.0):
            return self._hold(state)
        speed = min(max(float(speed), 0.0), self.maximum_horizontal_speed)
        speed *= math.cos(bearing)
        direction = heading + bearing
        if self.previous_velocity is None:
            self.previous_velocity = tuple(state.velocity)
        return self._velocity_command(
            state,
            (speed * math.cos(direction), speed * math.sin(direction),
             self.altitude_velocity_gain * (self.flight_altitude-state.position[2])),
            dt, takeoff_complete=False, target_available=False,
        )

    def search_velocity(self, state, anchor, dt):
        """Brake without target inputs, then servo the stopped search anchor."""
        if self.previous_velocity is None:
            self.previous_velocity = tuple(state.velocity)
        horizontal = ((0.0, 0.0) if anchor is None else (
            self.follow_position_gain * (anchor[0] - state.position[0]),
            self.follow_position_gain * (anchor[1] - state.position[1]),
        ))
        altitude = self.flight_altitude if anchor is None else anchor[2]
        command = self._velocity_command(
            state,
            (horizontal[0], horizontal[1],
             self.altitude_velocity_gain * (altitude - state.position[2])),
            dt, takeoff_complete=False, target_available=False,
        )
        return command if anchor is None else FlightGuidanceCommand(
            mode=command.mode, position=tuple(anchor), velocity=command.velocity,
            acceleration=command.acceleration, takeoff_complete=False,
            far_guidance_available=False, safety_state=command.safety_state,
            safety_margin=command.safety_margin,
        )

    def approach_target(self, state, target):
        """Return a moving preparation point sized by vertical reachability."""
        contact_z = self.sea_surface_z - self.approach_contact_clearance
        vertical_time = minimum_time_1d(
            displacement=contact_z - state.position[2],
            initial_velocity=state.velocity[2],
            final_velocity=target.velocity[2],
            maximum_speed=self.maximum_vertical_speed,
            maximum_acceleration=self.maximum_vertical_acceleration,
            maximum_braking_acceleration=(
                self.vertical_braking_acceleration
            ),
            response_delay=self.approach_response_delay,
            search_limit=self.approach_horizon,
        )
        if not math.isfinite(vertical_time):
            vertical_time = self.approach_horizon
        vertical_time = min(max(vertical_time, 0.0), self.approach_horizon)
        target_speed = math.hypot(target.velocity[0], target.velocity[1])
        if target_speed > 1e-6:
            direction = (
                target.velocity[0] / target_speed,
                target.velocity[1] / target_speed,
            )
        else:
            offset = (
                target.position[0] - state.position[0],
                target.position[1] - state.position[1],
            )
            distance = math.hypot(offset[0], offset[1])
            direction = (
                (offset[0] / distance, offset[1] / distance)
                if distance > 1e-6 else (1.0, 0.0)
            )
        standoff = self.approach_closing_speed * vertical_time
        return ApproachTarget(
            position=(
                target.position[0] - standoff * direction[0],
                target.position[1] - standoff * direction[1],
                self.sea_surface_z - self.approach_preparation_clearance,
            ),
            vertical_time=vertical_time,
            standoff=standoff,
        )

    def _approach_velocity(self, state, target):
        approach = self.approach_target(state, target)
        return (
            target.velocity[0]
            + self.follow_position_gain
            * (approach.position[0] - state.position[0]),
            target.velocity[1]
            + self.follow_position_gain
            * (approach.position[1] - state.position[1]),
        )

    def _takeoff_horizontal_weight(self, state):
        clearance = max(self.ground_position[2] - state.position[2], 0.0)
        if clearance <= self.takeoff_horizontal_start_height:
            return 0.0
        if clearance >= self.takeoff_horizontal_full_height:
            return 1.0
        return (
            (clearance - self.takeoff_horizontal_start_height)
            / (
                self.takeoff_horizontal_full_height
                - self.takeoff_horizontal_start_height
            )
        )

    def command(self, phase, state, target, dt=None):
        """Generate one bounded command for the named mission phase."""
        phase = str(phase)
        dt = self.control_dt if dt is None else max(float(dt), 1e-3)
        if self.ground_position is None:
            self.ground_position = tuple(state.position)
        if phase in ('INIT', 'GROUND_HOLD'):
            self.settled_duration = 0.0
            held_state = FlightKinematicState(
                self.ground_position,
                (0.0, 0.0, 0.0),
            )
            return self._hold(held_state)
        if phase == 'TAKEOFF':
            altitude_error = self.flight_altitude - state.position[2]
            desired_vz = max(
                min(
                    self.altitude_velocity_gain * altitude_error,
                    self.takeoff_maximum_vertical_speed,
                ),
                -self.takeoff_maximum_vertical_speed,
            )
            settled = (
                abs(altitude_error) <= self.takeoff_tolerance
                and abs(state.velocity[2]) <= self.takeoff_tolerance
            )
            self.settled_duration = (
                self.settled_duration + dt if settled else 0.0
            )
            horizontal = (0.0, 0.0)
            if target is not None:
                follow = self._follow_velocity(state, target)
                weight = self._takeoff_horizontal_weight(state)
                horizontal = follow[0] * weight, follow[1] * weight
            return self._velocity_command(
                state,
                (horizontal[0], horizontal[1], desired_vz),
                dt,
                self.settled_duration + 1e-9 >= self.takeoff_settle_time,
                target is not None,
                horizontal_acceleration=(
                    self.takeoff_maximum_horizontal_acceleration
                ),
                vertical_acceleration=(
                    self.takeoff_maximum_vertical_acceleration
                ),
            )
        if phase in ('FOLLOW', 'FAR_GUIDANCE'):
            self.settled_duration = 0.0
            if target is None:
                return self._hold(state)
            horizontal = (
                self._approach_velocity(state, target)
                if phase == 'FAR_GUIDANCE'
                else self._follow_velocity(state, target)
            )
            altitude = (
                self.sea_surface_z - self.approach_preparation_clearance
                if phase == 'FAR_GUIDANCE' else self.flight_altitude
            )
            desired = (
                horizontal[0],
                horizontal[1],
                self.altitude_velocity_gain
                * (altitude - state.position[2]),
            )
            if self.previous_velocity is None:
                self.previous_velocity = tuple(state.velocity)
            return self._velocity_command(
                state,
                desired,
                dt,
                True,
                True,
            )
        return self._hold(state)
