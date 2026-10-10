"""Realtime polynomial tracking independent of planning and ROS transport."""

import math
from dataclasses import dataclass
from enum import Enum

from uav_control.common.sea_safety import apply_sea_safety_guard


class TrajectoryRejectReason(str, Enum):
    """Reasons a new plan cannot atomically replace the active trajectory."""

    NONE = 'NONE'
    MISSION_MISMATCH = 'MISSION_MISMATCH'
    PREDICTION_MISMATCH = 'PREDICTION_MISMATCH'
    FRAME_MISMATCH = 'FRAME_MISMATCH'
    SOURCE_STALE = 'SOURCE_STALE'
    EXPIRED = 'EXPIRED'
    INSUFFICIENT_REMAINING_TIME = 'INSUFFICIENT_REMAINING_TIME'
    STATE_POSITION_MISMATCH = 'STATE_POSITION_MISMATCH'
    STATE_VELOCITY_MISMATCH = 'STATE_VELOCITY_MISMATCH'
    TARGET_ENDPOINT_MISMATCH = 'TARGET_ENDPOINT_MISMATCH'
    SAFETY_REJECTED = 'SAFETY_REJECTED'
    INVALID_TRAJECTORY = 'INVALID_TRAJECTORY'
    REFERENCE_POSITION_MISMATCH = 'REFERENCE_POSITION_MISMATCH'
    REFERENCE_VELOCITY_MISMATCH = 'REFERENCE_VELOCITY_MISMATCH'
    REFERENCE_ACCELERATION_MISMATCH = 'REFERENCE_ACCELERATION_MISMATCH'
    TERMINAL_COMMITTED = 'TERMINAL_COMMITTED'


@dataclass(frozen=True)
class PolynomialSegmentData:
    """One local-time three-axis quintic segment."""

    duration: float
    coefficients: tuple

    def __post_init__(self):
        if (
            not math.isfinite(float(self.duration))
            or self.duration <= 0.0
            or len(self.coefficients) != 18
            or not all(math.isfinite(value) for value in self.coefficients)
        ):
            raise ValueError('polynomial segment must be finite and complete')

    def sample(self, local_time):
        """Return position, velocity, and acceleration at local time."""
        local_time = min(max(float(local_time), 0.0), self.duration)
        position = []
        velocity = []
        acceleration = []
        for axis in range(3):
            offset = 6 * axis
            c0, c1, c2, c3, c4, c5 = self.coefficients[
                offset:offset + 6
            ]
            t = local_time
            position.append(
                c0 + c1 * t + c2 * t**2 + c3 * t**3
                + c4 * t**4 + c5 * t**5
            )
            velocity.append(
                c1 + 2.0 * c2 * t + 3.0 * c3 * t**2
                + 4.0 * c4 * t**3 + 5.0 * c5 * t**4
            )
            acceleration.append(
                2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t**2
                + 20.0 * c5 * t**3
            )
        return TrajectorySample(
            tuple(position),
            tuple(velocity),
            tuple(acceleration),
        )


@dataclass(frozen=True)
class TrajectorySample:
    """One desired state sampled from a polynomial trajectory."""

    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class PolynomialTrajectory:
    """Complete cross-process MINCO trajectory timed from its start stamp."""

    mission_id: int
    plan_id: int
    prediction_sequence_id: int
    source_stamp: float
    generated_stamp: float
    valid_until: float
    segments: tuple
    terminal_position: tuple
    terminal_velocity: tuple
    target_state_source: str
    frame_id: str = 'local_ned'
    contact_stamp: float = 0.0
    capture_entry_stamp: float = 0.0
    selected_t_go: float = 0.0
    remaining_t_go: float = 0.0
    terminal_mode: bool = False
    planned_capture_margin: float = 0.0
    capture_execution_margin: float = 0.0
    observation_stamp: float = 0.0

    @property
    def duration(self):
        """Return total piece duration."""
        return sum(segment.duration for segment in self.segments)

    def sample(self, elapsed):
        """Sample piecewise coefficients using each segment's local time."""
        if not self.segments:
            raise ValueError('trajectory must contain at least one segment')
        remaining = min(max(float(elapsed), 0.0), self.duration)
        for index, segment in enumerate(self.segments):
            if (
                remaining <= segment.duration
                or index == len(self.segments) - 1
            ):
                return segment.sample(remaining)
            remaining -= segment.duration
        raise RuntimeError('trajectory sampling failed')

    def sample_at_ros_time(self, now):
        """Sample from the MINCO start time stored in ``source_stamp``."""
        return self.sample(float(now) - self.source_stamp)


@dataclass(frozen=True)
class TrackerKinematicState:
    """Current UAV state in the ROS clock domain."""

    stamp: float
    position: tuple
    velocity: tuple


@dataclass(frozen=True)
class TrackingCommand:
    """Post-limit command ready for PX4 publication."""

    position: tuple
    velocity: tuple
    acceleration: tuple
    plan_id: int
    safety_state: str
    safety_margin: float


class TrajectoryTrackerCore:
    """Atomically accept and track valid plans with final sea protection."""

    def __init__(
        self,
        maximum_plan_age=0.125,
        minimum_remaining_time=0.20,
        maximum_position_error=0.30,
        maximum_velocity_error=0.50,
        target_endpoint_tolerance=0.50,
        expected_frame_id='local_ned',
        position_gain=1.2,
        maximum_horizontal_speed=6.5,
        maximum_vertical_speed=4.0,
        maximum_horizontal_acceleration=3.0,
        maximum_vertical_acceleration=3.0,
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        safety_response_delay=0.15,
        vertical_braking_acceleration=2.5,
        control_dt=0.05,
        recovery_clearance=0.5,
        recovery_climb_speed=1.0,
        maximum_command_dt=0.1,
        maximum_actual_vertical_acceleration=4.0,
    ):
        self.maximum_plan_age = float(maximum_plan_age)
        self.minimum_remaining_time = float(minimum_remaining_time)
        self.maximum_position_error = float(maximum_position_error)
        self.maximum_velocity_error = float(maximum_velocity_error)
        self.target_endpoint_tolerance = float(target_endpoint_tolerance)
        self.expected_frame_id = str(expected_frame_id)
        self.position_gain = float(position_gain)
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
        self.control_dt = float(control_dt)
        # Upper bound on the interval the command rate limiter may bank.  With
        # only a lower bound, a long gap (mode switch, plan loss) authorised a
        # command jump of acceleration_limit * gap, which PX4 answers with a
        # real acceleration far above the configured limit.
        self.maximum_command_dt = max(
            float(maximum_command_dt),
            self.control_dt,
        )
        # The configured acceleration limits bound the COMMAND.  PX4 answers a
        # bounded command with its own loop acceleration on top, which
        # measured 1.8-1.9x the commanded rate (3.0 m/s^2 commanded ->
        # 5.3 m/s^2 actual),
        # so the plant-side budget has to be enforced separately.
        self.maximum_actual_vertical_acceleration = max(
            float(maximum_actual_vertical_acceleration),
            0.0,
        )
        self.previous_state_stamp = None
        self.previous_state_vertical_velocity = None
        self.measured_vertical_acceleration = None
        # A lost plan must not park the airframe in the water margin: the
        # recovery reference climbs back to this clearance above the sea with a
        # bounded vertical speed.  The X500 body reaches 0.23 m below the PX4
        # reference, so 0.5 m keeps the gear roughly 0.27 m clear.
        self.recovery_clearance = max(float(recovery_clearance), 0.0)
        self.recovery_climb_speed = max(float(recovery_climb_speed), 0.0)
        self.active_trajectory = None
        self.status = 'NO_VALID_PLAN'
        self.previous_command_velocity = None
        self.previous_command_stamp = None
        self.last_reference_position = None
        self.recovery_position = None
        self.last_replacement_performed = False
        self.last_handover_position_error = 0.0
        self.last_handover_velocity_error = 0.0
        self.last_handover_acceleration_error = 0.0

    @staticmethod
    def _norm(values):
        return math.sqrt(sum(value * value for value in values))

    def reset(self):
        """Discard mission-scoped state without retaining prior commands."""
        self.active_trajectory = None
        self.status = 'NO_VALID_PLAN'
        self.previous_command_velocity = None
        self.previous_command_stamp = None
        self.last_reference_position = None
        self.recovery_position = None
        self.previous_state_stamp = None
        self.previous_state_vertical_velocity = None
        self.measured_vertical_acceleration = None
        self.last_replacement_performed = False
        self.last_handover_position_error = 0.0
        self.last_handover_velocity_error = 0.0
        self.last_handover_acceleration_error = 0.0

    def _observe_state(self, state):
        """Track the measured vertical acceleration for the plant limiter."""
        stamp = float(state.stamp)
        vertical = float(state.velocity[2])
        if (
            self.previous_state_stamp is not None
            and self.previous_state_vertical_velocity is not None
        ):
            step = stamp - self.previous_state_stamp
            if step > 1e-6:
                measured = abs(
                    (vertical - self.previous_state_vertical_velocity) / step
                )
                if self.measured_vertical_acceleration is None:
                    self.measured_vertical_acceleration = measured
                else:
                    # First-order filter: a raw finite difference is far too
                    # noisy to drive a limiter directly.
                    self.measured_vertical_acceleration = (
                        0.7 * self.measured_vertical_acceleration
                        + 0.3 * measured
                    )
        self.previous_state_stamp = stamp
        self.previous_state_vertical_velocity = vertical

    def accept(
        self,
        trajectory,
        state,
        mission_id,
        prediction_sequence_id=None,
        target_endpoint=None,
        maximum_position_error=None,
    ):
        """Validate then atomically replace the active trajectory."""
        self.last_replacement_performed = False
        if trajectory.mission_id != int(mission_id):
            return TrajectoryRejectReason.MISSION_MISMATCH
        if trajectory.frame_id != self.expected_frame_id:
            return TrajectoryRejectReason.FRAME_MISMATCH
        source_age = state.stamp - trajectory.source_stamp
        if source_age < 0.0 or source_age > self.maximum_plan_age:
            return TrajectoryRejectReason.SOURCE_STALE
        if state.stamp >= trajectory.valid_until or not trajectory.segments:
            return TrajectoryRejectReason.EXPIRED
        if (
            trajectory.valid_until - state.stamp
            < self.minimum_remaining_time
        ):
            return TrajectoryRejectReason.INSUFFICIENT_REMAINING_TIME
        try:
            expected = trajectory.sample_at_ros_time(state.stamp)
        except ValueError:
            return TrajectoryRejectReason.INVALID_TRAJECTORY
        position_error = self._norm(tuple(
            current - desired
            for current, desired in zip(state.position, expected.position)
        ))
        position_limit = (
            self.maximum_position_error
            if maximum_position_error is None
            else float(maximum_position_error)
        )
        if position_error > position_limit:
            return TrajectoryRejectReason.STATE_POSITION_MISMATCH
        velocity_error = self._norm(tuple(
            current - desired
            for current, desired in zip(state.velocity, expected.velocity)
        ))
        if velocity_error > self.maximum_velocity_error:
            return TrajectoryRejectReason.STATE_VELOCITY_MISMATCH
        if target_endpoint is not None:
            endpoint_error = self._norm(tuple(
                predicted - planned
                for predicted, planned in zip(
                    target_endpoint,
                    trajectory.terminal_position,
                )
            ))
            if endpoint_error > self.target_endpoint_tolerance:
                return TrajectoryRejectReason.TARGET_ENDPOINT_MISMATCH
        active = self.active_trajectory
        if (
            active is not None
            and active.mission_id == int(mission_id)
            and state.stamp < active.valid_until
        ):
            try:
                old_reference = active.sample_at_ros_time(state.stamp)
            except ValueError:
                old_reference = None
            if old_reference is not None:
                self.last_handover_position_error = self._norm(tuple(
                    new - old
                    for new, old in zip(
                        expected.position,
                        old_reference.position,
                    )
                ))
                self.last_handover_velocity_error = self._norm(tuple(
                    new - old
                    for new, old in zip(
                        expected.velocity,
                        old_reference.velocity,
                    )
                ))
                acceleration_delta = tuple(
                    new - old
                    for new, old in zip(
                        expected.acceleration,
                        old_reference.acceleration,
                    )
                )
                horizontal_acceleration_error = math.hypot(
                    acceleration_delta[0],
                    acceleration_delta[1],
                )
                vertical_acceleration_error = abs(acceleration_delta[2])
                self.last_handover_acceleration_error = max(
                    horizontal_acceleration_error,
                    vertical_acceleration_error,
                )
                if self.last_handover_position_error > position_limit:
                    return TrajectoryRejectReason.REFERENCE_POSITION_MISMATCH
                if (
                    self.last_handover_velocity_error
                    > self.maximum_velocity_error
                ):
                    return TrajectoryRejectReason.REFERENCE_VELOCITY_MISMATCH
                if (
                    horizontal_acceleration_error
                    > self.maximum_horizontal_acceleration
                    or vertical_acceleration_error
                    > self.maximum_vertical_acceleration
                ):
                    return (
                        TrajectoryRejectReason
                        .REFERENCE_ACCELERATION_MISMATCH
                    )
                same_contact = abs(
                    float(trajectory.contact_stamp)
                    - float(active.contact_stamp)
                ) <= 1e-3
                same_endpoint = self._norm(tuple(
                    new - old
                    for new, old in zip(
                        trajectory.terminal_position,
                        active.terminal_position,
                    )
                )) <= 1e-2
                if (
                    same_contact
                    and same_endpoint
                    and self.last_handover_position_error <= 1e-2
                    and self.last_handover_velocity_error <= 5e-2
                    and self.last_handover_acceleration_error <= 0.2
                ):
                    self.status = 'TRACKING'
                    return TrajectoryRejectReason.NONE
        safety = apply_sea_safety_guard(
            current_z=state.position[2],
            current_vz=state.velocity[2],
            proposed_vz=expected.velocity[2],
            sea_surface_z=self.sea_surface_z,
            reserve_clearance=self.reserve_clearance,
            response_delay=self.safety_response_delay,
            effective_braking_acceleration=(
                self.vertical_braking_acceleration
            ),
            control_dt=self.control_dt,
            maximum_vertical_speed=self.maximum_vertical_speed,
        )
        if safety.unrecoverable or safety.response_margin <= 0.0:
            return TrajectoryRejectReason.SAFETY_REJECTED
        self.active_trajectory = trajectory
        self.last_replacement_performed = True
        self.status = 'TRACKING'
        self.recovery_position = None
        return TrajectoryRejectReason.NONE

    @staticmethod
    def _limit_horizontal(values, maximum):
        magnitude = math.hypot(values[0], values[1])
        if magnitude <= maximum or magnitude <= 1e-12:
            return values[0], values[1]
        scale = maximum / magnitude
        return values[0] * scale, values[1] * scale

    def _shape_velocity(self, desired, stamp):
        horizontal = self._limit_horizontal(
            desired[:2],
            self.maximum_horizontal_speed,
        )
        limited = (
            horizontal[0],
            horizontal[1],
            max(min(desired[2], self.maximum_vertical_speed),
                -self.maximum_vertical_speed),
        )
        if self.previous_command_velocity is None:
            return limited
        dt = min(
            max(float(stamp) - self.previous_command_stamp, self.control_dt),
            self.maximum_command_dt,
        )
        delta_xy = (
            limited[0] - self.previous_command_velocity[0],
            limited[1] - self.previous_command_velocity[1],
        )
        limited_delta_xy = self._limit_horizontal(
            delta_xy,
            self.maximum_horizontal_acceleration * dt,
        )
        allowed_vertical = self.maximum_vertical_acceleration * dt
        measured = self.measured_vertical_acceleration
        if (
            measured is not None
            and self.maximum_actual_vertical_acceleration > 0.0
            and measured > self.maximum_actual_vertical_acceleration
        ):
            # The plant is already accelerating harder than its budget allows,
            # so shrink this step's allowance until the measured value settles
            # back inside the limit instead of asking for more.
            allowed_vertical *= (
                self.maximum_actual_vertical_acceleration / measured
            )
        delta_z = max(
            min(
                limited[2] - self.previous_command_velocity[2],
                allowed_vertical,
            ),
            -allowed_vertical,
        )
        return (
            self.previous_command_velocity[0] + limited_delta_xy[0],
            self.previous_command_velocity[1] + limited_delta_xy[1],
            self.previous_command_velocity[2] + delta_z,
        )

    def _terminal_cruise_velocity(self, state, control_stamp, target_at_time, target_velocity):
        """Aim at a short future target position without lowering cruise speed."""
        trajectory = self.active_trajectory
        remaining = trajectory.contact_stamp - control_stamp
        if (target_at_time is None or target_velocity is None
                or trajectory.contact_stamp <= 0. or not 0. <= remaining <= .7):
            return None
        # This is an explicit control-boundary model, not a retimed measurement.
        age = control_stamp - state.stamp
        origin = tuple(p + v * age for p, v in zip(state.position, state.velocity))
        target = target_at_time(control_stamp)
        relative = tuple(target[a] - origin[a] for a in (0, 1))
        distance = math.hypot(*relative)
        if not 1e-6 < distance <= 2.5:
            return None
        speed = self.maximum_horizontal_speed
        a = sum(v * v for v in target_velocity[:2]) - speed * speed
        b = 2. * sum(relative[i] * target_velocity[i] for i in (0, 1))
        c = distance * distance
        if a < -1e-6:
            lead = (-b - math.sqrt(max(b * b - 4. * a * c, 0.))) / (2. * a)
        else:
            lead = distance / speed
        target = target_at_time(control_stamp + min(max(lead, 0.), .7))
        aim = tuple(target[i] - origin[i] for i in (0, 1))
        norm = math.hypot(*aim)
        if norm <= 1e-6:
            return None
        desired = tuple(speed * value / norm for value in aim)
        previous = (self.previous_command_velocity or state.velocity)[:2]
        old_speed = min(math.hypot(*previous), speed)
        dt = self.control_dt if self.previous_command_stamp is None else min(
            max(control_stamp - self.previous_command_stamp, self.control_dt),
            self.maximum_command_dt,
        )
        budget = self.maximum_horizontal_acceleration * dt
        if old_speed <= 1e-6:
            return tuple(min(speed, budget) * value / speed for value in desired)
        heading = math.atan2(previous[1], previous[0])
        difference = math.atan2(math.sin(math.atan2(desired[1], desired[0]) - heading),
                                math.cos(math.atan2(desired[1], desired[0]) - heading))
        maximum_turn = 2. * math.asin(min(budget / (2. * old_speed), 1.))
        turn = max(-maximum_turn, min(difference, maximum_turn))
        # Spend the acceleration budget on steering first. Rotating at a
        # constant speed avoids the braking caused by vector interpolation.
        allowed_speed = old_speed * math.cos(turn) + math.sqrt(max(
            budget * budget - (old_speed * math.sin(turn)) ** 2, 0.,
        ))
        new_speed = max(old_speed, min(speed, allowed_speed))
        return (new_speed * math.cos(heading + turn), new_speed * math.sin(heading + turn))

    def command(self, state, mission_id, control_stamp=None,
                terminal_target_at_time=None, terminal_target_velocity=None,
                hold_terminal_velocity=False):
        """Return one post-Safety-Guard command or no valid-plan status."""
        control_stamp = state.stamp if control_stamp is None else float(control_stamp)
        if not math.isfinite(control_stamp) or control_stamp < state.stamp:
            raise ValueError('control epoch must not precede the measured state')
        self._observe_state(state)
        trajectory = self.active_trajectory
        if (
            trajectory is None
            or trajectory.mission_id != int(mission_id)
            or control_stamp >= trajectory.valid_until
        ):
            self.active_trajectory = None
            self.status = 'NO_VALID_PLAN'
            return None
        desired = trajectory.sample_at_ros_time(control_stamp)
        measured_reference = trajectory.sample_at_ros_time(state.stamp)
        # Remember the reference the vehicle was just asked to follow so a
        # later plan loss can continue from it instead of snapping away.
        self.last_reference_position = desired.position
        self.recovery_position = None
        feedback_velocity = tuple(
            velocity + self.position_gain * (reference - current)
            for velocity, reference, current in zip(
                desired.velocity,
                measured_reference.position,
                state.position,
            )
        )
        previous_velocity = self.previous_command_velocity
        previous_stamp = self.previous_command_stamp
        velocity = self._shape_velocity(feedback_velocity, control_stamp)
        terminal_velocity = self._terminal_cruise_velocity(
            state, control_stamp, terminal_target_at_time, terminal_target_velocity,
        )
        if (hold_terminal_velocity and terminal_velocity is None
                and trajectory.terminal_mode
                and 0. < trajectory.contact_stamp - control_stamp <= .7
                and previous_velocity is not None):
            # Explicit execution authority, not a stale target measurement:
            # preserve the last verified cruise vector until the plan expires.
            terminal_velocity = self._limit_horizontal(
                previous_velocity[:2], self.maximum_horizontal_speed)
        if terminal_velocity is not None:
            velocity = (*terminal_velocity, velocity[2])
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
            control_dt=self.control_dt,
            maximum_vertical_speed=self.maximum_vertical_speed,
        )
        velocity = (velocity[0], velocity[1], safety.command_vz)
        velocity_changed = self._norm(tuple(
            commanded - planned
            for commanded, planned in zip(velocity, desired.velocity)
        )) > 1e-6
        if velocity_changed and previous_velocity is not None:
            dt = max(
                control_stamp - float(previous_stamp),
                self.control_dt,
            )
            raw_acceleration = tuple(
                (current - previous) / dt
                for current, previous in zip(velocity, previous_velocity)
            )
            horizontal_acceleration = self._limit_horizontal(
                raw_acceleration[:2],
                self.maximum_horizontal_acceleration,
            )
            acceleration = (
                horizontal_acceleration[0],
                horizontal_acceleration[1],
                max(
                    min(
                        raw_acceleration[2],
                        self.maximum_vertical_acceleration,
                    ),
                    -self.maximum_vertical_acceleration,
                ),
            )
        elif velocity_changed:
            acceleration = (math.nan, math.nan, math.nan)
        else:
            acceleration = desired.acceleration
        self.previous_command_velocity = velocity
        self.previous_command_stamp = control_stamp
        self.status = 'TRACKING'
        return TrackingCommand(
            position=desired.position,
            velocity=velocity,
            acceleration=acceleration,
            plan_id=trajectory.plan_id,
            safety_state=safety.state.value,
            safety_margin=safety.response_margin,
        )

    def recovery_command(self, state, recovery_clearance=None):
        """
        Return a continuous, bounded command after the active plan is lost.

        The PX4 position setpoint is advanced from the last commanded
        reference with the same acceleration-limited velocity shaping used
        while tracking, so losing a plan no longer steps the reference back to
        the measured position with zero feed-forward.  While the reference is
        inside ``recovery_clearance`` of the sea the vertical target becomes a
        bounded climb, and the reference is never integrated past the reserve
        clearance, so a dropped terminal plan cannot park the airframe in the
        water margin.
        """
        self._observe_state(state)
        if self.recovery_position is None:
            if self.last_reference_position is not None:
                self.recovery_position = tuple(self.last_reference_position)
            else:
                self.recovery_position = tuple(state.position)
        previous_velocity = self.previous_command_velocity
        if previous_velocity is None:
            previous_velocity = (0.0, 0.0, 0.0)
        clearance = self.sea_surface_z - self.recovery_position[2]
        desired_clearance = (self.recovery_clearance
                             if recovery_clearance is None
                             else max(float(recovery_clearance),
                                      self.recovery_clearance))
        vertical_target = 0.0
        if clearance < desired_clearance:
            # Brake-to-target climb: never command more upward speed than the
            # vertical acceleration limit can arrest before reaching the
            # recovery clearance, so the hold settles instead of overshooting.
            remaining = desired_clearance - clearance
            vertical_target = -min(
                self.recovery_climb_speed,
                math.sqrt(
                    2.0 * self.maximum_vertical_acceleration * remaining
                ),
            )
        velocity = self._shape_velocity(
            (0.0, 0.0, vertical_target),
            state.stamp,
        )
        if self.previous_command_stamp is None:
            step = self.control_dt
        else:
            step = min(
                max(
                    float(state.stamp) - float(self.previous_command_stamp),
                    self.control_dt,
                ),
                self.maximum_command_dt,
            )
        position = tuple(
            current + 0.5 * (before + after) * step
            for current, before, after in zip(
                self.recovery_position,
                previous_velocity,
                velocity,
            )
        )
        ceiling = self.sea_surface_z - self.reserve_clearance
        if position[2] > ceiling:
            position = (position[0], position[1], ceiling)
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
            control_dt=self.control_dt,
            maximum_vertical_speed=self.maximum_vertical_speed,
        )
        velocity = (velocity[0], velocity[1], safety.command_vz)
        self.recovery_position = position
        self.previous_command_velocity = velocity
        self.previous_command_stamp = state.stamp
        self.status = 'RECOVERY'
        return TrackingCommand(
            position=position,
            velocity=velocity,
            acceleration=tuple(
                (after - before) / step
                for after, before in zip(velocity, previous_velocity)
            ),
            plan_id=0,
            safety_state=safety.state.value,
            safety_margin=safety.response_margin,
        )
