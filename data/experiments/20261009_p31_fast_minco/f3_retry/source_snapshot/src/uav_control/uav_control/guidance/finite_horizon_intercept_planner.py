"""Finite-horizon polynomial trajectory planning for moving interception."""

import math
import time
from dataclasses import dataclass, replace
from enum import Enum

import numpy as np
from scipy.optimize import minimize

from uav_control.guidance.minco_trajectory import MincoS3Trajectory
from uav_control.guidance.intercept_reachability import estimate_reachability


class PlanningFailureReason(str, Enum):
    """Auditable terminal-planning outcomes."""

    NONE = 'NONE'
    HORIZON_INSUFFICIENT = 'HORIZON_INSUFFICIENT'
    DYNAMIC_LIMIT = 'DYNAMIC_LIMIT'
    SEA_CLEARANCE = 'SEA_CLEARANCE'
    TRAJECTORY_INFEASIBLE = 'TRAJECTORY_INFEASIBLE'
    OPTIMIZATION_FAIL = 'OPTIMIZATION_FAIL'
    TARGET_SHIFT = 'TARGET_SHIFT'
    PLAN_EXPIRED = 'PLAN_EXPIRED'


@dataclass(frozen=True)
class PlannerDiagnostics:
    """Compact diagnostics from the most recent planning call."""

    failure_reason: PlanningFailureReason = PlanningFailureReason.NONE
    horizontal_min_time: float = math.nan
    vertical_min_time: float = math.nan
    sea_safe_min_time: float = math.nan
    required_time: float = math.nan
    search_min_time: float = math.nan
    search_max_time: float = math.nan
    selected_time: float = math.nan
    candidates_checked: int = 0
    total_compute_time: float = 0.0
    reachability_compute_time: float = 0.0
    generation_compute_time: float = 0.0
    validation_compute_time: float = 0.0
    optimization_compute_time: float = 0.0
    optimization_attempted: bool = False
    optimization_succeeded: bool = False
    candidate_diagnostics: tuple = ()


@dataclass(frozen=True)
class TrajectorySample:
    """Position, velocity, acceleration, and jerk at one trajectory time."""

    position: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple


@dataclass(frozen=True)
class QuinticAxis:
    """Quintic polynomial satisfying position, velocity, and acceleration."""

    coefficients: tuple

    @classmethod
    def from_boundary(cls, p0, v0, a0, pf, vf, af, duration):
        """Construct an axis polynomial from complete endpoint states."""
        duration = float(duration)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError('duration must be finite and positive')
        values = tuple(float(value) for value in (p0, v0, a0, pf, vf, af))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('boundary states must be finite')

        p0, v0, a0, pf, vf, af = values
        t2 = duration * duration
        t3 = t2 * duration
        t4 = t3 * duration
        t5 = t4 * duration
        c0 = p0
        c1 = v0
        c2 = 0.5 * a0
        c3 = (
            20.0 * (pf - p0)
            - (8.0 * vf + 12.0 * v0) * duration
            - (3.0 * a0 - af) * t2
        ) / (2.0 * t3)
        c4 = (
            30.0 * (p0 - pf)
            + (14.0 * vf + 16.0 * v0) * duration
            + (3.0 * a0 - 2.0 * af) * t2
        ) / (2.0 * t4)
        c5 = (
            12.0 * (pf - p0)
            - (6.0 * vf + 6.0 * v0) * duration
            - (a0 - af) * t2
        ) / (2.0 * t5)
        return cls((c0, c1, c2, c3, c4, c5))

    def sample(self, time):
        """Return position, velocity, acceleration, and jerk."""
        t = float(time)
        if not math.isfinite(t) or t < 0.0:
            raise ValueError('sample time must be finite and non-negative')
        c0, c1, c2, c3, c4, c5 = self.coefficients
        position = (
            c0 + c1 * t + c2 * t**2 + c3 * t**3
            + c4 * t**4 + c5 * t**5
        )
        velocity = (
            c1 + 2.0 * c2 * t + 3.0 * c3 * t**2
            + 4.0 * c4 * t**3 + 5.0 * c5 * t**4
        )
        acceleration = (
            2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t**2
            + 20.0 * c5 * t**3
        )
        jerk = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * t**2
        return position, velocity, acceleration, jerk


@dataclass(frozen=True)
class InterceptPlan:
    """One dynamically checked terminal interception trajectory."""

    axes: tuple
    duration: float
    closing_speed: float
    target_position: tuple
    target_velocity: tuple
    target_acceleration: tuple
    maximum_horizontal_speed: float
    maximum_vertical_speed: float
    maximum_horizontal_acceleration: float
    maximum_vertical_acceleration: float
    cost: float
    minco_trajectory: object = None
    planner_type: str = 'QUINTIC_S3'
    target_curve_weight: float = 0.0
    piece_durations: tuple = ()
    constraint_penalty: float = 0.0
    optimization_iterations: int = 0
    dynamically_feasible: bool = True
    sea_clearance_ceiling_z: float = math.inf

    def sample(self, time):
        """Sample all three axes of the planned trajectory."""
        if self.minco_trajectory is not None:
            return self.minco_trajectory.sample(time)
        values = [
            axis.sample(min(max(float(time), 0.0), self.duration))
            for axis in self.axes
        ]
        return TrajectorySample(
            tuple(value[0] for value in values),
            tuple(value[1] for value in values),
            tuple(value[2] for value in values),
            tuple(value[3] for value in values),
        )


class FiniteHorizonInterceptPlanner:
    """Search terminal time and closing speed for a feasible trajectory."""

    def __init__(
        self,
        minimum_duration,
        maximum_duration,
        duration_step,
        sample_step,
        maximum_horizontal_speed,
        maximum_vertical_speed,
        maximum_horizontal_acceleration,
        maximum_vertical_acceleration,
        desired_closing_speed,
        minimum_closing_speed,
        closing_speed_step,
        capture_radius,
        sea_surface_z,
        contact_clearance,
        minco_piece_count=1,
        minco_target_curve_weight=1.0,
        preferred_clearance=None,
        enable_minco_geometric_optimization=True,
        minco_optimization_max_iterations=3,
        minco_spatial_radius=0.75,
        minco_constraint_penalty_weight=1000.0,
        minco_quadrature_intervals_per_piece=8,
        absolute_maximum_duration=None,
        horizon_extra_margin=0.5,
        response_delay=0.15,
        effective_vertical_braking_acceleration=None,
    ):
        """Configure the finite-horizon feasibility search."""
        self.minimum_duration = self._positive(
            minimum_duration,
            'minimum duration',
        )
        self.maximum_duration = self._positive(
            maximum_duration,
            'maximum duration',
        )
        if self.maximum_duration < self.minimum_duration:
            raise ValueError('maximum duration must not be less than minimum')
        if absolute_maximum_duration is None:
            absolute_maximum_duration = self.maximum_duration
        self.absolute_maximum_duration = self._positive(
            absolute_maximum_duration,
            'absolute maximum duration',
        )
        if self.absolute_maximum_duration < self.maximum_duration:
            raise ValueError(
                'absolute maximum duration must not be less than maximum'
            )
        self.horizon_extra_margin = max(
            float(horizon_extra_margin),
            0.0,
        )
        self.response_delay = max(float(response_delay), 0.0)
        self.duration_step = self._positive(duration_step, 'duration step')
        self.sample_step = self._positive(sample_step, 'sample step')
        self.maximum_horizontal_speed = self._positive(
            maximum_horizontal_speed,
            'maximum horizontal speed',
        )
        self.maximum_vertical_speed = self._positive(
            maximum_vertical_speed,
            'maximum vertical speed',
        )
        self.maximum_horizontal_acceleration = self._positive(
            maximum_horizontal_acceleration,
            'maximum horizontal acceleration',
        )
        self.maximum_vertical_acceleration = self._positive(
            maximum_vertical_acceleration,
            'maximum vertical acceleration',
        )
        if effective_vertical_braking_acceleration is None:
            effective_vertical_braking_acceleration = (
                self.maximum_vertical_acceleration
            )
        self.effective_vertical_braking_acceleration = self._positive(
            effective_vertical_braking_acceleration,
            'effective vertical braking acceleration',
        )
        self.desired_closing_speed = self._positive(
            desired_closing_speed,
            'desired closing speed',
        )
        self.minimum_closing_speed = self._positive(
            minimum_closing_speed,
            'minimum closing speed',
        )
        self.minimum_closing_speed = min(
            self.minimum_closing_speed,
            self.desired_closing_speed,
        )
        self.closing_speed_step = self._positive(
            closing_speed_step,
            'closing speed step',
        )
        self.capture_radius = self._positive(capture_radius, 'capture radius')
        self.sea_surface_z = float(sea_surface_z)
        self.contact_clearance = self._positive(
            contact_clearance,
            'contact clearance',
        )
        if not math.isfinite(self.sea_surface_z):
            raise ValueError('sea surface z must be finite')
        if self.contact_clearance >= self.capture_radius:
            raise ValueError('contact clearance must be below capture radius')
        if preferred_clearance is None:
            preferred_clearance = self.contact_clearance
        self.preferred_clearance = self._positive(
            preferred_clearance,
            'preferred clearance',
        )
        self.minco_piece_count = int(minco_piece_count)
        if self.minco_piece_count < 1 or self.minco_piece_count > 6:
            raise ValueError('MINCO piece count must be between one and six')
        self.minco_target_curve_weight = float(minco_target_curve_weight)
        if (
            not math.isfinite(self.minco_target_curve_weight)
            or self.minco_target_curve_weight < 0.0
            or self.minco_target_curve_weight > 1.0
        ):
            raise ValueError('MINCO target curve weight must be in [0, 1]')
        self.enable_minco_geometric_optimization = bool(
            enable_minco_geometric_optimization
        )
        self.minco_optimization_max_iterations = max(
            int(minco_optimization_max_iterations),
            0,
        )
        self.minco_spatial_radius = self._positive(
            minco_spatial_radius,
            'MINCO spatial radius',
        )
        self.minco_constraint_penalty_weight = self._positive(
            minco_constraint_penalty_weight,
            'MINCO constraint penalty weight',
        )
        self.minco_quadrature_intervals_per_piece = max(
            int(minco_quadrature_intervals_per_piece),
            2,
        )
        self._last_duration = 0.5 * (
            self.minimum_duration + self.maximum_duration
        )
        self.last_diagnostics = PlannerDiagnostics()
        self._prewarm_minco_mappings()

    @staticmethod
    def _positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _vector(values, name):
        vector = tuple(float(value) for value in values)
        if (
            len(vector) != 3
            or not all(math.isfinite(value) for value in vector)
        ):
            raise ValueError(f'{name} must contain three finite values')
        return vector

    @staticmethod
    def _direction(start, end, fallback_velocity):
        delta = tuple(final - initial for initial, final in zip(start, end))
        distance = math.sqrt(sum(value * value for value in delta))
        if distance > 1e-9:
            return tuple(value / distance for value in delta)
        speed = math.sqrt(sum(value * value for value in fallback_velocity))
        if speed > 1e-9:
            return tuple(value / speed for value in fallback_velocity)
        return 1.0, 0.0, 0.0

    def _duration_candidates(self, minimum=None, maximum=None):
        if minimum is None:
            minimum = self.minimum_duration
        if maximum is None:
            maximum = self.maximum_duration
        count = int(math.floor(
            (maximum - minimum)
            / self.duration_step
            + 1e-9
        ))
        durations = [
            minimum + index * self.duration_step
            for index in range(count + 1)
        ]
        if durations[-1] < maximum - 1e-9:
            durations.append(maximum)
        if self._last_duration is not None:
            durations.sort(key=lambda value: abs(value - self._last_duration))
        return durations

    def _capture_contact_position(self, target_position):
        """Return a sea-safe point inside the configured capture sphere."""
        capture_compatible_clearance = max(
            self.sea_surface_z
            - (target_position[2] - self.capture_radius),
            self.contact_clearance,
        )
        effective_clearance = min(
            self.preferred_clearance,
            capture_compatible_clearance,
        )
        contact_position = list(target_position)
        contact_position[2] = min(
            contact_position[2],
            self.sea_surface_z - effective_clearance,
        )
        if abs(contact_position[2] - target_position[2]) > self.capture_radius:
            return None
        return tuple(contact_position)

    def _prewarm_minco_mappings(self):
        """Move constant MINCO matrix inversions outside the control loop."""
        if self.minco_piece_count == 1:
            return
        zero = (0.0, 0.0, 0.0)
        for duration in self._duration_candidates():
            piece_duration = duration / self.minco_piece_count
            MincoS3Trajectory(
                zero,
                zero,
                zero,
                zero,
                zero,
                zero,
                (zero,) * (self.minco_piece_count - 1),
                (piece_duration,) * self.minco_piece_count,
            )

    def _closing_speed_candidates(self):
        speeds = []
        speed = self.desired_closing_speed
        while speed > self.minimum_closing_speed + 1e-9:
            speeds.append(speed)
            speed -= self.closing_speed_step
        speeds.append(self.minimum_closing_speed)
        return speeds

    def _terminal_horizontal_velocity(self, target_velocity, direction, closing_speed):
        """Use the same target plus closing velocity in reachability and MINCO."""
        return tuple(target_velocity[a] + closing_speed * direction[a] for a in (0, 1))

    def _curve_weight_candidates(self):
        if self.minco_piece_count == 1:
            return (0.0,)
        weight = self.minco_target_curve_weight
        if weight <= 1e-9:
            return (0.0,)
        half_weight = 0.5 * weight
        if half_weight <= 0.05:
            return weight, 0.0
        quarter_weight = 0.25 * weight
        if quarter_weight <= 0.05:
            return weight, half_weight, 0.0
        return weight, half_weight, quarter_weight, 0.0

    def _candidate(
        self,
        initial_position,
        initial_velocity,
        initial_acceleration,
        target_position,
        target_velocity,
        target_acceleration,
        duration,
        closing_speed,
        previous_acceleration,
        target_start_position=None,
        intermediate_target_positions=(),
        target_curve_weight=0.0,
        guide_positions_override=None,
        piece_durations_override=None,
        optimization_iterations=0,
        return_infeasible=False,
    ):
        capture_compatible_clearance = max(
            self.sea_surface_z
            - (target_position[2] - self.capture_radius),
            self.contact_clearance,
        )
        effective_clearance = min(
            self.preferred_clearance,
            capture_compatible_clearance,
        )
        # Keep the validated ceiling on the hard floor rather than on the
        # preferred (larger) contact clearance.  The trajectory ends exactly at
        # the contact altitude, so using that same altitude as the ceiling made
        # millimetre-level interpolation sag mid-flight trip SEA_CLEARANCE.
        sea_clearance_ceiling_z = (
            self.sea_surface_z - self.contact_clearance
        )
        contact_position = self._capture_contact_position(target_position)
        if contact_position is None:
            return None
        horizontal_direction = self._direction(
            initial_position[:2],
            contact_position[:2],
            target_velocity[:2],
        )
        # Vertical target motion is sea-state heave, not a track to follow.
        # Matching it at contact forces the trajectory to dip below the
        # clearance ceiling just before the endpoint whenever the target
        # rises, so the UAV levels off vertically at the contact instead.  The
        # tiny upward bias keeps the contact strictly non-descending.
        terminal_horizontal = self._terminal_horizontal_velocity(
            target_velocity, horizontal_direction, closing_speed)
        terminal_velocity = (*terminal_horizontal, -1e-9)
        axes = tuple(
            QuinticAxis.from_boundary(
                initial_position[index],
                initial_velocity[index],
                initial_acceleration[index],
                contact_position[index],
                terminal_velocity[index],
                target_acceleration[index],
                duration,
            )
            for index in range(3)
        )

        minco_trajectory = None
        planner_type = 'QUINTIC_S3'
        if self.minco_piece_count > 1:
            if (
                target_start_position is None
                or len(intermediate_target_positions)
                != self.minco_piece_count - 1
            ):
                raise ValueError('MINCO target guide states are incomplete')
            if guide_positions_override is None:
                guide_positions = []
                for index, target_guide in enumerate(
                    intermediate_target_positions,
                    start=1,
                ):
                    fraction = index / self.minco_piece_count
                    baseline = tuple(
                        axis.sample(fraction * duration)[0]
                        for axis in axes
                    )
                    linear_target = tuple(
                        start + fraction * (end - start)
                        for start, end in zip(
                            target_start_position,
                            target_position,
                        )
                    )
                    # The target curve only shapes the horizontal intercept.
                    # Vertical target motion is sea-state heave, not a path to
                    # follow: blending it into the waypoints drags them below
                    # the sea-clearance ceiling and makes otherwise safe
                    # terminal dives infeasible.  Keep the vertical baseline
                    # and clamp it to the clearance floor as a hard backstop.
                    curve_weight = float(target_curve_weight)
                    guide_positions.append((
                        baseline[0] + curve_weight * (
                            target_guide[0] - linear_target[0]
                        ),
                        baseline[1] + curve_weight * (
                            target_guide[1] - linear_target[1]
                        ),
                        min(
                            baseline[2],
                            self.sea_surface_z - effective_clearance,
                        ),
                    ))
            else:
                guide_positions = [
                    self._vector(position, 'optimized MINCO waypoint')
                    for position in guide_positions_override
                ]
                if len(guide_positions) != self.minco_piece_count - 1:
                    raise ValueError(
                        'optimized MINCO waypoint count is invalid'
                    )
            if piece_durations_override is None:
                piece_duration = duration / self.minco_piece_count
                piece_durations = (
                    piece_duration,
                ) * self.minco_piece_count
            else:
                piece_durations = tuple(
                    float(value) for value in piece_durations_override
                )
                if (
                    len(piece_durations) != self.minco_piece_count
                    or not all(value > 0.0 for value in piece_durations)
                    or abs(sum(piece_durations) - duration) > 1e-6
                ):
                    raise ValueError('optimized MINCO durations are invalid')
            try:
                minco_trajectory = MincoS3Trajectory(
                    initial_position,
                    initial_velocity,
                    initial_acceleration,
                    contact_position,
                    terminal_velocity,
                    target_acceleration,
                    guide_positions,
                    piece_durations,
                )
            except ValueError:
                return None
            planner_type = 'MINCO_T3'

        def sample_trajectory(time):
            if minco_trajectory is not None:
                return minco_trajectory.sample(time)
            values = [axis.sample(time) for axis in axes]
            return TrajectorySample(
                tuple(value[0] for value in values),
                tuple(value[1] for value in values),
                tuple(value[2] for value in values),
                tuple(value[3] for value in values),
            )

        maximum_horizontal_speed = 0.0
        maximum_vertical_speed = 0.0
        maximum_horizontal_acceleration = 0.0
        maximum_vertical_acceleration = 0.0
        effort_cost = 0.0
        constraint_penalty = 0.0
        dynamically_feasible = True
        if minco_trajectory is not None:
            weighted_samples = minco_trajectory.quadrature_samples(
                self.minco_quadrature_intervals_per_piece
            )
        else:
            sample_count = max(
                int(math.ceil(duration / self.sample_step)),
                1,
            )
            step = duration / sample_count
            weighted_samples = (
                (
                    sample_trajectory(index * step),
                    step * (0.5 if index in (0, sample_count) else 1.0),
                )
                for index in range(sample_count + 1)
            )

        clearance_scale = max(effective_clearance, 0.05)
        for sample, integration_weight in weighted_samples:
            velocity = sample.velocity
            acceleration = sample.acceleration
            jerk = sample.jerk
            horizontal_speed = math.hypot(velocity[0], velocity[1])
            vertical_speed = abs(velocity[2])
            horizontal_acceleration = math.hypot(
                acceleration[0],
                acceleration[1],
            )
            vertical_acceleration = abs(acceleration[2])
            maximum_horizontal_speed = max(
                maximum_horizontal_speed,
                horizontal_speed,
            )
            maximum_vertical_speed = max(
                maximum_vertical_speed,
                vertical_speed,
            )
            maximum_horizontal_acceleration = max(
                maximum_horizontal_acceleration,
                horizontal_acceleration,
            )
            maximum_vertical_acceleration = max(
                maximum_vertical_acceleration,
                vertical_acceleration,
            )
            normalized_violations = (
                horizontal_speed / self.maximum_horizontal_speed - 1.0,
                vertical_speed / self.maximum_vertical_speed - 1.0,
                horizontal_acceleration
                / self.maximum_horizontal_acceleration - 1.0,
                vertical_acceleration
                / self.maximum_vertical_acceleration - 1.0,
                (
                    sample.position[2] - sea_clearance_ceiling_z
                ) / clearance_scale,
            )
            positive_violations = tuple(
                max(value, 0.0) for value in normalized_violations
            )
            if any(value > 1e-6 for value in positive_violations):
                dynamically_feasible = False
                if not return_infeasible:
                    return None
            # GCOPTER Eq. (87)-(90): differentiable cubic time-integral
            # penalty evaluated with trapezoidal quadrature.
            constraint_penalty += integration_weight * sum(
                value**3 for value in positive_violations
            )
            effort_cost += (
                horizontal_acceleration**2
                + vertical_acceleration**2
                + 0.01 * sum(component**2 for component in jerk)
            ) * integration_weight

        first_acceleration = sample_trajectory(
            min(self.sample_step, duration)
        ).acceleration
        continuity_cost = sum(
            (current - previous) ** 2
            for current, previous in zip(
                first_acceleration,
                previous_acceleration,
            )
        )
        closing_relaxation = (
            self.desired_closing_speed - closing_speed
        ) ** 2
        cost = (
            duration
            + 0.02 * effort_cost
            + 0.1 * continuity_cost
            + 0.2 * closing_relaxation
            + self.minco_constraint_penalty_weight * constraint_penalty
        )
        plan = InterceptPlan(
            axes,
            duration,
            closing_speed,
            contact_position,
            target_velocity,
            target_acceleration,
            maximum_horizontal_speed,
            maximum_vertical_speed,
            maximum_horizontal_acceleration,
            maximum_vertical_acceleration,
            cost,
            minco_trajectory,
            planner_type,
            target_curve_weight,
            (
                tuple(minco_trajectory.durations)
                if minco_trajectory is not None
                else (duration,)
            ),
            constraint_penalty,
            int(optimization_iterations),
            dynamically_feasible,
            sea_clearance_ceiling_z,
        )
        if dynamically_feasible or return_infeasible:
            return plan
        return None

    def _densely_validated_plan(self, plan):
        """Recheck a candidate at the configured hard-constraint resolution."""
        if plan is None:
            return None
        sample_count = max(
            int(math.ceil(plan.duration / self.sample_step)),
            1,
        )
        maximum_horizontal_speed = 0.0
        maximum_vertical_speed = 0.0
        maximum_horizontal_acceleration = 0.0
        maximum_vertical_acceleration = 0.0
        for index in range(sample_count + 1):
            sample = plan.sample(plan.duration * index / sample_count)
            horizontal_speed = math.hypot(
                sample.velocity[0],
                sample.velocity[1],
            )
            vertical_speed = abs(sample.velocity[2])
            horizontal_acceleration = math.hypot(
                sample.acceleration[0],
                sample.acceleration[1],
            )
            vertical_acceleration = abs(sample.acceleration[2])
            maximum_horizontal_speed = max(
                maximum_horizontal_speed,
                horizontal_speed,
            )
            maximum_vertical_speed = max(
                maximum_vertical_speed,
                vertical_speed,
            )
            maximum_horizontal_acceleration = max(
                maximum_horizontal_acceleration,
                horizontal_acceleration,
            )
            maximum_vertical_acceleration = max(
                maximum_vertical_acceleration,
                vertical_acceleration,
            )
            if (
                horizontal_speed > self.maximum_horizontal_speed + 1e-6
                or vertical_speed > self.maximum_vertical_speed + 1e-6
                or horizontal_acceleration
                > self.maximum_horizontal_acceleration + 1e-6
                or vertical_acceleration
                > self.maximum_vertical_acceleration + 1e-6
                or sample.position[2] > plan.sea_clearance_ceiling_z + 1e-6
            ):
                return None
        return replace(
            plan,
            maximum_horizontal_speed=maximum_horizontal_speed,
            maximum_vertical_speed=maximum_vertical_speed,
            maximum_horizontal_acceleration=maximum_horizontal_acceleration,
            maximum_vertical_acceleration=maximum_vertical_acceleration,
            dynamically_feasible=True,
        )

    def _validate_infeasible_candidates(self):
        """Return whether dense validation should diagnose rejected seeds."""
        return False

    def _geometrically_optimize_candidate(self, seed_plan, candidate_args):
        """Optimize MINCO waypoints and interval times."""
        if (
            seed_plan is None
            or seed_plan.minco_trajectory is None
            or self.minco_optimization_max_iterations <= 0
        ):
            return None

        waypoint_count = self.minco_piece_count - 1
        axes = seed_plan.axes
        target_start_position = candidate_args['target_start_position']
        target_position = candidate_args['target_position']
        intermediate_target_positions = candidate_args[
            'intermediate_target_positions'
        ]
        waypoint_baselines = []
        waypoint_curve_offsets = []
        for index, target_guide in enumerate(
            intermediate_target_positions,
            start=1,
        ):
            fraction = index / self.minco_piece_count
            baseline = tuple(
                axis.sample(fraction * seed_plan.duration)[0]
                for axis in axes
            )
            linear_target = tuple(
                start + fraction * (end - start)
                for start, end in zip(
                    target_start_position,
                    target_position,
                )
            )
            waypoint_baselines.append(baseline)
            # Match the horizontal-only curve guidance used by _candidate so
            # the optimizer never seeds or rewards waypoints pulled toward the
            # sea by the target's vertical heave.
            waypoint_curve_offsets.append((
                target_guide[0] - linear_target[0],
                target_guide[1] - linear_target[1],
                0.0,
            ))

        initial_weight = min(
            max(float(candidate_args['target_curve_weight']), 0.0),
            1.0,
        )
        corridor_radii = []
        initial_latents = []
        for curve_offset in waypoint_curve_offsets:
            offset_norm = math.sqrt(
                sum(value * value for value in curve_offset)
            )
            radius = max(self.minco_spatial_radius, offset_norm, 1e-6)
            desired_offset = min(initial_weight * offset_norm, radius)
            if desired_offset > 1e-9 and offset_norm > 1e-9:
                latent_norm = (
                    radius
                    - math.sqrt(max(radius**2 - desired_offset**2, 0.0))
                ) / desired_offset
                latent = tuple(
                    latent_norm * value / offset_norm
                    for value in curve_offset
                )
            else:
                latent = (0.0, 0.0, 0.0)
            corridor_radii.append(radius)
            initial_latents.append(latent)
        initial_variables = np.concatenate((
            np.asarray(initial_latents, dtype=float).reshape(-1),
            np.zeros(waypoint_count, dtype=float),
        ))
        bounds = [(-1.0, 1.0)] * (3 * waypoint_count)
        bounds.extend([(-1.75, 1.75)] * waypoint_count)
        best_feasible = None

        def evaluate(variables):
            nonlocal best_feasible
            variables = np.asarray(variables, dtype=float)
            spatial_latents = variables[:3 * waypoint_count].reshape((-1, 3))
            time_logits = variables[3 * waypoint_count:]
            guide_positions = tuple(
                MincoS3Trajectory.map_to_ball(
                    baseline,
                    radius,
                    latent,
                )
                for baseline, radius, latent in zip(
                    waypoint_baselines,
                    corridor_radii,
                    spatial_latents,
                )
            )
            curve_weights = []
            for waypoint, baseline, curve_offset in zip(
                guide_positions,
                waypoint_baselines,
                waypoint_curve_offsets,
            ):
                squared_offset = sum(value * value for value in curve_offset)
                if squared_offset <= 1e-12:
                    curve_weights.append(0.0)
                else:
                    curve_weights.append(sum(
                        (value - base) * offset
                        for value, base, offset in zip(
                            waypoint,
                            baseline,
                            curve_offset,
                        )
                    ) / squared_offset)
            piece_durations = MincoS3Trajectory.durations_from_logits(
                seed_plan.duration,
                time_logits,
            )
            candidate = self._candidate(
                **{
                    **candidate_args,
                    'target_curve_weight': float(np.mean(curve_weights)),
                },
                guide_positions_override=guide_positions,
                piece_durations_override=piece_durations,
                return_infeasible=True,
            )
            if candidate is None:
                return 1e12

            spatial_regularization = float(np.sum(
                (
                    spatial_latents
                    - np.asarray(initial_latents, dtype=float)
                ) ** 2
            ))
            temporal_regularization = float(np.dot(time_logits, time_logits))
            objective = (
                candidate.cost
                + 0.05 * spatial_regularization
                + 0.02 * temporal_regularization
            )
            if (
                candidate.dynamically_feasible
                and (
                    best_feasible is None
                    or objective < best_feasible[0]
                )
            ):
                best_feasible = objective, candidate
            return objective

        result = minimize(
            evaluate,
            initial_variables,
            method='L-BFGS-B',
            bounds=bounds,
            options={
                'maxiter': self.minco_optimization_max_iterations,
                'maxfun': 30,
                'maxls': 10,
                'ftol': 1e-5,
                'gtol': 1e-4,
            },
        )
        evaluate(result.x)
        if best_feasible is None:
            return None
        return replace(
            best_feasible[1],
            planner_type='MINCO_T3_OPT',
            optimization_iterations=int(result.nit),
        )

    def plan(
        self,
        initial_position,
        initial_velocity,
        initial_acceleration,
        target_state_at_time,
        previous_acceleration=(0.0, 0.0, 0.0),
    ):
        """
        Return the earliest feasible trajectory or ``None``.

        The search interval is rebuilt on every call from independent
        horizontal and vertical reachability bounds.  ``maximum_duration`` is
        the nominal upper bound; ``absolute_maximum_duration`` is the hard
        compute/mission cap.
        """
        compute_start = time.perf_counter()
        reachability_compute_time = 0.0
        generation_compute_time = 0.0
        validation_compute_time = 0.0
        optimization_compute_time = 0.0
        optimization_succeeded = False
        initial_position = self._vector(initial_position, 'initial position')
        initial_velocity = self._vector(initial_velocity, 'initial velocity')
        initial_acceleration = self._vector(
            initial_acceleration,
            'initial acceleration',
        )
        previous_acceleration = self._vector(
            previous_acceleration,
            'previous acceleration',
        )
        target_state_cache = {}

        def cached_target_state(time):
            key = round(float(time), 9)
            if key not in target_state_cache:
                target_state_cache[key] = target_state_at_time(float(time))
            return target_state_cache[key]

        def reachability_at(contact_time):
            target_state = cached_target_state(contact_time)
            if len(target_state) != 3:
                raise ValueError(
                    'target state must contain position, velocity, '
                    'acceleration'
                )
            target_position = self._vector(
                target_state[0],
                'target position',
            )
            target_velocity = self._vector(
                target_state[1],
                'target velocity',
            )
            contact = self._capture_contact_position(target_position)
            if contact is None:
                return None, None
            # Use the endpoint velocities that _candidate actually imposes.
            # Matching the target alone invents an extra deceleration and
            # rejects feasible short contacts with nonzero closing speed.
            direction = self._direction(
                initial_position[:2], contact[:2], target_velocity[:2],
            )
            estimates = []
            for closing_speed in self._closing_speed_candidates():
                horizontal_velocity = self._terminal_horizontal_velocity(
                    target_velocity, direction, closing_speed)
                guidance_velocity = (*horizontal_velocity, 0.0)
                estimates.append(estimate_reachability(
                    initial_position,
                    initial_velocity,
                    contact,
                    guidance_velocity,
                    self.maximum_horizontal_speed,
                    self.maximum_vertical_speed,
                    self.maximum_horizontal_acceleration,
                    self.maximum_vertical_acceleration,
                    self.effective_vertical_braking_acceleration,
                    self.response_delay,
                    self.absolute_maximum_duration,
                ))
            # This is only a search lower bound. Every generated candidate
            # still passes the full speed, acceleration and sea-safety gates.
            estimate = min(estimates, key=lambda value: value.required_time)
            return estimate, contact

        def first_finite_reachability(start_time):
            probe_time = min(
                max(float(start_time), self.minimum_duration),
                self.absolute_maximum_duration,
            )

            while True:
                probe_reachability, probe_contact = reachability_at(
                    probe_time
                )

                if probe_contact is None:
                    return probe_time, None, None

                if math.isfinite(
                    probe_reachability.required_time
                ):
                    return (
                        probe_time,
                        probe_reachability,
                        probe_contact,
                    )

                if (
                    probe_time
                    >= self.absolute_maximum_duration - 1e-9
                ):
                    return (
                        probe_time,
                        probe_reachability,
                        probe_contact,
                    )

                probe_time = min(
                    probe_time + self.duration_step,
                    self.absolute_maximum_duration,
                )

        # For a moving endpoint an early contact can be unreachable while a
        # later target state inside the same prediction horizon is reachable.
        # Do not classify the whole horizon from the first infinite bound.
        reachability_start = time.perf_counter()
        estimate_time, reachability, contact_position = (
            first_finite_reachability(self.minimum_duration)
        )

        if contact_position is None:
            reachability_compute_time = (
                time.perf_counter() - reachability_start
            )
            self.last_diagnostics = PlannerDiagnostics(
                failure_reason=PlanningFailureReason.SEA_CLEARANCE,
                total_compute_time=time.perf_counter() - compute_start,
                reachability_compute_time=reachability_compute_time,
            )
            return None

        # Fixed-point refinement starts at the first finite moving-target
        # contact probe and never moves the contact floor backward.
        for _ in range(4):
            required_time = reachability.required_time
            if not math.isfinite(required_time):
                break

            updated_time = max(
                estimate_time,
                required_time,
            )

            if abs(updated_time - estimate_time) <= 0.02:
                break

            if (
                estimate_time
                >= self.absolute_maximum_duration - 1e-9
            ):
                break

            estimate_time, reachability, contact_position = (
                first_finite_reachability(updated_time)
            )

            if contact_position is None:
                reachability_compute_time = (
                    time.perf_counter() - reachability_start
                )
                self.last_diagnostics = PlannerDiagnostics(
                    failure_reason=PlanningFailureReason.SEA_CLEARANCE,
                    total_compute_time=(
                        time.perf_counter() - compute_start
                    ),
                    reachability_compute_time=reachability_compute_time,
                )
                return None

        reachability_compute_time = (
            time.perf_counter() - reachability_start
        )

        required_time = (
            reachability.required_time
            if reachability is not None
            else math.inf
        )
        search_minimum = max(
            self.minimum_duration,
            estimate_time,
            required_time,
        )
        search_maximum = min(
            self.absolute_maximum_duration,
            max(
                self.maximum_duration,
                search_minimum + self.horizon_extra_margin,
            ),
        )
        base_diagnostics = dict(
            reachability_compute_time=reachability_compute_time,
            horizontal_min_time=(
                reachability.horizontal_min_time
                if reachability is not None else math.inf
            ),
            vertical_min_time=(
                reachability.vertical_min_time
                if reachability is not None else math.inf
            ),
            sea_safe_min_time=(
                reachability.sea_safe_min_time
                if reachability is not None else math.inf
            ),
            required_time=required_time,
            search_min_time=search_minimum,
            search_max_time=search_maximum,
        )
        if (
            not math.isfinite(required_time)
            or search_minimum > self.absolute_maximum_duration + 1e-9
        ):
            self.last_diagnostics = PlannerDiagnostics(
                failure_reason=PlanningFailureReason.HORIZON_INSUFFICIENT,
                total_compute_time=time.perf_counter() - compute_start,
                **base_diagnostics,
            )
            return None

        candidate_count = 0
        dynamic_rejection = False
        sea_rejection = False
        optimization_attempted = False

        for duration in self._duration_candidates(
            search_minimum,
            search_maximum,
        ):
            target_state = cached_target_state(duration)
            if len(target_state) != 3:
                raise ValueError(
                    'target state must contain position, velocity, '
                    'acceleration'
                )
            target_position = self._vector(target_state[0], 'target position')
            target_velocity = self._vector(target_state[1], 'target velocity')
            target_acceleration = self._vector(
                target_state[2],
                'target acceleration',
            )
            target_start_position = None
            intermediate_target_positions = ()
            if self.minco_piece_count > 1:
                start_state = cached_target_state(0.0)
                if len(start_state) != 3:
                    raise ValueError(
                        'target state must contain position, velocity, '
                        'acceleration'
                    )
                target_start_position = self._vector(
                    start_state[0],
                    'target start position',
                )
                intermediate_target_positions = tuple(
                    self._vector(
                        cached_target_state(
                            duration * index / self.minco_piece_count
                        )[0],
                        'target guide position',
                    )
                    for index in range(1, self.minco_piece_count)
                )
            for closing_speed in self._closing_speed_candidates():
                for curve_weight in self._curve_weight_candidates():
                    candidate_args = dict(
                        initial_position=initial_position,
                        initial_velocity=initial_velocity,
                        initial_acceleration=initial_acceleration,
                        target_position=target_position,
                        target_velocity=target_velocity,
                        target_acceleration=target_acceleration,
                        duration=duration,
                        closing_speed=closing_speed,
                        previous_acceleration=previous_acceleration,
                        target_start_position=target_start_position,
                        intermediate_target_positions=(
                            intermediate_target_positions
                        ),
                        target_curve_weight=curve_weight,
                    )
                    generation_start = time.perf_counter()
                    candidate = self._candidate(
                        **candidate_args,
                        return_infeasible=True,
                    )
                    generation_compute_time += (
                        time.perf_counter() - generation_start
                    )
                    candidate_count += 1
                    if candidate is None:
                        sea_rejection = True
                    elif not candidate.dynamically_feasible:
                        dynamic_rejection = True
                        if not self._validate_infeasible_candidates():
                            candidate = None
                    if candidate is not None:
                        validation_start = time.perf_counter()
                        candidate = self._densely_validated_plan(candidate)
                        validation_compute_time += (
                            time.perf_counter() - validation_start
                        )
                    if candidate is not None:
                        if (
                            self.enable_minco_geometric_optimization
                            and candidate.minco_trajectory is not None
                            and self.minco_optimization_max_iterations > 0
                        ):
                            optimization_attempted = True
                            optimization_start = time.perf_counter()
                            optimized = self._geometrically_optimize_candidate(
                                candidate,
                                candidate_args,
                            )
                            optimization_compute_time += (
                                time.perf_counter() - optimization_start
                            )
                            validation_start = time.perf_counter()
                            validated_optimized = self._densely_validated_plan(
                                optimized,
                            )
                            validation_compute_time += (
                                time.perf_counter() - validation_start
                            )
                            if validated_optimized is not None:
                                candidate = validated_optimized
                                optimization_succeeded = True
                        self._last_duration = candidate.duration
                        self.last_diagnostics = PlannerDiagnostics(
                            failure_reason=PlanningFailureReason.NONE,
                            selected_time=candidate.duration,
                            candidates_checked=candidate_count,
                            total_compute_time=(
                                time.perf_counter() - compute_start
                            ),
                            generation_compute_time=generation_compute_time,
                            validation_compute_time=validation_compute_time,
                            optimization_compute_time=(
                                optimization_compute_time
                            ),
                            optimization_attempted=optimization_attempted,
                            optimization_succeeded=optimization_succeeded,
                            **base_diagnostics,
                        )
                        return candidate

        if (
            self.enable_minco_geometric_optimization
            and self.minco_piece_count > 1
            and self.minco_optimization_max_iterations > 0
        ):
            # A single long-horizon, low-closing-speed seed bounds rescue
            # optimization cost.  The fast deterministic search above remains
            # the normal online path.
            duration = search_maximum
            target_state = cached_target_state(duration)
            target_position = self._vector(
                target_state[0],
                'target position',
            )
            target_velocity = self._vector(
                target_state[1],
                'target velocity',
            )
            target_acceleration = self._vector(
                target_state[2],
                'target acceleration',
            )
            target_start_position = self._vector(
                cached_target_state(0.0)[0],
                'target start position',
            )
            intermediate_target_positions = tuple(
                self._vector(
                    cached_target_state(
                        duration * index / self.minco_piece_count
                    )[0],
                    'target guide position',
                )
                for index in range(1, self.minco_piece_count)
            )
            candidate_args = dict(
                initial_position=initial_position,
                initial_velocity=initial_velocity,
                initial_acceleration=initial_acceleration,
                target_position=target_position,
                target_velocity=target_velocity,
                target_acceleration=target_acceleration,
                duration=duration,
                closing_speed=self.minimum_closing_speed,
                previous_acceleration=previous_acceleration,
                target_start_position=target_start_position,
                intermediate_target_positions=intermediate_target_positions,
                target_curve_weight=self.minco_target_curve_weight,
            )
            generation_start = time.perf_counter()
            seed = self._candidate(
                **candidate_args,
                return_infeasible=True,
            )
            generation_compute_time += time.perf_counter() - generation_start
            optimization_start = time.perf_counter()
            optimized = self._geometrically_optimize_candidate(
                seed,
                candidate_args,
            )
            optimization_compute_time += (
                time.perf_counter() - optimization_start
            )
            optimization_attempted = True
            validation_start = time.perf_counter()
            optimized = self._densely_validated_plan(optimized)
            validation_compute_time += (
                time.perf_counter() - validation_start
            )
            if optimized is not None:
                optimization_succeeded = True
                self._last_duration = optimized.duration
                self.last_diagnostics = PlannerDiagnostics(
                    failure_reason=PlanningFailureReason.NONE,
                    selected_time=optimized.duration,
                    candidates_checked=candidate_count + 1,
                    total_compute_time=time.perf_counter() - compute_start,
                    generation_compute_time=generation_compute_time,
                    validation_compute_time=validation_compute_time,
                    optimization_compute_time=optimization_compute_time,
                    optimization_attempted=optimization_attempted,
                    optimization_succeeded=optimization_succeeded,
                    **base_diagnostics,
                )
                return optimized
        if dynamic_rejection:
            failure_reason = PlanningFailureReason.DYNAMIC_LIMIT
        elif sea_rejection:
            failure_reason = PlanningFailureReason.SEA_CLEARANCE
        elif optimization_attempted:
            failure_reason = PlanningFailureReason.OPTIMIZATION_FAIL
        else:
            failure_reason = PlanningFailureReason.TRAJECTORY_INFEASIBLE
        self.last_diagnostics = PlannerDiagnostics(
            failure_reason=failure_reason,
            candidates_checked=candidate_count,
            total_compute_time=time.perf_counter() - compute_start,
            generation_compute_time=generation_compute_time,
            validation_compute_time=validation_compute_time,
            optimization_compute_time=optimization_compute_time,
            optimization_attempted=optimization_attempted,
            optimization_succeeded=optimization_succeeded,
            **base_diagnostics,
        )
        return None
