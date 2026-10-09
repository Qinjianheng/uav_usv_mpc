"""Deadline-aware MINCO planning with at most six realtime candidates."""

import math
import time
from dataclasses import dataclass, replace
from enum import Enum

from .finite_horizon_intercept_planner import (
    FiniteHorizonInterceptPlanner,
    PlanningFailureReason,
)


class FastPlanningFailure(str, Enum):
    """Auditable failure classes for the realtime planner."""

    NONE = 'NONE'
    STATE_STALE = 'STATE_STALE'
    PREDICTION_STALE = 'PREDICTION_STALE'
    HORIZON_INSUFFICIENT = 'HORIZON_INSUFFICIENT'
    CAPTURE_GEOMETRY = 'CAPTURE_GEOMETRY'
    DYNAMIC_LIMIT_HORIZONTAL = 'DYNAMIC_LIMIT_HORIZONTAL'
    DYNAMIC_LIMIT_VERTICAL = 'DYNAMIC_LIMIT_VERTICAL'
    SEA_CLEARANCE = 'SEA_CLEARANCE'
    MINCO_CONSTRUCTION_FAIL = 'MINCO_CONSTRUCTION_FAIL'
    OPTIMIZATION_FAIL = 'OPTIMIZATION_FAIL'
    DEADLINE_EXCEEDED = 'DEADLINE_EXCEEDED'
    PLAN_STALE_ON_ARRIVAL = 'PLAN_STALE_ON_ARRIVAL'


@dataclass(frozen=True)
class FastPlanningOutcome:
    """One fast planning result and its event-level diagnostics."""

    plan: object
    failure: FastPlanningFailure
    diagnostics: object


@dataclass(frozen=True)
class CandidateDiagnostic:
    """Bounded summary of one generated and densely checked candidate."""

    duration: float
    closing_speed: float
    curve_weight: float
    generation_time: float
    validation_time: float = 0.0
    maximum_horizontal_speed: float = 0.0
    maximum_vertical_speed: float = 0.0
    maximum_horizontal_acceleration: float = 0.0
    maximum_vertical_acceleration: float = 0.0
    maximum_sea_clearance_violation: float = 0.0
    sea_clearance_violation_time: float = -1.0
    maximum_constraint_violation: float = 0.0
    maximum_violation_time: float = -1.0
    maximum_violation_phase: str = 'NONE'
    failure: str = FastPlanningFailure.NONE.value
    violations: tuple = ()


class FastMincoPlanner(FiniteHorizonInterceptPlanner):
    """Use reachability-driven horizons and no realtime L-BFGS search."""

    def __init__(
        self,
        minimum_duration,
        maximum_duration,
        duration_margin,
        sample_step,
        maximum_horizontal_speed,
        maximum_vertical_speed,
        maximum_horizontal_acceleration,
        maximum_vertical_acceleration,
        preferred_closing_speed,
        conservative_closing_speed,
        capture_radius,
        sea_surface_z,
        contact_clearance,
        preferred_clearance,
        piece_count=3,
        target_curve_weight=0.7,
        deadline_seconds=0.08,
        response_delay=0.15,
        effective_vertical_braking_acceleration=2.5,
        quadrature_intervals_per_piece=6,
        clock=time.perf_counter,
    ):
        self.duration_margin = self._finite_positive(
            duration_margin,
            'duration margin',
        )
        self.deadline_seconds = self._finite_positive(
            deadline_seconds,
            'planning deadline',
        )
        self.preferred_realtime_closing_speed = self._finite_positive(
            preferred_closing_speed,
            'preferred closing speed',
        )
        self.conservative_realtime_closing_speed = self._finite_positive(
            conservative_closing_speed,
            'conservative closing speed',
        )
        self.realtime_curve_weight = float(target_curve_weight)
        self.clock = clock
        self.last_success_duration = None
        self._preferred_duration = None
        self._active_deadline = None
        self._deadline_exceeded = False
        self._candidate_failures = []
        self._candidate_diagnostics = []
        super().__init__(
            minimum_duration=minimum_duration,
            maximum_duration=maximum_duration,
            absolute_maximum_duration=maximum_duration,
            horizon_extra_margin=2.0 * self.duration_margin,
            duration_step=self.duration_margin,
            sample_step=sample_step,
            maximum_horizontal_speed=maximum_horizontal_speed,
            maximum_vertical_speed=maximum_vertical_speed,
            maximum_horizontal_acceleration=maximum_horizontal_acceleration,
            maximum_vertical_acceleration=maximum_vertical_acceleration,
            desired_closing_speed=preferred_closing_speed,
            minimum_closing_speed=conservative_closing_speed,
            closing_speed_step=max(
                preferred_closing_speed - conservative_closing_speed,
                1e-3,
            ),
            capture_radius=capture_radius,
            sea_surface_z=sea_surface_z,
            contact_clearance=contact_clearance,
            preferred_clearance=preferred_clearance,
            minco_piece_count=piece_count,
            minco_target_curve_weight=target_curve_weight,
            enable_minco_geometric_optimization=False,
            minco_optimization_max_iterations=0,
            minco_quadrature_intervals_per_piece=(
                quadrature_intervals_per_piece
            ),
            response_delay=response_delay,
            effective_vertical_braking_acceleration=(
                effective_vertical_braking_acceleration
            ),
        )

    @staticmethod
    def _finite_positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _unique(values):
        unique = []
        for value in values:
            if not any(abs(value - current) <= 1e-9 for current in unique):
                unique.append(value)
        return unique

    def _duration_candidates(self, minimum=None, maximum=None):
        minimum = self.minimum_duration if minimum is None else float(minimum)
        maximum = self.maximum_duration if maximum is None else float(maximum)
        attempted = []

        def available(value):
            return not any(abs(value - old) <= 1e-9 for old in attempted)

        preferred = self._preferred_duration
        if preferred is not None and minimum <= preferred <= maximum:
            first = preferred
        else:
            first = minimum
        attempted.append(first)
        yield first

        continuity = None
        if (
            self.last_success_duration is not None
            and minimum <= self.last_success_duration <= maximum
            and available(self.last_success_duration)
        ):
            continuity = self.last_success_duration
        elif available(minimum):
            continuity = minimum
        elif available(maximum):
            continuity = maximum
        if continuity is not None:
            attempted.append(continuity)
            yield continuity

        dynamic_failures = {
            FastPlanningFailure.DYNAMIC_LIMIT_HORIZONTAL,
            FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL,
        }
        if any(
            failure in dynamic_failures
            for failure in self._candidate_failures
        ):
            # A dynamic-limit rejection means the profile is too aggressive and
            # only more time can cure it, so probe the rest of the horizon
            # instead of advancing by a single duration_margin step and giving
            # up.  Two probes are enough: a quarter-span sweep measured the
            # same success rate for roughly twice the compute, and the extra
            # candidates pushed plans into DEADLINE_EXCEEDED.
            span = maximum - minimum
            for fraction in (0.5, 1.0):
                value = minimum + fraction * span
                if available(value):
                    attempted.append(value)
                    yield value
        else:
            for value in (
                minimum + 0.5 * (maximum - minimum),
                maximum,
                minimum,
            ):
                if available(value):
                    attempted.append(value)
                    yield value
                    return

    def _closing_speed_candidates(self):
        return tuple(self._unique((
            self.preferred_realtime_closing_speed,
            self.conservative_realtime_closing_speed,
        )))

    def _curve_weight_candidates(self):
        return (self.realtime_curve_weight,)

    def _deadline_reached(self):
        return (
            self._active_deadline is not None
            and self.clock() >= self._active_deadline
        )

    @staticmethod
    def _failure_from_violations(violations):
        if 'SEA_CLEARANCE' in violations:
            return FastPlanningFailure.SEA_CLEARANCE
        if any(value.startswith('VERTICAL_') for value in violations):
            return FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL
        if any(value.startswith('HORIZONTAL_') for value in violations):
            return FastPlanningFailure.DYNAMIC_LIMIT_HORIZONTAL
        return FastPlanningFailure.NONE

    def _record_candidate(
        self,
        duration,
        closing_speed,
        curve_weight,
        generation_time,
        candidate=None,
        failure=FastPlanningFailure.NONE,
        violations=(),
    ):
        if len(self._candidate_diagnostics) >= 6:
            return
        self._candidate_diagnostics.append(CandidateDiagnostic(
            duration=float(duration),
            closing_speed=float(
                candidate.closing_speed if candidate is not None else closing_speed),
            curve_weight=float(curve_weight),
            generation_time=max(float(generation_time), 0.0),
            maximum_horizontal_speed=(
                float(candidate.maximum_horizontal_speed)
                if candidate is not None else 0.0
            ),
            maximum_vertical_speed=(
                float(candidate.maximum_vertical_speed)
                if candidate is not None else 0.0
            ),
            maximum_horizontal_acceleration=(
                float(candidate.maximum_horizontal_acceleration)
                if candidate is not None else 0.0
            ),
            maximum_vertical_acceleration=(
                float(candidate.maximum_vertical_acceleration)
                if candidate is not None else 0.0
            ),
            failure=failure.value,
            violations=tuple(violations),
        ))

    def _candidate(self, *args, **kwargs):
        generation_start = time.perf_counter()
        duration = kwargs.get('duration', args[6] if len(args) > 6 else 0.0)
        closing_speed = kwargs.get(
            'closing_speed',
            args[7] if len(args) > 7 else 0.0,
        )
        curve_weight = kwargs.get('target_curve_weight', 0.0)
        if self._deadline_reached():
            self._deadline_exceeded = True
            self._record_candidate(
                duration,
                closing_speed,
                curve_weight,
                time.perf_counter() - generation_start,
                failure=FastPlanningFailure.DEADLINE_EXCEEDED,
                violations=('DEADLINE_EXCEEDED',),
            )
            return None
        target_position = kwargs.get('target_position')
        if target_position is None and len(args) >= 4:
            target_position = args[3]
        if self._capture_contact_position(target_position) is None:
            self._candidate_failures.append(
                FastPlanningFailure.CAPTURE_GEOMETRY
            )
            self._record_candidate(
                duration,
                closing_speed,
                curve_weight,
                time.perf_counter() - generation_start,
                failure=FastPlanningFailure.CAPTURE_GEOMETRY,
                violations=('CAPTURE_GEOMETRY',),
            )
            return None
        candidate = super()._candidate(*args, **kwargs)
        generation_time = time.perf_counter() - generation_start
        if self._deadline_reached():
            self._deadline_exceeded = True
            self._record_candidate(
                duration,
                closing_speed,
                curve_weight,
                generation_time,
                candidate=candidate,
                failure=FastPlanningFailure.DEADLINE_EXCEEDED,
                violations=('DEADLINE_EXCEEDED',),
            )
            return None
        if candidate is None:
            self._candidate_failures.append(
                FastPlanningFailure.MINCO_CONSTRUCTION_FAIL
            )
            self._record_candidate(
                duration,
                closing_speed,
                curve_weight,
                generation_time,
                failure=FastPlanningFailure.MINCO_CONSTRUCTION_FAIL,
                violations=('MINCO_CONSTRUCTION_FAIL',),
            )
        else:
            self._record_candidate(
                duration,
                closing_speed,
                curve_weight,
                generation_time,
                candidate=candidate,
            )
        return candidate

    def _validate_infeasible_candidates(self):
        return True

    def _densely_validated_plan(self, candidate):
        """Validate every hard constraint and retain one compact summary."""
        if candidate is None:
            return None
        validation_start = time.perf_counter()
        sample_count = max(
            int(math.ceil(candidate.duration / self.sample_step)),
            1,
        )
        maximum_horizontal_speed = 0.0
        maximum_vertical_speed = 0.0
        maximum_horizontal_acceleration = 0.0
        maximum_vertical_acceleration = 0.0
        maximum_sea_violation = 0.0
        sea_violation_time = -1.0
        maximum_constraint_violation = 0.0
        maximum_violation_time = -1.0
        violations = set()
        for index in range(sample_count + 1):
            sample_time = candidate.duration * index / sample_count
            sample = candidate.sample(sample_time)
            horizontal_speed = math.hypot(
                sample.velocity[0], sample.velocity[1]
            )
            vertical_speed = abs(sample.velocity[2])
            horizontal_acceleration = math.hypot(
                sample.acceleration[0], sample.acceleration[1]
            )
            vertical_acceleration = abs(sample.acceleration[2])
            maximum_horizontal_speed = max(
                maximum_horizontal_speed, horizontal_speed
            )
            maximum_vertical_speed = max(
                maximum_vertical_speed, vertical_speed
            )
            maximum_horizontal_acceleration = max(
                maximum_horizontal_acceleration, horizontal_acceleration
            )
            maximum_vertical_acceleration = max(
                maximum_vertical_acceleration, vertical_acceleration
            )
            sea_violation = max(
                sample.position[2] - candidate.sea_clearance_ceiling_z,
                0.0,
            )
            sample_maximum_violation = max(
                horizontal_speed - self.maximum_horizontal_speed,
                vertical_speed - self.maximum_vertical_speed,
                horizontal_acceleration
                - self.maximum_horizontal_acceleration,
                vertical_acceleration - self.maximum_vertical_acceleration,
                sea_violation,
                0.0,
            )
            if sample_maximum_violation > maximum_constraint_violation:
                maximum_constraint_violation = sample_maximum_violation
                maximum_violation_time = sample_time
            if sea_violation > max(maximum_sea_violation, 1e-6):
                maximum_sea_violation = sea_violation
                sea_violation_time = sample_time
            else:
                maximum_sea_violation = max(
                    maximum_sea_violation,
                    sea_violation,
                )
            if horizontal_speed > self.maximum_horizontal_speed + 1e-6:
                violations.add('HORIZONTAL_SPEED')
            if vertical_speed > self.maximum_vertical_speed + 1e-6:
                violations.add('VERTICAL_SPEED')
            if (
                horizontal_acceleration
                > self.maximum_horizontal_acceleration + 1e-6
            ):
                violations.add('HORIZONTAL_ACCELERATION')
            if (
                vertical_acceleration
                > self.maximum_vertical_acceleration + 1e-6
            ):
                violations.add('VERTICAL_ACCELERATION')
            if sea_violation > 1e-6:
                violations.add('SEA_CLEARANCE')

        validation_time = time.perf_counter() - validation_start
        if maximum_violation_time < 0.0:
            maximum_violation_phase = 'NONE'
        elif maximum_violation_time <= 0.2 * candidate.duration:
            maximum_violation_phase = 'START'
        elif maximum_violation_time >= 0.8 * candidate.duration:
            maximum_violation_phase = 'END'
        else:
            maximum_violation_phase = 'MIDDLE'
        failure = self._failure_from_violations(violations)
        if failure != FastPlanningFailure.NONE:
            self._candidate_failures.append(failure)
        if self._candidate_diagnostics:
            current = self._candidate_diagnostics[-1]
            self._candidate_diagnostics[-1] = replace(
                current,
                validation_time=validation_time,
                maximum_horizontal_speed=maximum_horizontal_speed,
                maximum_vertical_speed=maximum_vertical_speed,
                maximum_horizontal_acceleration=(
                    maximum_horizontal_acceleration
                ),
                maximum_vertical_acceleration=maximum_vertical_acceleration,
                maximum_sea_clearance_violation=maximum_sea_violation,
                sea_clearance_violation_time=sea_violation_time,
                maximum_constraint_violation=(
                    maximum_constraint_violation
                ),
                maximum_violation_time=maximum_violation_time,
                maximum_violation_phase=maximum_violation_phase,
                failure=failure.value,
                violations=tuple(sorted(violations)),
            )
        if self._deadline_reached():
            self._deadline_exceeded = True
            if self._candidate_diagnostics:
                current = self._candidate_diagnostics[-1]
                self._candidate_diagnostics[-1] = replace(
                    current,
                    failure=FastPlanningFailure.DEADLINE_EXCEEDED.value,
                    violations=tuple(sorted(
                        set(current.violations) | {'DEADLINE_EXCEEDED'}
                    )),
                )
            return None
        if violations:
            return None
        return replace(
            candidate,
            maximum_horizontal_speed=maximum_horizontal_speed,
            maximum_vertical_speed=maximum_vertical_speed,
            maximum_horizontal_acceleration=maximum_horizontal_acceleration,
            maximum_vertical_acceleration=maximum_vertical_acceleration,
            dynamically_feasible=True,
        )

    def _mapped_failure(self):
        if self._deadline_exceeded:
            return FastPlanningFailure.DEADLINE_EXCEEDED
        if self.last_diagnostics.failure_reason == (
            PlanningFailureReason.HORIZON_INSUFFICIENT
        ):
            return FastPlanningFailure.HORIZON_INSUFFICIENT
        if (
            self.last_diagnostics.failure_reason
            == PlanningFailureReason.SEA_CLEARANCE
            and self.last_diagnostics.candidates_checked == 0
        ):
            return FastPlanningFailure.CAPTURE_GEOMETRY
        priority = (
            FastPlanningFailure.CAPTURE_GEOMETRY,
            FastPlanningFailure.SEA_CLEARANCE,
            FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL,
            FastPlanningFailure.DYNAMIC_LIMIT_HORIZONTAL,
            FastPlanningFailure.MINCO_CONSTRUCTION_FAIL,
        )
        for failure in priority:
            if failure in self._candidate_failures:
                return failure
        return FastPlanningFailure.MINCO_CONSTRUCTION_FAIL

    def plan(self, *args, **kwargs):
        """Return the first feasible candidate within the hard deadline."""
        preferred_duration = kwargs.pop('preferred_duration', None)
        minimum_duration_override = kwargs.pop(
            'minimum_duration_override',
            None,
        )
        maximum_duration_override = kwargs.pop(
            'maximum_duration_override',
            None,
        )
        normal_minimum_duration = self.minimum_duration
        normal_maximum_duration = self.maximum_duration
        normal_absolute_maximum_duration = self.absolute_maximum_duration
        if minimum_duration_override is not None:
            minimum_duration_override = self._finite_positive(
                minimum_duration_override,
                'minimum duration override',
            )
            if minimum_duration_override > self.maximum_duration:
                raise ValueError(
                    'minimum duration override exceeds maximum duration'
                )
            self.minimum_duration = minimum_duration_override
        if maximum_duration_override is not None:
            self.maximum_duration = self._finite_positive(
                maximum_duration_override,
                'maximum duration override',
            )
            self.absolute_maximum_duration = self.maximum_duration
        self._preferred_duration = (
            None if preferred_duration is None else float(preferred_duration)
        )
        start = self.clock()
        self._active_deadline = start + self.deadline_seconds
        self._deadline_exceeded = False
        self._candidate_failures = []
        self._candidate_diagnostics = []
        try:
            plan = super().plan(*args, **kwargs)
            if self.clock() >= self._active_deadline:
                self._deadline_exceeded = True
                plan = None
        finally:
            self._active_deadline = None
            self._preferred_duration = None
            self.minimum_duration = normal_minimum_duration
            self.maximum_duration = normal_maximum_duration
            self.absolute_maximum_duration = normal_absolute_maximum_duration

        self.last_diagnostics = replace(
            self.last_diagnostics,
            candidate_diagnostics=tuple(self._candidate_diagnostics),
        )

        if plan is None:
            return FastPlanningOutcome(
                plan=None,
                failure=self._mapped_failure(),
                diagnostics=self.last_diagnostics,
            )
        self.last_success_duration = plan.duration
        plan = replace(plan, planner_type='MINCO_T3_FAST')
        return FastPlanningOutcome(
            plan=plan,
            failure=FastPlanningFailure.NONE,
            diagnostics=self.last_diagnostics,
        )
