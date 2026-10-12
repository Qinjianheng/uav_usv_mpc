"""Transport-neutral input handling for the independent MINCO planner."""

import math
import threading
from dataclasses import dataclass

from uav_control.common.sea_safety import apply_sea_safety_guard

from .fast_minco_planner import FastPlanningFailure


@dataclass(frozen=True)
class PredictionSample:
    """One timestamp-relative state in a target prediction."""

    relative_time: float
    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class PredictionSeries:
    """One immutable prediction snapshot."""

    mission_id: int
    sequence_id: int
    source_stamp: float
    valid_until: float
    samples: tuple
    source: str
    observation_stamp: float = None

    def __post_init__(self):
        if self.observation_stamp is None:
            object.__setattr__(self, 'observation_stamp', self.source_stamp)

    @property
    def end_stamp(self):
        """Return the absolute end time covered by prediction samples."""
        if not self.samples:
            return float(self.source_stamp)
        return float(self.source_stamp) + max(
            float(sample.relative_time)
            for sample in self.samples
        )

    @staticmethod
    def _blend(first, second, fraction):
        return tuple(
            start + fraction * (end - start)
            for start, end in zip(first, second)
        )

    def state_at(self, relative_time):
        """Linearly interpolate one state without extrapolating the horizon."""
        if not self.samples:
            raise ValueError('prediction series must contain samples')
        relative_time = float(relative_time)
        if not math.isfinite(relative_time) or relative_time < 0.0:
            raise ValueError('prediction time must be finite and non-negative')
        if relative_time <= self.samples[0].relative_time:
            sample = self.samples[0]
            return sample.position, sample.velocity, sample.acceleration
        for previous, current in zip(self.samples, self.samples[1:]):
            if relative_time <= current.relative_time:
                interval = current.relative_time - previous.relative_time
                fraction = (
                    (relative_time - previous.relative_time) / interval
                    if interval > 1e-12 else 0.0
                )
                return (
                    self._blend(
                        previous.position,
                        current.position,
                        fraction,
                    ),
                    self._blend(
                        previous.velocity,
                        current.velocity,
                        fraction,
                    ),
                    self._blend(
                        previous.acceleration,
                        current.acceleration,
                        fraction,
                    ),
                )
        raise ValueError('prediction time exceeds available horizon')

    def state_at_absolute_time(self, absolute_time):
        """Return the state at an absolute ROS time in this snapshot."""
        relative_time = float(absolute_time) - self.source_stamp
        return self.state_at(relative_time)


@dataclass(frozen=True)
class UavKinematicState:
    """One UAV state in the ROS clock domain."""

    stamp: float
    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class PlannerRequest:
    """Synchronized latest snapshots for one planning event."""

    mission_id: int
    prediction: PredictionSeries
    uav: UavKinematicState
    trajectory_start_stamp: float
    contact_stamp: float = None
    terminal_mode: bool = False
    minimum_duration: float = None
    planning_cycle_id: int = 0
    uav_source_stamp: float = None

    @property
    def source_stamp(self):
        """Return the oldest input stamp without hiding staleness."""
        state_stamp = (
            self.uav.stamp
            if self.uav_source_stamp is None
            else float(self.uav_source_stamp)
        )
        return min(self.observation_stamp, state_stamp)

    @property
    def observation_stamp(self):
        """Return immutable acquisition time of this request's prediction."""
        return self.prediction.observation_stamp

    @property
    def state_source_stamp(self):
        """Return measurement time even when the boundary is future-dated."""
        return (
            self.uav.stamp
            if self.uav_source_stamp is None
            else float(self.uav_source_stamp)
        )


class LatestRequestSlot:
    """A thread-safe single pending request with replacement accounting."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = None
        self.replaced_request_count = 0

    def submit(self, request):
        """Atomically replace any pending request."""
        with self._lock:
            if self._pending is not None:
                self.replaced_request_count += 1
            self._pending = request

    def take(self):
        """Atomically take the latest request, leaving the slot empty."""
        with self._lock:
            request = self._pending
            self._pending = None
            return request


@dataclass(frozen=True)
class PlanningDecision:
    """One cadence decision for normal or committed terminal planning."""

    submit: bool
    terminal_mode: bool
    minimum_duration: float


@dataclass(frozen=True)
class TerminalAdmissionDecision:
    """Explain whether one fully validated plan may start final approach."""

    admitted: bool
    reason: str


@dataclass(frozen=True)
class PlannedCaptureWindow:
    """Planned first capture entry and remaining executable trajectory time."""

    first_entry_t_go: float
    trajectory_end_t_go: float
    execution_margin: float


def evaluate_terminal_admission(
    horizontal_min_time,
    vertical_min_time,
    selected_t_go,
    planned_capture_margin,
    current_z,
    current_vz,
    planned_initial_vz,
    sea_surface_z,
    reserve_clearance,
    response_delay,
    braking_acceleration,
    maximum_vertical_speed,
    time_sync_tolerance,
    preparation_position_error=0.0,
    preparation_velocity_error=0.0,
    preparation_position_tolerance=math.inf,
    preparation_velocity_tolerance=math.inf,
    capture_execution_margin=math.inf,
):
    """Gate the first descent on synchronized reachability and sea margin."""
    values = (
        horizontal_min_time,
        vertical_min_time,
        selected_t_go,
        planned_capture_margin,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return TerminalAdmissionDecision(False, 'REACHABILITY_UNKNOWN')
    if planned_capture_margin < 0.0:
        return TerminalAdmissionDecision(False, 'CAPTURE_MARGIN_INSUFFICIENT')
    if (
        horizontal_min_time > selected_t_go + 1e-9
        or vertical_min_time > selected_t_go + 1e-9
    ):
        return TerminalAdmissionDecision(False, 'CONTACT_TIME_UNREACHABLE')
    if horizontal_min_time > vertical_min_time + time_sync_tolerance:
        return TerminalAdmissionDecision(False, 'HORIZONTAL_NOT_READY')
    if (
        float(preparation_position_error)
        > float(preparation_position_tolerance)
        or float(preparation_velocity_error)
        > float(preparation_velocity_tolerance)
    ):
        return TerminalAdmissionDecision(
            False,
            'HORIZONTAL_PREPARATION_NOT_READY',
        )
    if float(capture_execution_margin) + 1e-9 < float(response_delay):
        return TerminalAdmissionDecision(False, 'CAPTURE_WINDOW_INSUFFICIENT')
    safety = apply_sea_safety_guard(
        current_z=current_z,
        current_vz=current_vz,
        proposed_vz=planned_initial_vz,
        sea_surface_z=sea_surface_z,
        reserve_clearance=reserve_clearance,
        response_delay=response_delay,
        effective_braking_acceleration=braking_acceleration,
        control_dt=0.05,
        maximum_vertical_speed=maximum_vertical_speed,
    )
    if safety.unrecoverable or safety.response_margin <= 0.0:
        return TerminalAdmissionDecision(False, 'SEA_MARGIN_INSUFFICIENT')
    return TerminalAdmissionDecision(True, 'ADMITTED')


def planned_capture_window(
    plan,
    prediction,
    trajectory_start_stamp,
    capture_radius,
    sample_step=0.02,
):
    """Find first predicted capture entry without extending trajectory life."""
    duration = float(plan.duration)
    radius = float(capture_radius)
    step = max(float(sample_step), 1e-3)
    if duration <= 0.0 or radius <= 0.0:
        raise ValueError('capture window inputs must be positive')

    def distance_at(relative_time):
        planned = plan.sample(relative_time).position
        target = prediction.state_at_absolute_time(
            float(trajectory_start_stamp) + float(relative_time)
        )[0]
        return math.sqrt(sum(
            (float(left) - float(right)) ** 2
            for left, right in zip(planned, target)
        ))

    sample_count = max(int(math.ceil(duration / step)), 1)
    previous_time = 0.0
    previous_distance = distance_at(previous_time)
    first_entry = 0.0 if previous_distance <= radius else None
    for index in range(1, sample_count + 1):
        current_time = duration * index / sample_count
        current_distance = distance_at(current_time)
        if first_entry is None and current_distance <= radius:
            left = previous_time
            right = current_time
            for _ in range(24):
                middle = 0.5 * (left + right)
                if distance_at(middle) <= radius:
                    right = middle
                else:
                    left = middle
            first_entry = right
            break
        previous_time = current_time
        previous_distance = current_distance

    if first_entry is None:
        first_entry = math.inf
        margin = -math.inf
    else:
        margin = duration - first_entry
    return PlannedCaptureWindow(
        first_entry_t_go=first_entry,
        trajectory_end_t_go=duration,
        execution_margin=margin,
    )


def approach_preparation_errors(
    uav_position,
    uav_velocity,
    target_position,
    target_velocity,
    vertical_time,
    standoff_speed,
):
    """Measure error to the moving, reachability-sized preparation state."""
    target_speed = math.hypot(target_velocity[0], target_velocity[1])
    if target_speed > 1e-6:
        direction = (
            target_velocity[0] / target_speed,
            target_velocity[1] / target_speed,
        )
    else:
        delta = (
            target_position[0] - uav_position[0],
            target_position[1] - uav_position[1],
        )
        distance = math.hypot(*delta)
        direction = (
            (delta[0] / distance, delta[1] / distance)
            if distance > 1e-6 else (1.0, 0.0)
        )
    standoff = max(float(vertical_time), 0.0) * max(
        float(standoff_speed), 0.0
    )
    preparation = (
        target_position[0] - standoff * direction[0],
        target_position[1] - standoff * direction[1],
    )
    position_error = math.hypot(
        uav_position[0] - preparation[0],
        uav_position[1] - preparation[1],
    )
    velocity_error = math.hypot(
        uav_velocity[0] - target_velocity[0],
        uav_velocity[1] - target_velocity[1],
    )
    return position_error, velocity_error


def select_planning_start_state(
    measured_state,
    active_trajectory,
    expected_handover_stamp,
):
    """Use the old trajectory reference at handover, else measured state."""
    stamp = float(expected_handover_stamp)
    if (
        active_trajectory is None
        or stamp >= float(active_trajectory.valid_until)
    ):
        return measured_state
    try:
        reference = active_trajectory.sample_at_ros_time(stamp)
    except (TypeError, ValueError):
        return measured_state
    return UavKinematicState(
        stamp=stamp,
        position=tuple(reference.position),
        velocity=tuple(reference.velocity),
        acceleration=tuple(reference.acceleration),
    )


def terminal_mode_for_plan(
    mission_state,
    request_terminal_mode=False,
    remaining_t_go=None,
    terminal_time_threshold=1.0,
    terminal_state=7,
):
    """Return the shared terminal-state decision used by planner outputs."""
    return bool(
        request_terminal_mode
        or int(mission_state) == int(terminal_state)
        or (
            remaining_t_go is not None
            and float(remaining_t_go)
            <= float(terminal_time_threshold) + 1e-9
        )
    )


class PlanningRequestPolicy:
    """Select normal/terminal planning without changing motion limits."""

    def __init__(
        self,
        normal_minimum_duration=1.0,
        terminal_minimum_duration=0.30,
        terminal_freeze_time=0.30,
        terminal_time_threshold=1.0,
        terminal_state=7,
    ):
        self.normal_minimum_duration = float(normal_minimum_duration)
        self.terminal_minimum_duration = float(terminal_minimum_duration)
        self.terminal_freeze_time = float(terminal_freeze_time)
        self.terminal_time_threshold = float(terminal_time_threshold)
        self.terminal_state = int(terminal_state)

    def decide(self, mission_state, remaining_t_go=None):
        terminal = terminal_mode_for_plan(
            mission_state=mission_state,
            remaining_t_go=remaining_t_go,
            terminal_time_threshold=self.terminal_time_threshold,
            terminal_state=self.terminal_state,
        )
        if terminal:
            submit = (
                remaining_t_go is not None
                and float(remaining_t_go)
                > self.terminal_freeze_time + 1e-9
            )
            return PlanningDecision(
                submit=submit,
                terminal_mode=True,
                minimum_duration=self.terminal_minimum_duration,
            )
        return PlanningDecision(
            submit=True,
            terminal_mode=False,
            minimum_duration=self.normal_minimum_duration,
        )


class ContactTimeSchedule:
    """Keep committed and tracker-pending absolute contact times separate."""

    def __init__(
        self,
        terminal_threshold=1.0,
        freeze_time=0.30,
    ):
        self.terminal_threshold = max(float(terminal_threshold), 0.0)
        self.freeze_time = max(float(freeze_time), 0.0)
        self.contact_stamp = None
        self.committed_plan_id = 0
        self.pending_plan_id = 0
        self.pending_contact_stamp = None
        self.pending_planning_cycle_id = 0
        self.pending_since = None

    def reset(self):
        self.contact_stamp = None
        self.committed_plan_id = 0
        self._clear_pending()

    def _clear_pending(self):
        self.pending_plan_id = 0
        self.pending_contact_stamp = None
        self.pending_planning_cycle_id = 0
        self.pending_since = None

    def cancel_pending(self):
        """Discard the pending proposal while preserving committed contact."""
        had_pending = self.has_pending
        self._clear_pending()
        return had_pending

    @property
    def has_pending(self):
        return self.pending_plan_id > 0

    def propose(
        self,
        plan_id,
        contact_stamp,
        planning_cycle_id,
        proposed_at,
    ):
        """Replace the pending proposal without committing its contact time."""
        plan_id = int(plan_id)
        contact_stamp = float(contact_stamp)
        proposed_at = float(proposed_at)
        if (
            plan_id <= 0
            or not math.isfinite(contact_stamp)
            or not math.isfinite(proposed_at)
        ):
            return False
        self.pending_plan_id = plan_id
        self.pending_contact_stamp = contact_stamp
        self.pending_planning_cycle_id = int(planning_cycle_id)
        self.pending_since = proposed_at
        return True

    def confirm(self, plan_id, planning_cycle_id):
        """Commit only the exact proposal accepted by the tracker."""
        if (
            int(plan_id) != self.pending_plan_id
            or int(planning_cycle_id) != self.pending_planning_cycle_id
        ):
            return False
        self.contact_stamp = self.pending_contact_stamp
        self.committed_plan_id = self.pending_plan_id
        self._clear_pending()
        return True

    def reject(self, plan_id, planning_cycle_id):
        """Drop only the exact rejected proposal and keep the old contact."""
        if (
            int(plan_id) != self.pending_plan_id
            or int(planning_cycle_id) != self.pending_planning_cycle_id
        ):
            return False
        self._clear_pending()
        return True

    def expire(self, now, timeout):
        """Expire an unacknowledged proposal without changing the contact."""
        if not self.has_pending or self.pending_since is None:
            return False
        if float(now) - self.pending_since <= float(timeout):
            return False
        self._clear_pending()
        return True

    def accept_plan(self, source_stamp, selected_t_go, rescheduled=False):
        """Directly seed a committed contact for compatibility and startup."""
        candidate = float(source_stamp) + float(selected_t_go)
        if self.contact_stamp is None:
            self.contact_stamp = candidate
        elif (
            bool(rescheduled)
            and candidate > self.contact_stamp + 1e-9
        ):
            self.contact_stamp = candidate
        return self.contact_stamp

    def remaining_t_go(self, source_stamp):
        if self.contact_stamp is None:
            return None
        return self.contact_stamp - float(source_stamp)

    def preferred_t_go(self, source_stamp):
        remaining = self.remaining_t_go(source_stamp)
        if remaining is None or remaining <= 0.0:
            return None
        return remaining

    def is_terminal(self, stamp):
        remaining = self.remaining_t_go(stamp)
        return (
            remaining is not None
            and remaining <= self.terminal_threshold + 1e-9
        )

    def should_replan(self, source_stamp):
        """Allow terminal refreshes only before the final freeze window."""
        remaining = self.remaining_t_go(source_stamp)
        return remaining is None or remaining > self.freeze_time + 1e-9


def validate_input(request, now, maximum_age):
    """Classify stale planner inputs before any MINCO computation."""
    now = float(now)
    maximum_age = float(maximum_age)
    observation_stamp = float(request.observation_stamp)
    prediction_age = now - observation_stamp
    if (
        not math.isfinite(observation_stamp)
        or observation_stamp <= 0.0
        or not math.isfinite(now)
        or not math.isfinite(request.prediction.source_stamp)
        or request.prediction.source_stamp < observation_stamp
        or request.prediction.source_stamp > now
    ):
        return FastPlanningFailure.PREDICTION_STALE
    state_age = now - request.state_source_stamp
    if prediction_age < 0.0 or prediction_age > maximum_age:
        return FastPlanningFailure.PREDICTION_STALE
    if (
        not math.isfinite(state_age)
        or state_age < 0.0
        or state_age > maximum_age
    ):
        return FastPlanningFailure.STATE_STALE
    if request.prediction.mission_id != request.mission_id:
        return FastPlanningFailure.PREDICTION_STALE
    return FastPlanningFailure.NONE


def validate_plan_arrival(request, generated_stamp, maximum_age):
    """Reject a result whose oldest source was already stale on arrival."""
    if validate_input(request, generated_stamp, maximum_age) != (
        FastPlanningFailure.NONE
    ):
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    return FastPlanningFailure.NONE


def validate_target_shift(
    request,
    latest_prediction,
    intercept_time,
    tolerance,
    contact_stamp=None,
):
    """Reject a plan when a newer prediction moved its contact endpoint."""
    if latest_prediction is None:
        return FastPlanningFailure.PREDICTION_STALE
    if latest_prediction.mission_id != request.mission_id:
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    if contact_stamp is None:
        contact_stamp = (
            request.trajectory_start_stamp + float(intercept_time)
        )
    else:
        contact_stamp = float(contact_stamp)
    try:
        planned_position = request.prediction.state_at_absolute_time(
            contact_stamp
        )[0]
        latest_position = latest_prediction.state_at_absolute_time(
            contact_stamp
        )[0]
    except (TypeError, ValueError):
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    shift = math.sqrt(sum(
        (latest - planned) ** 2
        for latest, planned in zip(latest_position, planned_position)
    ))
    if shift > float(tolerance):
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    return FastPlanningFailure.NONE


def target_shift_distance(
    request,
    latest_prediction,
    intercept_time,
    contact_stamp=None,
):
    """Measure endpoint movement at one shared absolute contact time."""
    if latest_prediction is None:
        raise ValueError('latest prediction is unavailable')
    if latest_prediction.mission_id != request.mission_id:
        raise ValueError('latest prediction mission does not match request')
    if contact_stamp is None:
        contact_stamp = (
            request.trajectory_start_stamp + float(intercept_time)
        )
    planned_position = request.prediction.state_at_absolute_time(
        float(contact_stamp)
    )[0]
    latest_position = latest_prediction.state_at_absolute_time(
        float(contact_stamp)
    )[0]
    return math.sqrt(sum(
        (latest - planned) ** 2
        for latest, planned in zip(latest_position, planned_position)
    ))


def validate_total_deadline(elapsed, hard_deadline):
    """Apply the process-level budget including non-solver overhead."""
    if float(elapsed) >= float(hard_deadline):
        return FastPlanningFailure.DEADLINE_EXCEEDED
    return FastPlanningFailure.NONE
