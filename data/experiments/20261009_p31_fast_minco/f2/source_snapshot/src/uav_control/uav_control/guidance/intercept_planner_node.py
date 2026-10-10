"""Independent latest-input-only Fast MINCO planning node."""

import json
import math
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from uav_usv_interfaces.msg import ControllerDiagnostic, InterceptTrajectory
from uav_usv_interfaces.msg import MissionState
from uav_usv_interfaces.msg import PlannerDiagnostic, PolynomialSegment
from uav_usv_interfaces.msg import TargetPrediction

from uav_control.tracking.target_predictor_node import seconds_to_duration
from uav_control.tracking.target_predictor_node import seconds_to_time

from .fast_minco_planner import FastMincoPlanner, FastPlanningFailure
from .fast_minco_planner import FastPlanningOutcome
from .finite_horizon_intercept_planner import PlannerDiagnostics
from .planner_pipeline import LatestRequestSlot, PlannerRequest
from .planner_pipeline import ContactTimeSchedule
from .planner_pipeline import PlanningRequestPolicy
from .planner_pipeline import PredictionSample, PredictionSeries
from .planner_pipeline import terminal_mode_for_plan
from .planner_pipeline import evaluate_terminal_admission
from .planner_pipeline import approach_preparation_errors
from .planner_pipeline import planned_capture_window
from .planner_pipeline import select_planning_start_state
from .planner_pipeline import target_shift_distance
from .planner_pipeline import UavKinematicState, validate_input
from .planner_pipeline import validate_plan_arrival
from .planner_pipeline import validate_total_deadline


@dataclass(frozen=True)
class PlannerJobResult:
    """One completed worker event with ROS and monotonic timing."""

    request: PlannerRequest
    outcome: FastPlanningOutcome
    planning_started_stamp: float
    generated_stamp: float
    compute_time: float
    locked_contact_unreachable: bool = False
    rejection_stage: str = ''
    rejection_detail: str = ''


@dataclass(frozen=True)
class ActivePlanReference:
    """Planner-owned copy of a trajectory confirmed active by the tracker."""

    plan: object
    source_stamp: float

    @property
    def valid_until(self):
        return self.source_stamp + float(self.plan.duration)

    def sample_at_ros_time(self, stamp):
        return self.plan.sample(float(stamp) - self.source_stamp)


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def prediction_from_message(message):
    """Keep sample epoch and require the original image acquisition time."""
    source_stamp = _stamp_seconds(message.source_stamp)
    observation_stamp = _stamp_seconds(message.observation_stamp)
    if (
        not math.isfinite(source_stamp)
        or not math.isfinite(observation_stamp)
        or observation_stamp <= 0.0
        or source_stamp < observation_stamp
        or message.source != 'tracking'
    ):
        raise ValueError('prediction lacks valid tracking acquisition time')
    samples = tuple(PredictionSample(
        relative_time=_stamp_seconds(sample.relative_time),
        position=(
            float(sample.position.x),
            float(sample.position.y),
            float(sample.position.z),
        ),
        velocity=(
            float(sample.velocity.x),
            float(sample.velocity.y),
            float(sample.velocity.z),
        ),
        acceleration=(
            float(sample.acceleration.x),
            float(sample.acceleration.y),
            float(sample.acceleration.z),
        ),
    ) for sample in message.samples)
    previous_time = -1.0
    for sample in samples:
        if (
            sample.relative_time < 0.0
            or sample.relative_time <= previous_time
            or not all(math.isfinite(value) for value in (
                sample.relative_time,
                *sample.position, *sample.velocity, *sample.acceleration,
            ))
        ):
            raise ValueError('prediction contains invalid samples')
        previous_time = sample.relative_time
    return PredictionSeries(
        mission_id=int(message.mission_id),
        sequence_id=int(message.sequence_id),
        source_stamp=source_stamp,
        observation_stamp=observation_stamp,
        valid_until=_stamp_seconds(message.valid_until),
        samples=samples,
        source=str(message.source),
    )


def uav_state_from_message(message, received_stamp):
    """Convert PX4 state using ROS receipt time as the comparable timestamp."""
    values = []
    for name in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az'):
        value = float(getattr(message, name, math.nan))
        values.append(value if math.isfinite(value) else 0.0)
    return UavKinematicState(
        stamp=float(received_stamp),
        position=tuple(values[:3]),
        velocity=tuple(values[3:6]),
        acceleration=tuple(values[6:9]),
    )


def uav_state_from_navigation(message, now):
    """Preserve physical UAV sample time separately from future boundaries."""
    from ..common.navigation_state import navigation_components
    stamp, position, velocity, acceleration = navigation_components(message, now)
    return UavKinematicState(stamp, position, velocity, acceleration)


def plan_to_message(
    plan,
    mission_id,
    plan_id,
    prediction_sequence_id,
    trajectory_start_stamp,
    planning_started_stamp,
    generated_stamp,
    target_state_source,
    frame_id='local_ned',
    contact_stamp=None,
    remaining_t_go=None,
    terminal_mode=False,
    planned_capture_margin=0.0,
    capture_entry_stamp=None,
    capture_execution_margin=0.0,
    observation_stamp=None,
):
    """Serialize a MINCO polynomial using its true execution start time."""
    message = InterceptTrajectory()
    message.mission_id = int(mission_id)
    message.plan_id = int(plan_id)
    message.prediction_sequence_id = int(prediction_sequence_id)
    message.source_stamp = seconds_to_time(trajectory_start_stamp)
    message.observation_stamp = seconds_to_time(
        0.0 if observation_stamp is None else observation_stamp
    )
    message.planning_started_stamp = seconds_to_time(
        planning_started_stamp
    )
    message.generated_stamp = seconds_to_time(generated_stamp)
    message.valid_until = seconds_to_time(
        trajectory_start_stamp + plan.duration
    )
    if contact_stamp is None:
        contact_stamp = trajectory_start_stamp + plan.duration
    if remaining_t_go is None:
        remaining_t_go = plan.duration
    message.contact_stamp = seconds_to_time(contact_stamp)
    if capture_entry_stamp is None:
        capture_entry_stamp = contact_stamp
    message.capture_entry_stamp = seconds_to_time(capture_entry_stamp)
    message.target_state_source = str(target_state_source)
    message.frame_id = str(frame_id)
    message.planner_type = str(plan.planner_type)
    message.t_go = float(plan.duration)
    message.trajectory_duration = float(plan.duration)
    message.selected_t_go = float(plan.duration)
    message.remaining_t_go = max(float(remaining_t_go), 0.0)
    message.terminal_mode = bool(terminal_mode)
    message.planned_capture_margin = float(planned_capture_margin)
    message.capture_execution_margin = float(capture_execution_margin)

    segments = []
    coefficients = plan.minco_trajectory.coefficients
    for duration, piece_coefficients in zip(
        plan.piece_durations,
        coefficients,
    ):
        segment = PolynomialSegment()
        segment.duration = seconds_to_duration(duration)
        segment.coefficients = [
            float(piece_coefficients[power][axis])
            for axis in range(3)
            for power in range(6)
        ]
        segments.append(segment)
    message.segments = segments
    message.piece_count = len(segments)
    terminal = plan.sample(plan.duration)
    message.terminal_position.x = terminal.position[0]
    message.terminal_position.y = terminal.position[1]
    message.terminal_position.z = terminal.position[2]
    message.terminal_velocity.x = terminal.velocity[0]
    message.terminal_velocity.y = terminal.velocity[1]
    message.terminal_velocity.z = terminal.velocity[2]
    message.planned_closing_speed = float(plan.closing_speed)
    message.planned_max_horizontal_speed = float(
        plan.maximum_horizontal_speed
    )
    message.planned_max_vertical_speed = float(plan.maximum_vertical_speed)
    message.planned_max_horizontal_acceleration = float(
        plan.maximum_horizontal_acceleration
    )
    message.planned_max_vertical_acceleration = float(
        plan.maximum_vertical_acceleration
    )
    message.valid = True
    message.invalid_reason = ''
    return message


FAILURE_CONSTANTS = {
    FastPlanningFailure.NONE: PlannerDiagnostic.NONE,
    FastPlanningFailure.STATE_STALE: PlannerDiagnostic.STATE_STALE,
    FastPlanningFailure.PREDICTION_STALE: (
        PlannerDiagnostic.PREDICTION_STALE
    ),
    FastPlanningFailure.HORIZON_INSUFFICIENT: (
        PlannerDiagnostic.HORIZON_INSUFFICIENT
    ),
    FastPlanningFailure.CAPTURE_GEOMETRY: (
        PlannerDiagnostic.CAPTURE_GEOMETRY
    ),
    FastPlanningFailure.DYNAMIC_LIMIT_HORIZONTAL: (
        PlannerDiagnostic.DYNAMIC_LIMIT_HORIZONTAL
    ),
    FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL: (
        PlannerDiagnostic.DYNAMIC_LIMIT_VERTICAL
    ),
    FastPlanningFailure.SEA_CLEARANCE: PlannerDiagnostic.SEA_CLEARANCE,
    FastPlanningFailure.MINCO_CONSTRUCTION_FAIL: (
        PlannerDiagnostic.MINCO_CONSTRUCTION_FAIL
    ),
    FastPlanningFailure.OPTIMIZATION_FAIL: (
        PlannerDiagnostic.OPTIMIZATION_FAIL
    ),
    FastPlanningFailure.DEADLINE_EXCEEDED: (
        PlannerDiagnostic.DEADLINE_EXCEEDED
    ),
    FastPlanningFailure.PLAN_STALE_ON_ARRIVAL: (
        PlannerDiagnostic.PLAN_STALE_ON_ARRIVAL
    ),
}


class InterceptPlannerNode(Node):
    """Run Fast MINCO in its own process and retain only the latest request."""

    PLANNING_STATES = {
        MissionState.FAR_GUIDANCE,
        MissionState.MINCO_READY,
        MissionState.MINCO_TRACKING,
        MissionState.TERMINAL_MINCO,
    }
    RECOVERY_STATES = {
        MissionState.PLAN_RECOVERY,
        MissionState.SAFE_WAIT,
    } | {
        getattr(MissionState, name)
        for name in (
            'TARGET_ACQUIRE', 'TARGET_LOCK', 'REACQUIRE', 'SAFE_RECOVERY',
        )
        if hasattr(MissionState, name)
    }

    def __init__(self):
        super().__init__('intercept_planner_node')
        self.declare_parameter('planning_rate_hz', 5.0)
        self.declare_parameter('terminal_planning_rate_hz', 10.0)
        self.declare_parameter('completion_poll_rate_hz', 100.0)
        self.declare_parameter('minimum_duration', 1.0)
        self.declare_parameter('terminal_minimum_duration', 0.30)
        self.declare_parameter('terminal_freeze_time', 0.30)
        self.declare_parameter('maximum_duration', 1.5)
        self.declare_parameter('duration_margin', 0.35)
        self.declare_parameter('sample_step', 0.05)
        self.declare_parameter('maximum_horizontal_speed', 7.0)
        self.declare_parameter('maximum_vertical_speed', 4.0)
        self.declare_parameter('maximum_horizontal_acceleration', 3.0)
        self.declare_parameter('maximum_vertical_acceleration', 3.0)
        self.declare_parameter('preferred_closing_speed', 1.5)
        self.declare_parameter('conservative_closing_speed', 1.2)
        self.declare_parameter('planned_capture_radius', 0.35)
        self.declare_parameter('terminal_time_threshold', 1.0)
        self.declare_parameter('sea_surface_z', 0.0)
        self.declare_parameter('contact_clearance', 0.05)
        self.declare_parameter('preferred_clearance', 0.1)
        self.declare_parameter('piece_count', 3)
        self.declare_parameter('target_curve_weight', 0.7)
        self.declare_parameter('solver_deadline_seconds', 0.075)
        self.declare_parameter('deadline_seconds', 0.08)
        self.declare_parameter('maximum_input_age', 0.125)
        self.declare_parameter('endpoint_tolerance', 0.5)
        self.declare_parameter('approach_time_sync_tolerance', 0.35)
        self.declare_parameter('approach_reserve_clearance', 0.20)
        self.declare_parameter(
            'approach_preparation_standoff_speed',
            1.5,
        )
        self.declare_parameter(
            'approach_preparation_position_tolerance',
            0.75,
        )
        self.declare_parameter(
            'approach_preparation_velocity_tolerance',
            0.75,
        )
        self.declare_parameter('response_delay', 0.15)
        self.declare_parameter(
            'effective_vertical_braking_acceleration',
            2.5,
        )
        self.declare_parameter('quadrature_intervals_per_piece', 6)
        self.declare_parameter('frame_id', 'local_ned')

        self.maximum_input_age = float(
            self.get_parameter('maximum_input_age').value
        )
        self.endpoint_tolerance = float(
            self.get_parameter('endpoint_tolerance').value
        )
        self.approach_time_sync_tolerance = float(
            self.get_parameter('approach_time_sync_tolerance').value
        )
        self.approach_reserve_clearance = float(
            self.get_parameter('approach_reserve_clearance').value
        )
        self.approach_preparation_standoff_speed = float(
            self.get_parameter(
                'approach_preparation_standoff_speed'
            ).value
        )
        self.approach_preparation_position_tolerance = float(
            self.get_parameter(
                'approach_preparation_position_tolerance'
            ).value
        )
        self.approach_preparation_velocity_tolerance = float(
            self.get_parameter(
                'approach_preparation_velocity_tolerance'
            ).value
        )
        self.hard_deadline_seconds = float(
            self.get_parameter('deadline_seconds').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.planned_capture_radius = float(
            self.get_parameter('planned_capture_radius').value
        )
        self.terminal_time_threshold = float(
            self.get_parameter('terminal_time_threshold').value
        )
        self.terminal_minimum_duration = float(
            self.get_parameter('terminal_minimum_duration').value
        )
        self.terminal_freeze_time = float(
            self.get_parameter('terminal_freeze_time').value
        )
        self.planner = FastMincoPlanner(
            minimum_duration=self.get_parameter('minimum_duration').value,
            maximum_duration=self.get_parameter('maximum_duration').value,
            duration_margin=self.get_parameter('duration_margin').value,
            sample_step=self.get_parameter('sample_step').value,
            maximum_horizontal_speed=self.get_parameter(
                'maximum_horizontal_speed'
            ).value,
            maximum_vertical_speed=self.get_parameter(
                'maximum_vertical_speed'
            ).value,
            maximum_horizontal_acceleration=self.get_parameter(
                'maximum_horizontal_acceleration'
            ).value,
            maximum_vertical_acceleration=self.get_parameter(
                'maximum_vertical_acceleration'
            ).value,
            preferred_closing_speed=self.get_parameter(
                'preferred_closing_speed'
            ).value,
            conservative_closing_speed=self.get_parameter(
                'conservative_closing_speed'
            ).value,
            capture_radius=self.planned_capture_radius,
            sea_surface_z=self.get_parameter('sea_surface_z').value,
            contact_clearance=self.get_parameter(
                'contact_clearance'
            ).value,
            preferred_clearance=self.get_parameter(
                'preferred_clearance'
            ).value,
            piece_count=self.get_parameter('piece_count').value,
            target_curve_weight=self.get_parameter(
                'target_curve_weight'
            ).value,
            deadline_seconds=self.get_parameter(
                'solver_deadline_seconds'
            ).value,
            response_delay=self.get_parameter('response_delay').value,
            effective_vertical_braking_acceleration=self.get_parameter(
                'effective_vertical_braking_acceleration'
            ).value,
            quadrature_intervals_per_piece=self.get_parameter(
                'quadrature_intervals_per_piece'
            ).value,
        )
        planning_rate_hz = float(
            self.get_parameter('planning_rate_hz').value
        )
        if not math.isfinite(planning_rate_hz) or planning_rate_hz <= 0.0:
            raise ValueError('planning_rate_hz must be finite and positive')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.prediction_sub = self.create_subscription(
            TargetPrediction,
            '/planning/target_prediction',
            self.prediction_callback,
            px4_qos,
        )
        from uav_usv_interfaces.msg import UavState
        self.uav_sub = self.create_subscription(
            UavState,
            '/navigation/uav_state',
            self.uav_callback,
            px4_qos,
        )
        self.mission_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_callback,
            output_qos,
        )
        self.controller_diagnostic_sub = self.create_subscription(
            ControllerDiagnostic,
            '/control/diagnostic',
            self.controller_diagnostic_callback,
            output_qos,
        )
        self.trajectory_pub = self.create_publisher(
            InterceptTrajectory,
            '/planning/intercept_trajectory',
            output_qos,
        )
        self.diagnostic_pub = self.create_publisher(
            PlannerDiagnostic,
            '/planning/diagnostic',
            output_qos,
        )
        self.planning_timer = self.create_timer(
            1.0 / planning_rate_hz,
            self.planning_timer_callback,
        )
        terminal_planning_rate_hz = float(
            self.get_parameter('terminal_planning_rate_hz').value
        )
        if (
            not math.isfinite(terminal_planning_rate_hz)
            or terminal_planning_rate_hz <= 0.0
        ):
            raise ValueError(
                'terminal_planning_rate_hz must be finite and positive'
            )
        self.terminal_planning_timer = self.create_timer(
            1.0 / terminal_planning_rate_hz,
            self.terminal_planning_timer_callback,
        )
        self.planning_period = 1.0 / planning_rate_hz
        self.terminal_planning_period = 1.0 / terminal_planning_rate_hz
        self.last_worker_started_stamp = None
        completion_poll_rate_hz = float(
            self.get_parameter('completion_poll_rate_hz').value
        )
        if (
            not math.isfinite(completion_poll_rate_hz)
            or completion_poll_rate_hz <= 0.0
        ):
            raise ValueError(
                'completion_poll_rate_hz must be finite and positive'
            )
        self.completion_timer = self.create_timer(
            1.0 / completion_poll_rate_hz,
            self.completion_timer_callback,
        )
        self.completion_poll_period = 1.0 / completion_poll_rate_hz

        self.latest_prediction = None
        self.latest_uav = None
        self.mission_id = 0
        self.intercept_requested = False
        self.mission_state = MissionState.INIT
        self.planning_cycle_id = 0
        self.plan_id = 0
        self.completed_plan_count = 0
        self.completion_times = deque(maxlen=100)
        self.solver_compute_times = deque(maxlen=30)
        self.request_slot = LatestRequestSlot()
        self.worker_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='fast_minco',
        )
        self.future = None
        self.ready_job = None
        self.last_submitted_key = None
        self.active_plan_reference = None
        self.published_plan_references = {}
        self.contact_schedule = ContactTimeSchedule(
            terminal_threshold=self.terminal_time_threshold,
            freeze_time=self.terminal_freeze_time,
        )
        self.request_policy = PlanningRequestPolicy(
            normal_minimum_duration=self.planner.minimum_duration,
            terminal_minimum_duration=self.terminal_minimum_duration,
            terminal_freeze_time=self.terminal_freeze_time,
            terminal_time_threshold=self.terminal_time_threshold,
            terminal_state=MissionState.TERMINAL_MINCO,
        )
        self.get_logger().info(
            'Fast MINCO planner ready | rate='
            f'{planning_rate_hz:.1f} Hz | candidates<=6 | '
            f'solver deadline={self.planner.deadline_seconds * 1000.0:.0f} '
            f'ms | hard deadline={self.hard_deadline_seconds * 1000.0:.0f} ms'
        )

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def prediction_callback(self, message):
        if (
            not message.valid
            or not message.samples
            or message.frame_id != self.frame_id
        ):
            self.latest_prediction = None
            return
        try:
            prediction = prediction_from_message(message)
        except (AttributeError, TypeError, ValueError):
            self.latest_prediction = None
            return
        if (
            self.latest_prediction is not None
            and prediction.mission_id == self.latest_prediction.mission_id
            and (
                prediction.source_stamp < self.latest_prediction.source_stamp
                or (
                    prediction.source_stamp
                    == self.latest_prediction.source_stamp
                    and prediction.sequence_id
                    <= self.latest_prediction.sequence_id
                )
            )
        ):
            return
        now = self._ros_seconds()
        age = now - prediction.observation_stamp
        if (
            prediction.source_stamp > now
            or age < 0.0
            or age > self.maximum_input_age
            or now > prediction.valid_until
        ):
            self.latest_prediction = None
            return
        self.latest_prediction = prediction
        if self.intercept_requested:
            self._planning_tick(terminal_tick=None, prediction_triggered=True)

    def uav_callback(self, message):
        try:
            self.latest_uav = uav_state_from_navigation(
                message, self._ros_seconds(),
            )
        except (ValueError, TypeError):
            self.latest_uav = None

    def mission_callback(self, message):
        new_mission_id = int(message.mission_id)
        new_state = int(message.state)
        mission_changed = new_mission_id != self.mission_id
        old_state = self.mission_state
        if mission_changed:
            self.planning_cycle_id += 1
            self.request_slot.take()
            self.last_submitted_key = None
            self.contact_schedule.reset()
            self.active_plan_reference = None
            self.published_plan_references.clear()
        elif (
            new_state in self.RECOVERY_STATES
            and old_state not in self.RECOVERY_STATES
        ):
            self.planning_cycle_id += 1
            self.request_slot.take()
            self.last_submitted_key = None
            self.contact_schedule.cancel_pending()
            self.active_plan_reference = None
        elif (
            old_state in self.RECOVERY_STATES
            and new_state == MissionState.FAR_GUIDANCE
        ):
            self.planning_cycle_id += 1
            self.request_slot.take()
            self.last_submitted_key = None
            self.contact_schedule.reset()
            self.active_plan_reference = None
        self.mission_id = new_mission_id
        self.mission_state = new_state
        self.intercept_requested = bool(
            message.intercept_requested and not message.completed
        )

    def controller_diagnostic_callback(self, message):
        """Commit or discard only the tracker response for our pending plan."""
        if int(message.mission_id) != self.mission_id:
            return
        attempted_plan_id = int(message.attempted_plan_id)
        if message.status == 'PLAN_ACCEPTED':
            actually_replaced = bool(
                getattr(message, 'trajectory_replaced', False)
                and int(message.plan_id) == attempted_plan_id
            )
            confirmed = (
                self.contact_schedule.confirm(
                    attempted_plan_id,
                    self.planning_cycle_id,
                )
                if actually_replaced else False
            )
            if not actually_replaced:
                self.contact_schedule.reject(
                    attempted_plan_id,
                    self.planning_cycle_id,
                )
            references = getattr(self, 'published_plan_references', {})
            reference = references.pop(
                attempted_plan_id,
                None,
            )
            if confirmed and reference is not None and actually_replaced:
                self.active_plan_reference = reference
        elif message.status == 'PLAN_REJECTED':
            self.contact_schedule.reject(
                attempted_plan_id,
                self.planning_cycle_id,
            )
            getattr(self, 'published_plan_references', {}).pop(
                attempted_plan_id,
                None,
            )

    def _expected_handover_lead(self):
        if self.solver_compute_times:
            ordered = sorted(self.solver_compute_times)
            index = max(math.ceil(0.90 * len(ordered)) - 1, 0)
            compute = ordered[index]
        else:
            compute = min(self.planner.deadline_seconds, 0.05)
        return min(
            compute + self.completion_poll_period,
            self.hard_deadline_seconds,
        )

    def _current_request(self, decision):
        if self.latest_prediction is None or self.latest_uav is None:
            return None
        boundary = self.latest_uav
        # A target forecast can start after the physical UAV sample. Advance
        # only the planning boundary with P/V/A, retaining uav_source_stamp.
        delta = max(self.latest_prediction.source_stamp - boundary.stamp, 0.0)
        if delta > 0.0:
            boundary = UavKinematicState(
                boundary.stamp + delta,
                tuple(
                    p + v * delta + 0.5 * a * delta * delta
                    for p, v, a in zip(
                        boundary.position, boundary.velocity,
                        boundary.acceleration,
                    )
                ),
                tuple(
                    v + a * delta
                    for v, a in zip(boundary.velocity, boundary.acceleration)
                ),
                boundary.acceleration,
            )
        active_reference = getattr(self, 'active_plan_reference', None)
        if active_reference is not None:
            boundary = select_planning_start_state(
                measured_state=self.latest_uav,
                active_trajectory=active_reference,
                expected_handover_stamp=(
                    self._ros_seconds() + self._expected_handover_lead()
                ),
            )
        return PlannerRequest(
            mission_id=self.mission_id,
            prediction=self.latest_prediction,
            uav=boundary,
            trajectory_start_stamp=boundary.stamp,
            contact_stamp=self.contact_schedule.contact_stamp,
            terminal_mode=decision.terminal_mode,
            minimum_duration=decision.minimum_duration,
            planning_cycle_id=getattr(self, 'planning_cycle_id', 0),
            uav_source_stamp=self.latest_uav.stamp,
        )

    def _run_request(self, request):
        planning_started_stamp = self._ros_seconds()
        monotonic_start = time.perf_counter()
        rejection_stage = ''
        rejection_detail = ''
        locked_contact_unreachable = False

        failure = validate_input(
            request,
            planning_started_stamp,
            self.maximum_input_age,
        )

        if failure == FastPlanningFailure.NONE:
            minimum_duration = (
                self.planner.minimum_duration
                if request.minimum_duration is None
                else request.minimum_duration
            )

            available_prediction_duration = (
                request.prediction.end_stamp
                - request.trajectory_start_stamp
            )

            preferred_duration = None
            maximum_duration_override = min(
                self.planner.maximum_duration,
                available_prediction_duration,
            )

            horizon_insufficient = (
                not math.isfinite(available_prediction_duration)
                or available_prediction_duration
                < minimum_duration - 1e-9
            )

            if request.contact_stamp is not None:
                remaining = (
                    request.contact_stamp
                    - request.trajectory_start_stamp
                )

                if request.terminal_mode:
                    preferred_duration = remaining
                    minimum_duration = remaining

                    if (
                        remaining <= 0.0
                        or available_prediction_duration
                        < remaining - 1e-9
                    ):
                        horizon_insufficient = True
                    # Keep the committed contact as the preferred and
                    # earliest candidate, but retain the full feasible
                    # planning horizon for recovery if that contact is no
                    # longer dynamically reachable.

                elif remaining >= minimum_duration:
                    preferred_duration = remaining

            if horizon_insufficient:
                outcome = FastPlanningOutcome(
                    plan=None,
                    failure=FastPlanningFailure.HORIZON_INSUFFICIENT,
                    diagnostics=PlannerDiagnostics(),
                )
            else:
                outcome = self.planner.plan(
                    initial_position=request.uav.position,
                    initial_velocity=request.uav.velocity,
                    initial_acceleration=request.uav.acceleration,
                    target_state_at_time=(
                        lambda horizon: request.prediction
                        .state_at_absolute_time(
                            request.trajectory_start_stamp
                            + horizon
                        )
                    ),
                    preferred_duration=preferred_duration,
                    minimum_duration_override=(
                        minimum_duration
                        if request.terminal_mode
                        else None
                    ),
                    maximum_duration_override=(
                        maximum_duration_override
                    ),
                )

            if request.contact_stamp is not None:
                locked_remaining = (
                    request.contact_stamp - request.trajectory_start_stamp
                )
                required_time = outcome.diagnostics.required_time
                locked_contact_unreachable = bool(
                    math.isfinite(required_time)
                    and required_time > locked_remaining + 1e-9
                )

        else:
            rejection_stage = 'INPUT_VALIDATION'
            rejection_detail = failure.value
            outcome = FastPlanningOutcome(
                plan=None,
                failure=failure,
                diagnostics=PlannerDiagnostics(),
            )

        elapsed = time.perf_counter() - monotonic_start

        deadline_failure = validate_total_deadline(
            elapsed,
            self.hard_deadline_seconds,
        )

        if (
            outcome.plan is not None
            and deadline_failure != FastPlanningFailure.NONE
        ):
            rejection_stage = 'TOTAL_DEADLINE'
            rejection_detail = deadline_failure.value
            outcome = FastPlanningOutcome(
                plan=None,
                failure=deadline_failure,
                diagnostics=outcome.diagnostics,
            )

        generated_stamp = self._ros_seconds()

        if outcome.plan is not None:
            arrival_failure = validate_plan_arrival(
                request,
                generated_stamp,
                self.maximum_input_age,
            )

            if arrival_failure != FastPlanningFailure.NONE:
                rejection_stage = 'INPUT_AGE_AT_FINISH'
                rejection_detail = (
                    f'age={generated_stamp - request.source_stamp:.6f}'
                )
                outcome = FastPlanningOutcome(
                    plan=None,
                    failure=arrival_failure,
                    diagnostics=outcome.diagnostics,
                )

        return PlannerJobResult(
            request=request,
            outcome=outcome,
            planning_started_stamp=planning_started_stamp,
            generated_stamp=generated_stamp,
            compute_time=time.perf_counter() - monotonic_start,
            locked_contact_unreachable=locked_contact_unreachable,
            rejection_stage=rejection_stage,
            rejection_detail=rejection_detail,
        )

    def _completion_frequency(self):
        if len(self.completion_times) < 2:
            return 0.0
        elapsed = self.completion_times[-1] - self.completion_times[0]
        return (
            (len(self.completion_times) - 1) / elapsed
            if elapsed > 1e-9 else 0.0
        )

    def _publish_job(self, job):
        publish_stamp = self._ros_seconds()
        outcome = job.outcome
        candidate_contact_stamp = None
        contact_delay = math.nan
        target_prediction_shift = math.nan
        capture_entry_stamp = math.nan
        capture_execution_margin = -math.inf
        preparation_position_error = math.inf
        preparation_velocity_error = math.inf
        contact_recovery_reason = ''
        rejection_stage = job.rejection_stage
        rejection_detail = job.rejection_detail
        current_cycle = getattr(
            self,
            'planning_cycle_id',
            job.request.planning_cycle_id,
        )
        request_invalidated = (
            job.request.mission_id != getattr(
                self,
                'mission_id',
                job.request.mission_id,
            )
            or job.request.planning_cycle_id != current_cycle
        )
        if hasattr(self, 'intercept_requested'):
            request_invalidated = bool(
                request_invalidated
                or not self.intercept_requested
                or self.mission_state not in self.PLANNING_STATES
            )
        if outcome.plan is not None and request_invalidated:
            rejection_stage = 'MISSION_OR_REQUEST_INVALIDATED'
            rejection_detail = (
                f'request_cycle={job.request.planning_cycle_id},'
                f'current_cycle={current_cycle}'
            )
            outcome = FastPlanningOutcome(
                plan=None,
                failure=FastPlanningFailure.PLAN_STALE_ON_ARRIVAL,
                diagnostics=outcome.diagnostics,
            )
        if outcome.plan is not None:
            arrival_failure = validate_plan_arrival(
                job.request,
                publish_stamp,
                self.maximum_input_age,
            )
            if arrival_failure != FastPlanningFailure.NONE:
                rejection_stage = 'INPUT_AGE_AT_PUBLISH'
                rejection_detail = (
                    f'age={publish_stamp - job.request.source_stamp:.6f}'
                )
                outcome = FastPlanningOutcome(
                    plan=None,
                    failure=arrival_failure,
                    diagnostics=outcome.diagnostics,
                )
        if outcome.plan is not None:
            candidate_contact_stamp = (
                job.request.trajectory_start_stamp + outcome.plan.duration
            )
            if job.request.contact_stamp is not None:
                contact_delay = (
                    candidate_contact_stamp - job.request.contact_stamp
                )
                contact_allowed = (
                    abs(contact_delay) <= 1e-9
                    or (
                        job.request.terminal_mode
                        and contact_delay > 0.0
                    )
                    or (
                        not job.request.terminal_mode
                        and contact_delay > 0.0
                        and job.locked_contact_unreachable
                    )
                )
                if contact_allowed and contact_delay > 1e-9:
                    contact_recovery_reason = (
                        'TERMINAL_CONTACT_RECOVERY'
                        if job.request.terminal_mode
                        else 'CONTACT_UNREACHABLE_RECOVERY'
                    )
                if not contact_allowed:
                    rejection_stage = 'CONTACT_TIME_POLICY'
                    rejection_detail = (
                        f'delay={contact_delay:.6f},'
                        'locked_contact_unreachable='
                        f'{job.locked_contact_unreachable}'
                    )
                    outcome = FastPlanningOutcome(
                        plan=None,
                        failure=(
                            FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
                        ),
                        diagnostics=outcome.diagnostics,
                    )
        if outcome.plan is not None:
            try:
                target_prediction_shift = target_shift_distance(
                    request=job.request,
                    latest_prediction=self.latest_prediction,
                    intercept_time=outcome.plan.duration,
                    contact_stamp=candidate_contact_stamp,
                )
            except (TypeError, ValueError):
                shift_failure = FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
                rejection_stage = 'PREDICTION_HORIZON_UNAVAILABLE'
                rejection_detail = (
                    'absolute contact is outside an available prediction'
                )
            else:
                shift_failure = (
                    FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
                    if target_prediction_shift > self.endpoint_tolerance
                    else FastPlanningFailure.NONE
                )
                if shift_failure != FastPlanningFailure.NONE:
                    rejection_stage = 'TARGET_PREDICTION_SHIFT'
                    rejection_detail = (
                        f'shift={target_prediction_shift:.6f},'
                        f'tolerance={self.endpoint_tolerance:.6f}'
                    )
            if shift_failure != FastPlanningFailure.NONE:
                outcome = FastPlanningOutcome(
                    plan=None,
                    failure=shift_failure,
                    diagnostics=outcome.diagnostics,
                )

        contact_stamp = self.contact_schedule.contact_stamp
        remaining_t_go = 0.0
        terminal_mode = terminal_mode_for_plan(
            mission_state=self.mission_state,
            request_terminal_mode=job.request.terminal_mode,
            terminal_time_threshold=self.terminal_time_threshold,
            terminal_state=MissionState.TERMINAL_MINCO,
        )
        planned_capture_margin = 0.0
        terminal_admission = outcome.plan is not None
        terminal_admission_reason = (
            'ADMITTED' if terminal_admission else outcome.failure.value
        )
        if outcome.plan is not None:
            contact_stamp = candidate_contact_stamp
            remaining_t_go = max(
                contact_stamp - job.request.trajectory_start_stamp,
                0.0,
            )
            terminal_mode = terminal_mode_for_plan(
                mission_state=self.mission_state,
                request_terminal_mode=terminal_mode,
                remaining_t_go=remaining_t_go,
                terminal_time_threshold=self.terminal_time_threshold,
                terminal_state=MissionState.TERMINAL_MINCO,
            )
            try:
                target_position = (
                    job.request.prediction.state_at_absolute_time(
                        contact_stamp
                    )[0]
                )
                terminal_position = outcome.plan.sample(
                    outcome.plan.duration
                ).position
                terminal_error = math.sqrt(sum(
                    (planned - target) ** 2
                    for planned, target in zip(
                        terminal_position,
                        target_position,
                    )
                ))
                planned_capture_margin = (
                    self.planned_capture_radius - terminal_error
                )
            except (TypeError, ValueError):
                planned_capture_margin = -math.inf
            try:
                capture_window = planned_capture_window(
                    outcome.plan,
                    job.request.prediction,
                    job.request.trajectory_start_stamp,
                    self.planned_capture_radius,
                    sample_step=self.planner.sample_step,
                )
                capture_entry_stamp = (
                    job.request.trajectory_start_stamp
                    + capture_window.first_entry_t_go
                )
                capture_execution_margin = (
                    capture_window.execution_margin
                )
                target_start = (
                    job.request.prediction.state_at_absolute_time(
                        job.request.trajectory_start_stamp
                    )
                )
                (
                    preparation_position_error,
                    preparation_velocity_error,
                ) = approach_preparation_errors(
                    uav_position=job.request.uav.position,
                    uav_velocity=job.request.uav.velocity,
                    target_position=target_start[0],
                    target_velocity=target_start[1],
                    vertical_time=outcome.diagnostics.vertical_min_time,
                    standoff_speed=self.approach_preparation_standoff_speed,
                )
            except (TypeError, ValueError):
                capture_execution_margin = -math.inf

        if (
            outcome.plan is not None
            and self.mission_state == MissionState.FAR_GUIDANCE
        ):
            initial_reference = outcome.plan.sample(0.0)
            admission = evaluate_terminal_admission(
                horizontal_min_time=outcome.diagnostics.horizontal_min_time,
                vertical_min_time=outcome.diagnostics.vertical_min_time,
                selected_t_go=outcome.plan.duration,
                planned_capture_margin=planned_capture_margin,
                current_z=job.request.uav.position[2],
                current_vz=job.request.uav.velocity[2],
                planned_initial_vz=initial_reference.velocity[2],
                sea_surface_z=self.planner.sea_surface_z,
                reserve_clearance=getattr(
                    self,
                    'approach_reserve_clearance',
                    0.20,
                ),
                response_delay=self.planner.response_delay,
                braking_acceleration=(
                    self.planner.effective_vertical_braking_acceleration
                ),
                maximum_vertical_speed=self.planner.maximum_vertical_speed,
                time_sync_tolerance=getattr(
                    self,
                    'approach_time_sync_tolerance',
                    0.35,
                ),
                preparation_position_error=preparation_position_error,
                preparation_velocity_error=preparation_velocity_error,
                preparation_position_tolerance=(
                    self.approach_preparation_position_tolerance
                ),
                preparation_velocity_tolerance=(
                    self.approach_preparation_velocity_tolerance
                ),
                capture_execution_margin=capture_execution_margin,
            )
            terminal_admission = admission.admitted
            terminal_admission_reason = admission.reason

        trajectory_allowed = bool(
            outcome.plan is not None and terminal_admission
        )

        self.completed_plan_count += 1
        self.solver_compute_times.append(job.compute_time)
        self.completion_times.append(time.monotonic())
        self.plan_id += 1
        candidate_published = False
        if trajectory_allowed:
            candidate_published = self.contact_schedule.propose(
                plan_id=self.plan_id,
                contact_stamp=contact_stamp,
                planning_cycle_id=current_cycle,
                proposed_at=publish_stamp,
            )
            if not candidate_published:
                rejection_stage = 'CONTACT_PROPOSAL'
                rejection_detail = 'invalid pending contact proposal'
                outcome = FastPlanningOutcome(
                    plan=None,
                    failure=FastPlanningFailure.PLAN_STALE_ON_ARRIVAL,
                    diagnostics=outcome.diagnostics,
                )
                trajectory_allowed = False
        diagnostics = outcome.diagnostics
        message = PlannerDiagnostic()
        message.mission_id = job.request.mission_id
        message.plan_id = self.plan_id
        message.planning_cycle_id = int(current_cycle)
        message.prediction_sequence_id = (
            job.request.prediction.sequence_id
        )
        message.source_stamp = seconds_to_time(job.request.source_stamp)
        message.planning_started_stamp = seconds_to_time(
            job.planning_started_stamp
        )
        message.generated_stamp = seconds_to_time(job.generated_stamp)
        message.published_stamp = seconds_to_time(publish_stamp)
        if outcome.plan is None:
            message.result = PlannerDiagnostic.RESULT_FAILURE
        elif trajectory_allowed:
            message.result = PlannerDiagnostic.RESULT_SUCCESS
        else:
            message.result = PlannerDiagnostic.RESULT_IDLE
        message.failure_reason = FAILURE_CONSTANTS[outcome.failure]
        message.failure_detail = (
            rejection_detail or outcome.failure.value
        )
        message.rejection_stage = str(rejection_stage)
        message.rejection_detail = str(rejection_detail)
        message.contact_recovery_reason = str(contact_recovery_reason)
        message.compute_time = job.compute_time
        message.reachability_time = diagnostics.reachability_compute_time
        message.generation_time = diagnostics.generation_compute_time
        message.validation_time = diagnostics.validation_compute_time
        message.optimization_time = diagnostics.optimization_compute_time
        message.candidate_diagnostics = json.dumps(
            [asdict(value) for value in diagnostics.candidate_diagnostics],
            ensure_ascii=True,
            separators=(',', ':'),
        )
        message.input_age_at_start = (
            job.planning_started_stamp - job.request.source_stamp
        )
        message.input_age_at_finish = (
            job.generated_stamp - job.request.source_stamp
        )
        message.input_age_at_publish = (
            publish_stamp - job.request.source_stamp
        )
        message.completion_to_publish_delay = max(
            publish_stamp - job.generated_stamp,
            0.0,
        )
        message.contact_delay = float(contact_delay)
        message.target_prediction_shift = float(target_prediction_shift)
        message.candidate_published = bool(candidate_published)
        if outcome.plan is not None:
            message.selected_t_go = float(outcome.plan.duration)
            message.contact_stamp = seconds_to_time(contact_stamp)
            if math.isfinite(capture_entry_stamp):
                message.capture_entry_stamp = seconds_to_time(
                    capture_entry_stamp
                )
            message.remaining_t_go = float(remaining_t_go)
            message.terminal_mode = bool(terminal_mode)
            message.planned_capture_margin = float(
                planned_capture_margin
            )
            message.capture_execution_margin = float(
                capture_execution_margin
            )
            message.preparation_position_error = float(
                preparation_position_error
            )
            message.preparation_velocity_error = float(
                preparation_velocity_error
            )

        message.required_time = float(
            diagnostics.required_time
        )
        message.horizontal_min_time = float(
            diagnostics.horizontal_min_time
        )
        message.vertical_min_time = float(
            diagnostics.vertical_min_time
        )
        message.sea_safe_min_time = float(
            diagnostics.sea_safe_min_time
        )
        message.search_min_time = float(
            diagnostics.search_min_time
        )
        message.search_max_time = float(
            diagnostics.search_max_time
        )
        message.available_prediction_duration = float(
            job.request.prediction.end_stamp
            - job.request.trajectory_start_stamp
        )
        message.locked_remaining_t_go = (
            float(
                job.request.contact_stamp
                - job.request.trajectory_start_stamp
            )
            if job.request.contact_stamp is not None
            else math.nan
        )

        message.candidate_count = diagnostics.candidates_checked
        message.replaced_request_count = (
            self.request_slot.replaced_request_count
        )
        message.completed_plan_count = self.completed_plan_count
        message.actual_completion_frequency = self._completion_frequency()
        message.approach_phase = (
            'PREPARATION'
            if self.mission_state == MissionState.FAR_GUIDANCE
            else 'TERMINAL_APPROACH'
        )
        message.terminal_admission = bool(terminal_admission)
        message.terminal_admission_reason = str(
            terminal_admission_reason
        )
        self.diagnostic_pub.publish(message)

        if trajectory_allowed:
            trajectory = plan_to_message(
                outcome.plan,
                mission_id=job.request.mission_id,
                plan_id=self.plan_id,
                prediction_sequence_id=(
                    job.request.prediction.sequence_id
                ),
                trajectory_start_stamp=job.request.trajectory_start_stamp,
                planning_started_stamp=job.planning_started_stamp,
                generated_stamp=job.generated_stamp,
                target_state_source=job.request.prediction.source,
                observation_stamp=job.request.observation_stamp,
                frame_id=self.frame_id,
                contact_stamp=contact_stamp,
                remaining_t_go=remaining_t_go,
                terminal_mode=terminal_mode,
                planned_capture_margin=planned_capture_margin,
                capture_entry_stamp=capture_entry_stamp,
                capture_execution_margin=capture_execution_margin,
            )
            trajectory.published_stamp = seconds_to_time(publish_stamp)
            self.published_plan_references[self.plan_id] = (
                ActivePlanReference(
                    plan=outcome.plan,
                    source_stamp=job.request.trajectory_start_stamp,
                )
            )
            while len(self.published_plan_references) > 4:
                oldest = min(self.published_plan_references)
                self.published_plan_references.pop(oldest, None)
            self.trajectory_pub.publish(trajectory)

    def completion_timer_callback(self):
        if self.ready_job is not None:
            if self._ros_seconds() + 1e-9 < (
                self.ready_job.request.trajectory_start_stamp
            ):
                return
            job = self.ready_job
            self.ready_job = None
            self._publish_job(job)
            return
        if self.future is None or not self.future.done():
            return

        future = self.future
        self.future = None

        try:
            job = future.result()
        except Exception as exc:
            self.get_logger().error(
                'Planner worker failed: '
                f'{type(exc).__name__}: {exc}'
            )
            return

        if (
            job.outcome.plan is not None
            and self._ros_seconds() + 1e-9
            < job.request.trajectory_start_stamp
        ):
            self.ready_job = job
            return
        self._publish_job(job)

    def _planning_tick(self, terminal_tick, prediction_triggered=False):
        if (
            not self.intercept_requested
            or self.mission_state not in self.PLANNING_STATES
        ):
            return
        now = self._ros_seconds()
        self.contact_schedule.expire(
            now,
            getattr(self, 'maximum_input_age', 0.125),
        )
        if self.contact_schedule.has_pending:
            return
        remaining = None
        if self.contact_schedule.contact_stamp is not None:
            remaining = self.contact_schedule.remaining_t_go(
                now
            )
        if (
            remaining is not None and remaining <= 0.0
            and self.mission_state == MissionState.FAR_GUIDANCE
        ):
            # Recovery may pass through TARGET_LOCK before FAR_GUIDANCE, so
            # the direct recovery->far callback is not guaranteed. A missed,
            # expired contact must not keep the reacquired mission frozen.
            self.planning_cycle_id += 1
            self.contact_schedule.reset()
            self.active_plan_reference = None
            self.published_plan_references.clear()
            self.last_submitted_key = None
            remaining = None
        decision = self.request_policy.decide(
            self.mission_state,
            remaining_t_go=remaining,
        )
        if (
            terminal_tick is not None
            and decision.terminal_mode != bool(terminal_tick)
        ):
            return
        if not decision.submit:
            return
        # Timers maintain contact expiry/freeze policy. Start only on a new
        # prediction: equal-period timers can repeatedly select the previous
        # image just before its replacement arrives, wasting the age budget.
        if not prediction_triggered:
            return
        if self.future is not None or self.ready_job is not None:
            return
        period = (
            self.terminal_planning_period
            if decision.terminal_mode else self.planning_period
        )
        if (
            self.last_worker_started_stamp is not None
            and now - self.last_worker_started_stamp < period - 1e-8
        ):
            return
        current = self._current_request(decision)
        if current is not None:
            key = (
                current.mission_id,
                current.planning_cycle_id,
                current.prediction.sequence_id,
                current.uav.stamp,
            )
            if key != self.last_submitted_key:
                # Build the boundary at dispatch, never queue a future
                # handover state while another worker result is pending.
                self.last_submitted_key = key
                self.last_worker_started_stamp = now
                self.future = self.worker_executor.submit(
                    self._run_request,
                    current,
                )

    def planning_timer_callback(self):
        self._planning_tick(terminal_tick=False)

    def terminal_planning_timer_callback(self):
        self._planning_tick(terminal_tick=True)

    def destroy_node(self):
        self.worker_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InterceptPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
