"""Independent 20 Hz polynomial tracker and final PX4 safety barrier."""

import math
import json
import time
from collections import deque
from dataclasses import dataclass, replace

import rclpy
from builtin_interfaces.msg import Time
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from uav_usv_interfaces.msg import ControllerDiagnostic, InterceptTrajectory
from uav_usv_interfaces.msg import MissionState, PlannerDiagnostic
from uav_usv_interfaces.msg import TargetPrediction, TargetState
from uav_usv_interfaces.msg import TargetBearing, TargetObservation

from ..mission.target_visibility import TargetVisibilityState, VisibilityConfig
from ..common.navigation_state import navigation_components

from .flight_guidance import FlightGuidanceCore, FlightKinematicState
from .trajectory_tracking import PolynomialSegmentData
from .trajectory_tracking import PolynomialTrajectory
from .trajectory_tracking import TrackerKinematicState
from .trajectory_tracking import TrajectoryRejectReason
from .trajectory_tracking import TrajectoryTrackerCore


@dataclass(frozen=True)
class PendingTrajectory:
    """One future-start candidate awaiting a state at its execution time."""

    trajectory: PolynomialTrajectory
    planner_published_stamp: float
    tracker_receipt_stamp: float


@dataclass(frozen=True)
class TerminalExecution:
    """Non-renewable execution window for one already accepted trajectory."""

    trajectory: PolynomialTrajectory
    entered_at: float
    deadline: float


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _seconds_to_time(value):
    value = max(float(value), 0.0)
    seconds = int(value)
    nanoseconds = int(round((value - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def trajectory_from_message(message):
    """Reconstruct the complete polynomial without resetting source time."""
    if not message.valid:
        raise ValueError('invalid trajectory message')
    if message.piece_count != len(message.segments) or not message.segments:
        raise ValueError('invalid trajectory message piece count')
    segments = tuple(PolynomialSegmentData(
        duration=_duration_seconds(segment.duration),
        coefficients=tuple(float(value) for value in segment.coefficients),
    ) for segment in message.segments)
    return PolynomialTrajectory(
        mission_id=int(message.mission_id),
        plan_id=int(message.plan_id),
        prediction_sequence_id=int(message.prediction_sequence_id),
        source_stamp=_stamp_seconds(message.source_stamp),
        generated_stamp=_stamp_seconds(message.generated_stamp),
        valid_until=_stamp_seconds(message.valid_until),
        segments=segments,
        terminal_position=(
            float(message.terminal_position.x),
            float(message.terminal_position.y),
            float(message.terminal_position.z),
        ),
        terminal_velocity=(
            float(message.terminal_velocity.x),
            float(message.terminal_velocity.y),
            float(message.terminal_velocity.z),
        ),
        target_state_source=str(message.target_state_source),
        frame_id=str(message.frame_id),
        contact_stamp=_stamp_seconds(message.contact_stamp),
        capture_entry_stamp=_stamp_seconds(message.capture_entry_stamp),
        selected_t_go=float(message.selected_t_go),
        remaining_t_go=float(message.remaining_t_go),
        terminal_mode=bool(message.terminal_mode),
        planned_capture_margin=float(message.planned_capture_margin),
        capture_execution_margin=float(message.capture_execution_margin),
        observation_stamp=_stamp_seconds(message.observation_stamp),
    )


def trajectory_for_mission(trajectory, mission_state):
    """Make the mission's committed terminal state authoritative."""
    if int(mission_state) == MissionState.TERMINAL_MINCO:
        return replace(trajectory, terminal_mode=True)
    return trajectory


def tracker_state_from_message(message, received_stamp):
    """Use ROS receipt time, not the unrelated PX4 boot timestamp."""
    values = []
    for name in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
        value = float(getattr(message, name, math.nan))
        values.append(value if math.isfinite(value) else 0.0)
    return TrackerKinematicState(
        stamp=float(received_stamp),
        position=tuple(values[:3]),
        velocity=tuple(values[3:]),
    )


def tracker_state_from_navigation(message, now):
    """Preserve the causally mapped physical sample epoch for acceptance."""
    stamp, position, velocity, _ = navigation_components(message, now)
    return TrackerKinematicState(stamp, position, velocity)


def flight_target_from_message(message):
    """Convert the launch-selected target source for flight guidance."""
    values = (
        float(message.position.x),
        float(message.position.y),
        float(message.position.z),
        float(message.velocity.x),
        float(message.velocity.y),
        float(message.velocity.z),
    )
    if not message.valid or not all(math.isfinite(value) for value in values):
        raise ValueError('target state is invalid')
    return FlightKinematicState(values[:3], values[3:])


def measurement_age(now, stamp):
    """Return true acquisition age; missing/future clocks fail closed."""
    value = float(stamp)
    age = float(now) - value
    return age if math.isfinite(age) and value > 0.0 and age >= 0.0 else math.inf


def fresh_flight_target(message, now, maximum_age, frame_id='local_ned', source_max_age=None):
    """Convert KF state only when its originating image remains fresh."""
    if message is None or str(message.frame_id) != frame_id:
        return None
    source_age = measurement_age(now, _stamp_seconds(message.source_stamp))
    state_age = measurement_age(now, _stamp_seconds(message.stamp))
    if (source_age > (maximum_age if source_max_age is None else source_max_age)
            or state_age > maximum_age
            or source_age < state_age):
        return None
    try:
        return flight_target_from_message(message)
    except (TypeError, ValueError):
        return None


def anchored_search_hold(anchor, position, sea_surface_z, search_height):
    """Capture one XYZ reference on search entry rather than chasing drift."""
    if anchor is not None:
        return tuple(anchor)
    return (float(position[0]), float(position[1]),
            min(float(position[2]), float(sea_surface_z) - search_height))


def image_yaw_rate(bearing, gain, deadband, maximum_rate):
    """Positive image-right bearing requests positive NED yaw."""
    if not math.isfinite(float(bearing)) or abs(bearing) < deadband:
        return 0.0
    return max(-maximum_rate, min(maximum_rate, gain * bearing))


def _wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def target_facing_yaw(uav_position, target_position):
    """Return the local-NED heading that points from UAV to target."""
    return math.atan2(
        float(target_position[1]) - float(uav_position[1]),
        float(target_position[0]) - float(uav_position[0]),
    )


def rate_limited_target_yaw(previous_yaw, desired_yaw, maximum_rate, dt):
    """Move toward target yaw through the shortest wrapped angular delta."""
    desired_yaw = _wrap_angle(desired_yaw)
    if previous_yaw is None or not math.isfinite(float(previous_yaw)):
        return desired_yaw
    maximum_step = max(float(maximum_rate), 0.0) * max(float(dt), 0.0)
    delta = _wrap_angle(desired_yaw - float(previous_yaw))
    delta = max(min(delta, maximum_step), -maximum_step)
    return _wrap_angle(float(previous_yaw) + delta)


def command_to_setpoint(
    command,
    timestamp_us,
    yaw=math.nan,
    velocity_mode=False,
):
    """
    Map the post-guard command to a PX4 setpoint.

    ``velocity_mode`` sends the tracker's acceleration-limited velocity as the
    commanded setpoint with NaN position, so PX4 consumes the closing speed
    the tracker builds.  Position mode keeps the previous behaviour of sending
    the MINCO reference and is retained for callers that still want it.
    """
    message = TrajectorySetpoint()
    message.timestamp = int(timestamp_us)
    if velocity_mode:
        message.position = [math.nan, math.nan, math.nan]
        message.velocity = [float(value) for value in command.velocity]
        # PX4's velocity loop accepts acceleration feed-forward. The tracker
        # has already differentiated/limited the post-guard velocity; dropping
        # this braking term makes the plant lag a decelerating MINCO reference.
        message.acceleration = [float(value) for value in command.acceleration]
    else:
        message.position = [float(value) for value in command.position]
        message.velocity = [float(value) for value in command.velocity]
        message.acceleration = [float(value) for value in command.acceleration]
    message.jerk = [math.nan, math.nan, math.nan]
    message.yaw = math.nan if yaw is None else float(yaw)
    message.yawspeed = math.nan
    return message


def hold_setpoint(state, timestamp_us, yaw=math.nan):
    """Hold the last valid position without a high-speed fallback."""
    message = TrajectorySetpoint()
    message.timestamp = int(timestamp_us)
    message.position = [float(value) for value in state.position]
    message.velocity = [0.0, 0.0, 0.0]
    message.acceleration = [math.nan, math.nan, math.nan]
    message.jerk = [math.nan, math.nan, math.nan]
    message.yaw = math.nan if yaw is None else float(yaw)
    message.yawspeed = math.nan
    return message


def flight_command_to_setpoint(
        command, timestamp_us, yaw=math.nan, acceleration_feedforward=False):
    """Map non-MINCO flight guidance without mixing PX4 control modes."""
    message = TrajectorySetpoint()
    message.timestamp = int(timestamp_us)
    if command.mode == 'VELOCITY':
        message.position = [math.nan, math.nan, math.nan]
        message.velocity = [float(value) for value in command.velocity]
    elif command.mode == 'POSITION':
        message.position = [float(value) for value in command.position]
        message.velocity = [math.nan, math.nan, math.nan]
    else:
        raise ValueError(f'unsupported flight command mode: {command.mode}')
    message.acceleration = [math.nan, math.nan, math.nan]
    if acceleration_feedforward:
        if command.mode != 'VELOCITY' or len(command.acceleration) != 3:
            raise ValueError('INVALID_ACCELERATION_FEEDFORWARD_COMMAND')
        message.acceleration = [float(v) for v in command.acceleration]
    message.jerk = [math.nan, math.nan, math.nan]
    message.yaw = math.nan if yaw is None else float(yaw)
    message.yawspeed = math.nan
    return message


def prediction_endpoint_from_message(message, contact_stamp):
    """Interpolate a target endpoint at one absolute ROS contact time."""
    if not message.valid or not message.samples:
        raise ValueError('target prediction is unavailable')
    desired_time = float(contact_stamp) - _stamp_seconds(message.source_stamp)
    if not math.isfinite(desired_time) or desired_time < 0.0:
        raise ValueError('contact time precedes prediction source')
    samples = list(message.samples)
    first_time = _duration_seconds(samples[0].relative_time)
    if desired_time <= first_time:
        point = samples[0].position
        return float(point.x), float(point.y), float(point.z)
    for left, right in zip(samples, samples[1:]):
        left_time = _duration_seconds(left.relative_time)
        right_time = _duration_seconds(right.relative_time)
        if desired_time <= right_time:
            span = max(right_time - left_time, 1e-9)
            ratio = min(max((desired_time - left_time) / span, 0.0), 1.0)
            return tuple(
                float(getattr(left.position, axis))
                + ratio * (
                    float(getattr(right.position, axis))
                    - float(getattr(left.position, axis))
                )
                for axis in ('x', 'y', 'z')
            )
    if desired_time > _duration_seconds(samples[-1].relative_time) + 1e-9:
        raise ValueError('contact time exceeds prediction horizon')
    point = samples[-1].position
    return float(point.x), float(point.y), float(point.z)


def ground_flight_ready(offboard_active, vehicle_armed, status_fresh,
                        pre_flight_checks_pass):
    """Require PX4 preflight approval before exposing the X gate."""
    return bool(
        offboard_active and not vehicle_armed and status_fresh
        and pre_flight_checks_pass
    )


class TrajectoryTrackerNode(Node):
    """Track accepted plans while keeping optimization out of control."""

    CONTROL_STATES = {
        MissionState.MINCO_READY,
        MissionState.MINCO_TRACKING,
        MissionState.TERMINAL_MINCO,
        MissionState.PLAN_RECOVERY,
        MissionState.SAFE_WAIT,
    }
    TERMINAL_STATES = {
        MissionState.CAPTURE,
        MissionState.FAILURE,
        MissionState.ABORTED,
    }

    def __init__(self):
        super().__init__('trajectory_tracker_node')
        self.declare_parameter('p7_guidance_enabled', False)
        self.p7_guidance_enabled = bool(self.get_parameter('p7_guidance_enabled').value)
        self.declare_parameter('follow_minco_enabled', True)
        self.declare_parameter('follow_minco_shadow_check', False)
        self.follow_minco_requested = bool(self.get_parameter('follow_minco_enabled').value)
        self.declare_parameter('follow_minco_acceleration_feedforward', False)
        self.follow_minco_acceleration_feedforward = bool(
            self.get_parameter('follow_minco_acceleration_feedforward').value)
        if self.follow_minco_requested:
            from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
            self.follow_sitl_grant = require_isolated_sitl()
        self.follow_stable_ticks = 0
        from uav_control.guidance.follow_admission import AdmissionConfig, FollowAdmission
        self.declare_parameter('follow_admission_enabled', True)
        self.declare_parameter('follow_reference_mode', 'target')
        self.follow_reference_mode = self.get_parameter('follow_reference_mode').value
        if self.follow_reference_mode not in ('target', 'rear_point'):
            raise ValueError('INVALID_FOLLOW_REFERENCE_MODE')
        admission_values = {}
        for name, value in vars(AdmissionConfig()).items():
            self.declare_parameter('follow_admission_'+name, value)
            admission_values[name] = self.get_parameter('follow_admission_'+name).value
        self.follow_admission = (FollowAdmission(AdmissionConfig(**admission_values))
                                 if self.get_parameter('follow_admission_enabled').value else None)
        self.follow_admission_identity = None
        self.follow_navigation_history = deque(maxlen=128)
        self.follow_target_history = deque(maxlen=64)
        self.follow_debug_pub = self.create_publisher(
            String, '/control/follow_execution_debug', 10)
        # Nominal feedback execution does not claim independently qualified holding.
        self.follow_minco_authorized = False
        self.follow_generation = 0
        self.follow_receiver = None
        if self.follow_minco_requested or self.get_parameter('follow_minco_shadow_check').value:
            from uav_control.guidance.follow_contract import FollowReceiver
            from uav_usv_interfaces.msg import FollowTrajectory, FollowPlanAck
            from uav_control.guidance.follow_epoch import ReceiverClock
            from uav_usv_interfaces.msg import FollowReceiverState, UavState
            self.follow_clock = ReceiverClock()
            from rcl_interfaces.msg import ParameterDescriptor
            from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
            from uav_control.guidance.follow_limits import ResearchConfig, constraint_snapshot
            for name, value in (('follow_research_horizontal_acceleration', 3.),
                                ('follow_research_yaw_rate', 1.),
                                ('follow_target_time_profile', 'baseline_125')):
                self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            research = ResearchConfig(maximum_horizontal_acceleration=float(self.get_parameter(
                'follow_research_horizontal_acceleration').value), maximum_yaw_rate=float(
                self.get_parameter('follow_research_yaw_rate').value), target_time_profile=str(
                self.get_parameter('follow_target_time_profile').value))
            from uav_control.guidance.follow_time_contract import input_time_contract
            self.follow_time_contract = input_time_contract(research.target_time_profile)
            if (self.follow_admission is not None
                    and self.follow_admission.config.target_observation_age
                    != self.follow_time_contract.observation_age):
                raise ValueError('FOLLOW_ADMISSION_TARGET_TIME_MISMATCH')
            self.follow_receiver = FollowReceiver(
                limits=constraint_snapshot(FollowMpcSeed(research)))
            if self.follow_minco_requested:
                from uav_control.guidance.follow_math_runtime import configure_numerical_threads
                self.follow_math_runtime = configure_numerical_threads(1)
                from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
                self.follow_model = FollowMpcSeed(research)
                from uav_control.guidance.p6_follow import configure_p6
                self.p6_runtime = configure_p6(self, self.follow_model)
                self.p6_diagnostic_sub = self.create_subscription(
                    String, '/planning/imm_diagnostics', self.p6_runtime.diagnostic,
                    qos_profile_sensor_data)
                self.follow_receiver = NominalFollowReceiver(
                    limits=constraint_snapshot(self.follow_model), admission=self.follow_admission)
            epoch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=ReliabilityPolicy.RELIABLE)
            self.follow_epoch_pub = self.create_publisher(
                FollowReceiverState, '/control/follow_receiver_state', epoch_qos)
            self.follow_epoch_sub = self.create_subscription(
                UavState, '/navigation/uav_state', self._follow_native_epoch,
                qos_profile_sensor_data)
            self.follow_epoch_timer = self.create_timer(.05, self._publish_follow_epoch)
            self.follow_ack_pub = self.create_publisher(FollowPlanAck, '/control/follow_ack', 10)
            self.follow_sub = self.create_subscription(
                FollowTrajectory, '/planning/follow_trajectory',
                self.follow_trajectory_callback, 1)
            self.get_logger().warning(
                'FOLLOW controller: nominal rolling MINCO' if self.follow_minco_requested else
                'FOLLOW controller: original guidance with research rejection ACK')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('maximum_plan_age', 0.125)
        self.declare_parameter('minimum_remaining_time', 0.20)
        self.declare_parameter('maximum_position_error', 0.30)
        self.declare_parameter('maximum_velocity_error', 0.50)
        self.declare_parameter('target_endpoint_tolerance', 0.50)
        self.declare_parameter('position_gain', 1.2)
        self.declare_parameter('maximum_horizontal_speed', 6.5)
        self.declare_parameter(
            'guidance_maximum_horizontal_speed',
            6.2,
        )
        self.declare_parameter('maximum_vertical_speed', 4.0)
        self.declare_parameter('maximum_horizontal_acceleration', 3.0)
        self.declare_parameter(
            'guidance_maximum_horizontal_acceleration',
            self.get_parameter('maximum_horizontal_acceleration').value,
        )
        self.declare_parameter('maximum_vertical_acceleration', 3.0)
        self.declare_parameter('sea_surface_z', 0.0)
        self.declare_parameter('reserve_clearance', 0.07)
        self.declare_parameter('safety_response_delay', 0.15)
        self.declare_parameter('vertical_braking_acceleration', 2.5)
        self.declare_parameter('recovery_clearance', 0.5)
        self.declare_parameter('recovery_climb_speed', 1.0)
        self.declare_parameter('maximum_command_dt', 0.1)
        self.declare_parameter('maximum_actual_vertical_acceleration', 4.0)
        self.declare_parameter('terminal_cruise_enabled', False)
        self.terminal_cruise_enabled = bool(
            self.get_parameter('terminal_cruise_enabled').value)
        self.declare_parameter('maximum_state_age', 0.125)
        self.declare_parameter('use_velocity_control', True)
        self.declare_parameter('frame_id', 'local_ned')
        self.declare_parameter('target_state_topic', '/tracking/target_state')
        self.declare_parameter('offboard_prestream_time', 2.0)
        self.declare_parameter('px4_command_retry_time', 1.0)
        self.declare_parameter('vehicle_status_timeout', 2.0)
        self.declare_parameter('flight_altitude', -5.0)
        self.declare_parameter('takeoff_tolerance', 0.5)
        self.declare_parameter('takeoff_settle_time', 1.0)
        self.declare_parameter('takeoff_maximum_vertical_speed', 1.5)
        self.declare_parameter('takeoff_maximum_vertical_acceleration', 1.0)
        self.declare_parameter('takeoff_maximum_horizontal_acceleration', 1.5)
        self.declare_parameter('takeoff_horizontal_start_height', 0.5)
        self.declare_parameter('takeoff_horizontal_full_height', 1.5)
        self.declare_parameter('takeoff_visual_tracking', False)
        self.declare_parameter('follow_distance', 5.0)
        self.declare_parameter('follow_position_gain', 0.8)
        self.declare_parameter('bearing_approach_enabled', True)
        self.declare_parameter('bearing_approach_speed', 6.0)
        self.declare_parameter('altitude_velocity_gain', 1.0)
        self.declare_parameter('approach_contact_clearance', 0.33)
        self.declare_parameter('approach_closing_speed', 1.5)
        self.declare_parameter('approach_preparation_clearance', 1.2)
        self.declare_parameter('approach_horizon', 4.0)
        self.declare_parameter('approach_response_delay', 0.15)
        self.declare_parameter('terminal_replacement_position_error', 0.15)
        self.declare_parameter('max_observation_yaw_rate', 1.0)
        for name, value in vars(VisibilityConfig()).items():
            self.declare_parameter(name, value)

        control_rate = float(self.get_parameter('control_rate_hz').value)
        if not math.isfinite(control_rate) or control_rate <= 0.0:
            raise ValueError('control_rate_hz must be finite and positive')
        self.maximum_state_age = float(
            self.get_parameter('maximum_state_age').value
        )
        self.expected_frame_id = str(self.get_parameter('frame_id').value)
        self.takeoff_visual_tracking = bool(
            self.get_parameter('takeoff_visual_tracking').value)
        # Drive MINCO tracking with the tracker's acceleration-limited velocity
        # command.  Position mode re-sends a reference that is rebuilt from the
        # measured state every replan, so the position error never grows past a
        # few centimetres and the vehicle can only hold the speed it already
        # has; velocity mode is what lets it build the closing speed.
        self.use_velocity_control = bool(
            self.get_parameter('use_velocity_control').value
        )
        self.target_state_topic = str(
            self.get_parameter('target_state_topic').value
        )
        if self.target_state_topic != '/tracking/target_state':
            raise ValueError('strict tracker requires /tracking/target_state')
        if self.resolve_topic_name(self.target_state_topic) != '/tracking/target_state':
            raise ValueError('strict tracker forbids target state remapping')
        self.visibility = TargetVisibilityState(VisibilityConfig(**{
            name: self.get_parameter(name).value
            for name in vars(VisibilityConfig())
        }))
        self.bearing_approach_enabled = bool(
            self.get_parameter('bearing_approach_enabled').value)
        self.bearing_approach_speed = float(self.get_parameter('bearing_approach_speed').value)
        if not math.isfinite(self.bearing_approach_speed) or self.bearing_approach_speed <= 0.0:
            raise ValueError('bearing_approach_speed must be finite and positive')
        self.offboard_prestream_cycles = max(
            int(round(
                self.get_parameter('offboard_prestream_time').value
                * control_rate
            )),
            1,
        )
        self.px4_command_retry_cycles = max(
            int(round(
                self.get_parameter('px4_command_retry_time').value
                * control_rate
            )),
            1,
        )
        self.vehicle_status_timeout = float(
            self.get_parameter('vehicle_status_timeout').value
        )
        self.terminal_replacement_position_error = float(
            self.get_parameter('terminal_replacement_position_error').value
        )
        self.max_observation_yaw_rate = float(
            self.get_parameter('max_observation_yaw_rate').value
        )
        self.tracker = TrajectoryTrackerCore(
            maximum_plan_age=self.get_parameter('maximum_plan_age').value,
            minimum_remaining_time=self.get_parameter(
                'minimum_remaining_time'
            ).value,
            maximum_position_error=self.get_parameter(
                'maximum_position_error'
            ).value,
            maximum_velocity_error=self.get_parameter(
                'maximum_velocity_error'
            ).value,
            target_endpoint_tolerance=self.get_parameter(
                'target_endpoint_tolerance'
            ).value,
            expected_frame_id=self.expected_frame_id,
            position_gain=self.get_parameter('position_gain').value,
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
            sea_surface_z=self.get_parameter('sea_surface_z').value,
            reserve_clearance=self.get_parameter('reserve_clearance').value,
            safety_response_delay=self.get_parameter(
                'safety_response_delay'
            ).value,
            vertical_braking_acceleration=self.get_parameter(
                'vertical_braking_acceleration'
            ).value,
            control_dt=1.0 / control_rate,
            recovery_clearance=self.get_parameter(
                'recovery_clearance'
            ).value,
            recovery_climb_speed=self.get_parameter(
                'recovery_climb_speed'
            ).value,
            maximum_command_dt=self.get_parameter(
                'maximum_command_dt'
            ).value,
            maximum_actual_vertical_acceleration=self.get_parameter(
                'maximum_actual_vertical_acceleration'
            ).value,
        )
        self.flight_guidance = FlightGuidanceCore(
            flight_altitude=self.get_parameter('flight_altitude').value,
            takeoff_tolerance=self.get_parameter('takeoff_tolerance').value,
            takeoff_settle_time=self.get_parameter(
                'takeoff_settle_time'
            ).value,
            takeoff_maximum_vertical_speed=self.get_parameter(
                'takeoff_maximum_vertical_speed'
            ).value,
            takeoff_maximum_vertical_acceleration=self.get_parameter(
                'takeoff_maximum_vertical_acceleration'
            ).value,
            takeoff_maximum_horizontal_acceleration=self.get_parameter(
                'takeoff_maximum_horizontal_acceleration'
            ).value,
            takeoff_horizontal_start_height=self.get_parameter(
                'takeoff_horizontal_start_height'
            ).value,
            takeoff_horizontal_full_height=self.get_parameter(
                'takeoff_horizontal_full_height'
            ).value,
            follow_distance=self.get_parameter('follow_distance').value,
            follow_position_gain=self.get_parameter(
                'follow_position_gain'
            ).value,
            altitude_velocity_gain=self.get_parameter(
                'altitude_velocity_gain'
            ).value,
            maximum_horizontal_speed=self.get_parameter(
                'guidance_maximum_horizontal_speed'
            ).value,
            maximum_vertical_speed=self.get_parameter(
                'maximum_vertical_speed'
            ).value,
            maximum_horizontal_acceleration=self.get_parameter(
                'guidance_maximum_horizontal_acceleration'
            ).value,
            maximum_vertical_acceleration=self.get_parameter(
                'maximum_vertical_acceleration'
            ).value,
            sea_surface_z=self.get_parameter('sea_surface_z').value,
            reserve_clearance=self.get_parameter('reserve_clearance').value,
            safety_response_delay=self.get_parameter(
                'safety_response_delay'
            ).value,
            vertical_braking_acceleration=self.get_parameter(
                'vertical_braking_acceleration'
            ).value,
            approach_contact_clearance=self.get_parameter(
                'approach_contact_clearance'
            ).value,
            approach_closing_speed=self.get_parameter(
                'approach_closing_speed'
            ).value,
            approach_preparation_clearance=self.get_parameter(
                'approach_preparation_clearance'
            ).value,
            approach_horizon=self.get_parameter('approach_horizon').value,
            approach_response_delay=self.get_parameter(
                'approach_response_delay'
            ).value,
            control_dt=1.0 / control_rate,
        )

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        from uav_usv_interfaces.msg import UavState
        # Native bracket interpolation needs intermediate samples even when a
        # validation callback briefly occupies the sole executor. KEEP_LAST(1)
        # discards that physical history before uav_callback can record it.
        # Retention does not alter stamps or the 125 ms/50 ms validity checks.
        navigation_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=16,
        )
        self.uav_sub = self.create_subscription(
            UavState,
            '/navigation/uav_state',
            self.uav_callback,
            navigation_qos,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback,
            state_qos,
        )
        self.target_state_sub = self.create_subscription(
            TargetState,
            self.target_state_topic,
            self.target_state_callback,
            state_qos,
        )
        self.bearing_sub = self.create_subscription(
            TargetBearing, '/perception/front/target_bearing',
            self.bearing_callback, 10,
        )
        self.observation_sub = self.create_subscription(
            TargetObservation, '/perception/front/target_observation',
            self.observation_callback, 10,
        )
        self.trajectory_sub = self.create_subscription(
            InterceptTrajectory,
            '/planning/intercept_trajectory',
            self.trajectory_callback,
            reliable_qos,
        )
        self.prediction_sub = self.create_subscription(
            TargetPrediction,
            '/planning/target_prediction',
            self.prediction_callback,
            state_qos,
        )
        self.planner_diagnostic_sub = self.create_subscription(
            PlannerDiagnostic,
            '/planning/diagnostic',
            self.planner_diagnostic_callback,
            reliable_qos,
        )
        self.mission_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_callback,
            reliable_qos,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )
        self.reference_pub = self.create_publisher(
            TrajectorySetpoint,
            '/control/reference',
            reliable_qos,
        )
        self.diagnostic_pub = self.create_publisher(
            ControllerDiagnostic,
            '/control/diagnostic',
            reliable_qos,
        )
        self.flight_ready_pub = self.create_publisher(
            Bool,
            '/simulation/impact/flight_ready',
            latched_qos,
        )
        self.takeoff_complete_pub = self.create_publisher(
            Bool,
            '/control/takeoff_complete',
            latched_qos,
        )
        self.far_guidance_pub = self.create_publisher(
            Bool,
            '/control/far_guidance_available',
            reliable_qos,
        )
        self.timer = self.create_timer(1.0 / control_rate, self.timer_callback)

        self.latest_state = None
        self.latest_prediction = None
        self.terminal_execution = None
        self.latest_target_state = None
        self.latest_kf_message = None
        self.latest_body_bearing = None
        self.latest_center_bearing = None
        self.visibility_decision = None
        self.search_state = 'GROUND_HOLD'
        self.search_hold_position = None
        self.safe_recovery_latched = False
        self.bearing_approach_active = False
        self.intercept_requested = False
        self.yaw_owner = 'HOLD'
        self.search_yaw_rate_command = 0.0
        self.current_heading = 0.0
        self.latest_planner_diagnostic = None
        self.vehicle_status_stamp = None
        self.offboard_active = False
        self.vehicle_armed = False
        self.pre_flight_checks_pass = False
        self.preflight_wait_announced = False
        self.preflight_checks_announced = False
        self.mission_id = 0
        self.mission_state = MissionState.INIT
        self.mission_state_name = 'INIT'
        self.last_rejection = TrajectoryRejectReason.NONE
        self.last_rejection_subreason = ''
        self.pending_trajectory = None
        self.last_callback_time = 0.0
        self.control_counter = 0
        self.preflight_counter = 0
        self.last_timer_stamp = None
        self.flight_ready = False
        self.takeoff_complete_sent = False
        self.last_target_yaw = None
        self.terminal_hold_position = None
        self.terminal_mode_latched = False
        self.p7_image = self.p7_result = None
        self.declare_parameter('p7_capture_enabled', False)
        self.p7_capture_enabled = bool(self.get_parameter('p7_capture_enabled').value)
        if self.p7_guidance_enabled:
            from .p7_visual_guidance import GuidanceConfig, P7VisualGuidance
            parameters = vars(GuidanceConfig()).copy()
            for key, value in parameters.items():
                self.declare_parameter('p7_'+key, value)
                parameters[key] = float(self.get_parameter('p7_'+key).value)
            config = GuidanceConfig(**parameters)
            if (config.maximum_speed > self.flight_guidance.maximum_horizontal_speed
                    or config.maximum_acceleration
                    > self.flight_guidance.maximum_horizontal_acceleration
                    or config.maximum_yaw_rate > self.max_observation_yaw_rate):
                raise ValueError('P7_LIMIT_EXCEEDS_EXISTING_PHYSICAL_LIMIT')
            self.p7_controller = P7VisualGuidance(config)
            self.p7_diagnostic_pub = self.create_publisher(String, '/control/p7_guidance', 10)
        self.get_logger().info(
            'Trajectory tracker ready | rate='
            f'{control_rate:.1f} Hz | plan age<='
            f'{self.tracker.maximum_plan_age:.3f} s | vxy<='
            f'{self.tracker.maximum_horizontal_speed:.1f} m/s | vz<='
            f'{self.tracker.maximum_vertical_speed:.1f} m/s'
        )

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def uav_callback(self, message):
        received = self._ros_seconds()
        try:
            self.latest_state = tracker_state_from_navigation(
                message, received,
            )
        except (ValueError, TypeError):
            self.latest_state = None
            return
        self.follow_input_receipts = dict(
            getattr(self, 'follow_input_receipts', {}), navigation=received)
        if hasattr(self, 'follow_navigation_history'):
            self.follow_navigation_history.append(dict(
                valid=True, stamp=self.latest_state.stamp,
                p=self.latest_state.position, v=self.latest_state.velocity))
        heading = float(getattr(message, 'heading', math.nan))
        if math.isfinite(heading):
            self.current_heading = heading
        self._process_pending_trajectory()

    def _follow_native_epoch(self, message):
        """Observe native sample resets independently; never replace original navigation state."""
        try:
            self.follow_clock.observe(self._ros_seconds(), int(message.native_timestamp_sample))
        except (ValueError, TypeError):
            pass

    def _publish_follow_epoch(self):
        """Actual receiver owns this read-only heartbeat; it grants no holding or bridge."""
        from uav_usv_interfaces.msg import FollowReceiverState
        from uav_control.controllers.follow_transport import stamp
        context = self._follow_context()
        self._update_follow_admission(context)
        message = FollowReceiverState()
        message.stamp, message.receiver = stamp(context.now), 'trajectory_tracker_node'
        message.receiver_boot_id = self.follow_clock.boot_id
        message.clock_generation, message.mission_id = context.generation, context.mission_id
        message.prediction_sequence_id = context.prediction_id
        receiver = self.follow_receiver
        message.active_plan_id = receiver.active.plan_id if receiver.active else 0
        message.pending_plan_id = receiver.pending.plan_id if receiver.pending else 0
        gate = getattr(receiver, 'admission', None)
        if gate:
            message.research_state = gate.state
            message.entry_allowed = gate.allowed(context.now)
            message.follow_converged = gate.converged(context.now)
            message.active_lease_deadline = receiver.deadline if receiver.active else 0.
            message.recovery_until = max(0., gate.recover_until)
            message.entry_attempts, message.valid_probe_count = gate.entries, len(gate.probes)
            message.last_fallback_reason = (gate.loss or {}).get('reason', '')
            message.entry_inhibition_reason = gate.inhibition_reason(context.now)
        self.follow_epoch_pub.publish(message)

    def _update_follow_admission(self, context):
        """Only this receiver compares native navigation and KF at their common epoch."""
        from uav_control.guidance.follow_admission import FollowAdmission, follow_settling
        from uav_control.guidance.follow_contract import execution_context_errors
        receiver = self.follow_receiver
        gate = getattr(receiver, 'admission', None)
        if gate is None:
            return ''
        identity = (context.receiver_boot_id, context.generation, context.mission_id)
        if self.follow_admission_identity != identity:
            if receiver.active or receiver.pending:
                self._revoke_nominal_follow('RECEIVER_IDENTITY_CHANGED')
            receiver.admission = gate = FollowAdmission(gate.config)
            self.follow_admission_identity = identity
        self.follow_metrics = follow_settling(
            self.follow_navigation_history, self.follow_target_history, context.now,
            self.flight_guidance.follow_distance, self.flight_guidance.flight_altitude,
            admission_config=gate.config,
            reference_mode=getattr(self, 'follow_reference_mode', 'target'))
        status_stamp = getattr(self, 'vehicle_status_stamp', None)
        context_faults = execution_context_errors(context)
        legal = (not context_faults
                 and status_stamp is not None
                 and 0 <= context.now-status_stamp <= self.vehicle_status_timeout)
        reason = gate.update(context.now, self.follow_metrics, legal, active=bool(receiver.active))
        if receiver.pending and not legal:
            reason = reason or 'EXECUTION_CONTEXT_INVALID'
        # Heartbeat and proposal callbacks are also observations of execution legality.
        # Consume loss here so a healthy sample before the next control tick cannot erase it.
        if reason and (receiver.active or receiver.pending):
            reason = context_faults[0] if context_faults else reason
            self._revoke_nominal_follow(reason)
        return reason

    def _follow_context(self):
        """Build local receiver context; never accept a planner-supplied clock or boundary."""
        from uav_control.guidance.follow_contract import ReceiverContext
        now = self._ros_seconds()
        self.follow_generation = self.follow_clock.observe(now)
        prediction = self.latest_prediction
        from uav_control.guidance.follow_time_contract import input_time_contract
        contract = getattr(self, 'follow_time_contract', input_time_contract())
        navigation_fresh = bool(self.latest_state and 0 <= now-self.latest_state.stamp <= .125)
        kf = getattr(self, 'latest_kf_message', None)
        target_fresh = fresh_flight_target(
            kf, now, min(.125, self.maximum_state_age), self.expected_frame_id,
            source_max_age=contract.observation_age) is not None
        observation_stamp = self.visibility.last_valid_observation_stamp
        observation_fresh = bool(observation_stamp is not None
                                 and 0 <= now-observation_stamp
                                 <= self.visibility.config.target_lock_max_age)
        source = observation = generated = until = None
        try:
            source, observation, generated, until = (
                _stamp_seconds(getattr(prediction, name)) for name in (
                    'source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until'))
            prediction_fresh = bool(prediction is not None and prediction.valid
                                    and int(prediction.mission_id) == self.mission_id
                                    and 0 < observation <= source <= generated <= now < until
                                    and now-source <= contract.prediction_age
                                    and now-observation <= contract.observation_age
                                    and until <= observation+contract.observation_age+1e-7)
        except (AttributeError, ValueError, TypeError):
            prediction_fresh = False
        self.follow_input_status = dict(
            now=now, navigation_stamp=self.latest_state.stamp if self.latest_state else None,
            target_stamp=_stamp_seconds(kf.stamp) if kf else None,
            target_source_stamp=_stamp_seconds(kf.source_stamp) if kf else None,
            prediction_source=source, prediction_observation=observation,
            prediction_generated=generated, prediction_valid_until=until,
            observation_stamp=observation_stamp, observation_fresh=observation_fresh,
            callback_receipts=dict(getattr(self, 'follow_input_receipts', {})))
        return ReceiverContext(
            now, self.mission_id, self.follow_generation,
            int(prediction.sequence_id) if prediction is not None else 0,
            self.mission_state == MissionState.FOLLOW,
            self.offboard_active, self.vehicle_armed,
            bool(self.visibility_decision and (self.visibility_decision.locked
                 or getattr(self.visibility_decision, 'visual_locked', False))),
            receiver_boot_id=self.follow_clock.boot_id, navigation_fresh=navigation_fresh,
            target_fresh=target_fresh, prediction_fresh=prediction_fresh,
            recovering=self.safe_recovery_latched, observation_fresh=observation_fresh)

    def follow_trajectory_callback(self, message):
        """Real receiver rejection ACK. Qualification failure never mutates original control."""
        from uav_control.guidance.follow_profile import CycleProfile
        received_monotonic = time.perf_counter()
        with CycleProfile(int(message.plan_id)) as profile:
            response, context = self._follow_proposal_response(message)
        response.receiver_compute_seconds = time.perf_counter()-received_monotonic
        prepared = time.perf_counter()
        requested = getattr(self, 'follow_minco_requested', False)
        publish_start = self._ros_seconds() if requested else None
        self.follow_ack_pub.publish(response)
        published = time.perf_counter()
        publish_end = self._ros_seconds() if requested else None
        if response.state == 'ACCEPTED' and getattr(self, 'follow_minco_requested', False):
            self._schedule_follow_activation()
        if getattr(self, 'follow_minco_requested', False):
            self.follow_debug_pub.publish(String(data=json.dumps(dict(
                event='proposal_received', stamp=context.now, plan=int(message.plan_id),
                parent=int(message.parent_plan_id), state=response.state,
                reasons=list(response.reasons),
                receiver_compute_seconds=response.receiver_compute_seconds,
                receive_monotonic=received_monotonic, ack_monotonic=prepared,
                ack_publish_start_ros=publish_start, ack_publish_end_ros=publish_end,
                ack_publish_end_monotonic=published, validation_profile=profile.report(),
                math_runtime=getattr(self, 'follow_math_runtime', None)))))

    def _follow_proposal_response(self, message):
        """Independently validate with policy-specific inputs before refreshing local legality."""
        from uav_control.controllers.follow_transport import curve_from_message, ack_message
        from uav_control.guidance.follow_contract import FollowAck
        from uav_control.guidance.follow_profile import stage
        from numpy.linalg import LinAlgError
        context = self._follow_context()
        if getattr(self, 'follow_minco_requested', False):
            self._update_follow_admission(context)
        try:
            with stage('proposal_conversion'):
                candidate = curve_from_message(message)
            validated_prediction_id = None
            if getattr(self, 'follow_minco_requested', False):
                from uav_control.guidance.follow_minco_execution import (
                    nominal_curve, nominal_attitude,
                )
                from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
                endpoints = self.get_publishers_info_by_topic('/planning/follow_trajectory')
                if len(endpoints) != 1 or endpoints[0].node_name != 'p4_follow_planner_node':
                    raise ValueError('PLANNER_AUTHORITY_CONFLICT')
                if self.follow_stable_ticks < 10:
                    raise ValueError('FOLLOW_NOT_STABLE')
                with stage('proposal_model_conversion'):
                    prediction = self.p6_runtime.execution_prediction(
                        prediction_from_message(self.latest_prediction))
                validated_prediction_id = int(self.latest_prediction.sequence_id)
                with stage('proposal_attitude'):
                    attitude_ok = nominal_attitude(candidate, self.follow_model)
                with stage('proposal_geometry'):
                    geometry_ok = (attitude_ok and nominal_curve(
                        candidate, prediction, self.follow_model, candidate.start, candidate.end))
                if not geometry_ok:
                    raise ValueError('RECEIVER_NOMINAL_REVALIDATION_FAILED')
                context = self._follow_context()
                self._update_follow_admission(context)
            if getattr(self, 'follow_minco_requested', False):
                def final_context():
                    nonlocal context
                    with stage('proposal_final_context'):
                        context = self._follow_context()
                        self._update_follow_admission(context)
                    return context
                with stage('proposal_receiver_commit'):
                    ack = self.follow_receiver.propose(
                        candidate, context, validated_prediction_id=validated_prediction_id,
                        probe=message.reason == 'SHADOW_PROBE', context_provider=final_context)
            else:
                ack = self.follow_receiver.propose(candidate, context)
        except (ValueError, TypeError, AttributeError, OverflowError, FloatingPointError,
                LinAlgError) as error:
            gate = getattr(self.follow_receiver, 'admission', None)
            if gate is not None and message.reason == 'SHADOW_PROBE':
                gate.probe(context.now, int(message.plan_id), False)
            ack = FollowAck(int(message.plan_id), int(message.mission_id),
                            int(message.clock_generation), context.now, 'REJECTED',
                            (str(error) or 'INVALID_COEFFICIENTS',),
                            receiver_boot_id=context.receiver_boot_id,
                            planner_boot_id=getattr(message, 'planner_boot_id', ''))
        response = ack_message(ack)
        if getattr(self, 'follow_minco_requested', False) and self.follow_receiver.active:
            response.control_owner = 'MINCO_FOLLOW'
        return response, context

    def _schedule_follow_activation(self):
        """Wake the sole control path at an accepted start; never extend its window."""
        previous = getattr(self, 'follow_activation_timer', None)
        if previous is not None:
            previous.cancel()
            self.destroy_timer(previous)
            self.follow_activation_timer = None
        curve = self.follow_receiver.pending
        if curve is None:
            return
        now = self._ros_seconds()
        last = getattr(self, 'last_timer_stamp', None)
        minimum_dt = max(0., .001-(now-last)) if last is not None and now >= last else 0.

        def activate():
            if getattr(self, 'follow_activation_timer', None) is not timer:
                return
            timer.cancel()
            self.destroy_timer(timer)
            self.follow_activation_timer = None
            if self.follow_receiver.pending is not curve:
                return
            stamp = self._ros_seconds()
            last_control = getattr(self, 'last_timer_stamp', None)
            if (stamp < curve.start
                    or (last_control is not None and 0 <= stamp-last_control < .001)):
                self._schedule_follow_activation()
                return
            # The normal callback independently rechecks context, full prefix,
            # actual completion clock and command shaping before any ACTIVE.
            self.timer_callback()

        timer = self.create_timer(max(1e-9, curve.start-now, minimum_dt), activate)
        self.follow_activation_timer = timer

    def _revoke_nominal_follow(self, reason):
        """Every loss of the execution context emits a real revocation ACK, including nav loss."""
        if not getattr(self, 'follow_minco_requested', False):
            return
        from uav_control.controllers.follow_transport import ack_message
        context = self._follow_context()
        receiver = self.follow_receiver
        lost = receiver.active or receiver.pending
        curves = (receiver.pending, receiver.active)
        receiver.pending = receiver.active = None
        for curve in curves:
            if curve is not None:
                ack = receiver._ack(curve, context, 'REVOKED', (reason,))
                self.follow_ack_pub.publish(ack_message(ack))
        if lost is not None and getattr(receiver, 'admission', None):
            receiver.admission.lost(context.now, reason, lost)
        self.follow_stable_ticks = 0

    def _nominal_follow_command(self, current, now, dt):
        """Finite nominal reference lease; measured-feedback shaping and visual yaw stay local."""
        from uav_control.controllers.follow_transport import ack_message
        from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
        from uav_control.guidance.follow_minco_execution import nominal_curve, reference_velocity
        from uav_control.guidance.follow_contract import execution_context_errors
        receiver = self.follow_receiver
        started = time.perf_counter()
        started_cpu = time.thread_time()
        context = self._follow_context()
        self._update_follow_admission(context)
        admitted = time.perf_counter()
        self.follow_stable_ticks = self.follow_stable_ticks+1 if context.visible else 0
        nominal, prefix_details = {}, {}
        converted = None
        try:
            prediction = self.p6_runtime.execution_prediction(
                prediction_from_message(self.latest_prediction))
            converted = time.perf_counter()
            for curve in (receiver.active, receiver.pending):
                if curve and now < min(curve.end, curve.start+receiver.lease_seconds):
                    detail = dict(reason='CONTEXT_NOT_READY', visible=context.visible)
                    nominal[curve.plan_id] = (
                        not execution_context_errors(context) and nominal_curve(
                            curve, prediction, self.follow_model, max(now, curve.start),
                            min(curve.end, curve.start+receiver.lease_seconds), detail))
                    prefix_details[curve.plan_id] = detail
        except (ValueError, TypeError, AttributeError):
            nominal = {}
            if converted is None:
                converted = time.perf_counter()
        checked = time.perf_counter()
        # Recheck the actual clock/freshness after validation; computing cannot extend a lease.
        context = self._follow_context()
        ack = receiver.tick(context, nominal_valid=nominal)
        curve = receiver.active
        debug = dict(stamp=context.now, validation_seconds=time.perf_counter()-started,
                     validation_cpu_seconds=time.thread_time()-started_cpu,
                     validation_stages=dict(admission=admitted-started,
                                            prediction_conversion=converted-admitted,
                                            prefix_geometry=checked-converted,
                                            final_context_and_tick=time.perf_counter()-checked),
                     prefix_checks=prefix_details, prediction=context.prediction_id,
                     plan=curve.plan_id if curve else 0, state_stamp=self.latest_state.stamp,
                     active_start=curve.start if curve else None,
                     lease_deadline=receiver.deadline if curve else None,
                     initial=bool(ack and ack.state == 'ACTIVE' and not curve.parent_id)
                     if curve else False)
        if curve:
            sample = curve.sample(max(curve.start, self.latest_state.stamp))
            debug.update(reference=list(sample), actual_position=list(current.position),
                         actual_velocity=list(current.velocity), parent=curve.parent_id,
                         reference_position_error=list(
                sample[i]-current.position[i] for i in range(3)),
                reference_velocity_error=list(sample[i+3]-current.velocity[i] for i in range(3)),
                previous_command=self.tracker.previous_command_velocity)
        debug['follow_metrics'] = getattr(self, 'follow_metrics', {})
        gate = getattr(receiver, 'admission', None)
        debug['research_state'] = gate.state if gate else 'LEGACY_NOMINAL'
        debug['recovery'] = gate.loss if gate else None
        debug['entry_evidence'] = receiver.entry_evidence
        debug['execution_context'] = dict(schema_version=2,
                                          following=context.following, visible=context.visible,
                                          offboard=context.offboard, armed=context.armed,
                                          navigation_fresh=context.navigation_fresh,
                                          target_fresh=context.target_fresh,
                                          prediction_fresh=context.prediction_fresh,
                                          observation_fresh=context.observation_fresh,
                                          recovering=context.recovering)
        debug['input_freshness'] = dict(getattr(self, 'follow_input_status', {}))
        self.follow_cycle_debug = debug
        events = receiver.drain_events()
        for event in events:
            response = ack_message(event)
            response.control_owner = 'MINCO_FOLLOW' if receiver.active else 'ORIGINAL_FOLLOW'
            response.replaced = event.state == 'ACTIVE'
            self.follow_ack_pub.publish(response)
        curve = receiver.active
        if curve is None:
            return None
        try:
            desired = reference_velocity(curve, context.now, self.latest_state.stamp,
                                         current.position,
                                         self.flight_guidance.follow_position_gain)
            previous_velocity = self.flight_guidance.previous_velocity
            command = self.flight_guidance._velocity_command(
                FlightKinematicState(current.position, current.velocity), desired, dt, False, True)
            if getattr(self, 'follow_minco_acceleration_feedforward', False):
                from uav_control.guidance.follow_minco_execution import (
                    reference_acceleration, shaped_acceleration,
                )
                acceleration = reference_acceleration(
                    curve, context.now, self.latest_state.stamp, current.velocity,
                    self.flight_guidance.follow_position_gain,
                    self.follow_model.config.maximum_horizontal_acceleration)
                acceleration = shaped_acceleration(
                    acceleration, desired, command.velocity, previous_velocity, dt,
                    min(self.follow_model.config.maximum_horizontal_acceleration,
                        self.flight_guidance.maximum_horizontal_acceleration),
                    self.flight_guidance.maximum_horizontal_speed)
                command = replace(command, acceleration=acceleration)
            return command
        except (ValueError, TypeError, OverflowError):
            self._revoke_nominal_follow('REFERENCE_COMMAND_INVALID')
            return None

    def vehicle_status_callback(self, message):
        self.vehicle_status_stamp = self._ros_seconds()
        self.offboard_active = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        self.vehicle_armed = (
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        self.pre_flight_checks_pass = bool(message.pre_flight_checks_pass)

    def target_state_callback(self, message):
        now = self._ros_seconds()
        if message.valid and message.frame_id == self.expected_frame_id:
            previous = self.latest_kf_message
            stamp = _stamp_seconds(message.stamp)
            source = _stamp_seconds(message.source_stamp)
            if (0.0 < source <= stamp <= now and previous is not None
                    and (stamp < _stamp_seconds(previous.stamp)
                         or source < _stamp_seconds(previous.source_stamp))):
                return
        original_target = fresh_flight_target(
            message, now, self.maximum_state_age, self.expected_frame_id)
        converted = original_target
        contract = getattr(self, 'follow_time_contract', None)
        experimental_follow = bool(
            contract is not None and contract.profile != 'baseline_125'
            and getattr(self, 'follow_minco_requested', False)
            and getattr(self, 'mission_state', None) == MissionState.FOLLOW)
        if experimental_follow:
            converted = fresh_flight_target(
                message, now, min(.125, self.maximum_state_age), self.expected_frame_id,
                source_max_age=contract.observation_age)
        if hasattr(self, 'follow_target_history'):
            self.follow_target_history.append(dict(
                valid=converted is not None,
                stamp=_stamp_seconds(message.stamp), source=_stamp_seconds(message.source_stamp),
                p=tuple(float(getattr(message.position, a)) for a in 'xyz'),
                v=tuple(float(getattr(message.velocity, a)) for a in 'xyz')))
        if converted is None:
            self.latest_kf_message = None
            self.latest_target_state = None
            if getattr(self, 'follow_minco_requested', False):
                self._update_follow_admission(self._follow_context())
            return
        previous = self.latest_kf_message
        if (previous is not None and _stamp_seconds(message.stamp)
                < _stamp_seconds(previous.stamp)):
            return
        self.latest_kf_message = message
        # The experimental cache feeds independent MINCO admission/context only.
        # Original FOLLOW fallback/takeoff/intercept keep their source-age policy.
        self.latest_target_state = original_target
        self.follow_input_receipts = dict(
            getattr(self, 'follow_input_receipts', {}), target=now)

    def bearing_callback(self, message):
        stamp = _stamp_seconds(message.stamp)
        if stamp <= 0.0 and _stamp_seconds(message.raw_stamp) > 0.0:
            return
        if stamp <= 0.0:
            if getattr(self, 'terminal_execution', None) is not None:
                self.terminal_execution = None
                self.tracker.active_trajectory = None
                self.pending_trajectory = None
                self.safe_recovery_latched = True
            self.visibility.reset()
            self.latest_body_bearing = None
            self.latest_center_bearing = None
            self.latest_kf_message = None
            self.latest_target_state = None
            self.latest_prediction = None
            return
        accepted = self.visibility.observe(
            stamp, float(message.bearing), bool(message.valid),
            self._ros_seconds(),
        )
        if (accepted and message.valid
                and self.visibility.last_valid_bearing_stamp == stamp):
            body_bearing = float(getattr(message, 'body_bearing', message.bearing))
            self.latest_body_bearing = (stamp, body_bearing)

    def observation_callback(self, message):
        stamp = _stamp_seconds(message.stamp)
        now = self._ros_seconds()
        self.visibility.mark_observation(
            stamp,
            bool(message.valid and message.frame_id == self.expected_frame_id),
            now,
        )
        previous = getattr(self, 'latest_center_bearing', None)
        if previous is not None and 0.0 < stamp < previous[0]:
            return
        ray = tuple(float(getattr(message, 'center_camera_' + a, math.nan)) for a in 'xyz')
        trusted = (
            message.valid and message.frame_id == self.expected_frame_id
            and 0.0 <= measurement_age(now, stamp) <= self.maximum_state_age
            and all(math.isfinite(v) for v in ray) and ray[0] > 0.0
        )
        self.latest_center_bearing = (stamp, math.atan2(-ray[1], ray[0])) if trusted else None
        if getattr(self, 'p7_guidance_enabled', False):
            from .p7_visual_guidance import image_errors
            self.p7_image = (stamp, *image_errors(ray)) if trusted else None

    def prediction_callback(self, message):
        if not (message.valid and message.frame_id == self.expected_frame_id
                and message.source == 'tracking' and message.samples):
            self.latest_prediction = None
            return
        previous = self.latest_prediction
        if (previous is not None and message.mission_id == previous.mission_id
                and message.sequence_id < previous.sequence_id):
            return
        self.latest_prediction = message
        self.follow_input_receipts = dict(
            getattr(self, 'follow_input_receipts', {}), prediction=self._ros_seconds())

    def planner_diagnostic_callback(self, message):
        self.latest_planner_diagnostic = message

    def mission_callback(self, message):
        new_mission_id = int(message.mission_id)
        if new_mission_id != self.mission_id:
            if getattr(self, 'p7_guidance_enabled', False):
                self.p7_controller.reset()
                self.p7_image = self.p7_result = None
            self.terminal_execution = None
            self.tracker.reset()
            self.flight_guidance.reset()
            self.last_rejection = TrajectoryRejectReason.NONE
            self.last_rejection_subreason = ''
            self.pending_trajectory = None
            self.takeoff_complete_sent = False
            self.last_target_yaw = None
            self.terminal_hold_position = None
            self.terminal_mode_latched = False
            self.visibility.reset()
            self.visibility_decision = None
            self.latest_body_bearing = None
            self.latest_center_bearing = None
            self.latest_kf_message = None
            self.latest_target_state = None
            self.latest_prediction = None
            self.search_hold_position = None
            self.safe_recovery_latched = False
            self.bearing_approach_active = False
        previous_state = getattr(self, 'mission_state', MissionState.INIT)
        self.mission_id = new_mission_id
        self.intercept_requested = bool(getattr(message, 'intercept_requested', False))
        self.mission_state = int(message.state)
        self.mission_state_name = str(message.state_name) or 'INIT'
        if (getattr(self, 'p7_guidance_enabled', False)
                and self.mission_state == MissionState.FAR_GUIDANCE
                and previous_state != MissionState.FAR_GUIDANCE):
            previous_velocity = self.tracker.previous_command_velocity
            if previous_velocity is None and self.latest_state is not None:
                previous_velocity = self.latest_state.velocity
            self._revoke_nominal_follow('P7_GUIDANCE_ENTERED')
            self.pending_trajectory = self.tracker.active_trajectory = None
            self.terminal_execution = None
            self.terminal_mode_latched = False
            self.p7_controller.reset(previous_velocity,
                                     getattr(self, 'search_yaw_rate_command', 0.))
            self.p7_result = None
        if self.mission_state == MissionState.TERMINAL_MINCO:
            self.terminal_mode_latched = True
        if self.mission_state in self.TERMINAL_STATES:
            self.terminal_execution = None
            self.tracker.reset()
            self.pending_trajectory = None
            self.terminal_mode_latched = False
            if self.latest_state is not None:
                safe_z = min(
                    self.latest_state.position[2],
                    self.tracker.sea_surface_z
                    - self.tracker.reserve_clearance,
                )
                self.terminal_hold_position = (
                    self.latest_state.position[0],
                    self.latest_state.position[1],
                    safe_z,
                )

    def _candidate_endpoint(self, trajectory):
        if self.latest_prediction is None:
            raise ValueError('prediction unavailable')
        if int(self.latest_prediction.mission_id) != self.mission_id:
            raise ValueError('prediction mission mismatch')
        if (measurement_age(self._ros_seconds(), _stamp_seconds(
                self.latest_prediction.observation_stamp))
                > self.maximum_state_age):
            raise ValueError('prediction measurement is stale')
        return prediction_endpoint_from_message(
            self.latest_prediction,
            (
                trajectory.contact_stamp
                if trajectory.contact_stamp > 0.0
                else trajectory.source_stamp + trajectory.duration
            ),
        )

    def _evaluate_trajectory(self, trajectory):
        now = self._ros_seconds()
        committed = self._terminal_execution_active(now)
        fresh_replacement = not committed or bool(
            self._prediction_fresh(now)
            and self.latest_state is not None
            and 0. <= now - self.latest_state.stamp <= self.maximum_state_age
            and fresh_flight_target(self.latest_kf_message, now, self.maximum_state_age,
                                    self.expected_frame_id) is not None
            and self.visibility_decision and self.visibility_decision.locked
            and not self.safe_recovery_latched)
        if committed and not fresh_replacement:
            return TrajectoryRejectReason.TERMINAL_COMMITTED
        if now - trajectory.source_stamp > self.tracker.maximum_plan_age:
            return TrajectoryRejectReason.SOURCE_STALE
        if (trajectory.target_state_source != 'tracking'
                or measurement_age(now, trajectory.observation_stamp)
                > self.maximum_state_age
                or fresh_flight_target(
                    self.latest_kf_message, now, self.maximum_state_age,
                    self.expected_frame_id,
                ) is None
                or self.visibility.consecutive_lost_frames
                >= self.visibility.config.target_loss_frames
                or not self.visibility_decision
                or not self.visibility_decision.locked
                or self.safe_recovery_latched):
            return TrajectoryRejectReason.PREDICTION_MISMATCH
        if self.latest_state is None or self.latest_prediction is None:
            return TrajectoryRejectReason.PREDICTION_MISMATCH
        if int(self.latest_prediction.mission_id) != self.mission_id:
            return TrajectoryRejectReason.PREDICTION_MISMATCH
        # Geometry uses the physical sample epoch; execution authority must
        # satisfy the mission's remaining-time gate at the control epoch.
        if now >= trajectory.valid_until:
            return TrajectoryRejectReason.EXPIRED
        if trajectory.valid_until - now < self.tracker.minimum_remaining_time:
            return TrajectoryRejectReason.INSUFFICIENT_REMAINING_TIME
        try:
            endpoint = self._candidate_endpoint(trajectory)
        except (TypeError, ValueError):
            return TrajectoryRejectReason.TARGET_ENDPOINT_MISMATCH
        rejection = self.tracker.accept(
            trajectory,
            self.latest_state,
            self.mission_id,
            prediction_sequence_id=None,
            target_endpoint=endpoint,
            maximum_position_error=(
                self.terminal_replacement_position_error
                if self.mission_state == MissionState.TERMINAL_MINCO
                else None
            ),
        )
        if (
            rejection == TrajectoryRejectReason.NONE
            and trajectory.terminal_mode
        ):
            self.terminal_mode_latched = True
        if (committed and rejection == TrajectoryRejectReason.NONE
                and self.tracker.last_replacement_performed):
            # Only a completely accepted fresh plan grants a new deadline.
            # Rejected/stale candidates leave the old commitment untouched.
            self.terminal_execution = None
            self.visibility_decision = replace(self.visibility_decision, state='TARGET_LOCK')
            self.search_state = 'TARGET_LOCK'
        return rejection

    def _publish_candidate_result(self, pending, rejection, started):
        trajectory = pending.trajectory
        source_age = (
            self.latest_state.stamp - trajectory.source_stamp
            if self.latest_state is not None else math.nan
        )
        subreason = ''
        if rejection == TrajectoryRejectReason.SOURCE_STALE:
            subreason = (
                'FUTURE_TRAJECTORY_START'
                if source_age < 0.0 else 'TRAJECTORY_START_TOO_OLD'
            )
        self.last_rejection = rejection
        self.last_rejection_subreason = subreason
        self.last_callback_time = time.perf_counter() - started
        self._publish_diagnostic(
            self._ros_seconds(),
            None,
            ('PLAN_ACCEPTED' if rejection == TrajectoryRejectReason.NONE
             else 'PLAN_REJECTED'),
            self.last_callback_time,
            attempted_plan_id=trajectory.plan_id,
            candidate=pending,
        )

    def _process_pending_trajectory(self):
        pending = getattr(self, 'pending_trajectory', None)
        if pending is None or self.latest_state is None:
            return
        trajectory = pending.trajectory
        if self.latest_state.stamp < trajectory.source_stamp - 1e-9:
            if self._ros_seconds() - trajectory.source_stamp <= (
                self.tracker.maximum_plan_age
            ):
                return
            rejection = TrajectoryRejectReason.SOURCE_STALE
        else:
            rejection = self._evaluate_trajectory(trajectory)
        self.pending_trajectory = None
        self._publish_candidate_result(
            pending, rejection, time.perf_counter()
        )

    def trajectory_callback(self, message):
        if getattr(self, 'p7_guidance_enabled', False):
            return
        started = time.perf_counter()
        self.tracker.last_replacement_performed = False
        receipt_stamp = self._ros_seconds()
        attempted_plan_id = int(message.plan_id)
        rejection = TrajectoryRejectReason.INVALID_TRAJECTORY
        try:
            trajectory = trajectory_from_message(message)
            trajectory = trajectory_for_mission(
                trajectory,
                self.mission_state,
            )
        except (TypeError, ValueError):
            rejection = TrajectoryRejectReason.INVALID_TRAJECTORY
        else:
            pending = PendingTrajectory(
                trajectory=trajectory,
                planner_published_stamp=_stamp_seconds(
                    message.published_stamp
                ),
                tracker_receipt_stamp=receipt_stamp,
            )
            if (
                self.latest_state is not None
                and self.latest_state.stamp < trajectory.source_stamp - 1e-9
            ):
                current = getattr(self, 'pending_trajectory', None)
                if (
                    current is None
                    or trajectory.plan_id >= current.trajectory.plan_id
                ):
                    self.pending_trajectory = pending
                    self.last_rejection = TrajectoryRejectReason.NONE
                    self.last_rejection_subreason = 'FUTURE_TRAJECTORY_START'
                    self.last_callback_time = time.perf_counter() - started
                    self._publish_diagnostic(
                        receipt_stamp, None, 'PLAN_PENDING',
                        self.last_callback_time,
                        attempted_plan_id=attempted_plan_id,
                        candidate=pending,
                    )
                    return
            rejection = self._evaluate_trajectory(trajectory)
            self._publish_candidate_result(pending, rejection, started)
            return
        self.last_rejection = rejection
        self.last_rejection_subreason = ''
        self.last_callback_time = time.perf_counter() - started
        self._publish_diagnostic(
            receipt_stamp, None, 'PLAN_REJECTED', self.last_callback_time,
            attempted_plan_id=attempted_plan_id,
        )

    def _publish_offboard_mode(self, timestamp_us, velocity_control=False):
        message = OffboardControlMode()
        message.timestamp = int(timestamp_us)
        message.position = not bool(velocity_control)
        message.velocity = bool(velocity_control)
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False
        self.offboard_pub.publish(message)

    def _remember_emitted_velocity(self, setpoint, now):
        """Share actual velocity-command history across guidance/tracker owners."""
        velocity = tuple(float(value) for value in setpoint.velocity)
        if len(velocity) == 3 and all(math.isfinite(value) for value in velocity):
            self.tracker.previous_command_velocity = velocity
            self.tracker.previous_command_stamp = now

    def _publish_vehicle_command(self, command, param1, param2=0.0):
        message = VehicleCommand()
        message.timestamp = self._timestamp_us()
        message.command = int(command)
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.vehicle_command_pub.publish(message)

    @staticmethod
    def _publish_bool(publisher, value):
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    def _request_flight_mode(self):
        retry_due = self.control_counter % self.px4_command_retry_cycles == 0
        if not retry_due:
            return
        if not self.offboard_active:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )
        if not self.vehicle_armed:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
            )

    def _publish_diagnostic(
        self,
        now,
        command,
        status,
        callback_time,
        attempted_plan_id=0,
        candidate=None,
    ):
        message = ControllerDiagnostic()
        message.target_distance = math.inf
        message.stamp = _seconds_to_time(now)
        message.mission_id = self.mission_id
        message.attempted_plan_id = int(attempted_plan_id)
        active = self.tracker.active_trajectory
        if status == 'MINCO_FOLLOW' and getattr(self, 'follow_minco_requested', False):
            follow = self.follow_receiver.active
            if follow is not None:
                message.plan_id = follow.plan_id
                message.prediction_sequence_id = follow.prediction_id
                message.source_age = max(0., now-follow.source_stamp)
                message.remaining_time = max(0., self.follow_receiver.deadline-now)
        if active is not None:
            message.plan_id = active.plan_id
            message.prediction_sequence_id = active.prediction_sequence_id
            message.trajectory_age = max(now - active.source_stamp, 0.0)
            message.source_age = message.trajectory_age
            message.remaining_time = max(active.valid_until - now, 0.0)
            message.selected_t_go = active.selected_t_go
            message.contact_stamp = _seconds_to_time(active.contact_stamp)
            message.remaining_t_go = max(active.contact_stamp - now, 0.0)
            message.terminal_mode = active.terminal_mode
            message.planned_capture_margin = active.planned_capture_margin
        if self.latest_prediction is not None:
            message.prediction_age = measurement_age(
                now, _stamp_seconds(self.latest_prediction.observation_stamp),
            )
            message.prediction_sample_age = measurement_age(
                now, _stamp_seconds(self.latest_prediction.source_stamp),
            )
        else:
            message.prediction_age = math.inf
            message.prediction_sample_age = math.inf
        message.terminal_mode = bool(
            message.terminal_mode or self.terminal_mode_latched
        )
        if (
            self.latest_target_state is not None
            and self.latest_state is not None
        ):
            relative = tuple(
                target - current
                for target, current in zip(
                    self.latest_target_state.position,
                    self.latest_state.position,
                )
            )
            message.target_distance = math.sqrt(
                sum(value * value for value in relative)
            )
        message.target_yaw = (
            float(self.last_target_yaw)
            if self.last_target_yaw is not None
            else math.nan
        )
        visibility = getattr(self, 'visibility', None)
        decision = getattr(self, 'visibility_decision', None)
        message.target_visible = bool(decision and decision.visible)
        message.target_locked = bool(
            decision and decision.locked
            and not getattr(self, 'safe_recovery_latched', False)
        )
        message.search_state = getattr(self, 'search_state', 'SAFE_WAIT')
        message.search_direction = decision.search_direction if decision else 0
        message.last_valid_observation_age = (
            measurement_age(now, visibility.last_valid_observation_stamp)
            if visibility and visibility.last_valid_observation_stamp is not None
            else math.inf
        )
        message.last_valid_image_bearing = (
            visibility.last_valid_image_bearing if visibility else math.nan
        )
        if status == 'BEARING_APPROACH':
            message.source_age = measurement_age(now, visibility.last_valid_bearing_stamp)
        message.search_yaw_rate_command = getattr(
            self, 'search_yaw_rate_command', 0.0,
        )
        message.consecutive_valid_frames = (
            visibility.consecutive_valid_frames if visibility else 0
        )
        message.consecutive_lost_frames = (
            visibility.consecutive_lost_frames if visibility else 0
        )
        kf = getattr(self, 'latest_kf_message', None)
        message.kf_state_age = (
            measurement_age(now, _stamp_seconds(kf.source_stamp))
            if kf is not None else math.inf
        )
        message.planner_source_age = (
            measurement_age(now, active.observation_stamp)
            if active is not None else math.inf
        )
        message.yaw_owner = getattr(self, 'yaw_owner', 'HOLD')
        message.status = str(status)
        message.rejection_reason = self.last_rejection.value
        message.rejection_subreason = str(getattr(
            self, 'last_rejection_subreason', ''
        ))
        if candidate is not None:
            trajectory = candidate.trajectory
            message.trajectory_start_stamp = _seconds_to_time(
                trajectory.source_stamp
            )
            message.planner_published_stamp = _seconds_to_time(
                candidate.planner_published_stamp
            )
            message.tracker_receipt_stamp = _seconds_to_time(
                candidate.tracker_receipt_stamp
            )
            if self.latest_state is not None:
                message.tracker_state_stamp = _seconds_to_time(
                    self.latest_state.stamp
                )
                message.source_age = (
                    self.latest_state.stamp - trajectory.source_stamp
                )
                message.state_age_at_receipt = (
                    candidate.tracker_receipt_stamp - self.latest_state.stamp
                )
            message.remaining_valid_time = (
                trajectory.valid_until - candidate.tracker_receipt_stamp
            )
        message.trajectory_replaced = bool(
            self.tracker.last_replacement_performed
        )
        message.handover_position_error = float(
            self.tracker.last_handover_position_error
        )
        message.handover_velocity_error = float(
            self.tracker.last_handover_velocity_error
        )
        message.handover_acceleration_error = float(
            self.tracker.last_handover_acceleration_error
        )
        message.callback_compute_time = float(callback_time)
        if command is not None:
            message.safety_state = command.safety_state
            message.safety_margin = command.safety_margin
            message.reference_position.x = command.position[0]
            message.reference_position.y = command.position[1]
            message.reference_position.z = command.position[2]
            message.command_velocity.x = command.velocity[0]
            message.command_velocity.y = command.velocity[1]
            message.command_velocity.z = command.velocity[2]
            message.command_acceleration.x = command.acceleration[0]
            message.command_acceleration.y = command.acceleration[1]
            message.command_acceleration.z = command.acceleration[2]
        else:
            message.safety_state = 'HOLD'
        self.diagnostic_pub.publish(message)

    def _prediction_fresh(self, now):
        prediction = self.latest_prediction
        return bool(
            prediction is not None and prediction.valid
            and int(prediction.mission_id) == self.mission_id
            and measurement_age(now, _stamp_seconds(prediction.observation_stamp))
            <= self.maximum_state_age)

    def _terminal_execution_active(self, now):
        execution = getattr(self, 'terminal_execution', None)
        valid = bool(
            execution is not None
            and self.mission_state == MissionState.TERMINAL_MINCO
            and execution.trajectory is self.tracker.active_trajectory
            and execution.trajectory.mission_id == self.mission_id
            and execution.entered_at <= now < execution.deadline)
        if not valid:
            self.terminal_execution = None
        return valid

    def _arm_terminal_execution(self, current, now, decision):
        if (self._terminal_execution_active(now)
                or not getattr(self, 'terminal_cruise_enabled', False)
                or self.mission_state != MissionState.TERMINAL_MINCO
                or not self.intercept_requested or self.safe_recovery_latched
                or not decision.locked or self.latest_target_state is None
                or not self._prediction_fresh(now)):
            return
        trajectory = self.tracker.active_trajectory
        if (trajectory is None
                or not (trajectory.terminal_mode or self.terminal_mode_latched)
                or trajectory.mission_id != self.mission_id
                or trajectory.target_state_source != 'tracking'
                or not 0. < trajectory.contact_stamp - now <= .7
                or now >= trajectory.valid_until):
            return
        relative = tuple(self.latest_target_state.position[a] - current.position[a]
                         for a in (0, 1))
        if math.hypot(*relative) > 2.5:
            return
        # A plan accepted in MINCO_TRACKING can become the terminal plan
        # without an accepted replacement. Update only its execution tag.
        trajectory = trajectory_for_mission(trajectory, self.mission_state)
        self.tracker.active_trajectory = trajectory
        self.terminal_execution = TerminalExecution(
            trajectory, now, min(trajectory.valid_until, trajectory.contact_stamp,
                                 now + .7))

    def _update_visibility(self, current, now, dt):
        """Compute visual authority before selecting XYZ or yaw commands."""
        self.latest_target_state = fresh_flight_target(
            self.latest_kf_message, now, self.maximum_state_age,
            self.expected_frame_id,
        )
        height = self.tracker.sea_surface_z - current.position[2]
        search_height = self.visibility.config.target_search_enable_height
        if self.safe_recovery_latched and (
            height >= max(search_height, self.tracker.recovery_clearance)
            and abs(current.velocity[2]) <= self.flight_guidance.takeoff_tolerance
        ):
            self.safe_recovery_latched = False
            self.terminal_mode_latched = False
            self.search_hold_position = None
        terminal = (self.terminal_mode_latched
                    or self.mission_state == MissionState.TERMINAL_MINCO
                    or self.safe_recovery_latched)
        decision = self.visibility.update(
            now, self.current_heading, height, dt,
            self.latest_target_state is not None,
            terminal=terminal,
            separate_input_wait=self.mission_state == MissionState.FOLLOW,
        )
        self._arm_terminal_execution(current, now, decision)
        if self._terminal_execution_active(now):
            # Visibility remains truthful. Only execution ownership survives
            # temporary sensing loss; no new plan can enter this window.
            decision = replace(decision, state='TERMINAL_COMMITTED')
            self.visibility_decision = decision
            self.search_state = decision.state
            return decision
        if (terminal and not decision.locked) or (
            self.mission_state == MissionState.SAFE_RECOVERY
            and height < max(search_height, self.tracker.recovery_clearance)
        ):
            self.safe_recovery_latched = True
        if self.safe_recovery_latched:
            decision = replace(decision, state='SAFE_RECOVERY', locked=False,
                               yaw_rate=0.0, search_direction=0)
        self.visibility_decision = decision
        self.search_state = decision.state
        return decision

    def _search_or_recovery_command(self, current, dt=None):
        """Preempt stale descent; brake pre-intercept motion before anchoring."""
        self.pending_trajectory = None
        self.tracker.active_trajectory = None
        search_height = self.visibility.config.target_search_enable_height
        height = self.tracker.sea_surface_z - current.position[2]
        if self.terminal_mode_latched or (
            self.mission_state == MissionState.TERMINAL_MINCO
        ):
            self.safe_recovery_latched = True
        if self.safe_recovery_latched or height < max(
            search_height, self.tracker.recovery_clearance,
        ):
            self.safe_recovery_latched = True
            self.search_state = 'SAFE_RECOVERY'
            self.search_hold_position = None
            # Recovery uses the tracker's command owner. A later flight-guidance
            # handover must start from measured velocity, not a cached approach.
            self.flight_guidance.previous_velocity = None
            return self.tracker.recovery_command(
                current, recovery_clearance=max(
                    search_height, self.tracker.recovery_clearance,
                ),
            )
        # Y grants planning authority, not an abrupt stop on input expiry.
        # The low-altitude/terminal branch above still owns climb recovery.
        previous = self.flight_guidance.previous_velocity
        command_speed = (math.hypot(*previous[:2]) if previous is not None
                         else math.hypot(*current.velocity[:2]))
        if (self.search_hold_position is None
                and math.hypot(*current.velocity[:2]) <= 0.1
                and command_speed <= 0.1):
            self.search_hold_position = anchored_search_hold(
                None, current.position, self.tracker.sea_surface_z, search_height,
            )
        return self.flight_guidance.search_velocity(
            FlightKinematicState(current.position, current.velocity),
            self.search_hold_position,
            self.tracker.control_dt if dt is None else dt,
        )

    def _bearing_approach_available(self, current, now):
        """Keep RGB approach separate from locked FOLLOW and Y/MINCO authority."""
        return bool(
            getattr(self, 'bearing_approach_enabled', False)
            and self.mission_state in (
                MissionState.TARGET_ACQUIRE, MissionState.TARGET_LOCK,
                MissionState.REACQUIRE, MissionState.FOLLOW,
            )
            and not (self.mission_state == MissionState.FOLLOW
                     and self.visibility_decision.locked)
            and self.visibility_decision.state != 'INPUT_WAIT'
            and (self.mission_state == MissionState.FOLLOW
                 or not getattr(self, 'intercept_requested', False))
            and not self.safe_recovery_latched and not self.terminal_mode_latched
            and current.position[2] <= (self.flight_guidance.flight_altitude
                                        + self.flight_guidance.takeoff_tolerance)
            and abs(current.velocity[2]) <= self.flight_guidance.takeoff_tolerance
            and self.visibility.bearing_approach_ready(now, self.maximum_state_age)
            and math.isfinite(self._bearing_approach_direction())
            and abs(self._bearing_approach_direction()) < math.pi / 2.0
        )

    def _bearing_approach_direction(self):
        """Use the body ray only for the matching fresh RGB frame."""
        reading = getattr(self, 'latest_body_bearing', None)
        if (reading is not None
                and reading[0] == self.visibility.last_valid_bearing_stamp):
            return reading[1]
        return self.visibility.last_valid_image_bearing

    def _continuous_follow_command(self, current, now, dt):
        """Refine RGB into KF following without leaving FOLLOW or resetting V."""
        state = FlightKinematicState(current.position, current.velocity)
        if self.visibility_decision.state == 'INPUT_WAIT':
            return self._search_or_recovery_command(current, dt)
        if (self.latest_target_state is not None
                and (self.visibility_decision.visible
                     or self.visibility_decision.locked)
                and not self.safe_recovery_latched):
            self.search_hold_position = None
            return self.flight_guidance.command(
                'FOLLOW', state, self.latest_target_state, dt,
            )
        if self._bearing_approach_available(current, now):
            self.search_hold_position = None
            self.bearing_approach_active = True
            return self.flight_guidance.bearing_approach(
                state, self.current_heading, self.bearing_approach_speed, dt,
                bearing=self._bearing_approach_direction(),
            )
        return self._search_or_recovery_command(current, dt)

    def _flight_guidance_command(self, state, now, dt):
        """Route only fresh locked tracking inputs into opt-in takeoff guidance."""
        target = self.latest_target_state
        if self.mission_state == MissionState.TAKEOFF:
            target = None
            decision = self.visibility_decision
            if getattr(self, 'takeoff_visual_tracking', False):
                if self.flight_guidance.previous_velocity is None:
                    self.flight_guidance.previous_velocity = tuple(state.velocity)
                if decision is not None and decision.locked and decision.visible:
                    target = fresh_flight_target(
                        self.latest_kf_message, now, min(.125, self.maximum_state_age),
                        self.expected_frame_id)
        return self.flight_guidance.command(self.mission_state_name, state, target, dt)

    def _final_yaw(self, setpoint, current, dt):
        """One arbiter owns yaw for every final PX4 setpoint."""
        if (getattr(self, 'p7_guidance_enabled', False)
                and self.mission_state == MissionState.FAR_GUIDANCE):
            # P7 has no LOS-rate yaw feedforward: PNG owns horizontal motion only.
            result = getattr(self, 'p7_result', None)
            rate = result.yaw_rate if result is not None and result.valid else 0.
            self.yaw_owner = 'IBVS' if result is not None and result.valid else 'P7_WAIT'
            self.search_yaw_rate_command = rate
            self.last_target_yaw = _wrap_angle(self.current_heading+rate*dt)
            setpoint.yaw, setpoint.yawspeed = self.last_target_yaw, rate
            return
        decision = self.visibility_decision
        height = self.tracker.sea_surface_z - current.position[2]
        visual_intercept = (
            getattr(self, 'intercept_requested', False)
            and self.mission_state in (
                MissionState.FAR_GUIDANCE,
                MissionState.MINCO_READY, MissionState.MINCO_TRACKING,
                MissionState.TERMINAL_MINCO,
            )
            and decision is not None and decision.locked and decision.visible
        )
        visual_takeoff = (
            getattr(self, 'takeoff_visual_tracking', False)
            and self.mission_state == MissionState.TAKEOFF
            and decision is not None and decision.visible
            and measurement_age(current.stamp, self.visibility.last_valid_bearing_stamp or 0.)
            <= min(.125, self.maximum_state_age)
            and self.vehicle_armed and self.offboard_active
            and measurement_age(current.stamp, self.vehicle_status_stamp or 0.)
            <= self.vehicle_status_timeout)
        inhibited = (
            self.mission_state in (
                MissionState.INIT, MissionState.GROUND_HOLD,
                *self.TERMINAL_STATES,
            ) or self.safe_recovery_latched
            or (self._terminal_execution_active(current.stamp)
                and not decision.locked)
            # This is a launch/search gate. A fresh, locked terminal descent
            # still needs visual yaw below it to keep the target in the camera.
            or (height < self.visibility.config.target_search_enable_height
                and not (visual_intercept or visual_takeoff))
        )
        rate = 0.0
        self.yaw_owner = 'HOLD'
        if not inhibited and decision is not None:
            if decision.visible:
                visual_lock = decision.locked or getattr(decision, 'visual_locked', False)
                bearing = self.visibility.last_valid_image_bearing
                center = getattr(self, 'latest_center_bearing', None)
                if (visual_intercept and center is not None
                        and 0.0 <= measurement_age(current.stamp, center[0])
                        <= self.maximum_state_age):
                    # A clipped RGB centroid moves as pixels leave the frame.
                    # The reliable fitted center represents the target itself.
                    bearing = center[1]
                rate = image_yaw_rate(
                    bearing,
                    self.visibility.config.vision_yaw_gain,
                    self.visibility.config.target_center_deadband_rad,
                    (self.max_observation_yaw_rate if visual_lock
                     else self.visibility.config.maximum_search_yaw_rate),
                )
                if visual_intercept:
                    target = fresh_flight_target(
                        self.latest_kf_message, current.stamp,
                        self.maximum_state_age, self.expected_frame_id,
                    )
                    if target is not None:
                        rx = target.position[0] - current.position[0]
                        ry = target.position[1] - current.position[1]
                        range_squared = rx * rx + ry * ry
                        if range_squared > 1e-6:
                            vx = target.velocity[0] - current.velocity[0]
                            vy = target.velocity[1] - current.velocity[1]
                            los_rate = (rx * vy - ry * vx) / range_squared
                            if math.isfinite(los_rate):
                                # Image feedback alone needs a persistent
                                # bearing error to follow a rotating LOS.
                                rate += los_rate
                    rate = max(-self.max_observation_yaw_rate, min(
                        self.max_observation_yaw_rate, rate,
                    ))
                legacy_follow_feedforward = (
                    getattr(self, 'follow_reference_mode', 'target') == 'rear_point'
                    and getattr(getattr(self, 'follow_receiver', None), 'active', None)
                    is not None and decision.locked)
                p7_follow_feedforward = getattr(self, 'p7_guidance_enabled', False)
                if (self.mission_state == MissionState.FOLLOW
                        and (legacy_follow_feedforward or p7_follow_feedforward)
                        and measurement_age(current.stamp,
                                            self.visibility.last_valid_bearing_stamp or 0.)
                        <= min(.125, self.maximum_state_age)):
                    # The common-epoch KF/native pair supplies LOS rate. An
                    # expired or unresolved pair never grants feedforward.
                    metrics = getattr(self, 'follow_metrics', {})
                    limit = self.max_observation_yaw_rate
                    if p7_follow_feedforward and not visual_lock:
                        limit = min(limit, self.visibility.config.maximum_search_yaw_rate)
                    if hasattr(self, 'follow_model'):
                        limit = min(limit, self.follow_model.config.maximum_yaw_rate)
                    los_rate = metrics.get('los_yaw_rate', math.nan)
                    if (math.isfinite(los_rate)
                            and all(measurement_age(current.stamp, metrics.get(k, 0.))
                                    <= min(.125, self.maximum_state_age)
                                    for k in ('epoch', 'source_stamp'))):
                        rate = max(-limit, min(limit, rate+los_rate))
                self.yaw_owner = ('VISION' if visual_lock or getattr(
                    self, 'bearing_approach_active', False) else 'SEARCH')
            elif decision.state != 'SAFE_WAIT':
                rate = decision.yaw_rate
                self.yaw_owner = 'SEARCH'
        self.search_yaw_rate_command = rate
        if self.last_target_yaw is None or inhibited:
            self.last_target_yaw = self.current_heading
        # Do not accumulate a reference faster than the vehicle can follow.
        self.last_target_yaw = _wrap_angle(self.current_heading + rate * dt)
        setpoint.yaw = self.last_target_yaw
        setpoint.yawspeed = rate

    def _p7_guidance_command(self, current, now, dt):
        """Only the existing Tracker publishes; vertical shaping/sea safety remain local."""
        self.p7_result = None
        image = self.p7_image
        target = fresh_flight_target(self.latest_kf_message, now,
                                     min(.125, self.maximum_state_age), self.expected_frame_id)
        legal = (target is not None and image is not None and self.visibility_decision.locked
                 and not self.safe_recovery_latched and self.offboard_active and self.vehicle_armed
                 and self.vehicle_status_stamp is not None
                 and 0 <= now-self.vehicle_status_stamp <= self.vehicle_status_timeout)
        if not legal:
            self.p7_controller.reset()
            return self._search_or_recovery_command(current, dt)
        previous = self.tracker.previous_command_velocity
        if previous is None:
            previous = current.velocity
        if self.p7_controller.previous is None:
            self.p7_controller.reset(previous, getattr(self, 'search_yaw_rate_command', 0.))
        capture = getattr(self, 'p7_capture_enabled', False)
        approach_distance = None
        desired_z = self.flight_guidance.altitude_velocity_gain*(
            self.flight_guidance.flight_altitude-current.position[2])
        if capture:
            from .p7_visual_guidance import capture_reference
            approach_distance, desired_z = capture_reference(
                current.position, target.position, target.velocity,
                sea_surface_z=self.flight_guidance.sea_surface_z,
                contact_clearance=max(.33, self.flight_guidance.approach_contact_clearance))
        result = self.p7_controller.step(
            now, self.p7_actual_dt, current.position[:2], current.velocity[:2],
            target.position[:2], target.velocity[:2], *image, approach_distance=approach_distance)
        if not result.valid:
            return self._search_or_recovery_command(current, dt)
        self.flight_guidance.previous_velocity = previous
        command = self.flight_guidance._velocity_command(
            FlightKinematicState(current.position, current.velocity),
            (*result.velocity, desired_z), dt, True, True)
        self.p7_result = result
        self.p7_diagnostic_pub.publish(String(data=json.dumps(dict(
            vars(result), stamp=now, image_stamp=image[0], dt=self.p7_actual_dt,
            capture_enabled=capture, approach_distance=approach_distance,
            velocity=list(command.velocity)))))
        return command

    def timer_callback(self):
        started = time.perf_counter()
        now = self._ros_seconds()
        last_control = getattr(self, 'last_timer_stamp', None)
        self.p7_actual_dt = (now-last_control if last_control is not None
                             else self.tracker.control_dt)
        if (getattr(self, 'follow_minco_requested', False) and last_control is not None
                and 0 <= now-last_control < .001):
            # A start wakeup may precede the regular timer. Never shape a command
            # using the default 50ms dt for a sub-millisecond actual interval.
            return
        self._process_pending_trajectory()
        if self.mission_state != MissionState.FOLLOW:
            self._revoke_nominal_follow('FOLLOW_EXITED')
        if self.latest_state is None:
            self._revoke_nominal_follow('NAVIGATION_UNAVAILABLE')
            self._publish_bool(self.flight_ready_pub, False)
            self._publish_diagnostic(
                now,
                None,
                'WAITING_FOR_STATE',
                time.perf_counter() - started,
            )
            return
        if now - self.latest_state.stamp > self.maximum_state_age:
            self._revoke_nominal_follow('NAVIGATION_STALE')
            if getattr(self, 'terminal_execution', None) is not None:
                self.terminal_execution = None
                self.tracker.active_trajectory = None
                self.safe_recovery_latched = True
                self.search_state = 'SAFE_RECOVERY'
                if self.visibility_decision is not None:
                    self.visibility_decision = replace(
                        self.visibility_decision, state='SAFE_RECOVERY', locked=False)
            self._publish_bool(self.flight_ready_pub, False)
            self._publish_diagnostic(
                now,
                None,
                'STATE_STALE',
                time.perf_counter() - started,
            )
            return
        current = replace(self.latest_state, stamp=now)
        flight_state = FlightKinematicState(
            current.position,
            current.velocity,
        )
        timestamp_us = self._timestamp_us()
        dt = self.tracker.control_dt
        if self.last_timer_stamp is not None:
            measured_dt = now - self.last_timer_stamp
            if 0.001 <= measured_dt <= 0.25:
                dt = measured_dt
        self.last_timer_stamp = now
        self.control_counter += 1

        dt = min(dt, self.tracker.maximum_command_dt)
        decision = self._update_visibility(current, now, dt)
        committed = self._terminal_execution_active(now)
        self.bearing_approach_active = False
        target_yaw = self.current_heading

        if self.mission_state in (MissionState.INIT, MissionState.GROUND_HOLD):
            flight_command = self._flight_guidance_command(flight_state, now, dt)
            self._publish_offboard_mode(timestamp_us, velocity_control=False)
            setpoint = flight_command_to_setpoint(
                flight_command,
                timestamp_us,
                yaw=target_yaw,
            )
            self.preflight_counter += 1
            mode_retry_due = (
                self.preflight_counter >= self.offboard_prestream_cycles
                and (
                    self.preflight_counter - self.offboard_prestream_cycles
                ) % self.px4_command_retry_cycles == 0
            )
            if mode_retry_due and not self.offboard_active:
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                )
            if (
                self.vehicle_armed
                and self.preflight_counter % self.px4_command_retry_cycles == 0
            ):
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    0.0,
                )
            status_fresh = (
                self.vehicle_status_stamp is not None
                and now - self.vehicle_status_stamp
                <= self.vehicle_status_timeout
            )
            if not self.pre_flight_checks_pass:
                if not self.preflight_wait_announced:
                    self.get_logger().info(
                        'PREPARING | waiting for PX4 preflight checks'
                    )
                    self.preflight_wait_announced = True
            elif not self.preflight_checks_announced:
                self.get_logger().info('PX4 preflight checks passed')
                self.preflight_checks_announced = True
            ready = ground_flight_ready(
                self.offboard_active, self.vehicle_armed,
                status_fresh, self.pre_flight_checks_pass,
            )
            self._publish_bool(self.flight_ready_pub, ready)
            if ready and not self.flight_ready:
                self.get_logger().info(
                    'FLIGHT READY | PX4 is disarmed in OFFBOARD ground hold'
                )
            self.flight_ready = ready
            status = self.mission_state_name
            command = flight_command
        elif self.mission_state == MissionState.FOLLOW:
            self._publish_bool(self.flight_ready_pub, False)
            self._request_flight_mode()
            command = None
            if getattr(self, 'follow_minco_requested', False):
                # Both owners shape from the last actually emitted command at every switch.
                if self.tracker.previous_command_velocity is not None:
                    self.flight_guidance.previous_velocity = self.tracker.previous_command_velocity
                command = self._nominal_follow_command(current, now, dt)
            minco_active = command is not None
            if command is None:
                command = self._continuous_follow_command(current, now, dt)
            velocity_control = getattr(command, 'mode', '') == 'VELOCITY'
            self._publish_offboard_mode(
                timestamp_us, velocity_control=velocity_control,
            )
            setpoint = (flight_command_to_setpoint(
                command, timestamp_us, acceleration_feedforward=(minco_active and getattr(
                    self, 'follow_minco_acceleration_feedforward', False)))
                        if hasattr(command, 'mode')
                        else command_to_setpoint(command, timestamp_us))
            self._publish_bool(self.far_guidance_pub, False)
            status = 'MINCO_FOLLOW' if minco_active else 'FOLLOW'
        elif (getattr(self, 'p7_guidance_enabled', False)
              and self.mission_state == MissionState.FAR_GUIDANCE):
            self._publish_bool(self.flight_ready_pub, False)
            self._request_flight_mode()
            self.pending_trajectory = self.tracker.active_trajectory = None
            command = self._p7_guidance_command(current, now, dt)
            self._publish_offboard_mode(timestamp_us, velocity_control=True)
            setpoint = (flight_command_to_setpoint(command, timestamp_us)
                        if hasattr(command, 'mode') else command_to_setpoint(
                            command, timestamp_us, velocity_mode=True))
            setpoint.acceleration = [math.nan, math.nan, math.nan]
            self._publish_bool(self.far_guidance_pub, self.p7_result is not None)
            status = 'P7_GUIDANCE' if self.p7_result is not None else 'P7_BRAKING'
        elif self._bearing_approach_available(current, now):
            self._publish_bool(self.flight_ready_pub, False)
            self._request_flight_mode()
            self.pending_trajectory = None
            self.tracker.active_trajectory = None
            self.search_hold_position = None
            command = self.flight_guidance.bearing_approach(
                flight_state, self.current_heading, self.bearing_approach_speed, dt,
                bearing=self._bearing_approach_direction(),
            )
            self.bearing_approach_active = True
            self._publish_offboard_mode(timestamp_us, velocity_control=True)
            setpoint = flight_command_to_setpoint(command, timestamp_us)
            self._publish_bool(self.far_guidance_pub, False)
            status = 'BEARING_APPROACH'
        elif (self.mission_state in (
            MissionState.TARGET_ACQUIRE, MissionState.TARGET_LOCK,
            MissionState.REACQUIRE, MissionState.SAFE_RECOVERY,
        ) or (not committed and not decision.locked and self.mission_state not in (
            MissionState.TAKEOFF, *self.TERMINAL_STATES,
        ))):
            self._publish_bool(self.flight_ready_pub, False)
            self._request_flight_mode()
            command = self._search_or_recovery_command(current, dt)
            self._publish_offboard_mode(
                timestamp_us, velocity_control=getattr(command, 'mode', '') == 'VELOCITY',
            )
            if hasattr(command, 'mode'):
                setpoint = flight_command_to_setpoint(command, timestamp_us)
            else:
                setpoint = command_to_setpoint(command, timestamp_us)
            self._publish_bool(self.far_guidance_pub, False)
            status = self.search_state
            if (getattr(command, 'mode', '') == 'VELOCITY'
                    and self.search_hold_position is None):
                status = 'VISUAL_BRAKING'
        elif self.mission_state in (
            MissionState.TAKEOFF,
            MissionState.FOLLOW,
            MissionState.FAR_GUIDANCE,
        ):
            self.search_hold_position = None
            self._publish_bool(self.flight_ready_pub, False)
            self._request_flight_mode()
            flight_command = self._flight_guidance_command(flight_state, now, dt)
            velocity_control = flight_command.mode == 'VELOCITY'
            self._publish_offboard_mode(
                timestamp_us,
                velocity_control=velocity_control,
            )
            setpoint = flight_command_to_setpoint(
                flight_command,
                timestamp_us,
                yaw=target_yaw,
            )
            self._publish_bool(
                self.takeoff_complete_pub,
                flight_command.takeoff_complete,
            )
            self._publish_bool(
                self.far_guidance_pub,
                flight_command.far_guidance_available,
            )
            if (
                flight_command.takeoff_complete
                and not self.takeoff_complete_sent
            ):
                self.takeoff_complete_sent = True
                self.get_logger().info('TAKEOFF COMPLETE | entering FOLLOW')
            status = self.mission_state_name
            command = flight_command
        elif self.mission_state in self.CONTROL_STATES:
            self._publish_bool(self.flight_ready_pub, False)
            self._request_flight_mode()
            prediction = self.latest_prediction
            prediction_fresh = self._prediction_fresh(now)
            active = self.tracker.active_trajectory
            executable = bool(active is not None and active.mission_id == self.mission_id
                              and now < active.valid_until)
            if not prediction_fresh and not (
                committed or (executable and decision.locked
                              and self.latest_target_state is not None)
            ):
                command = self._search_or_recovery_command(current, dt)
                self._publish_offboard_mode(
                    timestamp_us,
                    velocity_control=getattr(command, 'mode', '') == 'VELOCITY',
                )
                setpoint = (flight_command_to_setpoint(command, timestamp_us)
                            if hasattr(command, 'mode')
                            else command_to_setpoint(command, timestamp_us))
                status = 'NO_VALID_PLAN'
                self._final_yaw(setpoint, current, dt)
                self.setpoint_pub.publish(setpoint)
                self._remember_emitted_velocity(setpoint, now)
                self.reference_pub.publish(setpoint)
                self._publish_diagnostic(now, command, status,
                                         time.perf_counter() - started)
                return
            self.search_hold_position = None
            # Position feedback must compare the physical measurement with
            # the reference at that same sample epoch. Feedforward and command
            # limits use the current publication epoch independently.
            terminal_target = (fresh_flight_target(
                self.latest_kf_message, now, self.maximum_state_age, self.expected_frame_id,
            ) if prediction_fresh
                and self.get_parameter('terminal_cruise_enabled').value else None)
            command = self.tracker.command(
                self.latest_state, self.mission_id, control_stamp=now,
                terminal_target_at_time=(
                    lambda stamp: prediction_endpoint_from_message(prediction, stamp)
                ) if terminal_target is not None else None,
                terminal_target_velocity=(
                    terminal_target.velocity if terminal_target is not None else None
                ),
                hold_terminal_velocity=committed,
                cruise_from_start=bool(
                    self.use_velocity_control and self.intercept_requested
                    and executable and active.target_state_source == 'tracking'
                    and decision.locked and terminal_target is not None),
            )
            velocity_mode = self.use_velocity_control
            if command is None:
                # Keep the recovery reference continuous instead of snapping
                # the position setpoint to the measured state, and climb out of
                # the sea margin while no plan is valid.
                command = self.tracker.recovery_command(current)
                setpoint = command_to_setpoint(
                    command,
                    timestamp_us,
                    yaw=target_yaw,
                    velocity_mode=velocity_mode,
                )
                self._publish_offboard_mode(
                    timestamp_us,
                    velocity_control=velocity_mode,
                )
                status = 'NO_VALID_PLAN'
            else:
                setpoint = command_to_setpoint(
                    command,
                    timestamp_us,
                    yaw=target_yaw,
                    velocity_mode=velocity_mode,
                )
                self._publish_offboard_mode(
                    timestamp_us,
                    velocity_control=velocity_mode,
                )
                status = 'TERMINAL_COMMITTED' if committed else 'TRACKING'
        else:
            self._publish_bool(self.flight_ready_pub, False)
            self._publish_offboard_mode(timestamp_us, velocity_control=False)
            hold_state = current
            if self.terminal_hold_position is not None:
                hold_state = replace(
                    current,
                    position=self.terminal_hold_position,
                    velocity=(0.0, 0.0, 0.0),
                )
            setpoint = hold_setpoint(
                hold_state,
                timestamp_us,
                yaw=target_yaw,
            )
            status = self.mission_state_name
            command = None
        self._final_yaw(setpoint, current, dt)
        self.setpoint_pub.publish(setpoint)
        if (getattr(self, 'follow_minco_requested', False)
                and self.mission_state == MissionState.FOLLOW):
            debug = getattr(self, 'follow_cycle_debug', {})
            debug.update(final_velocity=[float(v) for v in setpoint.velocity],
                         final_acceleration=[float(v) if math.isfinite(v) else None
                                             for v in setpoint.acceleration],
                         yaw=float(setpoint.yaw), yaw_rate=float(setpoint.yawspeed),
                         command_stamp=now, navigation_stamp=current.stamp,
                         shaping_previous=self.tracker.previous_command_velocity)
            self.follow_debug_pub.publish(String(data=json.dumps(debug)))
        self._remember_emitted_velocity(setpoint, now)
        self.reference_pub.publish(setpoint)
        self._publish_diagnostic(
            now,
            command,
            status,
            time.perf_counter() - started,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
