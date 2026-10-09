import csv
import copy
import json
import math
import subprocess
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Vector3
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from uav_usv_interfaces.msg import (
    InterceptResult,
    TargetObservation,
    TargetState,
)

from uav_control.guidance.finite_horizon_intercept_planner import (
    FiniteHorizonInterceptPlanner,
    PlanningFailureReason,
)
from uav_control.guidance.intercept_reachability import (
    SeaSafetyState,
    apply_sea_safety_guard,
)
from uav_control.guidance.rolling_intercept_guidance import (
    RollingReferenceFilter,
)
from uav_control.tracking.maneuvering_target_predictor import (
    ManeuveringTargetPredictor,
)
from uav_control.tracking.prediction_error_tracker import (
    PredictionErrorTracker,
)


PREDICTION_HORIZONS = (0.5, 1.0, 2.0)
PREDICTION_MODELS = ('guidance', 'kf')
PREDICTION_CSV_FIELDS = [
    f'{model}_prediction_{str(horizon).replace(".", "p")}_{field}'
    for model in PREDICTION_MODELS
    for horizon in PREDICTION_HORIZONS
    for field in ('x', 'y', 'z', 'error', 'age')
]


class TrajectoryImpactSim(Node):
    """
    Simulate and control moving-target interception.

    By default this node controls the PX4 Gazebo model. Pure numerical mode is
    available by setting enable_gazebo_control to false.
    """

    CSV_FIELDS = [
        'time',
        'uav_x',
        'uav_y',
        'uav_z',
        'uav_vx',
        'uav_vy',
        'uav_vz',
        'target_x',
        'target_y',
        'target_z',
        'target_vx',
        'target_vy',
        'target_vz',
        'camera_measurement_valid',
        'camera_x',
        'camera_y',
        'camera_z',
        'camera_position_error',
        'camera_confidence',
        'kf_state_valid',
        'kf_x',
        'kf_y',
        'kf_z',
        'kf_vx',
        'kf_vy',
        'kf_vz',
        'kf_position_error',
        'kf_state_age',
        *PREDICTION_CSV_FIELDS,
        'prediction_model',
        'estimated_turn_rate',
        'estimated_turn_acceleration',
        'estimated_longitudinal_acceleration',
        'control_dt',
        'sim_dt',
        'wall_dt',
        'callback_compute_time',
        'prediction_compute_time',
        'planner_compute_time',
        'minco_generation_compute_time',
        'minco_optimization_compute_time',
        'guidance_compute_time',
        'safety_compute_time',
        'logging_compute_time',
        'retained_plan_age',
        'distance',
        'horizontal_distance',
        'vertical_error',
        'intercept_x',
        'intercept_y',
        'intercept_z',
        'intercept_vx',
        'intercept_vy',
        'intercept_vz',
        'intercept_ax',
        'intercept_ay',
        'intercept_az',
        't_go',
        'guidance_altitude_reference',
        'guidance_closing_speed',
        't_horizontal_min',
        't_vertical_min',
        't_sea_safe_min',
        'planner_search_min_time',
        'planner_search_max_time',
        'planner_failure_reason',
        'guidance_phase',
        'sea_safety_state',
        'sea_safety_margin',
        'trajectory_plan_feasible',
        'planner_attempted',
        'planner_succeeded',
        'minco_optimization_attempted',
        'minco_optimization_succeeded',
        'plan_hold_active',
        'trajectory_planner',
        'minco_target_curve_weight',
        'minco_constraint_penalty',
        'minco_optimization_iterations',
        'minco_piece_durations',
        'planned_closing_speed',
        'planned_max_horizontal_speed',
        'planned_max_vertical_speed',
        'planned_max_horizontal_acceleration',
        'planned_max_vertical_acceleration',
        'uav_yaw',
        'observation_yaw',
        'command_vx',
        'command_vy',
        'command_vz',
        'command_ax',
        'command_ay',
        'command_az',
        'uav_ax',
        'uav_ay',
        'uav_az',
        'terminal_mode',
        'phase',
        'intercept_elapsed_time',
        'minimum_distance',
        'relative_speed',
        'closing_speed',
        'constraint_violation',
        'outcome',
        'failure_reason',
        'failure_detail',
    ]

    def __init__(self):
        raise RuntimeError(
            'Retired truth-control entry point; use modular_intercept.launch.py'
        )
        super().__init__('trajectory_impact_sim')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('offboard_prestream_time', 2.0)
        self.declare_parameter('px4_command_retry_time', 1.0)
        self.declare_parameter('max_speed', 7.0)
        self.declare_parameter('max_acceleration', 3.5)
        self.declare_parameter(
            'max_actual_horizontal_acceleration',
            4.0,
        )
        self.declare_parameter(
            'horizontal_acceleration_guard_margin',
            0.5,
        )
        self.declare_parameter('enable_maneuver_prediction', True)
        self.declare_parameter('turn_rate_filter_alpha', 0.25)
        self.declare_parameter('max_target_turn_rate', 0.7)
        self.declare_parameter('turn_acceleration_filter_alpha', 0.2)
        self.declare_parameter('max_target_turn_acceleration', 1.5)
        self.declare_parameter('speed_acceleration_filter_alpha', 0.2)
        self.declare_parameter('max_target_longitudinal_acceleration', 2.0)
        self.declare_parameter('maneuver_acceleration_decay_time', 1.0)
        self.declare_parameter('maneuver_prediction_horizon', 2.0)
        self.declare_parameter('minimum_target_speed', 0.2)
        self.declare_parameter('intercept_guidance_horizon_min', 1.0)
        self.declare_parameter('intercept_guidance_horizon_max', 2.0)
        self.declare_parameter(
            'intercept_guidance_horizon_distance',
            20.0,
        )
        self.declare_parameter('intercept_position_gain', 0.8)
        self.declare_parameter(
            'intercept_reference_position_gain',
            1.0,
        )
        self.declare_parameter('intercept_reference_max_speed', 6.2)
        self.declare_parameter(
            'intercept_reference_max_acceleration',
            3.2,
        )
        self.declare_parameter(
            'intercept_reference_max_vertical_acceleration',
            2.5,
        )
        self.declare_parameter('terminal_radius', 3.0)
        self.declare_parameter('terminal_closing_speed', 1.5)
        self.declare_parameter('terminal_min_closing_speed', 0.3)
        self.declare_parameter('terminal_closing_speed_step', 0.3)
        self.declare_parameter('terminal_plan_duration_step', 0.1)
        self.declare_parameter('terminal_plan_absolute_max_duration', 3.0)
        self.declare_parameter('terminal_plan_horizon_extra_margin', 0.5)
        self.declare_parameter('terminal_control_lookahead', 0.15)
        self.declare_parameter('terminal_replan_period', 0.1)
        self.declare_parameter('terminal_plan_max_hold_time', 0.2)
        self.declare_parameter('terminal_plan_target_error_limit', 0.5)
        self.declare_parameter('terminal_plan_min_remaining_time', 0.2)
        self.declare_parameter('terminal_plan_failure_limit', 3)
        self.declare_parameter('enable_async_terminal_planner', True)
        self.declare_parameter('terminal_trajectory_position_gain', 1.2)
        self.declare_parameter('terminal_trajectory_vertical_gain', 1.2)
        self.declare_parameter('enable_minco_planner', True)
        self.declare_parameter('minco_piece_count', 3)
        self.declare_parameter('minco_target_curve_weight', 0.7)
        self.declare_parameter('enable_minco_geometric_optimization', True)
        self.declare_parameter('minco_optimization_max_iterations', 3)
        self.declare_parameter('minco_spatial_radius', 0.75)
        self.declare_parameter('minco_constraint_penalty_weight', 1000.0)
        self.declare_parameter('minco_quadrature_intervals_per_piece', 8)
        self.declare_parameter('terminal_contact_clearance', 0.05)
        # Preferred sea-surface clearance for terminal pursuit.  Zero
        # disables it.  The runtime ceiling is relaxed when necessary so it
        # still intersects the target's capture sphere.
        self.declare_parameter('terminal_minimum_clearance', 0.1)
        self.declare_parameter('terminal_safety_response_time', 0.15)
        self.declare_parameter('terminal_safety_margin', 0.02)
        self.declare_parameter('terminal_safety_warning_margin', 0.15)
        self.declare_parameter(
            'terminal_effective_vertical_braking_acceleration',
            2.5,
        )
        self.declare_parameter('terminal_descent_release_distance', 4.0)
        self.declare_parameter(
            'terminal_dive_angle',
            math.radians(45.0),
        )
        self.declare_parameter(
            'front_camera_max_depression_angle',
            0.85,
        )
        self.declare_parameter('terminal_max_acceleration', 3.0)
        self.declare_parameter(
            'terminal_max_vertical_acceleration',
            3.0,
        )
        self.declare_parameter('impact_radius', 0.25)
        self.declare_parameter('enable_sea_contact_failure', True)
        self.declare_parameter('sea_surface_z', 0.0)
        self.declare_parameter('max_sim_duration', 30.0)
        self.declare_parameter('state_timeout', 0.5)
        self.declare_parameter('vehicle_status_timeout', 2.0)
        self.declare_parameter('mode_loss_grace_time', 0.5)
        self.declare_parameter('constraint_violation_duration', 0.25)
        self.declare_parameter('log_directory', 'experiment_logs')
        self.declare_parameter('enable_gazebo_control', True)
        self.declare_parameter('flight_altitude', -5.0)
        self.declare_parameter('takeoff_tolerance', 0.5)
        self.declare_parameter('takeoff_settle_time', 1.0)
        self.declare_parameter(
            'takeoff_max_horizontal_acceleration',
            1.5,
        )
        self.declare_parameter('takeoff_max_vertical_speed', 1.5)
        self.declare_parameter(
            'takeoff_max_vertical_acceleration',
            1.0,
        )
        self.declare_parameter(
            'takeoff_horizontal_start_height',
            0.5,
        )
        self.declare_parameter(
            'takeoff_horizontal_full_height',
            1.5,
        )
        self.declare_parameter('follow_distance', 5.0)
        # Interception needs more radial closure authority than following.
        # The follow value caps the closing speed that the terminal fallback
        # law may add on top of the target velocity, which is too small to
        # finish the last metres against a manoeuvring USV.
        self.declare_parameter(
            'terminal_pursuit_max_closing_speed',
            3.0,
        )
        self.declare_parameter('follow_position_gain', 0.8)
        self.declare_parameter('follow_max_closing_speed', 1.5)
        self.declare_parameter('follow_max_acceleration', 2.5)
        self.declare_parameter('altitude_velocity_gain', 1.0)
        self.declare_parameter('max_vertical_speed', 4.0)
        self.declare_parameter('max_vertical_acceleration', 3.0)
        self.declare_parameter(
            'max_actual_vertical_acceleration',
            4.0,
        )
        self.declare_parameter('speed_guard_margin', 0.5)
        self.declare_parameter('speed_governor_gain', 1.0)
        self.declare_parameter('max_observation_yaw_rate', 1.0)

        control_rate_hz = self.get_parameter(
            'control_rate_hz'
        ).value

        self.dt = 1.0 / max(float(control_rate_hz), 1.0)
        self.control_dt = self.dt
        self.sim_dt = self.dt
        self.last_control_callback_time_ns = None
        self.offboard_prestream_cycles = max(
            round(
                float(
                    self.get_parameter(
                        'offboard_prestream_time'
                    ).value
                ) / self.dt
            ),
            20,
        )
        self.px4_command_retry_cycles = max(
            round(
                float(
                    self.get_parameter(
                        'px4_command_retry_time'
                    ).value
                ) / self.dt
            ),
            1,
        )
        self.max_speed = max(
            float(self.get_parameter('max_speed').value),
            0.1
        )
        self.max_acceleration = max(
            float(self.get_parameter('max_acceleration').value),
            0.1
        )
        self.max_actual_horizontal_acceleration = max(
            float(
                self.get_parameter(
                    'max_actual_horizontal_acceleration'
                ).value
            ),
            0.1,
        )
        self.horizontal_acceleration_guard_margin = min(
            max(
                float(
                    self.get_parameter(
                        'horizontal_acceleration_guard_margin'
                    ).value
                ),
                0.0,
            ),
            max(self.max_actual_horizontal_acceleration - 0.1, 0.0),
        )
        self.enable_maneuver_prediction = bool(
            self.get_parameter('enable_maneuver_prediction').value
        )
        self.turn_rate_filter_alpha = min(
            max(
                float(
                    self.get_parameter(
                        'turn_rate_filter_alpha'
                    ).value
                ),
                0.0,
            ),
            1.0,
        )
        self.max_target_turn_rate = max(
            float(
                self.get_parameter('max_target_turn_rate').value
            ),
            0.01,
        )
        self.turn_acceleration_filter_alpha = min(
            max(
                float(
                    self.get_parameter(
                        'turn_acceleration_filter_alpha'
                    ).value
                ),
                0.0,
            ),
            1.0,
        )
        self.max_target_turn_acceleration = max(
            float(
                self.get_parameter(
                    'max_target_turn_acceleration'
                ).value
            ),
            0.01,
        )
        self.speed_acceleration_filter_alpha = min(
            max(
                float(
                    self.get_parameter(
                        'speed_acceleration_filter_alpha'
                    ).value
                ),
                0.0,
            ),
            1.0,
        )
        self.max_target_longitudinal_acceleration = max(
            float(
                self.get_parameter(
                    'max_target_longitudinal_acceleration'
                ).value
            ),
            0.01,
        )
        self.maneuver_acceleration_decay_time = max(
            float(
                self.get_parameter(
                    'maneuver_acceleration_decay_time'
                ).value
            ),
            self.dt,
        )
        self.maneuver_prediction_horizon = max(
            float(
                self.get_parameter(
                    'maneuver_prediction_horizon'
                ).value
            ),
            self.dt,
        )
        self.minimum_target_speed = max(
            float(
                self.get_parameter('minimum_target_speed').value
            ),
            0.01,
        )
        self.intercept_guidance_horizon_min = min(
            max(
                float(
                    self.get_parameter(
                        'intercept_guidance_horizon_min'
                    ).value
                ),
                self.dt,
            ),
            2.0,
        )
        self.intercept_guidance_horizon_max = min(
            max(
                float(
                    self.get_parameter(
                        'intercept_guidance_horizon_max'
                    ).value
                ),
                self.intercept_guidance_horizon_min,
            ),
            2.0,
        )
        self.intercept_guidance_horizon_distance = max(
            float(
                self.get_parameter(
                    'intercept_guidance_horizon_distance'
                ).value
            ),
            0.1,
        )
        self.intercept_position_gain = max(
            float(
                self.get_parameter('intercept_position_gain').value
            ),
            0.1,
        )
        self.intercept_reference_position_gain = max(
            float(
                self.get_parameter(
                    'intercept_reference_position_gain'
                ).value
            ),
            0.1,
        )
        self.terminal_radius = max(
            float(self.get_parameter('terminal_radius').value),
            0.1
        )
        self.terminal_closing_speed = max(
            float(self.get_parameter('terminal_closing_speed').value),
            0.1
        )
        self.terminal_min_closing_speed = min(
            max(
                float(
                    self.get_parameter(
                        'terminal_min_closing_speed'
                    ).value
                ),
                0.1,
            ),
            self.terminal_closing_speed,
        )
        self.terminal_closing_speed_step = max(
            float(
                self.get_parameter(
                    'terminal_closing_speed_step'
                ).value
            ),
            0.1,
        )
        self.terminal_plan_duration_step = max(
            float(
                self.get_parameter(
                    'terminal_plan_duration_step'
                ).value
            ),
            self.dt,
        )
        self.terminal_plan_absolute_max_duration = max(
            float(
                self.get_parameter(
                    'terminal_plan_absolute_max_duration'
                ).value
            ),
            self.intercept_guidance_horizon_max,
        )
        self.terminal_plan_horizon_extra_margin = max(
            float(
                self.get_parameter(
                    'terminal_plan_horizon_extra_margin'
                ).value
            ),
            self.terminal_plan_duration_step,
        )
        self.terminal_control_lookahead = max(
            float(
                self.get_parameter(
                    'terminal_control_lookahead'
                ).value
            ),
            self.dt,
        )
        self.terminal_replan_period = max(
            float(
                self.get_parameter('terminal_replan_period').value
            ),
            self.dt,
        )
        self.terminal_plan_max_hold_time = max(
            float(
                self.get_parameter(
                    'terminal_plan_max_hold_time'
                ).value
            ),
            0.0,
        )
        self.terminal_plan_target_error_limit = max(
            float(
                self.get_parameter(
                    'terminal_plan_target_error_limit'
                ).value
            ),
            0.01,
        )
        self.terminal_plan_min_remaining_time = max(
            float(
                self.get_parameter(
                    'terminal_plan_min_remaining_time'
                ).value
            ),
            self.terminal_control_lookahead,
        )
        self.terminal_plan_failure_limit = max(
            int(
                self.get_parameter(
                    'terminal_plan_failure_limit'
                ).value
            ),
            1,
        )
        self.enable_async_terminal_planner = bool(
            self.get_parameter('enable_async_terminal_planner').value
        )
        self.terminal_trajectory_position_gain = max(
            float(
                self.get_parameter(
                    'terminal_trajectory_position_gain'
                ).value
            ),
            0.0,
        )
        self.terminal_trajectory_vertical_gain = max(
            float(
                self.get_parameter(
                    'terminal_trajectory_vertical_gain'
                ).value
            ),
            0.0,
        )
        self.enable_minco_planner = bool(
            self.get_parameter('enable_minco_planner').value
        )
        self.minco_piece_count = min(
            max(
                int(self.get_parameter('minco_piece_count').value),
                1,
            ),
            6,
        )
        self.minco_target_curve_weight = min(
            max(
                float(
                    self.get_parameter(
                        'minco_target_curve_weight'
                    ).value
                ),
                0.0,
            ),
            1.0,
        )
        self.enable_minco_geometric_optimization = bool(
            self.get_parameter(
                'enable_minco_geometric_optimization'
            ).value
        )
        self.minco_optimization_max_iterations = min(
            max(
                int(
                    self.get_parameter(
                        'minco_optimization_max_iterations'
                    ).value
                ),
                0,
            ),
            20,
        )
        self.minco_spatial_radius = max(
            float(self.get_parameter('minco_spatial_radius').value),
            0.05,
        )
        self.minco_constraint_penalty_weight = max(
            float(
                self.get_parameter(
                    'minco_constraint_penalty_weight'
                ).value
            ),
            1.0,
        )
        self.minco_quadrature_intervals_per_piece = min(
            max(
                int(
                    self.get_parameter(
                        'minco_quadrature_intervals_per_piece'
                    ).value
                ),
                2,
            ),
            40,
        )
        self.terminal_max_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'terminal_max_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_acceleration,
        )
        self.impact_radius = max(
            float(self.get_parameter('impact_radius').value),
            0.01
        )
        self.terminal_plan_target_error_limit = max(
            self.terminal_plan_target_error_limit,
            self.impact_radius,
        )
        self.terminal_contact_clearance = min(
            max(
                float(
                    self.get_parameter(
                        'terminal_contact_clearance'
                    ).value
                ),
                0.001,
            ),
            0.9 * self.impact_radius,
        )
        self.terminal_minimum_clearance = (
            TrajectoryImpactSim.normalize_optional_clearance(
                self.get_parameter(
                    'terminal_minimum_clearance'
                ).value,
                self.terminal_contact_clearance,
                self.terminal_radius,
            )
        )
        self.terminal_safety_response_time = max(
            float(
                self.get_parameter(
                    'terminal_safety_response_time'
                ).value
            ),
            0.0,
        )
        self.terminal_safety_margin = min(
            max(
                float(
                    self.get_parameter('terminal_safety_margin').value
                ),
                0.0,
            ),
            0.5 * self.impact_radius,
        )
        self.terminal_safety_warning_margin = max(
            float(
                self.get_parameter(
                    'terminal_safety_warning_margin'
                ).value
            ),
            0.0,
        )
        self.terminal_descent_release_distance = max(
            float(
                self.get_parameter(
                    'terminal_descent_release_distance'
                ).value
            ),
            self.terminal_radius + 0.1,
        )
        self.front_camera_max_depression_angle = min(
            max(
                float(
                    self.get_parameter(
                        'front_camera_max_depression_angle'
                    ).value
                ),
                math.radians(5.0),
            ),
            math.radians(85.0),
        )
        self.terminal_dive_angle = min(
            max(
                float(
                    self.get_parameter('terminal_dive_angle').value
                ),
                math.radians(5.0),
            ),
            self.front_camera_max_depression_angle,
        )
        self.enable_sea_contact_failure = bool(
            self.get_parameter('enable_sea_contact_failure').value
        )
        self.sea_surface_z = float(
            self.get_parameter('sea_surface_z').value
        )
        self.max_sim_duration = max(
            float(self.get_parameter('max_sim_duration').value),
            self.dt
        )
        self.state_timeout = max(
            float(self.get_parameter('state_timeout').value),
            self.dt,
        )
        self.vehicle_status_timeout = max(
            float(
                self.get_parameter('vehicle_status_timeout').value
            ),
            self.state_timeout,
        )
        mode_loss_grace_time = max(
            float(self.get_parameter('mode_loss_grace_time').value),
            self.dt,
        )
        self.mode_loss_grace_cycles = max(
            round(mode_loss_grace_time / self.dt),
            1,
        )
        constraint_violation_duration = max(
            float(
                self.get_parameter(
                    'constraint_violation_duration'
                ).value
            ),
            self.dt,
        )
        self.constraint_violation_cycles_limit = max(
            round(constraint_violation_duration / self.dt),
            1,
        )
        self.enable_gazebo_control = bool(
            self.get_parameter('enable_gazebo_control').value
        )
        self.flight_altitude = float(
            self.get_parameter('flight_altitude').value
        )
        self.takeoff_tolerance = max(
            float(self.get_parameter('takeoff_tolerance').value),
            0.05
        )
        takeoff_settle_time = max(
            float(self.get_parameter('takeoff_settle_time').value),
            self.dt
        )
        self.takeoff_settle_cycles = max(
            round(takeoff_settle_time / self.dt),
            1
        )
        self.follow_distance = max(
            float(self.get_parameter('follow_distance').value),
            0.5
        )
        self.follow_position_gain = max(
            float(self.get_parameter('follow_position_gain').value),
            0.1
        )
        self.altitude_velocity_gain = max(
            float(self.get_parameter('altitude_velocity_gain').value),
            0.1
        )
        self.max_vertical_speed = max(
            float(self.get_parameter('max_vertical_speed').value),
            0.1
        )
        self.max_vertical_acceleration = max(
            float(
                self.get_parameter('max_vertical_acceleration').value
            ),
            0.1
        )
        self.max_actual_vertical_acceleration = max(
            float(
                self.get_parameter(
                    'max_actual_vertical_acceleration'
                ).value
            ),
            0.1,
        )
        self.terminal_max_vertical_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'terminal_max_vertical_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_vertical_acceleration,
        )
        self.terminal_effective_vertical_braking_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'terminal_effective_vertical_braking_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.terminal_max_vertical_acceleration,
        )
        requested_guard_margin = max(
            float(self.get_parameter('speed_guard_margin').value),
            0.0
        )
        self.speed_guard_margin = min(
            requested_guard_margin,
            0.5 * self.max_speed
        )
        self.command_speed_limit = (
            self.max_speed - self.speed_guard_margin
        )
        self.speed_governor_gain = max(
            float(self.get_parameter('speed_governor_gain').value),
            0.0,
        )
        self.max_observation_yaw_rate = max(
            float(
                self.get_parameter('max_observation_yaw_rate').value
            ),
            0.1,
        )
        self.follow_max_closing_speed = min(
            max(
                float(
                    self.get_parameter(
                        'follow_max_closing_speed'
                    ).value
                ),
                0.1,
            ),
            self.command_speed_limit,
        )
        self.terminal_pursuit_max_closing_speed = min(
            max(
                float(
                    self.get_parameter(
                        'terminal_pursuit_max_closing_speed'
                    ).value
                ),
                self.follow_max_closing_speed,
            ),
            self.command_speed_limit,
        )
        self.follow_max_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'follow_max_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_acceleration,
        )
        self.takeoff_max_horizontal_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'takeoff_max_horizontal_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.follow_max_acceleration,
        )
        self.takeoff_max_vertical_speed = min(
            max(
                float(
                    self.get_parameter(
                        'takeoff_max_vertical_speed'
                    ).value
                ),
                0.1,
            ),
            self.max_vertical_speed,
        )
        self.takeoff_max_vertical_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'takeoff_max_vertical_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_vertical_acceleration,
        )
        self.takeoff_horizontal_start_height = max(
            float(
                self.get_parameter(
                    'takeoff_horizontal_start_height'
                ).value
            ),
            0.0,
        )
        self.takeoff_horizontal_full_height = max(
            float(
                self.get_parameter(
                    'takeoff_horizontal_full_height'
                ).value
            ),
            self.takeoff_horizontal_start_height + 0.1,
        )
        self.intercept_reference_max_speed = min(
            max(
                float(
                    self.get_parameter(
                        'intercept_reference_max_speed'
                    ).value
                ),
                0.1,
            ),
            self.command_speed_limit,
        )
        self.intercept_reference_max_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'intercept_reference_max_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_acceleration,
        )
        self.intercept_reference_max_vertical_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'intercept_reference_max_vertical_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_vertical_acceleration,
        )

        self.target_predictor = ManeuveringTargetPredictor(
            turn_rate_filter_alpha=self.turn_rate_filter_alpha,
            max_turn_rate=self.max_target_turn_rate,
            maneuver_horizon=self.maneuver_prediction_horizon,
            minimum_speed=self.minimum_target_speed,
            turn_acceleration_filter_alpha=(
                self.turn_acceleration_filter_alpha
            ),
            max_turn_acceleration=self.max_target_turn_acceleration,
            speed_acceleration_filter_alpha=(
                self.speed_acceleration_filter_alpha
            ),
            max_longitudinal_acceleration=(
                self.max_target_longitudinal_acceleration
            ),
            acceleration_decay_time=(
                self.maneuver_acceleration_decay_time
            ),
        )
        self.intercept_reference_filter = RollingReferenceFilter(
            position_gain=self.intercept_reference_position_gain,
            max_horizontal_speed=self.intercept_reference_max_speed,
            max_vertical_speed=self.max_vertical_speed,
            max_horizontal_acceleration=(
                self.intercept_reference_max_acceleration
            ),
            max_vertical_acceleration=(
                self.intercept_reference_max_vertical_acceleration
            ),
        )
        self.intercept_trajectory_planner = (
            FiniteHorizonInterceptPlanner(
                minimum_duration=self.intercept_guidance_horizon_min,
                maximum_duration=self.intercept_guidance_horizon_max,
                duration_step=self.terminal_plan_duration_step,
                sample_step=self.dt,
                maximum_horizontal_speed=self.command_speed_limit,
                maximum_vertical_speed=self.max_vertical_speed,
                maximum_horizontal_acceleration=(
                    self.limit_horizontal_acceleration(
                        self.terminal_max_acceleration
                    )
                ),
                maximum_vertical_acceleration=(
                    self.terminal_max_vertical_acceleration
                ),
                desired_closing_speed=self.terminal_closing_speed,
                minimum_closing_speed=(
                    self.terminal_min_closing_speed
                ),
                closing_speed_step=self.terminal_closing_speed_step,
                capture_radius=self.impact_radius,
                sea_surface_z=self.sea_surface_z,
                contact_clearance=self.terminal_contact_clearance,
                minco_piece_count=(
                    self.minco_piece_count
                    if self.enable_minco_planner
                    else 1
                ),
                minco_target_curve_weight=(
                    self.minco_target_curve_weight
                ),
                preferred_clearance=max(
                    self.terminal_minimum_clearance,
                    self.terminal_contact_clearance,
                ),
                enable_minco_geometric_optimization=(
                    self.enable_minco_geometric_optimization
                ),
                minco_optimization_max_iterations=(
                    self.minco_optimization_max_iterations
                ),
                minco_spatial_radius=self.minco_spatial_radius,
                minco_constraint_penalty_weight=(
                    self.minco_constraint_penalty_weight
                ),
                minco_quadrature_intervals_per_piece=(
                    self.minco_quadrature_intervals_per_piece
                ),
                absolute_maximum_duration=(
                    self.terminal_plan_absolute_max_duration
                ),
                horizon_extra_margin=(
                    self.terminal_plan_horizon_extra_margin
                ),
                response_delay=self.terminal_safety_response_time,
                effective_vertical_braking_acceleration=(
                    self.terminal_effective_vertical_braking_acceleration
                ),
            )
        )
        self.planner_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='minco_planner',
        )
        self.planner_future = None
        self.planner_epoch = 0

        self.target_position_sub = self.create_subscription(
            Point,
            '/target/position',
            self.target_position_callback,
            10
        )
        self.target_velocity_sub = self.create_subscription(
            Vector3,
            '/target/velocity',
            self.target_velocity_callback,
            10
        )
        self.camera_observation_sub = self.create_subscription(
            TargetObservation,
            '/perception/front/target_observation',
            self.camera_observation_callback,
            10,
        )
        self.filtered_target_state_sub = self.create_subscription(
            TargetState,
            '/tracking/target_state',
            self.filtered_target_state_callback,
            10,
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        result_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vehicle_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_position_callback,
            px4_qos
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback,
            px4_qos,
        )

        self.sim_position_pub = self.create_publisher(
            Point,
            '/simulation/impact/uav_position',
            10
        )
        self.sim_velocity_pub = self.create_publisher(
            Vector3,
            '/simulation/impact/uav_velocity',
            10
        )
        self.intercept_point_pub = self.create_publisher(
            Point,
            '/simulation/impact/intercept_point',
            10
        )
        self.hit_pub = self.create_publisher(
            Bool,
            '/simulation/impact/hit',
            10
        )
        self.result_pub = self.create_publisher(
            InterceptResult,
            '/simulation/impact/result',
            result_qos,
        )
        self.flight_ready_pub = self.create_publisher(
            Bool,
            '/simulation/impact/flight_ready',
            result_qos,
        )
        self.command_sub = self.create_subscription(
            String,
            '/simulation/impact/command',
            self.command_callback,
            10
        )

        self.offboard_pub = None
        self.setpoint_pub = None
        self.command_pub = None

        if self.enable_gazebo_control:
            self.offboard_pub = self.create_publisher(
                OffboardControlMode,
                '/fmu/in/offboard_control_mode',
                10
            )
            self.setpoint_pub = self.create_publisher(
                TrajectorySetpoint,
                '/fmu/in/trajectory_setpoint',
                10
            )
            self.command_pub = self.create_publisher(
                VehicleCommand,
                '/fmu/in/vehicle_command',
                10
            )

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_position_time_ns = None
        self.target_velocity_time_ns = None
        self.target_position_received = False
        self.target_velocity_received = False
        self.target_path_history = deque()
        self.target_path_distance = 0.0
        self.camera_measurement_valid = False
        self.camera_measurement = (math.nan, math.nan, math.nan)
        self.camera_confidence = 0.0
        self.camera_measurement_time_ns = None
        self.kf_state_valid = False
        self.kf_position = (math.nan, math.nan, math.nan)
        self.kf_velocity = (math.nan, math.nan, math.nan)
        self.kf_state_time_ns = None
        self.prediction_error_tracker = PredictionErrorTracker(
            PREDICTION_HORIZONS
        )

        self.initial_uav_x = 0.0
        self.initial_uav_y = 0.0
        self.initial_uav_z = 0.0
        self.initial_uav_vx = 0.0
        self.initial_uav_vy = 0.0
        self.initial_uav_vz = 0.0
        self.current_uav_yaw = math.nan
        self.observation_yaw_command = math.nan
        self.desired_observation_yaw = math.nan
        self.uav_state_received = False
        self.vehicle_status_received = False
        self.uav_state_time_ns = None
        self.vehicle_status_time_ns = None
        self.offboard_active = False
        self.vehicle_armed = False
        self.pre_flight_checks_pass = False
        self.preflight_wait_announced = False
        self.preflight_checks_announced = False

        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_z = 0.0
        self.sim_vx = 0.0
        self.sim_vy = 0.0
        self.sim_vz = 0.0
        self.sim_time = 0.0
        self.started = False
        self.hit = False
        self.completed = False
        self.outcome = ''
        self.failure_reason = ''
        self.failure_detail = ''
        self.takeoff_requested = False
        self.takeoff_complete = False
        self.intercept_requested = False
        self.ready_for_takeoff_announced = False
        self.preflight_counter = 0
        self.flight_ready = False
        self.control_counter = 0
        self.takeoff_settle_counter = 0
        self.takeoff_x = None
        self.takeoff_y = None
        self.takeoff_z = None
        self.hold_x = None
        self.hold_y = None
        self.hold_z = None
        self.previous_relative_x = None
        self.previous_relative_y = None
        self.previous_relative_z = None
        self.previous_uav_x = None
        self.previous_uav_y = None
        self.previous_uav_z = None
        self.previous_target_x = None
        self.previous_target_y = None
        self.previous_target_z = None
        self.command_vx = None
        self.command_vy = None
        self.command_vz = None
        self.command_ax = 0.0
        self.command_ay = 0.0
        self.command_az = 0.0
        self.intercept_reference_vx = 0.0
        self.intercept_reference_vy = 0.0
        self.intercept_reference_vz = 0.0
        self.intercept_reference_ax = 0.0
        self.intercept_reference_ay = 0.0
        self.intercept_reference_az = 0.0
        self.measured_ax = 0.0
        self.measured_ay = 0.0
        self.measured_az = 0.0
        self.measured_acceleration = 0.0
        self.measured_vertical_acceleration = 0.0
        self.terminal_mode_active = False
        self.trajectory_plan_active = False
        self.planner_attempted_this_cycle = False
        self.planner_succeeded_this_cycle = False
        self.plan_hold_active = False
        self.optimization_attempted_this_cycle = False
        self.optimization_succeeded_this_cycle = False
        self.intercept_control_cycles = (
            getattr(self, 'intercept_control_cycles', 0) + 1
        )
        self.trajectory_planner_type = 'PURSUIT'
        self.guidance_phase = 'FOLLOW'
        self.minco_ever_engaged = False
        self.consecutive_plan_failures = 0
        self.plan_attempt_count = 0
        self.plan_success_count = 0
        self.plan_hold_count = 0
        self.intercept_control_cycles = 0
        self.minco_execution_cycles = 0
        self.optimization_attempt_count = 0
        self.optimization_success_count = 0
        self.planner_failure_counts = {}
        self.planner_failure_reason = PlanningFailureReason.NONE.value
        self.horizontal_min_time = math.nan
        self.vertical_min_time = math.nan
        self.sea_safe_min_time = math.nan
        self.planner_search_min_time = math.nan
        self.planner_search_max_time = math.nan
        self.sea_safety_state = SeaSafetyState.SAFE.value
        self.sea_safety_margin = math.inf
        self.first_unrecoverable_safety_detail = ''
        self.minimum_sea_safety_margin = math.inf
        self.first_safety_warning_time = None
        self.first_safety_brake_time = None
        self.final_relative_speed = math.nan
        self.final_vertical_relative_speed = math.nan
        self.wall_dt = self.dt
        self.last_control_wall_time = None
        self.callback_compute_time = 0.0
        self.prediction_compute_time = 0.0
        self.planner_compute_time = 0.0
        self.minco_generation_compute_time = 0.0
        self.minco_optimization_compute_time = 0.0
        self.guidance_compute_time = 0.0
        self.safety_compute_time = 0.0
        self.logging_compute_time = 0.0
        self.timing_samples = {
            name: [] for name in (
                'sim_dt',
                'wall_dt',
                'callback_compute_time',
                'prediction_compute_time',
                'planner_compute_time',
                'minco_generation_compute_time',
                'minco_optimization_compute_time',
                'guidance_compute_time',
                'safety_compute_time',
                'logging_compute_time',
            )
        }
        self.timing_samples_by_phase = {
            phase: {name: [] for name in self.timing_samples}
            for phase in ('FOLLOW', 'INTERCEPT')
        }
        self.retained_terminal_plan = None
        self.retained_terminal_plan_time_ns = None
        self.retained_terminal_plan_age = 0.0
        self.terminal_descent_committed = False
        self.planned_minco_target_curve_weight = 0.0
        self.planned_minco_constraint_penalty = 0.0
        self.planned_minco_optimization_iterations = 0
        self.planned_minco_piece_durations = ()
        self.guidance_altitude_reference = math.nan
        self.guidance_closing_speed = 0.0
        self.planned_closing_speed = 0.0
        self.planned_max_horizontal_speed = 0.0
        self.planned_max_vertical_speed = 0.0
        self.planned_max_horizontal_acceleration = 0.0
        self.planned_max_vertical_acceleration = 0.0
        self.previous_measured_vx = None
        self.previous_measured_vy = None
        self.previous_measured_vz = None
        self.previous_velocity_timestamp_us = None
        self.last_constraint_warning_time = -math.inf
        self.constraint_violation_cycles = 0
        self.active_constraint_violations = []
        self.mode_loss_cycles = 0
        self.minimum_distance = math.inf
        self.closest_horizontal_distance = math.inf
        self.closest_vertical_error = math.nan
        self.max_observed_horizontal_speed = 0.0
        self.max_observed_vertical_speed = 0.0
        self.max_observed_horizontal_acceleration = 0.0
        self.max_observed_vertical_acceleration = 0.0

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.log_start_time_ns = None

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        if self.enable_gazebo_control:
            self.get_logger().warn(
                'GAZEBO CONTROL ENABLED: do not run another PX4 controller.'
            )
        else:
            self.get_logger().warn(
                'SIMULATION ONLY: this node does not publish PX4 commands.'
            )
        self.get_logger().info(
            'Waiting for target state and UAV initial state.'
        )
        self.get_logger().info(
            'CONTROL LIMITS: '
            f'horizontal speed command={self.command_speed_limit:.2f} '
            f'm/s | horizontal acceleration command='
            f'{self.limit_horizontal_acceleration(self.max_acceleration):.2f} '
            f'm/s^2 | actual hard acceleration='
            f'{self.max_actual_horizontal_acceleration:.2f} m/s^2'
        )
        self.get_logger().info(
            'FINITE-HORIZON INTERCEPT: '
            f'{self.intercept_guidance_horizon_min:.2f}-'
            f'{self.intercept_guidance_horizon_max:.2f} s | '
            f'closing speed={self.terminal_min_closing_speed:.2f}-'
            f'{self.terminal_closing_speed:.2f} m/s | '
            f'control lookahead={self.terminal_control_lookahead:.2f} s'
        )
        self.get_logger().info(
            'TERMINAL TRAJECTORY GENERATOR: '
            + (
                f'MINCO-T3 | pieces={self.minco_piece_count} | '
                f'target-curve weight={self.minco_target_curve_weight:.2f} | '
                'geometric optimization='
                f'{self.enable_minco_geometric_optimization} | '
                'quadrature='
                f'{self.minco_quadrature_intervals_per_piece}/piece'
                if self.enable_minco_planner
                else 'single-piece quintic compatibility mode'
            )
            + ' | infeasible-plan fallback=HOLD/SAFE_WAIT'
        )
        self.get_logger().info(
            'SMOOTH TAKEOFF: velocity control on all axes | '
            f'XY acceleration='
            f'{self.takeoff_max_horizontal_acceleration:.2f} m/s^2 | '
            f'Z speed={self.takeoff_max_vertical_speed:.2f} m/s | '
            f'Z acceleration='
            f'{self.takeoff_max_vertical_acceleration:.2f} m/s^2 | '
            f'XY blend={self.takeoff_horizontal_start_height:.2f}-'
            f'{self.takeoff_horizontal_full_height:.2f} m AGL'
        )
        self.get_logger().info(
            'FRONT-VIEW DESCENT: '
            f'terminal flight-path angle='
            f'{math.degrees(self.terminal_dive_angle):.1f} deg | '
            f'max target depression='
            f'{math.degrees(self.front_camera_max_depression_angle):.1f} deg'
        )
        self.get_logger().info(
            'TARGET OBSERVATION YAW: enabled | '
            f'max yaw rate={self.max_observation_yaw_rate:.2f} rad/s'
        )
        if self.enable_maneuver_prediction:
            self.get_logger().info(
                'INTERCEPT PREDICTOR: bounded adaptive CTRA | '
                f'guidance horizon='
                f'{self.intercept_guidance_horizon_min:.2f}-'
                f'{self.intercept_guidance_horizon_max:.2f} s | '
                f'curve horizon={self.maneuver_prediction_horizon:.2f} s | '
                f'max turn rate={self.max_target_turn_rate:.2f} rad/s | '
                f'max turn acceleration='
                f'{self.max_target_turn_acceleration:.2f} rad/s^2 | '
                f'reference acceleration limit='
                f'{self.intercept_reference_max_acceleration:.2f} m/s^2'
            )
        else:
            self.get_logger().info(
                'INTERCEPT PREDICTOR: constant velocity fallback'
            )

    def target_position_callback(self, msg):
        self.target_x = float(msg.x)
        self.target_y = float(msg.y)
        self.target_z = float(msg.z)
        self.update_target_path_history(self.target_x, self.target_y)
        self.target_position_time_ns = (
            self.get_clock().now().nanoseconds
        )
        self.target_position_received = True

    def target_velocity_callback(self, msg):
        self.target_vx = float(msg.x)
        self.target_vy = float(msg.y)
        self.target_vz = float(msg.z)
        self.target_velocity_time_ns = self.get_clock().now().nanoseconds
        self.target_predictor.update_velocity(
            self.target_vx,
            self.target_vy,
            self.target_velocity_time_ns * 1e-9,
        )
        self.target_velocity_received = True

    def camera_observation_callback(self, msg):
        """Store the latest camera-only USV position for CSV evaluation."""
        position = (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        )
        self.camera_measurement_valid = bool(
            msg.valid and all(math.isfinite(value) for value in position)
        )
        self.camera_measurement = position
        self.camera_confidence = float(msg.confidence)
        self.camera_measurement_time_ns = self.get_clock().now().nanoseconds

    def filtered_target_state_callback(self, msg):
        """Store the camera/Kalman USV state for independent evaluation."""
        position = (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        )
        velocity = (
            float(msg.velocity.x),
            float(msg.velocity.y),
            float(msg.velocity.z),
        )
        self.kf_state_valid = bool(
            msg.valid
            and all(math.isfinite(value) for value in position + velocity)
        )
        self.kf_position = position
        self.kf_velocity = velocity
        self.kf_state_time_ns = self.get_clock().now().nanoseconds

    def command_callback(self, msg):
        command = msg.data.strip().upper()

        if command not in ('X', 'Y'):
            self.get_logger().warn(
                f'Ignoring unknown command: {msg.data!r}'
            )
            return

        if not self.enable_gazebo_control:
            self.get_logger().warn(
                f'{command} command is only used in Gazebo control mode.'
            )
            return

        if self.completed:
            self.get_logger().warn(
                f'{command} ignored: interception already completed.'
            )
            return

        if command == 'X':
            if self.takeoff_requested:
                self.get_logger().info(
                    'X ignored: takeoff/follow sequence already requested.'
                )
                return

            if not self.flight_ready:
                self.get_logger().warn(
                    'X rejected: flight preparation is not ready; wait for '
                    '/simulation/impact/flight_ready=true.'
                )
                return
            self.takeoff_requested = True
            self.log_start_time_ns = (
                self.get_clock().now().nanoseconds
            )
            self.begin_csv_logging()
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
            )
            self.get_logger().info(
                'X accepted. Requesting PX4 arming and starting the '
                'takeoff/follow sequence.'
            )
            return

        if not self.takeoff_requested:
            self.get_logger().warn(
                'Y rejected: send X first to start takeoff and follow.'
            )
            return

        if not self.takeoff_complete:
            self.get_logger().warn(
                'Y rejected: wait until FOLLOW MODE is active.'
            )
            return

        if self.started:
            self.get_logger().info(
                'Y ignored: interception is already active.'
            )
            return

        self.intercept_requested = True
        self.get_logger().info(
            'Y command received. Interception requested.'
        )

    def vehicle_position_callback(self, msg):
        current_vx = (
            float(msg.vx) if math.isfinite(msg.vx) else 0.0
        )
        current_vy = (
            float(msg.vy) if math.isfinite(msg.vy) else 0.0
        )
        current_vz = (
            float(msg.vz) if math.isfinite(msg.vz) else 0.0
        )
        timestamp_us = int(
            getattr(msg, 'timestamp_sample', 0)
            or getattr(msg, 'timestamp', 0)
        )

        if (
            self.previous_measured_vx is not None
            and timestamp_us > 0
            and self.previous_velocity_timestamp_us is not None
        ):
            sample_dt = (
                timestamp_us - self.previous_velocity_timestamp_us
            ) * 1e-6

            if 0.001 <= sample_dt <= 0.5:
                self.measured_ax = (
                    current_vx - self.previous_measured_vx
                ) / sample_dt
                self.measured_ay = (
                    current_vy - self.previous_measured_vy
                ) / sample_dt
                self.measured_acceleration = math.hypot(
                    self.measured_ax,
                    self.measured_ay,
                )
                if self.previous_measured_vz is not None:
                    self.measured_az = (
                        current_vz - self.previous_measured_vz
                    ) / sample_dt
                    self.measured_vertical_acceleration = abs(
                        self.measured_az
                    )

        self.previous_measured_vx = current_vx
        self.previous_measured_vy = current_vy
        self.previous_measured_vz = current_vz
        if timestamp_us > 0:
            self.previous_velocity_timestamp_us = timestamp_us

        self.initial_uav_x = float(msg.x)
        self.initial_uav_y = float(msg.y)
        self.initial_uav_z = float(msg.z)
        self.initial_uav_vx = current_vx
        self.initial_uav_vy = current_vy
        self.initial_uav_vz = current_vz
        heading = float(getattr(msg, 'heading', math.nan))
        if math.isfinite(heading):
            self.current_uav_yaw = self.wrap_angle(heading)
        self.uav_state_time_ns = self.get_clock().now().nanoseconds
        self.uav_state_received = True

    def vehicle_status_callback(self, msg):
        previous_offboard = self.offboard_active
        previous_armed = self.vehicle_armed

        self.vehicle_status_received = True
        self.vehicle_status_time_ns = (
            self.get_clock().now().nanoseconds
        )
        self.offboard_active = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        self.vehicle_armed = (
            msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        self.pre_flight_checks_pass = bool(msg.pre_flight_checks_pass)
        if self.offboard_active and not previous_offboard:
            self.get_logger().info('PX4 confirmed OFFBOARD mode.')
        if self.vehicle_armed and not previous_armed:
            self.get_logger().info('PX4 confirmed vehicle armed.')

    def timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    @staticmethod
    def wrap_angle(angle):
        """Wrap an angle to [-pi, pi)."""
        return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def target_observation_yaw(uav_x, uav_y, target_x, target_y):
        """Implement psi_e = atan2(e2^T(q-p), e1^T(q-p))."""
        delta_x = float(target_x) - float(uav_x)
        delta_y = float(target_y) - float(uav_y)
        if math.hypot(delta_x, delta_y) <= 1e-9:
            return math.nan
        return math.atan2(delta_y, delta_x)

    @classmethod
    def rate_limited_yaw(cls, current, desired, maximum_step):
        """Move toward desired yaw along the shortest wrapped direction."""
        if not math.isfinite(desired):
            return current
        if not math.isfinite(current):
            return cls.wrap_angle(desired)
        error = cls.wrap_angle(desired - current)
        step = max(min(error, maximum_step), -maximum_step)
        return cls.wrap_angle(current + step)

    def update_observation_yaw(self):
        """Keep the commanded body heading pointed toward the target."""
        target_x, target_y, _ = self.estimated_target_position()
        desired = self.target_observation_yaw(
            self.sim_x,
            self.sim_y,
            target_x,
            target_y,
        )
        self.desired_observation_yaw = desired
        reference = self.observation_yaw_command
        if not math.isfinite(reference):
            reference = self.current_uav_yaw
        self.observation_yaw_command = self.rate_limited_yaw(
            reference,
            desired,
            self.max_observation_yaw_rate * self.dt,
        )
        return self.observation_yaw_command

    def publish_offboard_mode(self):
        if self.offboard_pub is None:
            return

        msg = OffboardControlMode()
        msg.timestamp = self.timestamp()
        velocity_control = self.takeoff_requested and not self.completed
        msg.position = not velocity_control
        msg.velocity = velocity_control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def publish_vehicle_command(self, command, param1, param2=0.0):
        if self.command_pub is None:
            return

        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_flight_ready(self, ready):
        """Publish the ground-preparation gate used by both command clients."""
        ready = bool(ready)
        message = Bool()
        message.data = ready
        self.flight_ready_pub.publish(message)
        if ready and not self.flight_ready:
            self.get_logger().info(
                'FLIGHT READY | PX4 is disarmed in OFFBOARD ground hold; '
                'X may now start UAV takeoff and USV motion together.'
            )
        elif self.flight_ready and not ready and not self.takeoff_requested:
            self.get_logger().warn(
                'FLIGHT READY lost while waiting for X.'
            )
        self.flight_ready = ready

    def prepare_flight_on_ground(self):
        """Prestream and enter Offboard while remaining safely disarmed."""
        self.publish_offboard_mode()
        self.publish_gazebo_setpoint(
            self.takeoff_x,
            self.takeoff_y,
            self.takeoff_z,
            track_target_yaw=False,
        )
        self.preflight_counter += 1

        mode_retry_due = (
            self.preflight_counter >= self.offboard_prestream_cycles
            and (
                self.preflight_counter - self.offboard_prestream_cycles
            ) % self.px4_command_retry_cycles == 0
        )
        if mode_retry_due and not self.offboard_active:
            self.get_logger().info(
                'Preflight: requesting PX4 OFFBOARD mode'
            )
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )

        disarm_retry_due = (
            self.preflight_counter % self.px4_command_retry_cycles == 0
        )
        if disarm_retry_due and self.vehicle_armed:
            self.get_logger().warn(
                'PX4 armed before X; requesting safety disarm.'
            )
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                0.0,
            )

        status_fresh = False
        if self.vehicle_status_time_ns is not None:
            status_age = (
                self.get_clock().now().nanoseconds
                - self.vehicle_status_time_ns
            ) * 1e-9
            status_fresh = status_age <= self.vehicle_status_timeout
        if not self.pre_flight_checks_pass:
            if not self.preflight_wait_announced:
                self.get_logger().info(
                    'PREPARING | waiting for PX4 preflight checks'
                )
                self.preflight_wait_announced = True
        elif not self.preflight_checks_announced:
            self.get_logger().info('PX4 preflight checks passed')
            self.preflight_checks_announced = True
        self.publish_flight_ready(
            self.offboard_active
            and not self.vehicle_armed
            and status_fresh
            and self.pre_flight_checks_pass
        )

    def publish_gazebo_setpoint(
        self,
        position_x,
        position_y,
        position_z,
        velocity_x=None,
        velocity_y=None,
        track_target_yaw=True,
    ):
        if self.setpoint_pub is None:
            return

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp()
        msg.position = [
            float(position_x),
            float(position_y),
            float(position_z),
        ]

        if velocity_x is None or velocity_y is None:
            msg.velocity = [math.nan, math.nan, math.nan]
        else:
            msg.velocity = [
                float(velocity_x),
                float(velocity_y),
                0.0,
            ]

        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        if track_target_yaw:
            msg.yaw = self.update_observation_yaw()
        else:
            msg.yaw = (
                self.current_uav_yaw
                if math.isfinite(self.current_uav_yaw)
                else math.nan
            )
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def publish_gazebo_velocity_setpoint(
        self,
        velocity_x,
        velocity_y,
        velocity_z,
    ):
        if self.setpoint_pub is None:
            return

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [
            float(velocity_x),
            float(velocity_y),
            float(velocity_z),
        ]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self.update_observation_yaw()
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def publish_takeoff_follow_setpoint(
        self,
        velocity_x,
        velocity_y,
        velocity_z,
    ):
        """Climb and follow with a continuous three-axis velocity command."""
        if self.setpoint_pub is None:
            return

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [
            float(velocity_x),
            float(velocity_y),
            float(velocity_z),
        ]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self.update_observation_yaw()
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    @staticmethod
    def takeoff_horizontal_scale(clearance, start_height, full_height):
        """Smoothly enable horizontal motion after confirmed liftoff."""
        if clearance <= start_height:
            return 0.0
        if clearance >= full_height:
            return 1.0
        progress = (
            (clearance - start_height)
            / (full_height - start_height)
        )
        return progress * progress * (3.0 - 2.0 * progress)

    def reset_evaluation(self):
        self.planner_epoch = getattr(self, 'planner_epoch', 0) + 1
        planner_future = getattr(self, 'planner_future', None)
        if planner_future is not None:
            if planner_future.cancel():
                self.planner_future = None
        self.completed = False
        self.hit = False
        self.outcome = ''
        self.failure_reason = ''
        self.failure_detail = ''
        self.constraint_violation_cycles = 0
        self.active_constraint_violations = []
        self.mode_loss_cycles = 0
        self.minimum_distance = math.inf
        self.closest_horizontal_distance = math.inf
        self.closest_vertical_error = math.nan
        self.max_observed_horizontal_speed = 0.0
        self.max_observed_vertical_speed = 0.0
        self.max_observed_horizontal_acceleration = 0.0
        self.max_observed_vertical_acceleration = 0.0
        self.terminal_mode_active = False
        self.guidance_phase = 'FAR_GUIDANCE'
        self.minco_ever_engaged = False
        self.consecutive_plan_failures = 0
        self.plan_attempt_count = 0
        self.plan_success_count = 0
        self.plan_hold_count = 0
        self.intercept_control_cycles = 0
        self.minco_execution_cycles = 0
        self.optimization_attempt_count = 0
        self.optimization_success_count = 0
        self.planner_failure_counts = {}
        self.planner_failure_reason = PlanningFailureReason.NONE.value
        self.sea_safety_state = SeaSafetyState.SAFE.value
        self.sea_safety_margin = math.inf
        self.first_unrecoverable_safety_detail = ''
        self.minimum_sea_safety_margin = math.inf
        self.first_safety_warning_time = None
        self.first_safety_brake_time = None
        self.final_relative_speed = math.nan
        self.final_vertical_relative_speed = math.nan
        self.retained_terminal_plan = None
        for values in getattr(self, 'timing_samples', {}).values():
            values.clear()
        for phase_samples in getattr(
            self,
            'timing_samples_by_phase',
            {},
        ).values():
            for values in phase_samples.values():
                values.clear()
        self.retained_terminal_plan_time_ns = None
        self.retained_terminal_plan_age = 0.0
        self.terminal_descent_committed = False
        self.reset_trajectory_plan_diagnostics()

    @staticmethod
    def closest_relative_vector(relative_start, relative_end):
        delta = [
            end - start
            for start, end in zip(relative_start, relative_end)
        ]
        denominator = sum(value * value for value in delta)
        if denominator <= 1e-12:
            fraction = 0.0
        else:
            fraction = -sum(
                start * change
                for start, change in zip(relative_start, delta)
            ) / denominator
            fraction = min(max(fraction, 0.0), 1.0)

        return tuple(
            start + fraction * change
            for start, change in zip(relative_start, delta)
        )

    def update_closest_approach(self, relative_vector):
        horizontal_distance = math.hypot(
            relative_vector[0],
            relative_vector[1],
        )
        vertical_error = relative_vector[2]
        distance = math.sqrt(
            horizontal_distance * horizontal_distance
            + vertical_error * vertical_error
        )
        if distance < self.minimum_distance:
            self.minimum_distance = distance
            self.closest_horizontal_distance = horizontal_distance
            self.closest_vertical_error = vertical_error

    def update_interval_closest_approach(
        self,
        relative_start,
        relative_end,
    ):
        closest = self.closest_relative_vector(
            relative_start,
            relative_end,
        )
        self.update_closest_approach(closest)

    def runtime_failure_reason(self):
        if not self.started or self.completed:
            return None

        values = (
            self.sim_x,
            self.sim_y,
            self.sim_z,
            self.sim_vx,
            self.sim_vy,
            self.sim_vz,
            self.target_x,
            self.target_y,
            self.target_z,
            self.target_vx,
            self.target_vy,
            self.target_vz,
        )
        if not all(math.isfinite(value) for value in values):
            self.failure_detail = 'non-finite UAV or target state'
            return 'INVALID_STATE'

        now_ns = self.get_clock().now().nanoseconds
        state_timestamps = [
            ('target_position', self.target_position_time_ns),
            ('target_velocity', self.target_velocity_time_ns),
            ('uav_position', self.uav_state_time_ns),
        ]
        for name, timestamp in state_timestamps:
            if timestamp is None:
                self.failure_detail = f'{name} was never received'
                return 'STATE_LOST'
            age = (now_ns - timestamp) * 1e-9
            if age > self.state_timeout:
                self.failure_detail = (
                    f'{name} age {age:.3f} s exceeded '
                    f'{self.state_timeout:.3f} s'
                )
                return 'STATE_LOST'

        if self.enable_gazebo_control:
            if self.vehicle_status_time_ns is None:
                self.failure_detail = 'vehicle_status was never received'
                return 'STATE_LOST'
            status_age = (
                now_ns - self.vehicle_status_time_ns
            ) * 1e-9
            if status_age > self.vehicle_status_timeout:
                self.failure_detail = (
                    f'vehicle_status age {status_age:.3f} s exceeded '
                    f'{self.vehicle_status_timeout:.3f} s'
                )
                return 'STATE_LOST'

        if self.enable_gazebo_control:
            if self.offboard_active and self.vehicle_armed:
                self.mode_loss_cycles = 0
            else:
                self.mode_loss_cycles += 1
            if self.mode_loss_cycles >= self.mode_loss_grace_cycles:
                self.failure_detail = (
                    f'offboard={self.offboard_active}, '
                    f'armed={self.vehicle_armed}'
                )
                return 'OFFBOARD_LOST'

        if (
            self.constraint_violation_cycles
            >= self.constraint_violation_cycles_limit
        ):
            self.failure_detail = (
                f'hard limit exceeded: '
                f'{", ".join(self.active_constraint_violations)}; '
                f'persisted for '
                f'{self.constraint_violation_cycles * self.dt:.3f} s'
            )
            return 'CONSTRAINT_VIOLATION'

        return None

    def monitor_actual_constraints(self):
        actual_speed = math.hypot(
            self.initial_uav_vx,
            self.initial_uav_vy,
        )
        actual_vertical_speed = abs(self.initial_uav_vz)
        speed_violation = actual_speed > self.max_speed + 0.1
        (
            command_horizontal_acceleration_limit,
            command_vertical_acceleration_limit,
        ) = self.command_acceleration_limits(
            self.terminal_mode_active
        )
        acceleration_violation = (
            self.measured_acceleration
            > self.max_actual_horizontal_acceleration
        )
        vertical_speed_violation = (
            actual_vertical_speed > self.max_vertical_speed + 0.1
        )
        vertical_acceleration_violation = (
            self.measured_vertical_acceleration
            > self.max_actual_vertical_acceleration
        )
        guidance_response_warning = (
            self.terminal_mode_active
            and (
                self.measured_acceleration
                > command_horizontal_acceleration_limit + 0.5
                or self.measured_vertical_acceleration
                > command_vertical_acceleration_limit + 0.5
            )
        )
        violation = (
            speed_violation
            or acceleration_violation
            or vertical_speed_violation
            or vertical_acceleration_violation
        )
        active_violations = []
        if speed_violation:
            active_violations.append('horizontal_speed')
        if acceleration_violation:
            active_violations.append('horizontal_acceleration')
        if vertical_speed_violation:
            active_violations.append('vertical_speed')
        if vertical_acceleration_violation:
            active_violations.append('vertical_acceleration')
        self.active_constraint_violations = active_violations

        if self.started:
            self.max_observed_horizontal_speed = max(
                self.max_observed_horizontal_speed,
                actual_speed,
            )
            self.max_observed_vertical_speed = max(
                self.max_observed_vertical_speed,
                actual_vertical_speed,
            )
            self.max_observed_horizontal_acceleration = max(
                self.max_observed_horizontal_acceleration,
                self.measured_acceleration,
            )
            self.max_observed_vertical_acceleration = max(
                self.max_observed_vertical_acceleration,
                self.measured_vertical_acceleration,
            )
            if violation:
                self.constraint_violation_cycles += 1
            else:
                self.constraint_violation_cycles = 0

        if (
            (violation or guidance_response_warning)
            and self.sim_time - self.last_constraint_warning_time >= 1.0
        ):
            self.last_constraint_warning_time = self.sim_time
            warning_type = (
                'ACTUAL HARD CONSTRAINT WARNING'
                if violation
                else 'TERMINAL TRACKING RESPONSE'
            )
            self.get_logger().warn(
                f'{warning_type} | '
                f'speed={actual_speed:.2f}/{self.max_speed:.2f} m/s | '
                f'acceleration={self.measured_acceleration:.2f}/'
                f'{self.max_actual_horizontal_acceleration:.2f} '
                f'm/s^2 hard '
                f'(command shaping '
                f'{command_horizontal_acceleration_limit:.2f}) | '
                f'vertical speed={actual_vertical_speed:.2f}/'
                f'{self.max_vertical_speed:.2f} m/s | '
                f'vertical acceleration='
                f'{self.measured_vertical_acceleration:.2f}/'
                f'{self.max_actual_vertical_acceleration:.2f} '
                f'm/s^2 hard '
                f'(command shaping '
                f'{command_vertical_acceleration_limit:.2f})'
            )

        failed = (
            self.started
            and self.constraint_violation_cycles
            >= self.constraint_violation_cycles_limit
        )
        if failed:
            self.failure_detail = (
                f'hard limit exceeded: '
                f'{", ".join(self.active_constraint_violations)}; '
                f'persisted for '
                f'{self.constraint_violation_cycles * self.dt:.3f} s'
            )
        return failed

    def estimated_target_position(self):
        if self.target_position_time_ns is None:
            return self.target_x, self.target_y, self.target_z

        age = max(
            (
                self.get_clock().now().nanoseconds
                - self.target_position_time_ns
            ) * 1e-9,
            0.0
        )

        return self.predict_target_state(
            self.target_x,
            self.target_y,
            self.target_z,
            age,
        )[:3]

    def predict_target_state(
        self,
        target_x,
        target_y,
        target_z,
        horizon,
    ):
        """Predict target state using maneuver history or the CV fallback."""
        if self.enable_maneuver_prediction:
            return self.target_predictor.predict(
                target_x,
                target_y,
                target_z,
                self.target_vx,
                self.target_vy,
                self.target_vz,
                horizon,
            )

        return (
            target_x + self.target_vx * horizon,
            target_y + self.target_vy * horizon,
            target_z + self.target_vz * horizon,
            self.target_vx,
            self.target_vy,
            self.target_vz,
        )

    def update_target_path_history(self, target_x, target_y):
        if not self.target_path_history:
            self.target_path_history.append((0.0, target_x, target_y))
            return

        _, previous_x, previous_y = self.target_path_history[-1]
        segment_length = math.hypot(
            target_x - previous_x,
            target_y - previous_y,
        )
        if segment_length <= 1e-4:
            return

        self.target_path_distance += segment_length
        self.target_path_history.append((
            self.target_path_distance,
            target_x,
            target_y,
        ))

        retained_distance = max(3.0 * self.follow_distance, 100.0)
        oldest_required = self.target_path_distance - retained_distance
        while (
            len(self.target_path_history) > 2
            and self.target_path_history[1][0] < oldest_required
        ):
            self.target_path_history.popleft()

    @staticmethod
    def sample_path_reference(path_history, path_distance, speed):
        """Interpolate position and tangent velocity at an arc distance."""
        if not path_history:
            return None

        if path_distance <= path_history[0][0]:
            _, x, y = path_history[0]
            return x, y, 0.0, 0.0

        history = list(path_history)
        previous = history[0]
        for current in history[1:]:
            if path_distance <= current[0]:
                segment_length = current[0] - previous[0]
                if segment_length <= 1e-9:
                    return current[1], current[2], 0.0, 0.0
                fraction = (
                    (path_distance - previous[0]) / segment_length
                )
                x = previous[1] + fraction * (
                    current[1] - previous[1]
                )
                y = previous[2] + fraction * (
                    current[2] - previous[2]
                )
                direction_x = (current[1] - previous[1]) / segment_length
                direction_y = (current[2] - previous[2]) / segment_length
                return (
                    x,
                    y,
                    speed * direction_x,
                    speed * direction_y,
                )
            previous = current

        _, x, y = history[-1]
        if len(history) < 2:
            return x, y, 0.0, 0.0

        before_last = history[-2]
        segment_length = history[-1][0] - before_last[0]
        if segment_length <= 1e-9:
            return x, y, 0.0, 0.0
        return (
            x,
            y,
            speed * (x - before_last[1]) / segment_length,
            speed * (y - before_last[2]) / segment_length,
        )

    def calculate_follow_reference(self, target_x, target_y):
        target_speed = math.hypot(self.target_vx, self.target_vy)
        if self.target_path_history:
            available_distance = (
                self.target_path_distance
                - self.target_path_history[0][0]
            )
            requested_distance = min(
                self.follow_distance,
                available_distance,
            )
            reference_distance = (
                self.target_path_distance - requested_distance
            )
            reference = TrajectoryImpactSim.sample_path_reference(
                self.target_path_history,
                reference_distance,
                target_speed,
            )
            if reference is not None:
                if requested_distance < self.follow_distance:
                    return reference[0], reference[1], 0.0, 0.0
                return reference

        target_speed = math.hypot(
            self.target_vx,
            self.target_vy
        )

        if target_speed > 1e-6:
            direction_x = self.target_vx / target_speed
            direction_y = self.target_vy / target_speed
        else:
            direction_x = 1.0
            direction_y = 0.0

        return (
            target_x - self.follow_distance * direction_x,
            target_y - self.follow_distance * direction_y,
            self.target_vx,
            self.target_vy,
        )

    def plan_follow_velocity(
        self,
        follow_x,
        follow_y,
        follow_vx,
        follow_vy,
    ):
        """Approach the moving follow point without overshooting it."""
        error_x = follow_x - self.sim_x
        error_y = follow_y - self.sim_y
        distance = math.hypot(error_x, error_y)

        if distance <= 1e-6:
            return self.clamp_command_speed(
                follow_vx,
                follow_vy,
            )

        direction_x = error_x / distance
        direction_y = error_y / distance
        braking_speed = math.sqrt(
            2.0 * self.follow_max_acceleration * distance
        )
        closing_speed = min(
            self.follow_position_gain * distance,
            self.follow_max_closing_speed,
            braking_speed,
        )

        return self.clamp_command_speed(
            follow_vx + closing_speed * direction_x,
            follow_vy + closing_speed * direction_y,
        )

    def calculate_intercept_solution(
        self,
        uav_x,
        uav_y,
        uav_z,
        target_x,
        target_y,
        target_z,
    ):
        distance = math.sqrt(
            (target_x - uav_x) ** 2
            + (target_y - uav_y) ** 2
            + (target_z - uav_z) ** 2
        )
        horizon_ratio = min(
            distance / self.intercept_guidance_horizon_distance,
            1.0,
        )
        guidance_horizon = (
            self.intercept_guidance_horizon_min
            + horizon_ratio
            * (
                self.intercept_guidance_horizon_max
                - self.intercept_guidance_horizon_min
            )
        )
        predicted_state = self.predict_target_state(
            target_x,
            target_y,
            target_z,
            guidance_horizon,
        )
        self.intercept_reference_vx = predicted_state[3]
        self.intercept_reference_vy = predicted_state[4]
        self.intercept_reference_vz = predicted_state[5]
        if (
            getattr(self, 'enable_maneuver_prediction', True)
            and self.target_predictor.maneuver_model_active
        ):
            acceleration = self.target_predictor.acceleration(
                predicted_state[3],
                predicted_state[4],
                guidance_horizon,
            )
        else:
            acceleration = (0.0, 0.0, 0.0)
        self.intercept_reference_ax = acceleration[0]
        self.intercept_reference_ay = acceleration[1]
        self.intercept_reference_az = acceleration[2]

        return (
            predicted_state[0],
            predicted_state[1],
            predicted_state[2],
            guidance_horizon,
        )

    def continuous_intercept_solution(
        self,
        target_x,
        target_y,
        target_z,
    ):
        """Update the bounded position/velocity/acceleration reference."""
        (
            raw_x,
            raw_y,
            raw_z,
            guidance_horizon,
        ) = self.calculate_intercept_solution(
            self.sim_x,
            self.sim_y,
            self.sim_z,
            target_x,
            target_y,
            target_z,
        )
        reference = self.intercept_reference_filter.update(
            (raw_x, raw_y, raw_z),
            (
                self.intercept_reference_vx,
                self.intercept_reference_vy,
                self.intercept_reference_vz,
            ),
            (
                self.control_dt
                if self.enable_gazebo_control
                else self.dt
            ),
        )
        self.intercept_reference_vx = reference.vx
        self.intercept_reference_vy = reference.vy
        self.intercept_reference_vz = reference.vz
        self.intercept_reference_ax = reference.ax
        self.intercept_reference_ay = reference.ay
        self.intercept_reference_az = reference.az

        return (
            reference.x,
            reference.y,
            reference.z,
            guidance_horizon,
        )

    def clamp_speed(self, vx, vy):
        speed = math.hypot(vx, vy)

        if speed <= self.max_speed:
            return vx, vy

        scale = self.max_speed / speed
        return vx * scale, vy * scale

    def clamp_command_speed(self, vx, vy):
        speed = math.hypot(vx, vy)

        if speed <= self.command_speed_limit:
            return vx, vy

        scale = self.command_speed_limit / speed
        return vx * scale, vy * scale

    def speed_governed_velocity(self, vx, vy):
        """Reduce forward demand when measured speed exceeds the soft cap."""
        vx, vy = self.clamp_command_speed(vx, vy)
        actual_speed = math.hypot(self.sim_vx, self.sim_vy)
        if (
            actual_speed <= self.command_speed_limit
            or actual_speed <= 1e-9
            or self.speed_governor_gain <= 0.0
        ):
            return vx, vy

        actual_direction_x = self.sim_vx / actual_speed
        actual_direction_y = self.sim_vy / actual_speed
        desired_forward_speed = (
            vx * actual_direction_x + vy * actual_direction_y
        )
        speed_excess = actual_speed - self.command_speed_limit
        governed_forward_speed = max(
            self.command_speed_limit
            - self.speed_governor_gain * speed_excess,
            0.0,
        )
        if desired_forward_speed <= governed_forward_speed:
            return vx, vy

        correction = desired_forward_speed - governed_forward_speed
        return (
            vx - correction * actual_direction_x,
            vy - correction * actual_direction_y,
        )

    def limit_horizontal_acceleration(self, requested_limit):
        """Reserve response margin below the measured hard acceleration."""
        limit = min(
            max(float(requested_limit), 0.1),
            self.max_acceleration,
        )
        if not self.enable_gazebo_control:
            return limit

        safe_actual_limit = max(
            self.max_actual_horizontal_acceleration
            - self.horizontal_acceleration_guard_margin,
            0.1,
        )
        return min(limit, safe_actual_limit)

    def target_planning_state(self, target_x, target_y, target_z, horizon):
        """Return predicted target position, velocity, and acceleration."""
        state = self.predict_target_state(
            target_x,
            target_y,
            target_z,
            horizon,
        )
        acceleration = (
            self.target_predictor.acceleration(
                state[3],
                state[4],
                horizon,
            )
            if (
                self.enable_maneuver_prediction
                and self.target_predictor.maneuver_model_active
            )
            else (0.0, 0.0, 0.0)
        )
        return (
            state[:3],
            state[3:6],
            acceleration,
        )

    def reset_trajectory_plan_diagnostics(self):
        """Clear diagnostics when no feasible terminal plan is active."""
        self.trajectory_plan_active = False
        self.trajectory_planner_type = 'PURSUIT'
        self.planned_minco_target_curve_weight = 0.0
        self.planned_minco_constraint_penalty = 0.0
        self.planned_minco_optimization_iterations = 0
        self.planned_minco_piece_durations = ()
        self.guidance_altitude_reference = math.nan
        self.guidance_closing_speed = 0.0
        self.planned_closing_speed = 0.0
        self.planned_max_horizontal_speed = 0.0
        self.planned_max_vertical_speed = 0.0
        self.planned_max_horizontal_acceleration = 0.0
        self.planned_max_vertical_acceleration = 0.0

    def terminal_trajectory_plan(self, target_x, target_y, target_z):
        """Search a dynamically feasible, sea-safe dynamic-horizon plan."""
        return self.intercept_trajectory_planner.plan(
            initial_position=(self.sim_x, self.sim_y, self.sim_z),
            initial_velocity=(self.sim_vx, self.sim_vy, self.sim_vz),
            initial_acceleration=(
                self.command_ax,
                self.command_ay,
                self.command_az,
            ),
            target_state_at_time=lambda horizon: (
                self.target_planning_state(
                    target_x,
                    target_y,
                    target_z,
                    horizon,
                )
            ),
            previous_acceleration=(
                self.command_ax,
                self.command_ay,
                self.command_az,
            ),
        )

    def _terminal_planning_snapshot(self, target_x, target_y, target_z):
        """Freeze inputs so the planning worker never reads changing state."""
        predictor = copy.deepcopy(self.target_predictor)
        target_velocity = (self.target_vx, self.target_vy, self.target_vz)
        maneuver_enabled = self.enable_maneuver_prediction

        def target_state_at_time(horizon):
            if maneuver_enabled:
                state = predictor.predict(
                    target_x,
                    target_y,
                    target_z,
                    *target_velocity,
                    horizon,
                )
            else:
                state = (
                    target_x + target_velocity[0] * horizon,
                    target_y + target_velocity[1] * horizon,
                    target_z + target_velocity[2] * horizon,
                    *target_velocity,
                )
            acceleration = (
                predictor.acceleration(state[3], state[4], horizon)
                if maneuver_enabled and predictor.maneuver_model_active
                else (0.0, 0.0, 0.0)
            )
            return state[:3], state[3:6], acceleration

        return {
            'initial_position': (self.sim_x, self.sim_y, self.sim_z),
            'initial_velocity': (self.sim_vx, self.sim_vy, self.sim_vz),
            'initial_acceleration': (
                self.command_ax,
                self.command_ay,
                self.command_az,
            ),
            'target_state_at_time': target_state_at_time,
            'previous_acceleration': (
                self.command_ax,
                self.command_ay,
                self.command_az,
            ),
        }

    def _run_terminal_planning_job(self, epoch, snapshot):
        plan = self.intercept_trajectory_planner.plan(**snapshot)
        return epoch, plan, self.intercept_trajectory_planner.last_diagnostics

    def poll_terminal_trajectory_plan(self, target_x, target_y, target_z):
        """Poll one worker result or submit a new 10 Hz planning job."""
        if self.planner_future is not None:
            if not self.planner_future.done():
                return None, None, False, False
            try:
                epoch, plan, diagnostics = self.planner_future.result()
            except Exception as error:  # pragma: no cover - ROS log path
                self.get_logger().error(f'MINCO worker failed: {error}')
                epoch, plan, diagnostics = self.planner_epoch, None, None
            self.planner_future = None
            if epoch != self.planner_epoch:
                return None, None, False, False
            return plan, diagnostics, False, True

        snapshot = self._terminal_planning_snapshot(
            target_x,
            target_y,
            target_z,
        )
        self.planner_future = self.planner_executor.submit(
            self._run_terminal_planning_job,
            self.planner_epoch,
            snapshot,
        )
        return None, None, True, False

    def retain_terminal_trajectory_plan(self, plan, timestamp_ns=None):
        """Remember a feasible trajectory for brief replanning dropouts."""
        if timestamp_ns is None:
            timestamp_ns = self.get_clock().now().nanoseconds
        self.retained_terminal_plan = plan
        self.retained_terminal_plan_time_ns = int(timestamp_ns)
        self.retained_terminal_plan_age = 0.0

    def retained_terminal_trajectory_sample(self, timestamp_ns=None):
        """Return a retained plan, shifted sample, and remaining time."""
        if (
            self.retained_terminal_plan is None
            or self.retained_terminal_plan_time_ns is None
        ):
            return None
        if timestamp_ns is None:
            timestamp_ns = self.get_clock().now().nanoseconds
        elapsed = max(
            (int(timestamp_ns) - self.retained_terminal_plan_time_ns) * 1e-9,
            0.0,
        )
        maximum_hold_time = float(
            getattr(self, 'terminal_plan_max_hold_time', math.inf)
        )
        if elapsed > maximum_hold_time + 1e-9:
            self.retained_terminal_plan = None
            self.retained_terminal_plan_time_ns = None
            self.retained_terminal_plan_age = 0.0
            return None
        self.retained_terminal_plan_age = elapsed
        sample_time = elapsed + self.terminal_control_lookahead
        if sample_time > self.retained_terminal_plan.duration + 1e-9:
            self.retained_terminal_plan = None
            self.retained_terminal_plan_time_ns = None
            self.retained_terminal_plan_age = 0.0
            return None
        remaining_time = max(
            self.retained_terminal_plan.duration - elapsed,
            0.0,
        )
        if remaining_time < float(
            getattr(self, 'terminal_plan_min_remaining_time', 0.0)
        ):
            self.retained_terminal_plan = None
            self.retained_terminal_plan_time_ns = None
            self.retained_terminal_plan_age = 0.0
            return None
        return self.retained_terminal_plan, sample_time, remaining_time

    def retained_plan_matches_target(
        self,
        plan,
        remaining_time,
        target_x,
        target_y,
        target_z,
    ):
        """Reject a retained endpoint after the target prediction moves."""
        target_state_at_time = getattr(self, 'target_planning_state', None)
        if target_state_at_time is None:
            return True
        predicted_position = target_state_at_time(
            target_x,
            target_y,
            target_z,
            remaining_time,
        )[0]
        endpoint_error = math.sqrt(sum(
            (predicted - planned) ** 2
            for predicted, planned in zip(
                predicted_position,
                plan.target_position,
            )
        ))
        descent_speed = max(float(getattr(self, 'sim_vz', 0.0)), 0.0)
        braking = max(
            float(
                getattr(
                    self,
                    'terminal_effective_vertical_braking_acceleration',
                    2.5,
                )
            ),
            0.1,
        )
        reserve = (
            float(getattr(self, 'terminal_contact_clearance', 0.05))
            + float(getattr(self, 'terminal_safety_margin', 0.02))
        )
        sea_margin = (
            float(getattr(self, 'sea_surface_z', 0.0))
            - float(getattr(self, 'sim_z', -math.inf))
            - descent_speed
            * float(getattr(self, 'terminal_safety_response_time', 0.0))
            - descent_speed * descent_speed / (2.0 * braking)
            - reserve
        )
        return (
            endpoint_error <= self.terminal_plan_target_error_limit
            and sea_margin > 0.0
            and remaining_time >= float(
                getattr(self, 'terminal_plan_min_remaining_time', 0.0)
            )
        )

    def pursuit_altitude_reference(self, horizontal_distance, target_z):
        """Follow the terminal dive line while retaining front visibility."""
        horizontal_distance = max(float(horizontal_distance), 0.0)
        target_z = float(target_z)
        capture_z = min(
            target_z - self.terminal_contact_clearance,
            self.sea_surface_z - self.terminal_contact_clearance,
        )
        dive_z = (
            capture_z
            - horizontal_distance * math.tan(self.terminal_dive_angle)
        )
        profile_z = max(self.flight_altitude, dive_z)

        # At very short horizontal range, continuing to hold a fixed height
        # would drive the target below the forward camera's lower FOV edge.
        # Lower the vehicle with the remaining horizontal gap so pursuit can
        # continue without stopping to reacquire the target vertically.
        visibility_height = max(
            horizontal_distance
            * math.tan(self.front_camera_max_depression_angle),
            self.terminal_contact_clearance,
        )
        visibility_z = target_z - visibility_height
        altitude_ceiling_z = TrajectoryImpactSim.terminal_altitude_ceiling(
            self,
            target_z,
        )
        return min(
            max(profile_z, visibility_z),
            self.sea_surface_z - self.terminal_contact_clearance,
            altitude_ceiling_z,
        )

    @staticmethod
    def normalize_optional_clearance(value, minimum_enabled, maximum):
        """Clamp an optional positive clearance while preserving zero."""
        requested = max(float(value), 0.0)
        if requested <= 0.0:
            return 0.0
        return min(max(requested, float(minimum_enabled)), float(maximum))

    @staticmethod
    def terminal_altitude_ceiling(controller, target_z):
        """
        Return the maximum allowed NED z for terminal pursuit.

        The geometric dive line is designed to reach the sea-surface capture
        height only at zero horizontal range.  When horizontal closure is
        slower than the dive, the vehicle can otherwise reach that height
        while the horizontal gap is still open and end in sea contact.

        NED z grows downwards, so a clearance constraint is an upper bound on
        z.  A requested sea clearance is relaxed to the top of the capture
        sphere when necessary.  This preserves at least one reachable point
        inside the sphere instead of turning a safety preference into an
        impossible capture.  Zero disables the ceiling and returns +infinity.

        This is a static helper so the unbound guidance methods stay usable
        from lightweight test controllers that are not full nodes.
        """
        impact_radius = float(getattr(controller, 'impact_radius', 0.25))
        minimum_clearance = getattr(
            controller,
            'terminal_minimum_clearance',
            None,
        )
        if minimum_clearance is None or float(minimum_clearance) <= 0.0:
            return math.inf
        capture_sphere_top = float(target_z) - impact_radius
        return max(
            capture_sphere_top,
            controller.sea_surface_z - minimum_clearance,
        )

    def terminal_fallback_altitude_reference(
        self,
        horizontal_distance,
        target_z,
    ):
        """Keep descending after terminal commitment despite plan loss."""
        pursuit_z = self.pursuit_altitude_reference(
            horizontal_distance,
            target_z,
        )
        if not self.terminal_descent_committed:
            return pursuit_z, False
        if horizontal_distance > self.terminal_descent_release_distance:
            self.terminal_descent_committed = False
            return pursuit_z, False

        capture_z = min(
            float(target_z) - self.terminal_contact_clearance,
            self.sea_surface_z - self.terminal_contact_clearance,
        )
        reference_z = max(pursuit_z, capture_z)
        altitude_ceiling_z = TrajectoryImpactSim.terminal_altitude_ceiling(
            self,
            target_z,
        )
        return min(reference_z, altitude_ceiling_z), True

    def pursuit_vertical_velocity(
        self,
        horizontal_distance,
        target_z,
        target_vz,
        altitude_reference,
        horizontal_closing_speed,
    ):
        """Track the dive line using feed-forward and altitude feedback."""
        capture_z = min(
            float(target_z) - self.terminal_contact_clearance,
            self.sea_surface_z - self.terminal_contact_clearance,
        )
        unclamped_dive_z = (
            capture_z
            - max(float(horizontal_distance), 0.0)
            * math.tan(self.terminal_dive_angle)
        )
        feedforward_vz = 0.0
        if unclamped_dive_z >= self.flight_altitude:
            feedforward_vz = max(
                float(horizontal_closing_speed),
                0.0,
            ) * math.tan(self.terminal_dive_angle)
            sea_clearance_z = (
                self.sea_surface_z - self.terminal_contact_clearance
            )
            if capture_z < sea_clearance_z:
                feedforward_vz += float(target_vz)

        desired_vz = (
            feedforward_vz
            + self.altitude_velocity_gain
            * (float(altitude_reference) - self.sim_z)
        )
        desired_vz = TrajectoryImpactSim.terminal_safe_vertical_velocity(
            self,
            desired_vz,
            target_z,
        )
        return max(
            min(desired_vz, self.max_vertical_speed),
            -self.max_vertical_speed,
        )

    def pursuit_closing_speed(
        self,
        horizontal_distance,
        vertical_distance=0.0,
        maximum_closing_speed=None,
    ):
        """
        Continue forward closure until the actual capture neighborhood.

        The braking law itself is unchanged: the closing speed still falls
        to zero at the capture neighbourhood.  Only the ceiling differs,
        because intercepting needs more radial authority than following.
        """
        if maximum_closing_speed is None:
            maximum_closing_speed = self.follow_max_closing_speed
        acceleration_limit = self.limit_horizontal_acceleration(
            self.terminal_max_acceleration
        )
        remaining_radius_squared = max(
            self.impact_radius * self.impact_radius
            - float(vertical_distance) * float(vertical_distance),
            0.0,
        )
        horizontal_capture_radius = math.sqrt(
            remaining_radius_squared
        )
        braking_distance = max(
            float(horizontal_distance) - horizontal_capture_radius,
            0.0,
        )
        return min(
            float(maximum_closing_speed),
            math.sqrt(2.0 * acceleration_limit * braking_distance),
        )

    def measured_horizontal_closing_speed(self, line_x, line_y):
        """Return actual UAV-minus-target velocity along the horizontal LOS."""
        return (
            (self.sim_vx - self.target_vx) * float(line_x)
            + (self.sim_vy - self.target_vy) * float(line_y)
        )

    def terminal_safe_vertical_velocity(self, desired_vz, target_z):
        """Apply a capture-compatible stopping barrier above the sea."""
        requested_clearance = max(
            float(getattr(self, 'terminal_minimum_clearance', 0.0)),
            float(getattr(self, 'terminal_contact_clearance', 0.05)),
        )
        capture_top = float(target_z) - float(
            getattr(self, 'impact_radius', 0.25)
        )
        altitude_ceiling = max(
            capture_top,
            float(getattr(self, 'sea_surface_z', 0.0)) - requested_clearance,
        )
        safety_margin = float(getattr(self, 'terminal_safety_margin', 0.0))
        altitude_ceiling = max(capture_top, altitude_ceiling - safety_margin)
        current_z = float(self.sim_z)
        control_step = max(
            float(
                getattr(
                    self,
                    'control_dt',
                    getattr(self, 'dt', 0.05),
                )
            ),
            1e-3,
        )
        if current_z >= altitude_ceiling:
            return min(
                float(desired_vz),
                (altitude_ceiling - current_z) / control_step,
            )

        response_time = float(
            getattr(self, 'terminal_safety_response_time', 0.0)
        )
        current_descent_speed = max(float(getattr(self, 'sim_vz', 0.0)), 0.0)
        available_distance = max(
            altitude_ceiling
            - current_z
            - current_descent_speed * response_time,
            0.0,
        )
        acceleration = max(
            float(
                getattr(
                    self,
                    'terminal_max_vertical_acceleration',
                    getattr(self, 'max_vertical_acceleration', 3.0),
                )
            ),
            0.1,
        )
        safe_descent_speed = (
            -acceleration * response_time
            + math.sqrt(
                (acceleration * response_time) ** 2
                + 2.0 * acceleration * available_distance
            )
        )
        return min(float(desired_vz), safe_descent_speed)

    def terminal_trajectory_velocity(self, sample, target_z):
        """Track a planned sample with bounded position feedback."""
        horizontal_gain = float(
            getattr(self, 'terminal_trajectory_position_gain', 0.0)
        )
        vertical_gain = float(
            getattr(self, 'terminal_trajectory_vertical_gain', 0.0)
        )
        desired_vx = (
            sample.velocity[0]
            + horizontal_gain
            * (sample.position[0] - self.sim_x)
        )
        desired_vy = (
            sample.velocity[1]
            + horizontal_gain
            * (sample.position[1] - self.sim_y)
        )
        clamp_command_speed = getattr(self, 'clamp_command_speed', None)
        if clamp_command_speed is not None:
            desired_vx, desired_vy = clamp_command_speed(
                desired_vx,
                desired_vy,
            )
        desired_vz = (
            sample.velocity[2]
            + vertical_gain
            * (sample.position[2] - self.sim_z)
        )
        desired_vz = TrajectoryImpactSim.terminal_safe_vertical_velocity(
            self,
            desired_vz,
            target_z,
        )
        desired_vz = max(
            min(
                desired_vz,
                float(getattr(self, 'max_vertical_speed', math.inf)),
            ),
            -float(getattr(self, 'max_vertical_speed', math.inf)),
        )
        return desired_vx, desired_vy, desired_vz

    def plan_velocity(self, target_x, target_y, target_z):
        """Generate pursuit guidance or one step of a feasible trajectory."""
        guidance_start = time.perf_counter()
        self.planner_attempted_this_cycle = False
        self.planner_succeeded_this_cycle = False
        self.plan_hold_active = False
        self.optimization_attempted_this_cycle = False
        self.optimization_succeeded_this_cycle = False
        dx = target_x - self.sim_x
        dy = target_y - self.sim_y
        horizontal_distance = math.hypot(dx, dy)

        if horizontal_distance > 1e-9:
            horizontal_los_x = dx / horizontal_distance
            horizontal_los_y = dy / horizontal_distance
        else:
            horizontal_los_x = 0.0
            horizontal_los_y = 0.0

        intercept_x, intercept_y, intercept_z, t_go = (
            self.continuous_intercept_solution(
                target_x,
                target_y,
                target_z,
            )
        )

        retained = self.retained_terminal_trajectory_sample()
        retained_plan = None
        retained_sample_time = 0.0
        retained_time_to_go = 0.0
        if retained is not None:
            (
                retained_plan,
                retained_sample_time,
                retained_time_to_go,
            ) = retained
            if not TrajectoryImpactSim.retained_plan_matches_target(
                self,
                retained_plan,
                retained_time_to_go,
                target_x,
                target_y,
                target_z,
            ):
                self.retained_terminal_plan = None
                self.retained_terminal_plan_time_ns = None
                self.retained_terminal_plan_age = 0.0
                retained_plan = None
                self.planner_failure_reason = (
                    PlanningFailureReason.TARGET_SHIFT.value
                )

        replan_period = float(
            getattr(self, 'terminal_replan_period', 0.0)
        )
        replan_due = (
            retained_plan is None
            or self.retained_terminal_plan_age >= replan_period
        )
        plan = None
        diagnostics = None
        attempt_submitted = False
        attempt_completed = False
        if replan_due:
            if getattr(self, 'enable_async_terminal_planner', False):
                (
                    plan,
                    diagnostics,
                    attempt_submitted,
                    attempt_completed,
                ) = self.poll_terminal_trajectory_plan(
                    target_x,
                    target_y,
                    target_z,
                )
            else:
                attempt_submitted = True
                attempt_completed = True
                plan = self.terminal_trajectory_plan(
                    target_x,
                    target_y,
                    target_z,
                )
                planner = getattr(self, 'intercept_trajectory_planner', None)
                diagnostics = getattr(planner, 'last_diagnostics', None)
            if attempt_submitted:
                self.planner_attempted_this_cycle = True
                self.plan_attempt_count = (
                    getattr(self, 'plan_attempt_count', 0) + 1
                )
            if attempt_completed and diagnostics is None:
                self.planner_failure_reason = (
                    PlanningFailureReason.OPTIMIZATION_FAIL.value
                )
            if diagnostics is not None:
                self.planner_compute_time = diagnostics.total_compute_time
                self.minco_generation_compute_time = (
                    diagnostics.generation_compute_time
                )
                self.minco_optimization_compute_time = (
                    diagnostics.optimization_compute_time
                )
                self.horizontal_min_time = diagnostics.horizontal_min_time
                self.vertical_min_time = diagnostics.vertical_min_time
                self.sea_safe_min_time = diagnostics.sea_safe_min_time
                self.planner_search_min_time = diagnostics.search_min_time
                self.planner_search_max_time = diagnostics.search_max_time
                self.planner_failure_reason = diagnostics.failure_reason.value
                if diagnostics.optimization_attempted:
                    self.optimization_attempted_this_cycle = True
                    self.optimization_attempt_count += 1
                if diagnostics.optimization_succeeded:
                    self.optimization_succeeded_this_cycle = True
                    self.optimization_success_count += 1
            if attempt_completed and plan is None:
                self.consecutive_plan_failures = (
                    getattr(self, 'consecutive_plan_failures', 0) + 1
                )
                if diagnostics is not None:
                    reason = diagnostics.failure_reason.value
                    failure_counts = getattr(
                        self,
                        'planner_failure_counts',
                        {},
                    )
                    failure_counts[reason] = failure_counts.get(reason, 0) + 1
                    self.planner_failure_counts = failure_counts
            elif attempt_completed:
                self.planner_succeeded_this_cycle = True
                self.plan_success_count = (
                    getattr(self, 'plan_success_count', 0) + 1
                )
                self.consecutive_plan_failures = 0
        plan_sample_time = self.terminal_control_lookahead
        plan_time_to_go = plan.duration if plan is not None else 0.0
        retained_plan_active = False
        if plan is not None:
            self.minco_execution_cycles = (
                getattr(self, 'minco_execution_cycles', 0) + 1
            )
            self.retain_terminal_trajectory_plan(plan)
        elif retained_plan is not None:
            plan = retained_plan
            plan_sample_time = retained_sample_time
            plan_time_to_go = retained_time_to_go
            retained_plan_active = True
            self.plan_hold_active = True
            self.plan_hold_count = getattr(self, 'plan_hold_count', 0) + 1
        if plan is not None:
            sample = plan.sample(
                min(plan_sample_time, plan.duration)
            )
            desired_vx, desired_vy, desired_vz = (
                TrajectoryImpactSim.terminal_trajectory_velocity(
                    self,
                    sample,
                    target_z,
                )
            )
            if horizontal_distance <= self.terminal_radius:
                self.terminal_descent_committed = True
            self.trajectory_plan_active = True
            first_minco_entry = not getattr(self, 'minco_ever_engaged', False)
            self.minco_ever_engaged = True
            if first_minco_entry:
                TrajectoryImpactSim.update_guidance_phase(
                    self,
                    'MINCO_ENTRY',
                )
            elif horizontal_distance <= self.terminal_radius:
                TrajectoryImpactSim.update_guidance_phase(
                    self,
                    'TERMINAL_MINCO',
                )
            else:
                TrajectoryImpactSim.update_guidance_phase(
                    self,
                    'MINCO_TRACKING',
                )
            self.trajectory_planner_type = plan.planner_type + (
                '_HOLD' if retained_plan_active else ''
            )
            self.planned_minco_target_curve_weight = (
                plan.target_curve_weight
            )
            self.planned_minco_constraint_penalty = (
                getattr(plan, 'constraint_penalty', 0.0)
            )
            self.planned_minco_optimization_iterations = (
                getattr(plan, 'optimization_iterations', 0)
            )
            self.planned_minco_piece_durations = getattr(
                plan,
                'piece_durations',
                (plan.duration,),
            )
            self.guidance_altitude_reference = sample.position[2]
            self.guidance_closing_speed = plan.closing_speed
            self.planned_closing_speed = plan.closing_speed
            self.planned_max_horizontal_speed = (
                plan.maximum_horizontal_speed
            )
            self.planned_max_vertical_speed = plan.maximum_vertical_speed
            self.planned_max_horizontal_acceleration = (
                plan.maximum_horizontal_acceleration
            )
            self.planned_max_vertical_acceleration = (
                plan.maximum_vertical_acceleration
            )
            self.intercept_reference_vx = sample.velocity[0]
            self.intercept_reference_vy = sample.velocity[1]
            self.intercept_reference_vz = sample.velocity[2]
            self.intercept_reference_ax = sample.acceleration[0]
            self.intercept_reference_ay = sample.acceleration[1]
            self.intercept_reference_az = sample.acceleration[2]
            self.guidance_compute_time = time.perf_counter() - guidance_start
            return (
                desired_vx,
                desired_vy,
                desired_vz,
                plan.target_position[0],
                plan.target_position[1],
                plan.target_position[2],
                plan_time_to_go,
                True,
            )

        # Before first entry, pursuit only drives the vehicle into the 3-D
        # reachable set.  Once MINCO has been entered it is latched: plan loss
        # never falls back to a high-speed terminal dive.
        self.reset_trajectory_plan_diagnostics()
        if not getattr(self, 'minco_ever_engaged', False):
            TrajectoryImpactSim.update_guidance_phase(
                self,
                'FAR_GUIDANCE',
            )
            self.trajectory_planner_type = 'FAR_GUIDANCE'
            closing_speed = self.pursuit_closing_speed(
                horizontal_distance,
                abs(target_z - self.sim_z),
                self.follow_max_closing_speed,
            )
            desired_vx = self.target_vx + closing_speed * horizontal_los_x
            desired_vy = self.target_vy + closing_speed * horizontal_los_y
            staging_z = self.pursuit_altitude_reference(
                horizontal_distance,
                target_z,
            )
            measured_closing_speed = max(
                self.measured_horizontal_closing_speed(
                    horizontal_los_x,
                    horizontal_los_y,
                ),
                0.0,
            )
            desired_vz = self.pursuit_vertical_velocity(
                horizontal_distance,
                target_z,
                self.target_vz,
                staging_z,
                measured_closing_speed,
            )
        else:
            TrajectoryImpactSim.update_guidance_phase(self, 'SAFE_WAIT')
            self.trajectory_planner_type = 'MINCO_SAFE_WAIT'
            closing_speed = 0.0
            desired_vx = self.target_vx
            desired_vy = self.target_vy
            staging_z = min(self.sim_z, self.flight_altitude)
            desired_vz = max(
                min(
                    self.altitude_velocity_gain * (staging_z - self.sim_z),
                    self.max_vertical_speed,
                ),
                -self.max_vertical_speed,
            )
        desired_vx, desired_vy = self.clamp_command_speed(
            desired_vx,
            desired_vy,
        )
        self.guidance_altitude_reference = staging_z
        self.guidance_closing_speed = closing_speed
        self.guidance_compute_time = time.perf_counter() - guidance_start

        return (
            desired_vx,
            desired_vy,
            desired_vz,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
            getattr(self, 'minco_ever_engaged', False),
        )

    def update_guidance_phase(self, phase):
        """Record and announce only actual guidance-state transitions."""
        previous = getattr(self, 'guidance_phase', '')
        self.guidance_phase = str(phase)
        get_logger = getattr(self, 'get_logger', None)
        if previous != self.guidance_phase and get_logger is not None:
            get_logger().info(
                f'GUIDANCE PHASE: {previous or "UNSET"} -> '
                f'{self.guidance_phase}'
            )

    def acceleration_limited_velocity(
        self,
        desired_vx,
        desired_vy,
        acceleration_limit=None,
    ):
        if acceleration_limit is None:
            acceleration_limit = self.max_acceleration
        acceleration_limit = self.limit_horizontal_acceleration(
            acceleration_limit
        )

        if self.enable_gazebo_control:
            desired_vx, desired_vy = self.speed_governed_velocity(
                desired_vx,
                desired_vy,
            )

            if self.command_vx is None or self.command_vy is None:
                base_vx, base_vy = self.clamp_command_speed(
                    self.sim_vx,
                    self.sim_vy,
                )
            else:
                base_vx = self.command_vx
                base_vy = self.command_vy
        else:
            base_vx = self.sim_vx
            base_vy = self.sim_vy

        delta_vx = desired_vx - base_vx
        delta_vy = desired_vy - base_vy
        delta_speed = math.hypot(delta_vx, delta_vy)
        control_step = (
            getattr(self, 'control_dt', self.dt)
            if self.enable_gazebo_control
            else self.dt
        )
        max_delta_speed = acceleration_limit * control_step

        if delta_speed > max_delta_speed:
            scale = max_delta_speed / delta_speed
            delta_vx *= scale
            delta_vy *= scale

        command_vx = base_vx + delta_vx
        command_vy = base_vy + delta_vy

        if self.enable_gazebo_control:
            command_vx, command_vy = self.clamp_command_speed(
                command_vx,
                command_vy,
            )
        else:
            command_vx, command_vy = self.clamp_speed(
                command_vx,
                command_vy,
            )

        self.command_vx = command_vx
        self.command_vy = command_vy
        self.command_ax = (command_vx - base_vx) / control_step
        self.command_ay = (command_vy - base_vy) / control_step

        return command_vx, command_vy

    def command_acceleration_limits(self, terminal_mode):
        """Return the XY and Z acceleration limits for a guidance phase."""
        if terminal_mode:
            return (
                self.limit_horizontal_acceleration(
                    self.terminal_max_acceleration
                ),
                self.terminal_max_vertical_acceleration,
            )
        return (
            self.limit_horizontal_acceleration(self.max_acceleration),
            self.max_vertical_acceleration,
        )

    def acceleration_limited_vertical_velocity(
        self,
        desired_vz,
        acceleration_limit=None,
    ):
        if acceleration_limit is None:
            acceleration_limit = self.max_vertical_acceleration
        acceleration_limit = min(
            max(float(acceleration_limit), 0.1),
            self.max_vertical_acceleration,
        )
        desired_vz = max(
            min(desired_vz, self.max_vertical_speed),
            -self.max_vertical_speed,
        )

        if self.enable_gazebo_control and self.command_vz is not None:
            base_vz = self.command_vz
        else:
            base_vz = self.sim_vz

        control_step = (
            getattr(self, 'control_dt', self.dt)
            if self.enable_gazebo_control
            else self.dt
        )
        max_delta = acceleration_limit * control_step
        delta_vz = max(
            min(desired_vz - base_vz, max_delta),
            -max_delta,
        )
        command_vz = max(
            min(base_vz + delta_vz, self.max_vertical_speed),
            -self.max_vertical_speed,
        )

        self.command_vz = command_vz
        self.command_az = (command_vz - base_vz) / control_step

        return command_vz

    def apply_final_sea_safety(self, command_vz):
        """Apply the sea barrier after trajectory and acceleration limits."""
        started = time.perf_counter()
        control_step = (
            self.control_dt if self.enable_gazebo_control else self.dt
        )
        result = apply_sea_safety_guard(
            current_z=self.sim_z,
            current_vz=self.sim_vz,
            proposed_vz=command_vz,
            sea_surface_z=self.sea_surface_z,
            reserve_clearance=(
                self.terminal_contact_clearance
                + self.terminal_safety_margin
            ),
            response_delay=self.terminal_safety_response_time,
            effective_braking_acceleration=(
                self.terminal_effective_vertical_braking_acceleration
            ),
            control_dt=control_step,
            warning_margin=self.terminal_safety_warning_margin,
            maximum_vertical_speed=self.max_vertical_speed,
        )
        prior_command_vz = command_vz - self.command_az * control_step
        self.command_vz = result.command_vz
        self.command_az = (
            result.command_vz - prior_command_vz
        ) / control_step
        self.sea_safety_state = result.state.value
        self.sea_safety_margin = result.response_margin
        self.minimum_sea_safety_margin = min(
            getattr(self, 'minimum_sea_safety_margin', math.inf),
            result.response_margin,
        )
        if (
            result.state == SeaSafetyState.WARNING
            and getattr(self, 'first_safety_warning_time', None) is None
        ):
            self.first_safety_warning_time = self.sim_time
            self.get_logger().warn(
                f'SEA SAFETY WARNING | margin={result.response_margin:.3f} m'
            )
        if (
            result.state in (
                SeaSafetyState.BRAKE,
                SeaSafetyState.UNRECOVERABLE,
            )
            and getattr(self, 'first_safety_brake_time', None) is None
        ):
            self.first_safety_brake_time = self.sim_time
            self.get_logger().warn(
                f'SEA SAFETY BRAKE | margin={result.response_margin:.3f} m'
            )
        if (
            result.state == SeaSafetyState.UNRECOVERABLE
            and not self.first_unrecoverable_safety_detail
        ):
            self.first_unrecoverable_safety_detail = (
                f'z={self.sim_z:.3f}, vz={self.sim_vz:.3f}, '
                f'clearance={result.clearance:.3f}, '
                f'immediate_margin={result.immediate_margin:.3f}, '
                f'response_margin={result.response_margin:.3f}'
            )
            self.get_logger().error(
                'SEA SAFETY UNRECOVERABLE | '
                + self.first_unrecoverable_safety_detail
            )
        self.safety_compute_time = time.perf_counter() - started
        return result.command_vz

    def impact_fraction(self, relative_start, relative_end):
        deltas = [
            end - start
            for start, end in zip(relative_start, relative_end)
        ]

        c = (
            sum(value * value for value in relative_start)
            - self.impact_radius * self.impact_radius
        )

        if c <= 0.0:
            return 0.0

        a = sum(delta * delta for delta in deltas)

        if a < 1e-12:
            return None

        b = 2.0 * sum(
            start * delta
            for start, delta in zip(relative_start, deltas)
        )
        discriminant = b * b - 4.0 * a * c

        if discriminant < 0.0:
            return None

        sqrt_discriminant = math.sqrt(discriminant)
        roots = [
            (-b - sqrt_discriminant) / (2.0 * a),
            (-b + sqrt_discriminant) / (2.0 * a),
        ]
        valid_roots = [root for root in roots if 0.0 <= root <= 1.0]

        return min(valid_roots) if valid_roots else None

    def sea_contact_fraction(self, start_z, end_z):
        """Return the interval fraction where NED z reaches sea level."""
        if not self.enable_sea_contact_failure:
            return None

        if start_z >= self.sea_surface_z:
            return 0.0

        delta_z = end_z - start_z
        if delta_z <= 0.0:
            return None

        fraction = (self.sea_surface_z - start_z) / delta_z
        if 0.0 <= fraction <= 1.0:
            return fraction
        return None

    def terminal_event(
        self,
        relative_start,
        relative_end,
        start_z,
        end_z,
    ):
        """Select the first capture or sea-contact event in an interval."""
        hit_fraction = TrajectoryImpactSim.impact_fraction(
            self,
            relative_start,
            relative_end,
        )
        sea_fraction = TrajectoryImpactSim.sea_contact_fraction(
            self,
            start_z,
            end_z,
        )

        if (
            hit_fraction is not None
            and (
                sea_fraction is None
                or hit_fraction <= sea_fraction
            )
        ):
            return 'CAPTURE_RADIUS_REACHED', hit_fraction
        if sea_fraction is not None:
            return 'SEA_CONTACT', sea_fraction
        return None, None

    def open_csv_log(self):
        log_directory = Path(
            self.get_parameter(
                'log_directory'
            ).get_parameter_value().string_value
        ).expanduser()

        if not log_directory.is_absolute():
            log_directory = Path.cwd() / log_directory

        log_directory.mkdir(parents=True, exist_ok=True)
        filename = (
            'trajectory_impact_sim_'
            + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            + '.csv'
        )
        self.csv_path = log_directory / filename
        self.csv_file = self.csv_path.open(
            mode='x',
            newline='',
            encoding='utf-8',
            buffering=1
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.CSV_FIELDS)

    def begin_csv_logging(self):
        if self.csv_file is not None:
            return

        if self.log_start_time_ns is None:
            self.log_start_time_ns = (
                self.get_clock().now().nanoseconds
            )
        self.prediction_error_tracker.reset()

        try:
            self.open_csv_log()
        except (OSError, ValueError) as error:
            self.csv_file = None
            self.csv_writer = None
            self.csv_path = None
            self.get_logger().error(
                f'Failed to open CSV log: {error}'
            )
            return

        self.get_logger().info(
            f'CSV logging started at X command: {self.csv_path}'
        )

    @staticmethod
    def _timing_summary(values):
        finite = sorted(
            float(value) for value in values if math.isfinite(value)
        )
        if not finite:
            return {'count': 0, 'mean': None, 'p95': None, 'max': None}
        rank = min(
            max(math.ceil(0.95 * len(finite)) - 1, 0),
            len(finite) - 1,
        )
        return {
            'count': len(finite),
            'mean': sum(finite) / len(finite),
            'p95': finite[rank],
            'max': finite[-1],
        }

    def write_run_artifacts(self, csv_path):
        """Write one compact summary and exact runtime parameter snapshot."""
        if csv_path is None:
            return
        csv_path = Path(csv_path)
        summary_path = csv_path.with_name(csv_path.stem + '_summary.json')
        config_path = csv_path.with_name(csv_path.stem + '_config.yaml')
        attempts = self.plan_attempt_count
        cycles = self.intercept_control_cycles
        prediction_rmse = {}
        try:
            with csv_path.open(newline='', encoding='utf-8') as stream:
                logged_rows = list(csv.DictReader(stream))
        except OSError:
            logged_rows = []
        if self.failure_reason == 'SEA_CONTACT' and logged_rows:
            tail_path = csv_path.with_name(csv_path.stem + '_sea_tail.csv')
            timestamps = []
            for row in logged_rows:
                try:
                    timestamps.append(float(row.get('time', '')))
                except (TypeError, ValueError):
                    timestamps.append(math.nan)
            finite_timestamps = [
                value for value in timestamps if math.isfinite(value)
            ]
            if finite_timestamps:
                cutoff = max(finite_timestamps) - 2.0
                tail_rows = [
                    row for row, timestamp in zip(logged_rows, timestamps)
                    if math.isfinite(timestamp) and timestamp >= cutoff
                ]
                tail_fields = [
                    field for field in (
                        'time',
                        'uav_z',
                        'uav_vz',
                        'target_z',
                        'horizontal_distance',
                        'vertical_error',
                        't_horizontal_min',
                        't_vertical_min',
                        't_go',
                        'sea_safety_margin',
                        'trajectory_planner',
                        'retained_plan_age',
                        'command_vz',
                        'command_az',
                        'sea_safety_state',
                        'sim_dt',
                        'wall_dt',
                        'callback_compute_time',
                    )
                    if field in (logged_rows[0] if logged_rows else {})
                ]
                try:
                    with tail_path.open(
                        'w', newline='', encoding='utf-8'
                    ) as stream:
                        writer = csv.DictWriter(stream, fieldnames=tail_fields)
                        writer.writeheader()
                        writer.writerows(
                            {
                                field: row.get(field, '')
                                for field in tail_fields
                            }
                            for row in tail_rows
                        )
                except OSError as error:
                    self.get_logger().error(
                        f'Failed to write sea-contact tail: {error}'
                    )
        for horizon in ('0p5', '1p0', '2p0'):
            field = f'guidance_prediction_{horizon}_error'
            errors = []
            for row in logged_rows:
                try:
                    value = float(row.get(field, ''))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    errors.append(value)
            prediction_rmse[field] = (
                math.sqrt(sum(value * value for value in errors) / len(errors))
                if errors else None
            )
        try:
            git_commit = subprocess.run(
                ['git', 'rev-parse', 'HEAD'],
                cwd=Path.cwd(),
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            git_dirty = bool(subprocess.run(
                ['git', 'status', '--porcelain'],
                cwd=Path.cwd(),
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip())
        except (OSError, subprocess.CalledProcessError):
            git_commit = None
            git_dirty = None
        summary = {
            'csv_file': csv_path.name,
            'git_commit': git_commit,
            'git_dirty': git_dirty,
            'outcome': self.outcome or 'UNFINISHED',
            'failure_reason': self.failure_reason,
            'failure_detail': self.failure_detail,
            'elapsed_time': self.sim_time,
            'capture_time': self.sim_time if self.hit else None,
            'relative_speed_at_capture': (
                self.final_relative_speed if self.hit else None
            ),
            'vertical_relative_speed_at_capture': (
                self.final_vertical_relative_speed if self.hit else None
            ),
            'minimum_distance': (
                self.minimum_distance
                if math.isfinite(self.minimum_distance) else None
            ),
            'closest_horizontal_distance': (
                self.closest_horizontal_distance
                if math.isfinite(self.closest_horizontal_distance) else None
            ),
            'closest_vertical_error': (
                self.closest_vertical_error
                if math.isfinite(self.closest_vertical_error) else None
            ),
            'planner': {
                'attempts': attempts,
                'successes': self.plan_success_count,
                'success_rate': (
                    self.plan_success_count / attempts if attempts else None
                ),
                'hold_cycles': self.plan_hold_count,
                'attempt_rate': attempts / cycles if cycles else None,
                'execution_rate': (
                    self.minco_execution_cycles / cycles if cycles else None
                ),
                'hold_rate': (
                    self.plan_hold_count / cycles if cycles else None
                ),
                'optimization_attempts': self.optimization_attempt_count,
                'optimization_successes': self.optimization_success_count,
                'optimization_success_rate': (
                    self.optimization_success_count
                    / self.optimization_attempt_count
                    if self.optimization_attempt_count else None
                ),
                'failure_counts': self.planner_failure_counts,
                'minco_ever_engaged': self.minco_ever_engaged,
            },
            'sea_safety': {
                'final_state': self.sea_safety_state,
                'final_response_margin': (
                    self.sea_safety_margin
                    if math.isfinite(self.sea_safety_margin) else None
                ),
                'minimum_response_margin': (
                    self.minimum_sea_safety_margin
                    if math.isfinite(self.minimum_sea_safety_margin)
                    else None
                ),
                'first_warning_time': self.first_safety_warning_time,
                'first_brake_time': self.first_safety_brake_time,
                'first_unrecoverable_detail': (
                    self.first_unrecoverable_safety_detail
                ),
            },
            'timing_seconds': {
                name: self._timing_summary(values)
                for name, values in self.timing_samples.items()
            },
            'timing_by_phase_seconds': {
                phase: {
                    name: self._timing_summary(values)
                    for name, values in phase_samples.items()
                }
                for phase, phase_samples in (
                    self.timing_samples_by_phase.items()
                )
            },
            'prediction_rmse': prediction_rmse,
        }
        parameters = {}
        # ``Node.list_parameters`` is not part of the ROS 2 Humble Python API.
        # ``get_parameters_by_prefix('')`` returns every declared parameter
        # and keeps run-artifact generation portable across Humble builds.
        for name, parameter in self.get_parameters_by_prefix('').items():
            value = parameter.value
            if isinstance(value, tuple):
                value = list(value)
            parameters[name] = value
        config_snapshot = {
            'trajectory_impact_sim': {'ros__parameters': parameters},
        }
        baseline_path = (
            Path.cwd()
            / 'src/uav_usv_bringup/config/baseline.yaml'
        )
        if baseline_path.is_file():
            config_snapshot['baseline_yaml_source'] = (
                baseline_path.read_text(encoding='utf-8')
            )
        try:
            summary_path.write_text(
                json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
                encoding='utf-8',
            )
            # JSON is valid YAML 1.2 and avoids adding a runtime dependency.
            config_path.write_text(
                json.dumps(config_snapshot, ensure_ascii=False, indent=2)
                + '\n',
                encoding='utf-8',
            )
        except (OSError, ValueError) as error:
            self.get_logger().error(f'Failed to write run artifacts: {error}')

    def close_csv_log(self):
        if self.csv_file is None:
            return

        csv_path = self.csv_path

        try:
            self.csv_file.flush()
            self.csv_file.close()
        except (OSError, ValueError) as error:
            message = f'Failed to close CSV log: {error}'
            if rclpy.ok():
                self.get_logger().error(message)
            else:
                print(message, flush=True)
        finally:
            self.csv_file = None
            self.csv_writer = None

        self.write_run_artifacts(csv_path)

        message = f'CSV log saved: {csv_path}'
        if rclpy.ok():
            self.get_logger().info(message)
        else:
            print(message, flush=True)

    def record_row(
        self,
        target_x,
        target_y,
        target_z,
        intercept_x,
        intercept_y,
        intercept_z,
        t_go,
        phase=None,
    ):
        if self.csv_writer is None:
            return
        logging_start = time.perf_counter()

        if self.log_start_time_ns is None:
            elapsed_time = self.sim_time
        else:
            elapsed_time = max(
                (
                    self.get_clock().now().nanoseconds
                    - self.log_start_time_ns
                ) * 1e-9,
                0.0,
            )

        if phase is None:
            phase = (
                'intercept'
                if self.enable_gazebo_control
                else 'virtual_intercept'
            )

        horizontal_distance = math.hypot(
            target_x - self.sim_x,
            target_y - self.sim_y,
        )
        vertical_error = target_z - self.sim_z
        distance = math.sqrt(
            horizontal_distance * horizontal_distance
            + vertical_error * vertical_error
        )
        if self.started:
            self.update_closest_approach((
                target_x - self.sim_x,
                target_y - self.sim_y,
                vertical_error,
            ))
        relative_velocity = (
            self.target_vx - self.sim_vx,
            self.target_vy - self.sim_vy,
            self.target_vz - self.sim_vz,
        )
        relative_speed = math.sqrt(
            sum(value * value for value in relative_velocity)
        )
        if distance > 1e-9:
            closing_speed = -(
                (target_x - self.sim_x) * relative_velocity[0]
                + (target_y - self.sim_y) * relative_velocity[1]
                + vertical_error * relative_velocity[2]
            ) / distance
        else:
            closing_speed = 0.0
        minimum_distance = (
            self.minimum_distance
            if math.isfinite(self.minimum_distance)
            else distance
        )
        prediction_model = (
            'BCTRA'
            if (
                self.enable_maneuver_prediction
                and self.target_predictor.maneuver_model_active
            )
            else 'CV'
        )
        logged_command_vx = (
            self.command_vx
            if self.command_vx is not None
            else self.sim_vx
        )
        logged_command_vy = (
            self.command_vy
            if self.command_vy is not None
            else self.sim_vy
        )
        logged_command_vz = (
            self.command_vz
            if self.command_vz is not None
            else self.sim_vz
        )
        now_ns = self.get_clock().now().nanoseconds
        camera_age = (
            math.inf
            if self.camera_measurement_time_ns is None
            else max(
                (now_ns - self.camera_measurement_time_ns) * 1e-9,
                0.0,
            )
        )
        camera_valid = (
            self.camera_measurement_valid
            and camera_age <= self.state_timeout
        )
        kf_state_age = (
            math.inf
            if self.kf_state_time_ns is None
            else max((now_ns - self.kf_state_time_ns) * 1e-9, 0.0)
        )
        kf_valid = (
            self.kf_state_valid
            and kf_state_age <= self.state_timeout
        )
        truth_position = (target_x, target_y, target_z)
        camera_error = (
            math.sqrt(sum(
                (estimate - truth) ** 2
                for estimate, truth in zip(
                    self.camera_measurement,
                    truth_position,
                )
            ))
            if camera_valid
            else math.nan
        )
        kf_error = (
            math.sqrt(sum(
                (estimate - truth) ** 2
                for estimate, truth in zip(
                    self.kf_position,
                    truth_position,
                )
            ))
            if kf_valid
            else math.nan
        )

        prediction_start = time.perf_counter()
        prediction_evaluations = {
            (model, horizon): self.prediction_error_tracker.evaluate(
                model,
                horizon,
                elapsed_time,
                truth_position,
            )
            for model in PREDICTION_MODELS
            for horizon in PREDICTION_HORIZONS
        }
        guidance_predictions = {
            horizon: self.target_planning_state(
                target_x,
                target_y,
                target_z,
                horizon,
            )[0]
            for horizon in PREDICTION_HORIZONS
        }
        self.prediction_error_tracker.add(
            'guidance',
            elapsed_time,
            guidance_predictions,
        )
        if kf_valid:
            self.prediction_error_tracker.add(
                'kf',
                elapsed_time,
                {
                    horizon: tuple(
                        position + velocity * horizon
                        for position, velocity in zip(
                            self.kf_position,
                            self.kf_velocity,
                        )
                    )
                    for horizon in PREDICTION_HORIZONS
                },
            )
        self.prediction_compute_time = time.perf_counter() - prediction_start
        prediction_values = []
        for model in PREDICTION_MODELS:
            for horizon in PREDICTION_HORIZONS:
                evaluation = prediction_evaluations[(model, horizon)]
                if evaluation is None:
                    prediction_values.extend(['', '', '', '', ''])
                else:
                    prediction_values.extend([
                        f'{evaluation.position[0]:.6f}',
                        f'{evaluation.position[1]:.6f}',
                        f'{evaluation.position[2]:.6f}',
                        f'{evaluation.error:.6f}',
                        f'{evaluation.age:.6f}',
                    ])

        def valid_position_values(valid, position):
            return [
                f'{value:.6f}' if valid else ''
                for value in position
            ]

        row = [
            f'{elapsed_time:.6f}',
            f'{self.sim_x:.6f}',
            f'{self.sim_y:.6f}',
            f'{self.sim_z:.6f}',
            f'{self.sim_vx:.6f}',
            f'{self.sim_vy:.6f}',
            f'{self.sim_vz:.6f}',
            f'{target_x:.6f}',
            f'{target_y:.6f}',
            f'{target_z:.6f}',
            f'{self.target_vx:.6f}',
            f'{self.target_vy:.6f}',
            f'{self.target_vz:.6f}',
            '1' if camera_valid else '0',
            *valid_position_values(
                camera_valid,
                self.camera_measurement,
            ),
            f'{camera_error:.6f}' if camera_valid else '',
            f'{self.camera_confidence:.6f}' if camera_valid else '',
            '1' if kf_valid else '0',
            *valid_position_values(kf_valid, self.kf_position),
            *valid_position_values(kf_valid, self.kf_velocity),
            f'{kf_error:.6f}' if kf_valid else '',
            f'{kf_state_age:.6f}' if kf_valid else '',
            *prediction_values,
            prediction_model,
            f'{self.target_predictor.turn_rate:.6f}',
            f'{self.target_predictor.turn_acceleration:.6f}',
            f'{self.target_predictor.speed_acceleration:.6f}',
            f'{self.control_dt:.6f}',
            f'{self.sim_dt:.6f}',
            f'{self.wall_dt:.6f}',
            f'{self.callback_compute_time:.6f}',
            f'{self.prediction_compute_time:.6f}',
            f'{self.planner_compute_time:.6f}',
            f'{self.minco_generation_compute_time:.6f}',
            f'{self.minco_optimization_compute_time:.6f}',
            f'{self.guidance_compute_time:.6f}',
            f'{self.safety_compute_time:.6f}',
            f'{self.logging_compute_time:.6f}',
            f'{self.retained_terminal_plan_age:.6f}',
            f'{distance:.6f}',
            f'{horizontal_distance:.6f}',
            f'{vertical_error:.6f}',
            f'{intercept_x:.6f}',
            f'{intercept_y:.6f}',
            f'{intercept_z:.6f}',
            f'{self.intercept_reference_vx:.6f}',
            f'{self.intercept_reference_vy:.6f}',
            f'{self.intercept_reference_vz:.6f}',
            f'{self.intercept_reference_ax:.6f}',
            f'{self.intercept_reference_ay:.6f}',
            f'{self.intercept_reference_az:.6f}',
            f'{t_go:.6f}',
            f'{self.guidance_altitude_reference:.6f}',
            f'{self.guidance_closing_speed:.6f}',
            f'{self.horizontal_min_time:.6f}',
            f'{self.vertical_min_time:.6f}',
            f'{self.sea_safe_min_time:.6f}',
            f'{self.planner_search_min_time:.6f}',
            f'{self.planner_search_max_time:.6f}',
            self.planner_failure_reason,
            self.guidance_phase,
            self.sea_safety_state,
            f'{self.sea_safety_margin:.6f}',
            '1' if self.trajectory_plan_active else '0',
            '1' if self.planner_attempted_this_cycle else '0',
            '1' if self.planner_succeeded_this_cycle else '0',
            '1' if self.optimization_attempted_this_cycle else '0',
            '1' if self.optimization_succeeded_this_cycle else '0',
            '1' if self.plan_hold_active else '0',
            self.trajectory_planner_type,
            f'{self.planned_minco_target_curve_weight:.6f}',
            f'{self.planned_minco_constraint_penalty:.9f}',
            str(self.planned_minco_optimization_iterations),
            ';'.join(
                f'{duration:.6f}'
                for duration in self.planned_minco_piece_durations
            ),
            f'{self.planned_closing_speed:.6f}',
            f'{self.planned_max_horizontal_speed:.6f}',
            f'{self.planned_max_vertical_speed:.6f}',
            f'{self.planned_max_horizontal_acceleration:.6f}',
            f'{self.planned_max_vertical_acceleration:.6f}',
            f'{self.current_uav_yaw:.6f}',
            f'{self.observation_yaw_command:.6f}',
            f'{logged_command_vx:.6f}',
            f'{logged_command_vy:.6f}',
            f'{logged_command_vz:.6f}',
            f'{self.command_ax:.6f}',
            f'{self.command_ay:.6f}',
            f'{self.command_az:.6f}',
            f'{self.measured_ax:.6f}',
            f'{self.measured_ay:.6f}',
            f'{self.measured_az:.6f}',
            '1' if self.terminal_mode_active else '0',
            phase,
            f'{self.sim_time:.6f}' if self.started else '',
            f'{minimum_distance:.6f}',
            f'{relative_speed:.6f}',
            f'{closing_speed:.6f}',
            '1' if self.constraint_violation_cycles > 0 else '0',
            self.outcome,
            self.failure_reason,
            self.failure_detail,
        ]
        if len(row) != len(self.CSV_FIELDS):
            raise RuntimeError(
                f'CSV schema has {len(self.CSV_FIELDS)} fields but row has '
                f'{len(row)} values'
            )
        self.csv_writer.writerow(row)
        self.logging_compute_time = time.perf_counter() - logging_start

    def publish_simulation_state(
        self,
        intercept_x,
        intercept_y,
        intercept_z,
    ):
        position = Point()
        position.x = self.sim_x
        position.y = self.sim_y
        position.z = self.sim_z
        self.sim_position_pub.publish(position)

        velocity = Vector3()
        velocity.x = self.sim_vx
        velocity.y = self.sim_vy
        velocity.z = self.sim_vz
        self.sim_velocity_pub.publish(velocity)

        intercept = Point()
        intercept.x = intercept_x
        intercept.y = intercept_y
        intercept.z = intercept_z
        self.intercept_point_pub.publish(intercept)

    def start_simulation(self):
        self.sim_x = self.initial_uav_x
        self.sim_y = self.initial_uav_y
        self.sim_z = self.initial_uav_z
        self.sim_vx = self.initial_uav_vx
        self.sim_vy = self.initial_uav_vy
        self.sim_vz = self.initial_uav_vz
        self.sim_time = 0.0
        self.log_start_time_ns = self.get_clock().now().nanoseconds
        self.begin_csv_logging()

        self.reset_evaluation()
        self.started = True
        self.intercept_reference_filter.reset()
        target_x, target_y, target_z = self.estimated_target_position()
        intercept_x, intercept_y, intercept_z, t_go = (
            self.continuous_intercept_solution(
                target_x,
                target_y,
                target_z,
            )
        )
        self.record_row(
            target_x,
            target_y,
            target_z,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
        )
        self.publish_simulation_state(
            intercept_x,
            intercept_y,
            intercept_z,
        )

        hit_message = Bool()
        hit_message.data = False
        self.hit_pub.publish(hit_message)

        self.get_logger().info(
            'Starting VIRTUAL TRAJECTORY INTERCEPTION'
        )
        if self.csv_path is not None:
            self.get_logger().info(
                f'CSV logging to: {self.csv_path}'
            )

    def start_gazebo_interception(self):
        self.sim_x = self.initial_uav_x
        self.sim_y = self.initial_uav_y
        self.sim_z = self.initial_uav_z
        self.sim_vx = self.initial_uav_vx
        self.sim_vy = self.initial_uav_vy
        self.sim_vz = self.initial_uav_vz
        self.sim_time = 0.0
        self.begin_csv_logging()

        self.reset_evaluation()
        self.started = True
        self.intercept_reference_filter.reset()
        target_x, target_y, target_z = self.estimated_target_position()
        intercept_x, intercept_y, intercept_z, t_go = (
            self.continuous_intercept_solution(
                target_x,
                target_y,
                target_z,
            )
        )

        self.previous_relative_x = target_x - self.sim_x
        self.previous_relative_y = target_y - self.sim_y
        self.previous_relative_z = target_z - self.sim_z
        self.previous_uav_x = self.sim_x
        self.previous_uav_y = self.sim_y
        self.previous_uav_z = self.sim_z
        self.previous_target_x = target_x
        self.previous_target_y = target_y
        self.previous_target_z = target_z

        self.record_row(
            target_x,
            target_y,
            target_z,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
        )
        self.publish_simulation_state(
            intercept_x,
            intercept_y,
            intercept_z,
        )

        hit_message = Bool()
        hit_message.data = False
        self.hit_pub.publish(hit_message)

        self.get_logger().info(
            'Y accepted. Starting GAZEBO TRAJECTORY INTERCEPTION'
        )
        if self.csv_path is not None:
            self.get_logger().info(
                f'CSV logging to: {self.csv_path}'
            )

    def update_control_timing(self, timestamp_ns=None):
        """Measure the real callback interval used by command limiters."""
        if timestamp_ns is None:
            timestamp_ns = self.get_clock().now().nanoseconds
        timestamp_ns = int(timestamp_ns)
        if self.last_control_callback_time_ns is None:
            self.control_dt = self.dt
        else:
            measured_dt = (
                timestamp_ns - self.last_control_callback_time_ns
            ) * 1e-9
            if 0.001 <= measured_dt <= 0.25:
                self.control_dt = measured_dt
            else:
                self.control_dt = self.dt
        self.last_control_callback_time_ns = timestamp_ns
        self.sim_dt = self.control_dt
        return self.control_dt

    def gazebo_timer_callback(self):
        self.update_control_timing()
        if self.completed:
            self.publish_offboard_mode()
            if self.hold_x is not None:
                self.publish_gazebo_setpoint(
                    self.hold_x,
                    self.hold_y,
                    self.hold_z,
                )
            return

        state_ready = (
            self.target_position_received
            and self.target_velocity_received
            and self.uav_state_received
        )
        if not state_ready:
            self.publish_flight_ready(False)
            return

        if self.takeoff_x is None:
            self.takeoff_x = self.initial_uav_x
            self.takeoff_y = self.initial_uav_y
            self.takeoff_z = self.initial_uav_z

        if not self.takeoff_requested:
            self.prepare_flight_on_ground()
            if not self.ready_for_takeoff_announced:
                self.ready_for_takeoff_announced = True
                self.get_logger().info(
                    'PREPARING | prestreaming OFFBOARD ground hold while '
                    'disarmed'
                )
            return

        self.publish_offboard_mode()
        self.control_counter += 1

        failure_reason = self.runtime_failure_reason()
        if failure_reason is not None:
            target_x, target_y, target_z = (
                self.estimated_target_position()
            )
            self.finish_interception(
                False,
                failure_reason,
                target_x,
                target_y,
                target_z,
            )
            return

        if not self.takeoff_complete:
            self.sim_x = self.initial_uav_x
            self.sim_y = self.initial_uav_y
            self.sim_z = self.initial_uav_z
            self.sim_vx = self.initial_uav_vx
            self.sim_vy = self.initial_uav_vy
            self.sim_vz = self.initial_uav_vz
            target_x, target_y, target_z = (
                self.estimated_target_position()
            )
            (
                follow_x,
                follow_y,
                follow_vx,
                follow_vy,
            ) = self.calculate_follow_reference(target_x, target_y)
            desired_vx, desired_vy = self.plan_follow_velocity(
                follow_x,
                follow_y,
                follow_vx,
                follow_vy,
            )
            takeoff_clearance = max(
                self.takeoff_z - self.sim_z,
                0.0,
            )
            horizontal_scale = self.takeoff_horizontal_scale(
                takeoff_clearance,
                self.takeoff_horizontal_start_height,
                self.takeoff_horizontal_full_height,
            )
            desired_vx *= horizontal_scale
            desired_vy *= horizontal_scale
            if self.offboard_active and self.vehicle_armed:
                command_vx, command_vy = (
                    self.acceleration_limited_velocity(
                        desired_vx,
                        desired_vy,
                        self.takeoff_max_horizontal_acceleration,
                    )
                )
            else:
                command_vx, command_vy = self.clamp_command_speed(
                    self.sim_vx,
                    self.sim_vy,
                )
                self.command_vx = command_vx
                self.command_vy = command_vy
                self.command_ax = 0.0
                self.command_ay = 0.0
            altitude_error = self.flight_altitude - self.sim_z
            desired_vz = max(
                min(
                    self.altitude_velocity_gain * altitude_error,
                    self.takeoff_max_vertical_speed,
                ),
                -self.takeoff_max_vertical_speed,
            )
            command_vz = self.acceleration_limited_vertical_velocity(
                desired_vz,
                self.takeoff_max_vertical_acceleration,
            )
            self.publish_takeoff_follow_setpoint(
                command_vx,
                command_vy,
                command_vz,
            )

            mode_retry_due = (
                self.control_counter
                >= self.offboard_prestream_cycles
                and (
                    self.control_counter
                    - self.offboard_prestream_cycles
                ) % self.px4_command_retry_cycles == 0
            )
            if mode_retry_due and not self.offboard_active:
                self.get_logger().info('Requesting PX4 OFFBOARD mode')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                )

            arm_retry_due = (
                self.control_counter == 1
                or self.control_counter % self.px4_command_retry_cycles == 0
            )
            if arm_retry_due and not self.vehicle_armed:
                self.get_logger().info('Requesting PX4 arming')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                )
            intercept_x, intercept_y, intercept_z, t_go = (
                self.calculate_intercept_solution(
                    self.sim_x,
                    self.sim_y,
                    self.sim_z,
                    target_x,
                    target_y,
                    target_z,
                )
            )
            self.record_row(
                target_x,
                target_y,
                target_z,
                intercept_x,
                intercept_y,
                intercept_z,
                t_go,
                phase='takeoff',
            )
            self.publish_simulation_state(
                follow_x,
                follow_y,
                self.flight_altitude,
            )

            altitude_ready = (
                abs(self.initial_uav_z - self.flight_altitude)
                <= self.takeoff_tolerance
                and abs(self.initial_uav_vz) <= 0.5
                and self.offboard_active
                and self.vehicle_armed
            )

            if altitude_ready:
                self.takeoff_settle_counter += 1
            else:
                self.takeoff_settle_counter = 0

            if (
                self.takeoff_settle_counter
                >= self.takeoff_settle_cycles
            ):
                self.takeoff_complete = True
                self.command_vx, self.command_vy = (
                    self.clamp_command_speed(
                        self.initial_uav_vx,
                        self.initial_uav_vy,
                    )
                )
                self.command_vz = max(
                    min(
                        self.initial_uav_vz,
                        self.max_vertical_speed,
                    ),
                    -self.max_vertical_speed,
                )
                self.get_logger().info(
                    'Takeoff complete. Entering FOLLOW MODE.'
                )
                self.get_logger().info(
                    'Send Y to start interception.'
                )

            if self.control_counter % 40 == 0:
                takeoff_follow_error = math.hypot(
                    follow_x - self.sim_x,
                    follow_y - self.sim_y,
                )
                self.get_logger().info(
                    f'TAKEOFF | z={self.initial_uav_z:.2f} m | '
                    f'target z={self.flight_altitude:.2f} m | '
                    f'follow error={takeoff_follow_error:.2f} m | '
                    f'XY blend={100.0 * horizontal_scale:.0f}% | '
                    f'XY speed={math.hypot(self.sim_vx, self.sim_vy):.2f} '
                    f'm/s'
                )
            return

        if not self.started:
            self.terminal_mode_active = False
            target_x, target_y, target_z = (
                self.estimated_target_position()
            )
            (
                follow_x,
                follow_y,
                follow_vx,
                follow_vy,
            ) = self.calculate_follow_reference(target_x, target_y)
            self.sim_x = self.initial_uav_x
            self.sim_y = self.initial_uav_y
            self.sim_z = self.initial_uav_z
            self.sim_vx = self.initial_uav_vx
            self.sim_vy = self.initial_uav_vy
            self.sim_vz = self.initial_uav_vz

            desired_vx, desired_vy = self.plan_follow_velocity(
                follow_x,
                follow_y,
                follow_vx,
                follow_vy,
            )
            command_vx, command_vy = (
                self.acceleration_limited_velocity(
                    desired_vx,
                    desired_vy,
                    self.follow_max_acceleration,
                )
            )
            altitude_error = self.flight_altitude - self.sim_z
            desired_vz = max(
                min(
                    self.altitude_velocity_gain * altitude_error,
                    self.max_vertical_speed,
                ),
                -self.max_vertical_speed,
            )
            command_vz = self.acceleration_limited_vertical_velocity(
                desired_vz
            )
            self.publish_gazebo_velocity_setpoint(
                command_vx,
                command_vy,
                command_vz,
            )
            self.monitor_actual_constraints()
            self.publish_simulation_state(
                follow_x,
                follow_y,
                self.flight_altitude,
            )
            intercept_x, intercept_y, intercept_z, t_go = (
                self.calculate_intercept_solution(
                    self.sim_x,
                    self.sim_y,
                    self.sim_z,
                    target_x,
                    target_y,
                    target_z,
                )
            )
            self.record_row(
                target_x,
                target_y,
                target_z,
                intercept_x,
                intercept_y,
                intercept_z,
                t_go,
                phase='follow',
            )

            if self.intercept_requested:
                self.start_gazebo_interception()
                return

            if self.control_counter % 20 == 0:
                follow_error = math.hypot(
                    follow_x - self.sim_x,
                    follow_y - self.sim_y,
                )
                self.get_logger().info(
                    f'FOLLOW MODE | error={follow_error:.2f} m | '
                    f'speed={math.hypot(self.sim_vx, self.sim_vy):.2f} '
                    f'm/s | waiting for Y'
                )
            return

        self.sim_x = self.initial_uav_x
        self.sim_y = self.initial_uav_y
        self.sim_z = self.initial_uav_z
        self.sim_vx = self.initial_uav_vx
        self.sim_vy = self.initial_uav_vy
        self.sim_vz = self.initial_uav_vz
        target_x, target_y, target_z = self.estimated_target_position()
        current_relative = (
            target_x - self.sim_x,
            target_y - self.sim_y,
            target_z - self.sim_z,
        )
        previous_relative = (
            self.previous_relative_x,
            self.previous_relative_y,
            self.previous_relative_z,
        )
        event_reason, event_fraction = self.terminal_event(
            previous_relative,
            current_relative,
            self.previous_uav_z,
            self.sim_z,
        )

        if event_fraction is not None:
            hit_uav_x = (
                self.previous_uav_x
                + event_fraction * (self.sim_x - self.previous_uav_x)
            )
            hit_uav_y = (
                self.previous_uav_y
                + event_fraction * (self.sim_y - self.previous_uav_y)
            )
            hit_uav_z = (
                self.previous_uav_z
                + event_fraction * (self.sim_z - self.previous_uav_z)
            )
            hit_target_x = (
                self.previous_target_x
                + event_fraction
                * (target_x - self.previous_target_x)
            )
            hit_target_y = (
                self.previous_target_y
                + event_fraction
                * (target_y - self.previous_target_y)
            )
            hit_target_z = (
                self.previous_target_z
                + event_fraction
                * (target_z - self.previous_target_z)
            )
            self.sim_x = hit_uav_x
            self.sim_y = hit_uav_y
            self.sim_z = hit_uav_z
            self.sim_time += self.control_dt * event_fraction
            success = event_reason == 'CAPTURE_RADIUS_REACHED'
            if not success:
                self.failure_detail = (
                    f'UAV NED z reached sea surface '
                    f'{self.sea_surface_z:.3f} m before capture'
                )
            self.finish_interception(
                success,
                event_reason,
                hit_target_x,
                hit_target_y,
                hit_target_z,
            )
            self.publish_offboard_mode()
            self.publish_gazebo_setpoint(
                self.hold_x,
                self.hold_y,
                self.hold_z,
            )
            return

        self.update_interval_closest_approach(
            previous_relative,
            current_relative,
        )

        (
            desired_vx,
            desired_vy,
            desired_vz,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
            terminal_mode,
        ) = self.plan_velocity(target_x, target_y, target_z)
        self.terminal_mode_active = terminal_mode
        (
            horizontal_acceleration_limit,
            vertical_acceleration_limit,
        ) = self.command_acceleration_limits(
            terminal_mode
        )
        command_vx, command_vy = self.acceleration_limited_velocity(
            desired_vx,
            desired_vy,
            horizontal_acceleration_limit,
        )
        command_vz = self.acceleration_limited_vertical_velocity(
            desired_vz,
            vertical_acceleration_limit,
        )
        command_vz = self.apply_final_sea_safety(command_vz)

        self.publish_gazebo_velocity_setpoint(
            command_vx,
            command_vy,
            command_vz,
        )
        if self.monitor_actual_constraints():
            self.finish_interception(
                False,
                'CONSTRAINT_VIOLATION',
                target_x,
                target_y,
                target_z,
            )
            return

        self.sim_time += self.control_dt
        self.record_row(
            target_x,
            target_y,
            target_z,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
        )
        self.publish_simulation_state(
            intercept_x,
            intercept_y,
            intercept_z,
        )

        self.previous_relative_x = current_relative[0]
        self.previous_relative_y = current_relative[1]
        self.previous_relative_z = current_relative[2]
        self.previous_uav_x = self.sim_x
        self.previous_uav_y = self.sim_y
        self.previous_uav_z = self.sim_z
        self.previous_target_x = target_x
        self.previous_target_y = target_y
        self.previous_target_z = target_z

        step_index = round(self.sim_time / self.dt)
        if step_index % round(1.0 / self.dt) == 0:
            horizontal_distance = math.hypot(
                current_relative[0],
                current_relative[1],
            )
            distance = math.sqrt(
                horizontal_distance * horizontal_distance
                + current_relative[2] * current_relative[2]
            )
            speed = math.hypot(self.sim_vx, self.sim_vy)
            command_speed = math.hypot(command_vx, command_vy)
            command_acceleration = math.hypot(
                self.command_ax,
                self.command_ay,
            )
            self.get_logger().info(
                f'Time={self.sim_time:.2f} s | '
                f'Distance={distance:.2f} m | '
                f'XY={horizontal_distance:.2f} m | '
                f'Z error={current_relative[2]:.2f} m | '
                f'T_go={t_go:.2f} s | '
                f'UAV speed={speed:.2f} m/s | '
                f'Command speed={command_speed:.2f} m/s | '
                f'Vz={self.sim_vz:.2f}/{command_vz:.2f} m/s | '
                f'Command acceleration={command_acceleration:.2f} m/s^2 | '
                f'Command az={self.command_az:.2f} m/s^2 | '
                f'UAV acceleration={self.measured_acceleration:.2f} m/s^2 | '
                f'Mode={self.guidance_phase}'
            )

        if self.sim_time >= self.max_sim_duration:
            self.finish_interception(
                False,
                'TIMEOUT',
                target_x,
                target_y,
                target_z,
            )

    def relative_motion_metrics(self, target_x, target_y, target_z):
        relative_position = (
            target_x - self.sim_x,
            target_y - self.sim_y,
            target_z - self.sim_z,
        )
        relative_velocity = (
            self.target_vx - self.sim_vx,
            self.target_vy - self.sim_vy,
            self.target_vz - self.sim_vz,
        )
        relative_speed = math.sqrt(
            sum(value * value for value in relative_velocity)
        )
        distance = math.sqrt(
            sum(value * value for value in relative_position)
        )
        horizontal_distance = math.hypot(
            relative_position[0],
            relative_position[1],
        )
        if distance > 1e-9:
            closing_speed = -sum(
                position * velocity
                for position, velocity in zip(
                    relative_position,
                    relative_velocity,
                )
            ) / distance
        else:
            closing_speed = 0.0

        return (
            distance,
            horizontal_distance,
            relative_position[2],
            relative_speed,
            closing_speed,
        )

    def finish_interception(
        self,
        success,
        reason,
        target_x,
        target_y,
        target_z,
    ):
        if self.completed:
            return

        self.completed = True
        self.hit = bool(success)
        if success:
            self.outcome = 'SUCCESS'
        elif reason == 'ABORTED':
            self.outcome = 'ABORTED'
        else:
            self.outcome = 'FAILURE'
        self.failure_reason = '' if success else reason
        if success:
            self.failure_detail = ''
        elif not self.failure_detail:
            if reason == 'TIMEOUT':
                self.failure_detail = (
                    f'elapsed time reached '
                    f'{self.max_sim_duration:.3f} s'
                )
            elif reason == 'ABORTED':
                self.failure_detail = 'node shutdown before completion'
        if success:
            self.hold_x = target_x
            self.hold_y = target_y
            self.hold_z = target_z
        else:
            self.hold_x = self.sim_x
            self.hold_y = self.sim_y
            self.hold_z = self.sim_z

        (
            distance,
            horizontal_distance,
            vertical_error,
            relative_speed,
            closing_speed,
        ) = self.relative_motion_metrics(
            target_x,
            target_y,
            target_z,
        )
        self.final_relative_speed = relative_speed
        self.final_vertical_relative_speed = self.target_vz - self.sim_vz
        self.update_closest_approach((
            target_x - self.sim_x,
            target_y - self.sim_y,
            vertical_error,
        ))
        self.max_observed_horizontal_speed = max(
            self.max_observed_horizontal_speed,
            math.hypot(self.sim_vx, self.sim_vy),
        )
        self.max_observed_vertical_speed = max(
            self.max_observed_vertical_speed,
            abs(self.sim_vz),
        )
        minimum_distance = (
            self.minimum_distance
            if math.isfinite(self.minimum_distance)
            else distance
        )

        self.record_row(
            target_x,
            target_y,
            target_z,
            target_x,
            target_y,
            target_z,
            0.0,
        )
        self.publish_simulation_state(target_x, target_y, target_z)

        hit_message = Bool()
        hit_message.data = bool(success)
        self.hit_pub.publish(hit_message)

        result = InterceptResult()
        result.stamp = self.get_clock().now().to_msg()
        result.success = bool(success)
        result.outcome = self.outcome
        result.reason = 'CAPTURE_RADIUS_REACHED' if success else reason
        result.detail = self.failure_detail
        result.elapsed_time = self.sim_time
        result.capture_radius = self.impact_radius
        result.minimum_distance = minimum_distance
        result.horizontal_distance = self.closest_horizontal_distance
        result.vertical_error = self.closest_vertical_error
        result.relative_speed = relative_speed
        result.closing_speed = closing_speed
        result.max_horizontal_speed = self.max_observed_horizontal_speed
        result.max_vertical_speed = self.max_observed_vertical_speed
        result.max_horizontal_acceleration = (
            self.max_observed_horizontal_acceleration
        )
        result.max_vertical_acceleration = (
            self.max_observed_vertical_acceleration
        )
        self.result_pub.publish(result)

        self.get_logger().info(
            '========================================'
        )
        if success:
            log_method = self.get_logger().info
        elif reason == 'ABORTED':
            log_method = self.get_logger().warn
        else:
            log_method = self.get_logger().error
        log_method(
            f'INTERCEPTION {self.outcome} | '
            f'Reason={result.reason} | '
            f'Detail={result.detail or "none"}'
        )
        self.get_logger().info(
            f'Time = {self.sim_time:.3f} s | '
            f'Distance = {distance:.3f} m | '
            f'Minimum distance = {minimum_distance:.3f} m | '
            f'Relative speed = {relative_speed:.3f} m/s'
        )
        self.get_logger().info(
            '========================================'
        )
        self.close_csv_log()

    def timer_callback(self):
        """Measure wall scheduling and callback compute time."""
        callback_start = time.perf_counter()
        if self.last_control_wall_time is not None:
            measured_wall_dt = callback_start - self.last_control_wall_time
            if 0.001 <= measured_wall_dt <= 1.0:
                self.wall_dt = measured_wall_dt
        self.last_control_wall_time = callback_start
        try:
            self._timer_callback_impl()
        finally:
            self.callback_compute_time = time.perf_counter() - callback_start
            for name in self.timing_samples:
                value = float(getattr(self, name, math.nan))
                if math.isfinite(value):
                    self.timing_samples[name].append(value)
                    phase = 'INTERCEPT' if self.started else 'FOLLOW'
                    self.timing_samples_by_phase[phase][name].append(value)

    def _timer_callback_impl(self):
        if self.enable_gazebo_control:
            self.gazebo_timer_callback()
            return

        if self.completed:
            return

        ready = (
            self.target_position_received
            and self.target_velocity_received
            and self.uav_state_received
        )

        if not self.started:
            if ready:
                self.start_simulation()
            return

        failure_reason = self.runtime_failure_reason()
        if failure_reason is not None:
            target_x, target_y, target_z = (
                self.estimated_target_position()
            )
            self.finish_interception(
                False,
                failure_reason,
                target_x,
                target_y,
                target_z,
            )
            return

        target_x, target_y, target_z = self.estimated_target_position()
        relative_start = (
            target_x - self.sim_x,
            target_y - self.sim_y,
            target_z - self.sim_z,
        )

        (
            desired_vx,
            desired_vy,
            desired_vz,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
            terminal_mode,
        ) = self.plan_velocity(target_x, target_y, target_z)
        self.terminal_mode_active = terminal_mode
        (
            horizontal_acceleration_limit,
            vertical_acceleration_limit,
        ) = self.command_acceleration_limits(
            terminal_mode
        )

        old_x = self.sim_x
        old_y = self.sim_y
        old_vx = self.sim_vx
        old_vy = self.sim_vy
        old_z = self.sim_z
        old_vz = self.sim_vz
        new_vx, new_vy = self.acceleration_limited_velocity(
            desired_vx,
            desired_vy,
            horizontal_acceleration_limit,
        )
        new_vz = self.acceleration_limited_vertical_velocity(
            desired_vz,
            vertical_acceleration_limit,
        )
        new_vz = self.apply_final_sea_safety(new_vz)

        acceleration_x = (new_vx - old_vx) / self.dt
        acceleration_y = (new_vy - old_vy) / self.dt
        acceleration_z = (new_vz - old_vz) / self.dt
        next_x = (
            old_x
            + old_vx * self.dt
            + 0.5 * acceleration_x * self.dt * self.dt
        )
        next_y = (
            old_y
            + old_vy * self.dt
            + 0.5 * acceleration_y * self.dt * self.dt
        )
        next_z = (
            old_z
            + old_vz * self.dt
            + 0.5 * acceleration_z * self.dt * self.dt
        )
        next_target_x = target_x + self.target_vx * self.dt
        next_target_y = target_y + self.target_vy * self.dt
        next_target_z = target_z + self.target_vz * self.dt
        relative_end = (
            next_target_x - next_x,
            next_target_y - next_y,
            next_target_z - next_z,
        )

        event_reason, event_fraction = self.terminal_event(
            relative_start,
            relative_end,
            old_z,
            next_z,
        )

        if event_fraction is not None:
            partial_dt = self.dt * event_fraction
            self.sim_x = (
                old_x
                + old_vx * partial_dt
                + 0.5 * acceleration_x * partial_dt * partial_dt
            )
            self.sim_y = (
                old_y
                + old_vy * partial_dt
                + 0.5 * acceleration_y * partial_dt * partial_dt
            )
            self.sim_z = (
                old_z
                + old_vz * partial_dt
                + 0.5 * acceleration_z * partial_dt * partial_dt
            )
            self.sim_vx = old_vx + acceleration_x * partial_dt
            self.sim_vy = old_vy + acceleration_y * partial_dt
            self.sim_vz = old_vz + acceleration_z * partial_dt
            self.sim_time += partial_dt
            self.max_observed_horizontal_speed = max(
                self.max_observed_horizontal_speed,
                math.hypot(self.sim_vx, self.sim_vy),
            )
            self.max_observed_vertical_speed = max(
                self.max_observed_vertical_speed,
                abs(self.sim_vz),
            )
            self.max_observed_horizontal_acceleration = max(
                self.max_observed_horizontal_acceleration,
                math.hypot(acceleration_x, acceleration_y),
            )
            self.max_observed_vertical_acceleration = max(
                self.max_observed_vertical_acceleration,
                abs(acceleration_z),
            )
            hit_target_x = target_x + self.target_vx * partial_dt
            hit_target_y = target_y + self.target_vy * partial_dt
            hit_target_z = target_z + self.target_vz * partial_dt
            success = event_reason == 'CAPTURE_RADIUS_REACHED'
            if not success:
                self.failure_detail = (
                    f'UAV NED z reached sea surface '
                    f'{self.sea_surface_z:.3f} m before capture'
                )
            self.finish_interception(
                success,
                event_reason,
                hit_target_x,
                hit_target_y,
                hit_target_z,
            )
            return

        self.update_interval_closest_approach(
            relative_start,
            relative_end,
        )

        self.sim_x = next_x
        self.sim_y = next_y
        self.sim_z = next_z
        self.sim_vx = new_vx
        self.sim_vy = new_vy
        self.sim_vz = new_vz
        self.sim_time += self.dt
        self.max_observed_horizontal_speed = max(
            self.max_observed_horizontal_speed,
            math.hypot(self.sim_vx, self.sim_vy),
        )
        self.max_observed_vertical_speed = max(
            self.max_observed_vertical_speed,
            abs(self.sim_vz),
        )
        self.max_observed_horizontal_acceleration = max(
            self.max_observed_horizontal_acceleration,
            math.hypot(acceleration_x, acceleration_y),
        )
        self.max_observed_vertical_acceleration = max(
            self.max_observed_vertical_acceleration,
            abs(acceleration_z),
        )

        (
            next_intercept_x,
            next_intercept_y,
            next_intercept_z,
            next_t_go,
        ) = (
            self.calculate_intercept_solution(
                self.sim_x,
                self.sim_y,
                self.sim_z,
                next_target_x,
                next_target_y,
                next_target_z,
            )
        )
        self.record_row(
            next_target_x,
            next_target_y,
            next_target_z,
            next_intercept_x,
            next_intercept_y,
            next_intercept_z,
            next_t_go,
        )
        self.publish_simulation_state(
            next_intercept_x,
            next_intercept_y,
            next_intercept_z,
        )

        step_index = round(self.sim_time / self.dt)
        if step_index % round(1.0 / self.dt) == 0:
            distance = math.sqrt(
                (next_target_x - self.sim_x) ** 2
                + (next_target_y - self.sim_y) ** 2
                + (next_target_z - self.sim_z) ** 2
            )
            speed = math.sqrt(
                self.sim_vx * self.sim_vx
                + self.sim_vy * self.sim_vy
                + self.sim_vz * self.sim_vz
            )
            self.get_logger().info(
                f'Time={self.sim_time:.2f} s | '
                f'Distance={distance:.2f} m | '
                f'T_go={next_t_go:.2f} s | '
                f'Virtual UAV speed={speed:.2f} m/s'
            )

        if self.sim_time >= self.max_sim_duration:
            self.finish_interception(
                False,
                'TIMEOUT',
                next_target_x,
                next_target_y,
                next_target_z,
            )

    def destroy_node(self):
        if self.started and not self.completed:
            target_x, target_y, target_z = (
                self.estimated_target_position()
            )
            self.finish_interception(
                False,
                'ABORTED',
                target_x,
                target_y,
                target_z,
            )
        self.close_csv_log()
        planner_executor = getattr(self, 'planner_executor', None)
        if planner_executor is not None:
            planner_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TrajectoryImpactSim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
