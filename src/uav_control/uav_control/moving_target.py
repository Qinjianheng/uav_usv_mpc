from dataclasses import fields
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, Vector3
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, String
from uav_control.usv_motion_profiles import (
    elapsed_motion_step, MotionModeSelector, MotionParameters, UsvMotionProfile,
)
from uav_usv_interfaces.msg import MissionState, TargetState


def measured_motion_step(previous_time_ns, current_time_ns, nominal_step):
    """Return elapsed ROS-clock time while rejecting invalid clock jumps."""
    nominal_step = float(nominal_step)
    if previous_time_ns is None:
        return nominal_step
    elapsed = (int(current_time_ns) - int(previous_time_ns)) * 1e-9
    if not math.isfinite(elapsed) or elapsed <= 0.0 or elapsed > 1.0:
        return nominal_step
    return elapsed


def target_state_message(stamp_seconds, position, velocity, acceleration=(0., 0., 0.)):
    """Build a timestamped truth state without exposing future motion."""
    nanoseconds = round(float(stamp_seconds) * 1e9)
    message = TargetState()
    message.stamp.sec = nanoseconds // 1_000_000_000
    message.stamp.nanosec = nanoseconds % 1_000_000_000
    message.frame_id = 'local_ned'
    message.position.x = float(position[0])
    message.position.y = float(position[1])
    message.position.z = float(position[2])
    message.velocity.x = float(velocity[0])
    message.velocity.y = float(velocity[1])
    message.velocity.z = float(velocity[2])
    message.acceleration.x = float(acceleration[0])
    message.acceleration.y = float(acceleration[1])
    message.acceleration.z = float(acceleration[2])
    message.covariance = [0.0] * 36
    message.valid = True
    return message


class MovingTarget(Node):

    def __init__(self, **kwargs):
        super().__init__('moving_target', **kwargs)

        self.declare_parameter('initial_x', 8.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('velocity_x', 0.0)
        self.declare_parameter('velocity_y', 4.0)
        self.declare_parameter('velocity_z', 0.0)
        self.declare_parameter('trajectory_type', 'figure_eight')
        self.declare_parameter('horizontal_speed', 4.0)
        self.declare_parameter('horizontal_acceleration_limit', 1.0)
        self.declare_parameter('maximum_turn_rate', 0.65)
        self.declare_parameter('maximum_lateral_acceleration', 3.2)
        self.declare_parameter('figure_eight_x_amplitude', 40.0)
        self.declare_parameter('figure_eight_y_amplitude', 20.0)
        self.declare_parameter('vertical_oscillation_amplitude', 0.15)
        self.declare_parameter('vertical_oscillation_frequency', 0.25)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('start_on_command', True)
        self.declare_parameter('enable_gazebo_visualization', True)
        self.declare_parameter('gazebo_world_name', 'default')
        self.declare_parameter('gazebo_entity_name', 'usv_target')
        self.declare_parameter('gazebo_sphere_diameter', 0.5)
        self.declare_parameter('gazebo_visual_height_offset', 0.42)
        self.declare_parameter('pause_gazebo_on_hit', True)
        # Old parameter names and defaults above remain compatible. The new
        # profile bounds/shape parameters are startup configuration only.
        profile_defaults = MotionParameters()
        for field in fields(profile_defaults):
            if field.name in ('linear_vx', 'linear_vy', 'integration_step'):
                continue
            if not self.has_parameter(field.name):
                self.declare_parameter(
                    field.name, getattr(profile_defaults, field.name))

        self.position_pub = self.create_publisher(
            Point,
            '/target/position',
            10
        )

        self.velocity_pub = self.create_publisher(
            Vector3,
            '/target/velocity',
            10
        )
        self.state_pub = self.create_publisher(
            TargetState,
            '/target/state',
            10,
        )
        self.motion_mode_pub = self.create_publisher(
            String, '/simulation/impact/motion_mode', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.motion_truth_pub = self.create_publisher(
            String, '/simulation/impact/motion_truth', 10)

        self.command_sub = self.create_subscription(
            String,
            '/simulation/impact/command',
            self.command_callback,
            10,
        )
        self.mission_state_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_state_callback,
            10,
        )
        self.flight_ready_sub = self.create_subscription(
            Bool,
            '/simulation/impact/flight_ready',
            self.flight_ready_callback,
            10,
        )

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.initial_z = float(self.get_parameter('initial_z').value)
        self.z = self.initial_z

        self.linear_vx = float(self.get_parameter('velocity_x').value)
        self.linear_vy = float(self.get_parameter('velocity_y').value)
        self.vx = self.linear_vx
        self.vy = self.linear_vy
        self.vz = float(self.get_parameter('velocity_z').value)
        self.horizontal_acceleration_limit = max(
            float(
                self.get_parameter(
                    'horizontal_acceleration_limit'
                ).value
            ),
            0.1,
        )
        self.maximum_turn_rate = max(
            float(self.get_parameter('maximum_turn_rate').value),
            0.1,
        )
        self.maximum_lateral_acceleration = max(
            float(
                self.get_parameter(
                    'maximum_lateral_acceleration'
                ).value
            ),
            0.1,
        )
        self.cruise_horizontal_speed = float(
            self.get_parameter('horizontal_speed').value
        )
        self.commanded_horizontal_speed = 0.0
        self.trajectory_type = str(
            self.get_parameter('trajectory_type').value
        ).strip().lower()
        self.vertical_oscillation_amplitude = float(
            self.get_parameter('vertical_oscillation_amplitude').value
        )
        self.vertical_oscillation_frequency = float(
            self.get_parameter('vertical_oscillation_frequency').value
        )
        self.vertical_angular_frequency = (
            2.0 * math.pi * self.vertical_oscillation_frequency
        )
        self.elapsed_time = 0.0

        self.hit = False
        self.flight_ready = False
        self.started = not bool(
            self.get_parameter('start_on_command').value
        )

        update_rate_hz = max(
            float(self.get_parameter('update_rate_hz').value),
            1.0,
        )
        self.dt = 1.0 / update_rate_hz
        values = {
            field.name: self.get_parameter(field.name).value
            for field in fields(profile_defaults)
            if field.name not in ('linear_vx', 'linear_vy', 'integration_step')
        }
        values.update(linear_vx=self.linear_vx, linear_vy=self.linear_vy,
                      integration_step=self.dt)
        self.motion_parameters = MotionParameters(**values)
        self.motion_selector = MotionModeSelector(self.trajectory_type)
        self.motion_profile = UsvMotionProfile(
            self.motion_selector.mode, self.motion_parameters)
        self.trajectory_type = self.motion_selector.mode
        self._sync_motion_sample(self.motion_profile.state())
        if self.started:
            self.motion_selector.start()
        self.add_on_set_parameters_callback(self.mode_parameters_callback)
        self.last_motion_update_time_ns = (
            self.get_clock().now().nanoseconds if self.started else None
        )

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        self.gazebo_visualizer = None
        self.gazebo_visualization_ready = False
        self.gazebo_world_paused = False
        self.last_gazebo_warning_time = -math.inf
        self.pause_gazebo_on_hit = bool(
            self.get_parameter('pause_gazebo_on_hit').value
        )
        self.gazebo_visual_height_offset = max(
            float(
                self.get_parameter(
                    'gazebo_visual_height_offset'
                ).value
            ),
            0.0,
        )
        if bool(
            self.get_parameter('enable_gazebo_visualization').value
        ):
            try:
                from uav_control.gazebo_target_visualizer import (
                    GazeboTargetVisualizer,
                )

                self.gazebo_visualizer = GazeboTargetVisualizer(
                    world_name=self.get_parameter(
                        'gazebo_world_name'
                    ).value,
                    entity_name=self.get_parameter(
                        'gazebo_entity_name'
                    ).value,
                    diameter=self.get_parameter(
                        'gazebo_sphere_diameter'
                    ).value,
                )
            except (ImportError, RuntimeError, ValueError) as exc:
                self.get_logger().warn(
                    'Gazebo target visualization disabled: '
                    f'{exc}'
                )

        self.get_logger().info(
            'Moving target node ready; waiting for X to start motion.'
            if not self.started
            else 'Moving target node started in immediate-motion mode.'
        )
        self.publish_motion_mode()

    def _sync_motion_sample(self, sample):
        self.current_motion_sample = sample
        self.x, self.y, self.z = sample.position
        self.vx, self.vy = sample.velocity[:2]
        self.commanded_horizontal_speed = self.motion_profile.speed
        self.elapsed_time = self.motion_profile.elapsed_time
        self.figure_eight_trajectory = self.motion_profile.figure_eight

    def publish_motion_mode(self, acknowledgement=None):
        acknowledgement = acknowledgement or self.motion_selector.status()
        message = String()
        message.data = json.dumps(acknowledgement.as_dict(), sort_keys=True)
        self.motion_mode_pub.publish(message)

    def mode_parameters_callback(self, parameters):
        """Use the standard SetParameters service as independent selection ACK."""
        motion_names = {field.name for field in fields(MotionParameters())}
        motion_names.update(('velocity_x', 'velocity_y', 'update_rate_hz'))
        selections = [p for p in parameters if p.name == 'trajectory_type']
        changes = [p for p in parameters if p.name in motion_names]
        self._pending_motion_profile = None
        if changes or len(selections) > 1 or (selections and len(parameters) != 1):
            ack = self.motion_selector.status(
                False, 'motion configuration requires restart; select mode separately')
        elif not selections:
            return SetParametersResult(successful=True)
        else:
            selector = MotionModeSelector(self.trajectory_type)
            selector.started = self.started
            ack = selector.select(selections[0].value)
            if ack.successful and selector.mode != self.trajectory_type:
                try:
                    profile = UsvMotionProfile(selector.mode, self.motion_parameters)
                except ValueError as exc:
                    ack = self.motion_selector.status(False, str(exc))
                else:
                    self._pending_motion_profile = profile
        return SetParametersResult(
            successful=ack.successful,
            reason=json.dumps(ack.as_dict(), sort_keys=True))

    def set_parameters(self, parameters):
        # Keep the public non-atomic API consistent with ParameterService,
        # which invokes set_parameters_atomically once for each parameter.
        return [self.set_parameters_atomically([parameter])
                for parameter in parameters]

    def set_parameters_atomically(self, parameters):
        """
        Commit profile only after ROS validation, and preserve success ACK.

        Humble discards successful callback ``reason`` values. The standard
        ParameterService delegates here, so attach the applied state after
        the framework commits instead of replacing or duplicating a service.
        """
        result = super().set_parameters_atomically(parameters)
        selections = [p for p in parameters if p.name == 'trajectory_type']
        if not selections or not hasattr(self, 'motion_selector'):
            return result
        if result.successful:
            selector = MotionModeSelector(self.trajectory_type)
            selector.started = self.started
            ack = selector.select(selections[0].value)
            if selector.mode != self.trajectory_type:
                self.motion_profile = self._pending_motion_profile
                self.motion_selector = selector
                self.trajectory_type = selector.mode
                self._sync_motion_sample(self.motion_profile.state())
        else:
            try:
                reason = json.loads(result.reason)['reason']
            except (ValueError, KeyError, TypeError):
                reason = result.reason
            ack = self.motion_selector.status(False, reason)
        self.publish_motion_mode(ack)
        result.reason = json.dumps(ack.as_dict(), sort_keys=True)
        return result

    def _start_motion(self, message):
        if self.started:
            return
        self.started = True
        selector = getattr(self, 'motion_selector', None)
        if selector is not None:
            MovingTarget.publish_motion_mode(self, selector.start())
        get_clock = getattr(self, 'get_clock', None)
        self.last_motion_update_time_ns = (
            get_clock().now().nanoseconds
            if get_clock is not None
            else None
        )
        self.get_logger().info(message)

    def command_callback(self, msg):
        if msg.data.strip().upper() == 'X' and not self.started:
            if not self.flight_ready:
                self.get_logger().warn(
                    'X ignored: UAV flight preparation is not ready.'
                )
                return
            MovingTarget._start_motion(
                self,
                'X received. Moving target motion started.',
            )

    def mission_state_callback(self, msg):
        if int(msg.state) == MissionState.TAKEOFF and not self.started:
            MovingTarget._start_motion(
                self,
                'Mission TAKEOFF accepted. Moving target motion started.',
            )

    def flight_ready_callback(self, msg):
        self.flight_ready = bool(msg.data)

    def timer_callback(self):
        now_ns = self.get_clock().now().nanoseconds
        clock_stalled = False
        # The renderer can accept set_pose requests even while physics is
        # paused. Follow the native world pause flag so wall-clock ROS timers
        # cannot keep moving the target after the UAV simulation has stopped.
        native_paused = getattr(self.gazebo_visualizer, 'world_paused', None)
        if native_paused is not None:
            self.gazebo_world_paused = bool(native_paused)
        if self.gazebo_world_paused:
            # Keep an actual clock anchor, never invent a nominal resume step.
            self.last_motion_update_time_ns = now_ns
        # 更新目标位置
        if self.started and not self.hit and not self.gazebo_world_paused:
            motion_step = elapsed_motion_step(
                self.last_motion_update_time_ns,
                now_ns,
            )
            self.last_motion_update_time_ns = now_ns
            clock_stalled = motion_step == 0.

            profile = getattr(self, 'motion_profile', None)
            if profile is not None:
                self._sync_motion_sample(profile.advance(motion_step))
            else:
                # Retain the original legacy integration for existing callers
                # that supply the historical explicit state interface.
                MovingTarget._advance_legacy_state(self, motion_step)

        # 发布位置
        position_msg = Point()
        position_msg.x = self.x
        position_msg.y = self.y
        position_msg.z = self.z
        self.position_pub.publish(position_msg)

        stationary = (self.hit or not self.started or self.gazebo_world_paused
                      or clock_stalled)
        velocity_msg = Vector3()
        acceleration = (0., 0., 0.)
        if not stationary:
            velocity_msg.x = self.vx
            velocity_msg.y = self.vy
            velocity_msg.z = (
                self.vz
                + self.vertical_oscillation_amplitude
                * self.vertical_angular_frequency
                * math.cos(self.vertical_angular_frequency * self.elapsed_time))
            sample = getattr(self, 'current_motion_sample', None)
            if sample is not None:
                acceleration = sample.acceleration
        self.velocity_pub.publish(velocity_msg)
        self.state_pub.publish(target_state_message(
            stamp_seconds=now_ns * 1e-9,
            position=(position_msg.x, position_msg.y, position_msg.z),
            velocity=(velocity_msg.x, velocity_msg.y, velocity_msg.z),
            acceleration=acceleration))
        truth_pub = getattr(self, 'motion_truth_pub', None)
        if truth_pub is not None:
            sample = self.current_motion_sample
            diagnostic = String()
            diagnostic.data = json.dumps({
                'stamp_ns': now_ns, 'mode': self.trajectory_type,
                'elapsed_time': self.elapsed_time,
                'position': sample.position,
                'velocity': (velocity_msg.x, velocity_msg.y, velocity_msg.z),
                'acceleration': acceleration,
                'yaw': sample.yaw,
                'turn_rate': 0. if stationary else sample.turn_rate,
                'curvature': sample.curvature, 'phase': sample.phase,
                'stage': 'stationary' if stationary else sample.stage,
                'longitudinal_acceleration': (
                    0. if stationary else sample.longitudinal_acceleration),
                'lateral_acceleration': (
                    0. if stationary else sample.lateral_acceleration),
                'angular_acceleration': (
                    0. if stationary else sample.angular_acceleration),
                'evaluation_only': True,
            }, sort_keys=True)
            truth_pub.publish(diagnostic)
        if not self.gazebo_world_paused:
            self.update_gazebo_visualization()

    def _advance_legacy_state(self, motion_step):
        # Original explicit-state interface, including its numerical order.
        remaining = motion_step
        while remaining > 1e-12:
            integration_step = min(remaining, self.dt)
            self.commanded_horizontal_speed = min(
                self.commanded_horizontal_speed
                + self.horizontal_acceleration_limit * integration_step,
                self.cruise_horizontal_speed,
            )
            if self.figure_eight_trajectory is None:
                requested_speed = math.hypot(self.linear_vx, self.linear_vy)
                velocity_scale = (
                    self.commanded_horizontal_speed / requested_speed
                    if requested_speed > 1e-9 else 0.0)
                self.vx = self.linear_vx * velocity_scale
                self.vy = self.linear_vy * velocity_scale
                self.x += self.vx * integration_step
                self.y += self.vy * integration_step
            else:
                self.figure_eight_trajectory.set_speed(self.commanded_horizontal_speed)
                self.x, self.y, self.vx, self.vy = (
                    self.figure_eight_trajectory.advance(integration_step))
            remaining -= integration_step

        self.elapsed_time += motion_step
        self.z = (
            self.initial_z
            + self.vz * self.elapsed_time
            + self.vertical_oscillation_amplitude * math.sin(
                self.vertical_angular_frequency * self.elapsed_time)
        )

    def pause_gazebo_world(self):
        if self.gazebo_visualizer is None:
            self.get_logger().warn(
                'Cannot pause Gazebo: target visualizer is unavailable.'
            )
            return

        try:
            paused = self.gazebo_visualizer.pause_world()
        except RuntimeError as exc:
            paused = False
            self.gazebo_visualizer.last_error = str(exc)

        if paused:
            self.gazebo_world_paused = True
            self.get_logger().info(
                'Gazebo world paused at the experiment terminal event.'
            )
        else:
            detail = (
                self.gazebo_visualizer.last_error
                or 'unknown Gazebo Transport error'
            )
            self.get_logger().warn(
                f'Unable to pause Gazebo after terminal event: {detail}'
            )

    def update_gazebo_visualization(self):
        if self.gazebo_visualizer is None:
            return

        try:
            updated = self.gazebo_visualizer.update(
                self.x,
                self.y,
                self.z - self.gazebo_visual_height_offset,
            )
        except RuntimeError as exc:
            updated = False
            self.gazebo_visualizer.last_error = str(exc)

        if updated:
            if not self.gazebo_visualization_ready:
                self.gazebo_visualization_ready = True
                self.get_logger().info(
                    'Gazebo red USV target sphere is visible.'
                )
            return

        warning_time = self.get_clock().now().nanoseconds * 1e-9
        if warning_time - self.last_gazebo_warning_time >= 5.0:
            self.last_gazebo_warning_time = warning_time
            detail = (
                self.gazebo_visualizer.last_error
                or 'waiting for Gazebo entity creation'
            )
            self.get_logger().warn(
                f'Gazebo red target sphere not ready: {detail}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MovingTarget()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
