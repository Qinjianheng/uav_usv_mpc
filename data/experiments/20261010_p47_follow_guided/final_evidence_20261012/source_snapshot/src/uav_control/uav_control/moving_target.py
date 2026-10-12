import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, String
from uav_control.figure_eight_trajectory import FigureEightTrajectory
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


def target_state_message(stamp_seconds, position, velocity):
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
    message.acceleration.x = 0.0
    message.acceleration.y = 0.0
    message.acceleration.z = 0.0
    message.covariance = [0.0] * 36
    message.valid = True
    return message


class MovingTarget(Node):

    def __init__(self):
        super().__init__('moving_target')

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
        self.figure_eight_trajectory = None
        if self.trajectory_type == 'figure_eight':
            self.figure_eight_trajectory = FigureEightTrajectory(
                initial_x=self.x,
                initial_y=self.y,
                x_amplitude=self.get_parameter(
                    'figure_eight_x_amplitude'
                ).value,
                y_amplitude=self.get_parameter(
                    'figure_eight_y_amplitude'
                ).value,
                speed=self.cruise_horizontal_speed,
            )
            (
                self.peak_turn_rate,
                self.peak_lateral_acceleration,
            ) = self.figure_eight_trajectory.kinematic_envelope(
                self.cruise_horizontal_speed
            )
            if self.peak_turn_rate > self.maximum_turn_rate:
                raise ValueError(
                    'USV figure-eight exceeds maximum_turn_rate: '
                    f'{self.peak_turn_rate:.3f} > '
                    f'{self.maximum_turn_rate:.3f} rad/s'
                )
            if (
                self.peak_lateral_acceleration
                > self.maximum_lateral_acceleration
            ):
                raise ValueError(
                    'USV figure-eight exceeds '
                    'maximum_lateral_acceleration: '
                    f'{self.peak_lateral_acceleration:.3f} > '
                    f'{self.maximum_lateral_acceleration:.3f} m/s^2'
                )
            self.figure_eight_trajectory.set_speed(0.0)
            self.x, self.y, self.vx, self.vy = (
                self.figure_eight_trajectory.state()
            )
        elif self.trajectory_type != 'linear':
            raise ValueError(
                'trajectory_type must be "linear" or "figure_eight".'
            )
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
        if self.figure_eight_trajectory is not None:
            x_min, x_max = self.figure_eight_trajectory.x_limits
            y_min, y_max = self.figure_eight_trajectory.y_limits
            self.get_logger().info(
                'USV figure-eight trajectory enabled | '
                f'speed=0.00->{self.cruise_horizontal_speed:.2f} m/s | '
                f'acceleration<={self.horizontal_acceleration_limit:.2f} '
                f'm/s^2 | turn rate<={self.peak_turn_rate:.2f} rad/s | '
                f'lateral acceleration<='
                f'{self.peak_lateral_acceleration:.2f} m/s^2 | '
                f'X=[{x_min:.1f}, {x_max:.1f}] m | '
                f'Y=[{y_min:.1f}, {y_max:.1f}] m'
            )

    def _start_motion(self, message):
        if self.started:
            return
        self.started = True
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
        # The renderer can accept set_pose requests even while physics is
        # paused. Follow the native world pause flag so wall-clock ROS timers
        # cannot keep moving the target after the UAV simulation has stopped.
        native_paused = getattr(self.gazebo_visualizer, 'world_paused', None)
        if native_paused is not None:
            self.gazebo_world_paused = bool(native_paused)
        if self.gazebo_world_paused:
            # Resume from a nominal step, never integrate the paused interval.
            self.last_motion_update_time_ns = None
        # 更新目标位置
        if self.started and not self.hit and not self.gazebo_world_paused:
            now_ns = self.get_clock().now().nanoseconds
            motion_step = measured_motion_step(
                self.last_motion_update_time_ns,
                now_ns,
                self.dt,
            )
            self.last_motion_update_time_ns = now_ns

            # Integrate delayed callbacks in nominal-sized substeps.  The
            # elapsed trajectory time still follows the ROS clock, while the
            # acceleration ramp and curved path retain numerical fidelity.
            remaining = motion_step
            while remaining > 1e-12:
                integration_step = min(remaining, self.dt)
                self.commanded_horizontal_speed = min(
                    self.commanded_horizontal_speed
                    + self.horizontal_acceleration_limit * integration_step,
                    self.cruise_horizontal_speed,
                )
                if self.figure_eight_trajectory is None:
                    requested_speed = math.hypot(
                        self.linear_vx,
                        self.linear_vy,
                    )
                    velocity_scale = (
                        self.commanded_horizontal_speed / requested_speed
                        if requested_speed > 1e-9
                        else 0.0
                    )
                    self.vx = self.linear_vx * velocity_scale
                    self.vy = self.linear_vy * velocity_scale
                    self.x += self.vx * integration_step
                    self.y += self.vy * integration_step
                else:
                    self.figure_eight_trajectory.set_speed(
                        self.commanded_horizontal_speed
                    )
                    self.x, self.y, self.vx, self.vy = (
                        self.figure_eight_trajectory.advance(integration_step)
                    )
                remaining -= integration_step

            self.elapsed_time += motion_step
            self.z = (
                self.initial_z
                + self.vz * self.elapsed_time
                + self.vertical_oscillation_amplitude * math.sin(
                    self.vertical_angular_frequency * self.elapsed_time
                )
            )

        # 发布位置
        position_msg = Point()

        position_msg.x = self.x
        position_msg.y = self.y
        position_msg.z = self.z

        self.position_pub.publish(position_msg)

        # 发布速度
        velocity_msg = Vector3()

        if self.hit or not self.started or self.gazebo_world_paused:
            velocity_msg.x = 0.0
            velocity_msg.y = 0.0
            velocity_msg.z = 0.0
        else:
            velocity_msg.x = self.vx
            velocity_msg.y = self.vy
            velocity_msg.z = (
                self.vz
                + self.vertical_oscillation_amplitude
                * self.vertical_angular_frequency
                * math.cos(
                    self.vertical_angular_frequency * self.elapsed_time
                )
            )

        self.velocity_pub.publish(velocity_msg)
        state_stamp = self.get_clock().now().nanoseconds * 1e-9
        self.state_pub.publish(target_state_message(
            stamp_seconds=state_stamp,
            position=(position_msg.x, position_msg.y, position_msg.z),
            velocity=(velocity_msg.x, velocity_msg.y, velocity_msg.z),
        ))

        if not self.gazebo_world_paused:
            self.update_gazebo_visualization()

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
