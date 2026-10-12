#!/usr/bin/env python3
"""Isolated safety-gated UAV axis motion, without mission control."""

import argparse
import math
import time

import rclpy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


MOVE_DISTANCE_M = 8.0
MOVE_SPEED_MPS = 1.2
MOVE_PHASE_SECONDS = 10.0
HOVER_SECONDS = 25.0
YAW_PHASE_SECONDS = 7.0
ALTITUDE_NED_M = -5.0


def ready_for_takeoff(status, position_fresh, prestream_seconds):
    """Never expose X until PX4 safety and OFFBOARD ground hold are true."""
    return bool(
        status is not None and position_fresh
        and prestream_seconds >= 2.0
        and status.pre_flight_checks_pass
        and not status.failsafe
        and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        and status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
    )


def clamp_rate(current, desired, rate, dt):
    delta = max(min(desired - current, rate * dt), -rate * dt)
    return current + delta


def should_publish_reference(state):
    """PX4 AUTO_LAND must not receive the prior hover setpoint."""
    return state not in ('LANDING', 'DONE')


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def phase_target(phase, elapsed, origin_xy, initial_yaw,
                 yaw_north_offset_m=0.0):
    """Generate a bounded position/yaw reference from independent phases."""
    north, east = origin_xy
    fraction = max(0.0, min(1.0,
                   MOVE_SPEED_MPS * elapsed / MOVE_DISTANCE_M))
    yaw = initial_yaw
    if phase == 'NORTH_OUT':
        north += MOVE_DISTANCE_M * fraction
    elif phase == 'NORTH_RETURN':
        north += MOVE_DISTANCE_M * (1.0 - fraction)
    elif phase == 'EAST_OUT':
        east += MOVE_DISTANCE_M * fraction
    elif phase == 'EAST_RETURN':
        east += MOVE_DISTANCE_M * (1.0 - fraction)
    elif phase.startswith('YAW_'):
        north += yaw_north_offset_m
    if phase == 'YAW_P45':
        yaw = wrap_angle(initial_yaw + math.pi / 4)
    elif phase == 'YAW_P90':
        yaw = wrap_angle(initial_yaw + math.pi / 2)
    elif phase == 'YAW_M45':
        yaw = wrap_angle(initial_yaw - math.pi / 4)
    return (north, east, ALTITUDE_NED_M), yaw


def phase_schedule(profile):
    yaw = (
        ('YAW_0', YAW_PHASE_SECONDS),
        ('YAW_P45', YAW_PHASE_SECONDS),
        ('YAW_P90', YAW_PHASE_SECONDS),
        ('YAW_BACK_0', YAW_PHASE_SECONDS),
        ('YAW_M45', YAW_PHASE_SECONDS),
    )
    if profile == 'yaw_offset':
        return (('HOVER', 10.0),
                ('NORTH_OUT', MOVE_PHASE_SECONDS)) + yaw + (
                ('NORTH_RETURN', MOVE_PHASE_SECONDS),)
    if profile == 'full':
        return (('HOVER', HOVER_SECONDS),
                ('NORTH_OUT', MOVE_PHASE_SECONDS),
                ('NORTH_RETURN', MOVE_PHASE_SECONDS),
                ('EAST_OUT', MOVE_PHASE_SECONDS),
                ('EAST_RETURN', MOVE_PHASE_SECONDS)) + yaw
    raise ValueError('unknown P7 axis profile')


class AxisMotion(Node):
    """One PX4 command owner in a dedicated P7 launch, with X safety gate."""

    def __init__(self, profile="full"):
        super().__init__('p7_px4_local_axis_motion')
        self.profile = profile
        self.phases = phase_schedule(profile)
        self.status = None
        self.status_time = None
        self.position = None
        self.position_time = None
        self.velocity = None
        self.heading = None
        self.origin = None
        self.initial_yaw = None
        self.position_ref = None
        self.yaw_ref = None
        self.state = 'GROUND'
        self.phase_index = -1
        self.phase_started = None
        self.takeoff_settled_at = None
        self.prestream_started = time.monotonic()
        self.last_tick = time.monotonic()
        self.last_command = 0.0
        self.abort_reason = None
        self.last_armed = False
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.status_callback, qos,
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback, qos,
        )
        self.create_subscription(
            String, '/simulation/impact/command', self.command_callback, 10,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10,
        )
        self.ready_pub = self.create_publisher(
            Bool, '/simulation/impact/flight_ready', 10,
        )
        self.phase_pub = self.create_publisher(
            String, '/p7/axis/phase', 10,
        )
        self.create_timer(0.05, self.tick)

    def now(self):
        return time.monotonic()

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def status_callback(self, message):
        self.status = message
        self.status_time = self.now()
        self.last_armed = (
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def position_callback(self, message):
        values = (message.x, message.y, message.z,
                  message.vx, message.vy, message.vz, message.heading)
        if not all(math.isfinite(float(value)) for value in values):
            return
        self.position = tuple(float(value) for value in values[:3])
        self.velocity = tuple(float(value) for value in values[3:6])
        self.heading = float(message.heading)
        self.position_time = self.now()
        if self.origin is None and self.state == 'GROUND':
            self.origin = self.position[:2]
            self.initial_yaw = self.heading
            self.position_ref = self.position
            self.yaw_ref = self.heading

    def command_callback(self, message):
        if str(message.data).upper() != 'X' or self.state != 'GROUND':
            return
        if not self.is_ready():
            self.get_logger().warning('X ignored: PX4 preflight gate closed')
            return
        self.state = 'TAKEOFF'
        self.get_logger().info('P7 AXIS | X accepted: safe takeoff')

    def is_ready(self):
        now = self.now()
        fresh = (self.position_time is not None
                 and now - self.position_time <= 0.5
                 and self.status_time is not None
                 and now - self.status_time <= 1.0)
        return ready_for_takeoff(
            self.status, fresh, now - self.prestream_started,
        )

    def send_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.timestamp = self.timestamp_us()
        message.command = command
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_pub.publish(message)
        self.last_command = self.now()

    def publish_reference(self):
        if self.position_ref is None:
            return
        timestamp = self.timestamp_us()
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self.offboard_pub.publish(mode)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = [float(x) for x in self.position_ref]
        setpoint.velocity = [math.nan] * 3
        setpoint.acceleration = [math.nan] * 3
        setpoint.jerk = [math.nan] * 3
        setpoint.yaw = float(self.yaw_ref)
        setpoint.yawspeed = math.nan
        self.setpoint_pub.publish(setpoint)

    def next_phase(self, now):
        self.phase_index += 1
        if self.phase_index >= len(self.phases):
            self.state = 'LANDING'
            self.get_logger().info('P7 AXIS | phases complete; landing')
            return
        self.state = self.phases[self.phase_index][0]
        self.phase_started = now
        self.get_logger().info(f'P7 AXIS | phase {self.state}')

    def tick(self):
        now = self.now()
        dt = min(max(now - self.last_tick, 0.0), 0.1)
        self.last_tick = now
        status = self.status
        armed = self.last_armed
        if self.state != 'GROUND' and self.state != 'DONE':
            if (status is None or self.status_time is None
                    or now - self.status_time > 1.0
                    or self.position_time is None
                    or now - self.position_time > 0.5
                    or status.failsafe):
                self.abort_reason = 'status_stale_or_failsafe'
                self.state = 'LANDING'
            if self.origin is not None and self.position is not None:
                radius = math.hypot(
                    self.position[0] - self.origin[0],
                    self.position[1] - self.origin[1],
                )
                if radius > 20.0 or self.position[2] < -8.0:
                    self.abort_reason = 'geofence_or_altitude'
                    self.state = 'LANDING'
        ready_message = Bool()
        ready_message.data = (
            self.is_ready() if self.state == 'GROUND' else False
        )
        self.ready_pub.publish(ready_message)
        phase_message = String()
        phase_message.data = self.state
        self.phase_pub.publish(phase_message)
        if self.position_ref is None:
            return
        if self.state == 'GROUND':
            if (now - self.prestream_started >= 2.0
                    and not self.is_ready()
                    and status is not None
                    and status.pre_flight_checks_pass
                    and not status.failsafe
                    and now - self.last_command >= 1.0):
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif self.state == 'TAKEOFF':
            if not armed and now - self.last_command >= 1.0:
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if armed:
                z = clamp_rate(self.position_ref[2], ALTITUDE_NED_M, 1.0,
                               dt)
                self.position_ref = (self.origin[0], self.origin[1], z)
                settled = (abs(self.position[2] - ALTITUDE_NED_M) < 0.3
                           and abs(self.velocity[2]) < 0.3)
                if settled:
                    if self.takeoff_settled_at is None:
                        self.takeoff_settled_at = now
                    elif now - self.takeoff_settled_at >= 2.0:
                        self.next_phase(now)
                else:
                    self.takeoff_settled_at = None
        elif self.state not in ('LANDING', 'DONE'):
            duration = self.phases[self.phase_index][1]
            elapsed = now - self.phase_started
            desired_position, desired_yaw = phase_target(
                self.state, elapsed, self.origin, self.initial_yaw,
                yaw_north_offset_m=(
                    MOVE_DISTANCE_M if self.profile == 'yaw_offset'
                    else 0.0),
            )
            self.position_ref = desired_position
            yaw_error = wrap_angle(desired_yaw - self.yaw_ref)
            self.yaw_ref = wrap_angle(self.yaw_ref + max(min(
                yaw_error, 0.45 * dt), -0.45 * dt))
            if elapsed >= duration:
                self.next_phase(now)
        elif self.state == 'LANDING':
            if status is not None and not armed:
                self.state = 'DONE'
                self.get_logger().info('P7 AXIS | disarmed; capture complete')
            elif now - self.last_command >= 1.0:
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if should_publish_reference(self.state):
            self.publish_reference()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('full', 'yaw_offset'),
                        default='full')
    args = parser.parse_args()
    rclpy.init()
    node = AxisMotion(args.profile)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
