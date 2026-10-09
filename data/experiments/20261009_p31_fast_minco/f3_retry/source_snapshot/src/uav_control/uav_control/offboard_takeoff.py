import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)

import math


class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__('offboard_takeoff')

        # 发布器
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

        # 20Hz发送
        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.counter = 0

        self.get_logger().info(
            "Offboard takeoff node started"
        )

    def timestamp(self):
        return int(
            self.get_clock()
            .now()
            .nanoseconds / 1000
        )

    def publish_offboard_mode(self):
        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        # 位置控制
        msg.position = True

        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    def publish_setpoint(self):
        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # 目标位置
        # NED坐标
        msg.position = [
            5.0,
            0.0,
            -2.0
        ]

        # 不控制速度
        msg.velocity = [
            math.nan,
            math.nan,
            math.nan
        ]

        msg.acceleration = [
            math.nan,
            math.nan,
            math.nan
        ]

        msg.yaw = 0.0

        self.setpoint_pub.publish(msg)

    def send_command(self, command):
        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = (
            VehicleCommand
            .VEHICLE_CMD_COMPONENT_ARM_DISARM
        )

        msg.param1 = 1.0

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    def offboard_mode(self):
        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = (
            VehicleCommand
            .VEHICLE_CMD_DO_SET_MODE
        )

        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param1 = 1.0

        # PX4 custom mode: OFFBOARD
        msg.param2 = 6.0

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    def timer_callback(self):
        self.publish_offboard_mode()

        self.publish_setpoint()

        self.counter += 1

        # 先发送一秒setpoint
        if self.counter == 20:

            self.get_logger().info(
                "Switching to OFFBOARD"
            )

            self.offboard_mode()

        if self.counter == 40:

            self.get_logger().info(
                "Arming"
            )

            self.arm()


def main(args=None):

    rclpy.init(args=args)

    node = OffboardTakeoff()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
