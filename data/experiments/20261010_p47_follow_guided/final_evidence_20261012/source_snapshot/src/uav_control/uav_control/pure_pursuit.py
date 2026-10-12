import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)


class PurePursuit(Node):

    def __init__(self):
        raise RuntimeError(
            'Retired truth-control entry point; use modular_intercept.launch.py'
        )
        super().__init__('pure_pursuit')

        # ========================================
        # PX4 发布器
        # ========================================

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

        # ========================================
        # 虚拟目标订阅
        # ========================================

        self.target_sub = self.create_subscription(
            Point,
            '/target/position',
            self.target_callback,
            10
        )

        # 目标速度只用于实验记录，不参与 Pure Pursuit 控制。
        self.target_velocity_sub = self.create_subscription(
            Vector3,
            '/target/velocity',
            self.target_velocity_callback,
            10
        )

        # ========================================
        # PX4输出话题QoS
        # ========================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ========================================
        # 无人机当前位置订阅
        # 注意你的PX4使用_v1话题
        # ========================================

        self.vehicle_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_callback,
            px4_qos
        )

        # ========================================
        # 目标状态
        # ========================================

        self.target_x = 20.0
        self.target_y = 0.0

        self.target_vx = 0.0
        self.target_vy = 0.0

        self.target_received = False
        self.target_velocity_received = False

        # ========================================
        # UAV状态
        # ========================================

        self.uav_x = 0.0
        self.uav_y = 0.0
        self.uav_z = 0.0

        self.uav_vx = 0.0
        self.uav_vy = 0.0
        self.uav_vz = 0.0

        self.uav_state_received = False

        # ========================================
        # 截击判定
        # ========================================

        # 与 Predictive Intercept 使用相同成功阈值。
        self.intercept_distance = 0.5
        self.intercepted = False

        # ========================================
        # 实验记录
        # ========================================

        # 相对路径以启动 ros2 run 时的当前目录为基准。
        self.declare_parameter(
            'log_directory',
            'experiment_logs'
        )

        self.experiment_start_time_ns = None
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

        # ========================================
        # 飞行参数
        # ========================================

        # PX4使用NED坐标
        # z=-5 表示高度约5m
        self.flight_altitude = -5.0

        # 20 Hz，与 Predictive Intercept 相同。
        self.dt = 0.05

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        self.counter = 0

        # 4秒后开始追目标
        self.takeoff_counter = 80

        self.get_logger().info(
            'Pure pursuit node started'
        )

    # ============================================
    # 时间戳
    # ============================================

    def timestamp(self):

        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    # ============================================
    # 实验记录
    # ============================================

    def start_experiment_logging(self):

        if self.experiment_start_time_ns is not None:
            return

        log_directory = Path(
            self.get_parameter(
                'log_directory'
            ).get_parameter_value().string_value
        ).expanduser()

        if not log_directory.is_absolute():
            log_directory = Path.cwd() / log_directory

        try:

            log_directory.mkdir(
                parents=True,
                exist_ok=True
            )

            filename = (
                'pure_pursuit_'
                + datetime.now().strftime(
                    '%Y%m%d_%H%M%S_%f'
                )
                + '.csv'
            )

            self.csv_path = log_directory / filename

            self.csv_file = self.csv_path.open(
                mode='x',
                newline='',
                encoding='utf-8',
                buffering=1
            )

            self.csv_writer = csv.writer(
                self.csv_file
            )

            self.csv_writer.writerow([
                'time',
                'uav_x',
                'uav_y',
                'uav_z',
                'uav_vx',
                'uav_vy',
                'target_x',
                'target_y',
                'target_vx',
                'target_vy',
                'distance',
                'intercept_x',
                'intercept_y',
                't_go',
            ])

        except OSError as error:

            self.csv_file = None
            self.csv_writer = None
            self.csv_path = None

            self.get_logger().error(
                f'Failed to open CSV log: {error}'
            )

        self.experiment_start_time_ns = (
            self.get_clock().now().nanoseconds
        )

        self.get_logger().info(
            'Starting PURE PURSUIT'
        )

        if self.csv_path is not None:

            self.get_logger().info(
                f'CSV logging to: {self.csv_path}'
            )

    def elapsed_experiment_time(self):

        if self.experiment_start_time_ns is None:
            return None

        elapsed_ns = (
            self.get_clock().now().nanoseconds
            - self.experiment_start_time_ns
        )

        return elapsed_ns * 1e-9

    def record_experiment_data(self):

        if (
            self.csv_writer is None
            or self.experiment_start_time_ns is None
            or not self.target_received
            or not self.target_velocity_received
            or not self.uav_state_received
        ):
            return

        elapsed_time = self.elapsed_experiment_time()

        dx = self.target_x - self.uav_x
        dy = self.target_y - self.uav_y

        distance = math.sqrt(
            dx * dx
            + dy * dy
        )

        # Pure Pursuit 的控制目标就是目标当前位置。
        intercept_x = self.target_x
        intercept_y = self.target_y
        t_go = 0.0

        self.csv_writer.writerow([
            f'{elapsed_time:.6f}',
            f'{self.uav_x:.6f}',
            f'{self.uav_y:.6f}',
            f'{self.uav_z:.6f}',
            f'{self.uav_vx:.6f}',
            f'{self.uav_vy:.6f}',
            f'{self.target_x:.6f}',
            f'{self.target_y:.6f}',
            f'{self.target_vx:.6f}',
            f'{self.target_vy:.6f}',
            f'{distance:.6f}',
            f'{intercept_x:.6f}',
            f'{intercept_y:.6f}',
            f'{t_go:.6f}',
        ])

    def close_csv_log(self):

        if self.csv_file is None:
            return

        csv_path = self.csv_path

        try:
            self.csv_file.flush()
            self.csv_file.close()

        except OSError as error:

            message = f'Failed to close CSV log: {error}'

            if rclpy.ok():
                self.get_logger().error(message)
            else:
                print(message, flush=True)

        finally:

            self.csv_file = None
            self.csv_writer = None

        message = f'CSV log saved: {csv_path}'

        # Ctrl+C 时 ROS context 可能已关闭，此时不能再发布 rosout。
        if rclpy.ok():
            self.get_logger().info(message)
        else:
            print(message, flush=True)

    def destroy_node(self):

        self.close_csv_log()

        return super().destroy_node()

    # ============================================
    # 接收目标位置
    # ============================================

    def target_callback(self, msg):

        self.target_x = msg.x
        self.target_y = msg.y

        if not self.target_received:

            self.get_logger().info(
                f'Target received: '
                f'x={msg.x:.2f}, '
                f'y={msg.y:.2f}'
            )

            self.target_received = True

    # ============================================
    # 接收目标速度（只用于记录）
    # ============================================

    def target_velocity_callback(self, msg):

        self.target_vx = msg.x
        self.target_vy = msg.y

        if not self.target_velocity_received:

            self.get_logger().info(
                f'Target velocity received: '
                f'vx={msg.x:.2f}, '
                f'vy={msg.y:.2f}'
            )

        self.target_velocity_received = True

    # ============================================
    # 接收无人机状态
    # ============================================

    def vehicle_callback(self, msg):

        self.uav_x = msg.x
        self.uav_y = msg.y
        self.uav_z = msg.z

        self.uav_vx = msg.vx
        self.uav_vy = msg.vy
        self.uav_vz = msg.vz

        if not self.uav_state_received:

            self.get_logger().info(
                f'UAV state received: '
                f'x={msg.x:.2f}, '
                f'y={msg.y:.2f}, '
                f'z={msg.z:.2f}'
            )

            self.uav_state_received = True

    # ============================================
    # Offboard heartbeat
    # ============================================

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    # ============================================
    # 发布位置目标
    # ============================================

    def publish_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # 前4秒：原地起飞
        if self.counter < self.takeoff_counter:

            msg.position = [
                0.0,
                0.0,
                self.flight_altitude
            ]

        # 4秒以后：Pure Pursuit
        else:

            msg.position = [
                self.target_x,
                self.target_y,
                self.flight_altitude
            ]

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

        msg.yaw = math.nan

        self.setpoint_pub.publish(msg)

    # ============================================
    # 切换OFFBOARD
    # ============================================

    def offboard_mode(self):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = (
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        )

        msg.param1 = 1.0
        msg.param2 = 6.0

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # ============================================
    # 解锁
    # ============================================

    def arm(self):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = (
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        )

        msg.param1 = 1.0

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # ============================================
    # 计算目标距离
    # ============================================

    def check_interception(self):

        if not self.target_received:
            return

        if not self.uav_state_received:
            return

        dx = self.uav_x - self.target_x
        dy = self.uav_y - self.target_y

        distance = math.sqrt(
            dx * dx + dy * dy
        )

        # 每秒显示一次
        if self.counter % 20 == 0:

            self.get_logger().info(
                f'UAV=({self.uav_x:.2f}, '
                f'{self.uav_y:.2f})  '
                f'Target=({self.target_x:.2f}, '
                f'{self.target_y:.2f})  '
                f'Distance={distance:.2f} m'
            )

        # 截击成功
        if (
            distance < self.intercept_distance
            and not self.intercepted
            and self.experiment_start_time_ns is not None
        ):

            self.intercepted = True

            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                'INTERCEPTION SUCCESS!'
            )

            self.get_logger().info(
                f'Distance = {distance:.3f} m'
            )

            elapsed_time = self.elapsed_experiment_time()

            self.get_logger().info(
                f'Time = {elapsed_time:.2f} s'
            )

            self.get_logger().info(
                '================================'
            )

    # ============================================
    # 主循环
    # ============================================

    def timer_callback(self):

        self.publish_offboard_mode()

        self.publish_setpoint()

        self.counter += 1

        # 1秒后切OFFBOARD
        if self.counter == 20:

            self.get_logger().info(
                'Switching to OFFBOARD'
            )

            self.offboard_mode()

        # 2秒后解锁
        if self.counter == 40:

            self.get_logger().info(
                'Arming'
            )

            self.arm()

        # 4秒后开始追击
        if self.counter == self.takeoff_counter:

            if not self.target_received:

                self.get_logger().warn(
                    'No target received yet.'
                )

            self.start_experiment_logging()

        self.check_interception()

        # 只记录实验阶段；正常情况下频率与主循环一致，为 20 Hz。
        self.record_experiment_data()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = PurePursuit()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        if node is not None:
            node.destroy_node()

        # SIGINT 可能已经关闭了 rclpy context，避免重复 shutdown。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
