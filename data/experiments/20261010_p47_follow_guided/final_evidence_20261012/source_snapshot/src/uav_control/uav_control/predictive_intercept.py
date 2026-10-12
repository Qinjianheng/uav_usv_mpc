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


class PredictiveIntercept(Node):

    def __init__(self):
        raise RuntimeError(
            'Retired truth-control entry point; use modular_intercept.launch.py'
        )
        super().__init__('predictive_intercept')

        # ============================================================
        # PX4 Publishers
        # ============================================================

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

        # ============================================================
        # Target Subscribers
        # ============================================================

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

        # ============================================================
        # PX4 QoS
        # ============================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 你的 PX4 v1.18 当前实际话题带 _v1
        self.vehicle_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_position_callback,
            px4_qos
        )

        # ============================================================
        # Target State
        # ============================================================

        self.target_x = 0.0
        self.target_y = 0.0

        self.target_vx = 0.0
        self.target_vy = 0.0

        self.target_position_received = False
        self.target_velocity_received = False

        # ============================================================
        # UAV State
        # ============================================================

        self.uav_x = 0.0
        self.uav_y = 0.0
        self.uav_z = 0.0

        self.uav_vx = 0.0
        self.uav_vy = 0.0
        self.uav_vz = 0.0

        self.uav_state_received = False

        # ============================================================
        # Interception Parameters
        # ============================================================

        # 用于计算预测截击时间
        # 注意：
        # 这是算法假设的无人机水平截击速度，
        # 不是直接发送给 PX4 的速度命令。
        self.interceptor_speed = 5.0

        # 防止预测点跑得太远
        self.max_prediction_time = 8.0

        # 飞行高度
        # PX4 NED 坐标系：
        # z < 0 表示向上
        self.flight_altitude = -5.0

        # 截击成功判定距离
        self.intercept_distance = 0.5

        self.intercepted = False

        # ============================================================
        # Experiment Logging
        # ============================================================

        # 相对路径以启动 ros2 run 时的当前目录为基准。
        self.declare_parameter(
            'log_directory',
            'experiment_logs'
        )

        self.experiment_start_time_ns = None
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

        # ============================================================
        # Timing
        # ============================================================

        # 20 Hz
        self.dt = 0.05

        self.counter = 0

        # 80 * 0.05 = 4 秒
        self.takeoff_counter = 80

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        self.get_logger().info(
            'Predictive interception node started'
        )

    # ================================================================
    # Timestamp
    # ================================================================

    def timestamp(self):

        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    # ================================================================
    # Experiment Logging
    # ================================================================

    def start_experiment_logging(self):

        if self.experiment_start_time_ns is not None:
            return

        # 先准备日志文件，随后用同一时刻作为实验时间零点。
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
                'predictive_intercept_'
                + datetime.now().strftime(
                    '%Y%m%d_%H%M%S_%f'
                )
                + '.csv'
            )

            self.csv_path = log_directory / filename

            # 行缓冲让每一行尽快落盘，异常退出时也尽量保留数据。
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
            'Starting PREDICTIVE INTERCEPTION'
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
            or not self.target_position_received
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

        intercept_x, intercept_y, t_go = (
            self.calculate_intercept_point()
        )

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

    # ================================================================
    # Target Position Callback
    # ================================================================

    def target_position_callback(self, msg):

        self.target_x = msg.x
        self.target_y = msg.y

        if not self.target_position_received:

            self.get_logger().info(
                f'Target position received: '
                f'x={msg.x:.2f}, y={msg.y:.2f}'
            )

        self.target_position_received = True

    # ================================================================
    # Target Velocity Callback
    # ================================================================

    def target_velocity_callback(self, msg):

        self.target_vx = msg.x
        self.target_vy = msg.y

        if not self.target_velocity_received:

            self.get_logger().info(
                f'Target velocity received: '
                f'vx={msg.x:.2f}, vy={msg.y:.2f}'
            )

        self.target_velocity_received = True

    # ================================================================
    # UAV Position Callback
    # ================================================================

    def vehicle_position_callback(self, msg):

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

    # ================================================================
    # Calculate Intercept Time
    # ================================================================

    def calculate_intercept_time(self):

        if (
            not self.target_position_received
            or not self.target_velocity_received
            or not self.uav_state_received
        ):
            return 0.0

        # 相对位置：
        # r = target - UAV
        rx = self.target_x - self.uav_x
        ry = self.target_y - self.uav_y

        # 目标速度
        vx = self.target_vx
        vy = self.target_vy

        # 假设无人机截击速度
        vu = self.interceptor_speed

        # ------------------------------------------------------------
        # 解：
        #
        # |r + vt*T| = Vu*T
        #
        # 得到：
        #
        # a*T^2 + b*T + c = 0
        # ------------------------------------------------------------

        a = (
            vx * vx
            + vy * vy
            - vu * vu
        )

        b = 2.0 * (
            rx * vx
            + ry * vy
        )

        c = (
            rx * rx
            + ry * ry
        )

        # 已经非常靠近
        if c < 1e-6:
            return 0.0

        # ------------------------------------------------------------
        # 接近一次方程
        # ------------------------------------------------------------

        if abs(a) < 1e-6:

            if abs(b) < 1e-6:

                distance = math.sqrt(c)

                return min(
                    distance / max(vu, 0.1),
                    self.max_prediction_time
                )

            t = -c / b

            if t > 0.0:

                return min(
                    t,
                    self.max_prediction_time
                )

            distance = math.sqrt(c)

            return min(
                distance / max(vu, 0.1),
                self.max_prediction_time
            )

        # ------------------------------------------------------------
        # 二次方程
        # ------------------------------------------------------------

        discriminant = (
            b * b
            - 4.0 * a * c
        )

        # 没有实根
        if discriminant < 0.0:

            distance = math.sqrt(c)

            return min(
                distance / max(vu, 0.1),
                self.max_prediction_time
            )

        sqrt_d = math.sqrt(discriminant)

        t1 = (
            -b + sqrt_d
        ) / (2.0 * a)

        t2 = (
            -b - sqrt_d
        ) / (2.0 * a)

        positive_times = []

        if t1 > 0.0:
            positive_times.append(t1)

        if t2 > 0.0:
            positive_times.append(t2)

        if len(positive_times) == 0:

            distance = math.sqrt(c)

            return min(
                distance / max(vu, 0.1),
                self.max_prediction_time
            )

        intercept_time = min(positive_times)

        return min(
            intercept_time,
            self.max_prediction_time
        )

    # ================================================================
    # Calculate Intercept Point
    # ================================================================

    def calculate_intercept_point(self):

        t_go = self.calculate_intercept_time()

        intercept_x = (
            self.target_x
            + self.target_vx * t_go
        )

        intercept_y = (
            self.target_y
            + self.target_vy * t_go
        )

        return (
            intercept_x,
            intercept_y,
            t_go
        )

    # ================================================================
    # Publish Offboard Control Mode
    # ================================================================

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        # 主控制仍然是位置控制
        msg.position = True

        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    # ================================================================
    # Publish Trajectory Setpoint
    # ================================================================

    def publish_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # ------------------------------------------------------------
        # 起飞阶段
        # ------------------------------------------------------------

        if self.counter < self.takeoff_counter:

            msg.position = [
                0.0,
                0.0,
                self.flight_altitude
            ]

            msg.velocity = [
                math.nan,
                math.nan,
                math.nan
            ]

        # ------------------------------------------------------------
        # Predictive Interception
        # ------------------------------------------------------------

        else:

            if (
                self.target_position_received
                and self.target_velocity_received
                and self.uav_state_received
            ):

                intercept_x, intercept_y, _ = (
                    self.calculate_intercept_point()
                )

                # ====================================================
                # 位置：
                # 飞向预测截击点
                # ====================================================

                msg.position = [
                    intercept_x,
                    intercept_y,
                    self.flight_altitude
                ]

                # ====================================================
                # 速度前馈：
                #
                # 非常关键
                #
                # 告诉 PX4：
                # 这个位置目标本身还在以 USV 的速度运动
                # ====================================================

                msg.velocity = [
                    self.target_vx,
                    self.target_vy,
                    0.0
                ]

            else:

                # 目标信息没有准备好时保持原地
                msg.position = [
                    0.0,
                    0.0,
                    self.flight_altitude
                ]

                msg.velocity = [
                    0.0,
                    0.0,
                    0.0
                ]

        # 当前不使用加速度前馈
        msg.acceleration = [
            math.nan,
            math.nan,
            math.nan
        ]

        # 当前不控制 jerk
        msg.jerk = [
            math.nan,
            math.nan,
            math.nan
        ]

        # 不限定机头方向
        msg.yaw = math.nan

        # 不限定偏航角速度
        msg.yawspeed = math.nan

        self.setpoint_pub.publish(msg)

    # ================================================================
    # Switch to OFFBOARD
    # ================================================================

    def offboard_mode(self):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = (
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        )

        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param1 = 1.0

        # PX4 OFFBOARD
        msg.param2 = 6.0

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # ================================================================
    # ARM
    # ================================================================

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

    # ================================================================
    # Check Interception
    # ================================================================

    def check_interception(self):

        if (
            not self.target_position_received
            or not self.uav_state_received
        ):
            return

        dx = (
            self.target_x
            - self.uav_x
        )

        dy = (
            self.target_y
            - self.uav_y
        )

        distance = math.sqrt(
            dx * dx
            + dy * dy
        )

        # 每秒输出一次
        if self.counter % 20 == 0:

            intercept_x, intercept_y, t_go = (
                self.calculate_intercept_point()
            )

            uav_speed = math.sqrt(
                self.uav_vx * self.uav_vx
                + self.uav_vy * self.uav_vy
            )

            self.get_logger().info(
                f'Distance={distance:.2f} m | '
                f'T_go={t_go:.2f} s | '
                f'UAV_speed={uav_speed:.2f} m/s | '
                f'Target=({self.target_x:.2f}, '
                f'{self.target_y:.2f}) | '
                f'Intercept=({intercept_x:.2f}, '
                f'{intercept_y:.2f})'
            )

        # ------------------------------------------------------------
        # 第一次进入截击范围
        # ------------------------------------------------------------

        if (
            distance < self.intercept_distance
            and not self.intercepted
            and self.experiment_start_time_ns is not None
        ):

            self.intercepted = True

            elapsed_time = self.elapsed_experiment_time()

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'INTERCEPTION SUCCESS!'
            )

            self.get_logger().info(
                f'Distance = {distance:.3f} m'
            )

            self.get_logger().info(
                f'Time = {elapsed_time:.2f} s'
            )

            self.get_logger().info(
                '========================================'
            )

    # ================================================================
    # Main Timer
    # ================================================================

    def timer_callback(self):

        # Offboard heartbeat 必须持续发送
        self.publish_offboard_mode()

        # 持续发送 TrajectorySetpoint
        self.publish_setpoint()

        self.counter += 1

        # ------------------------------------------------------------
        # 1 秒后切入 OFFBOARD
        # ------------------------------------------------------------

        if self.counter == 20:

            self.get_logger().info(
                'Switching to OFFBOARD'
            )

            self.offboard_mode()

        # ------------------------------------------------------------
        # 2 秒后 ARM
        # ------------------------------------------------------------

        if self.counter == 40:

            self.get_logger().info(
                'Arming'
            )

            self.arm()

        # ------------------------------------------------------------
        # 4 秒后开始预测截击
        # ------------------------------------------------------------

        if self.counter == self.takeoff_counter:

            self.start_experiment_logging()

        # ------------------------------------------------------------
        # 截击距离判断
        # ------------------------------------------------------------

        self.check_interception()

        # 只记录实验阶段；正常情况下频率与主循环一致，为 20 Hz。
        self.record_experiment_data()


# ====================================================================
# Main
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = PredictiveIntercept()

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
