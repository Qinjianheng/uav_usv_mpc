"""ROS 2 node that filters camera-derived target observations."""

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from uav_usv_interfaces.msg import TargetObservation, TargetState

from .constant_velocity_kalman import ConstantVelocityKalmanFilter


def _stamp_nanoseconds(stamp):
    return (
        int(stamp.sec) * 1_000_000_000
        + int(stamp.nanosec)
    )


def measurement_from_observation(
    message,
    expected_frame_id,
):
    """Return position, covariance, and observation time."""
    if not bool(message.valid):
        raise ValueError('target observation is invalid')
    if str(message.frame_id) != str(expected_frame_id):
        raise ValueError('target observation frame mismatch')

    measurement = np.asarray(
        (
            message.position.x,
            message.position.y,
            message.position.z,
        ),
        dtype=float,
    )
    if not np.all(np.isfinite(measurement)):
        raise ValueError('target observation position is not finite')

    covariance = np.asarray(
        message.covariance,
        dtype=float,
    )
    if covariance.size != 9:
        raise ValueError(
            'target observation covariance must have nine values'
        )
    covariance = covariance.reshape((3, 3))
    if not np.all(np.isfinite(covariance)):
        raise ValueError(
            'target observation covariance is not finite'
        )

    timestamp_ns = _stamp_nanoseconds(message.stamp)
    if timestamp_ns <= 0:
        raise ValueError(
            'target observation timestamp must be positive'
        )

    return measurement, covariance, timestamp_ns


class TargetKalmanFilterNode(Node):
    """Filter RGB-D target observations without entering control."""

    def __init__(self):
        super().__init__('target_kalman_filter')

        self.declare_parameter('frame_id', 'local_ned')
        self.declare_parameter(
            'input_topic',
            '/perception/front/target_observation',
        )
        self.declare_parameter(
            'state_topic',
            '/tracking/target_state',
        )
        self.declare_parameter(
            'prediction_topic',
            '/tracking/predicted_position',
        )
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter(
            'process_acceleration_std',
            0.5,
        )
        self.declare_parameter(
            'measurement_position_std',
            0.02,
        )
        self.declare_parameter(
            'initial_velocity_std',
            5.0,
        )
        self.declare_parameter(
            'prediction_horizon',
            0.5,
        )
        self.declare_parameter(
            'measurement_timeout',
            0.5,
        )

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )
        self.prediction_horizon = max(
            float(
                self.get_parameter(
                    'prediction_horizon'
                ).value
            ),
            0.0,
        )
        self.measurement_timeout = max(
            float(
                self.get_parameter(
                    'measurement_timeout'
                ).value
            ),
            0.01,
        )
        update_rate_hz = max(
            float(
                self.get_parameter(
                    'update_rate_hz'
                ).value
            ),
            1.0,
        )

        self.filter = ConstantVelocityKalmanFilter(
            process_acceleration_std=(
                self.get_parameter(
                    'process_acceleration_std'
                ).value
            ),
            measurement_position_std=(
                self.get_parameter(
                    'measurement_position_std'
                ).value
            ),
            initial_velocity_std=(
                self.get_parameter(
                    'initial_velocity_std'
                ).value
            ),
        )

        self.filter_time_ns = None
        self.last_measurement_time_ns = None

        input_topic = str(
            self.get_parameter('input_topic').value
        )
        state_topic = str(
            self.get_parameter('state_topic').value
        )
        prediction_topic = str(
            self.get_parameter('prediction_topic').value
        )

        self.measurement_sub = self.create_subscription(
            TargetObservation,
            input_topic,
            self.measurement_callback,
            10,
        )
        self.state_pub = self.create_publisher(
            TargetState,
            state_topic,
            10,
        )
        self.prediction_pub = self.create_publisher(
            PointStamped,
            prediction_topic,
            10,
        )
        self.timer = self.create_timer(
            1.0 / update_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            'Target Kalman filter ready | '
            f'input={input_topic} | '
            f'state={state_topic} | '
            f'horizon={self.prediction_horizon:.2f} s'
        )

    def measurement_callback(self, message):
        if (
            not message.valid
            or message.frame_id != self.frame_id
        ):
            return

        try:
            measurement, covariance, timestamp_ns = (
                measurement_from_observation(
                    message,
                    self.frame_id,
                )
            )
        except ValueError:
            return

        if (
            self.last_measurement_time_ns is not None
            and timestamp_ns
            <= self.last_measurement_time_ns
        ):
            return

        try:
            if not self.filter.initialized:
                self.filter.initialize(
                    measurement,
                    covariance,
                )
                self.filter_time_ns = timestamp_ns
            else:
                self.predict_to(timestamp_ns)
                self.filter.update(
                    measurement,
                    covariance,
                )
        except ValueError:
            return

        self.last_measurement_time_ns = timestamp_ns
        # A new image must not wait another publication period before control
        # sees it. Projection keeps the original source stamp and KF state.
        self.timer_callback()

    def predict_to(self, timestamp_ns):
        if (
            not self.filter.initialized
            or self.filter_time_ns is None
        ):
            return

        dt = max(
            (
                int(timestamp_ns)
                - int(self.filter_time_ns)
            ) * 1e-9,
            0.0,
        )
        if dt > 0.0:
            self.filter.predict(dt)
            self.filter_time_ns = int(timestamp_ns)

    def timer_callback(self):
        if (
            not self.filter.initialized
            or self.filter_time_ns is None
            or self.last_measurement_time_ns is None
        ):
            return

        now = self.get_clock().now()

        measurement_age = (
            now.nanoseconds
            - self.last_measurement_time_ns
        ) * 1e-9
        valid = (
            0.0 <= measurement_age
            <= self.measurement_timeout
        )

        projection_dt = max(
            (
                now.nanoseconds
                - self.filter_time_ns
            ) * 1e-9,
            0.0,
        )
        projected_state, projected_covariance = (
            self.filter.project(projection_dt)
        )

        state_message = TargetState()
        state_message.stamp = now.to_msg()
        state_message.source_stamp = Time(
            nanoseconds=self.last_measurement_time_ns
        ).to_msg()
        state_message.frame_id = self.frame_id

        state_message.position.x = float(
            projected_state[0]
        )
        state_message.position.y = float(
            projected_state[1]
        )
        state_message.position.z = float(
            projected_state[2]
        )
        state_message.velocity.x = float(
            projected_state[3]
        )
        state_message.velocity.y = float(
            projected_state[4]
        )
        state_message.velocity.z = float(
            projected_state[5]
        )

        state_message.acceleration.x = 0.0
        state_message.acceleration.y = 0.0
        state_message.acceleration.z = 0.0

        state_message.covariance = [
            float(value)
            for value in projected_covariance.reshape(-1)
        ]
        state_message.valid = valid
        self.state_pub.publish(state_message)

        future_state, _ = self.filter.project(
            projection_dt + self.prediction_horizon
        )
        predicted_position = future_state[:3]

        prediction_message = PointStamped()
        prediction_time_ns = (
            now.nanoseconds
            + round(
                self.prediction_horizon * 1e9
            )
        )
        prediction_message.header.stamp = Time(
            nanoseconds=prediction_time_ns
        ).to_msg()
        prediction_message.header.frame_id = self.frame_id
        prediction_message.point.x = float(
            predicted_position[0]
        )
        prediction_message.point.y = float(
            predicted_position[1]
        )
        prediction_message.point.z = float(
            predicted_position[2]
        )
        self.prediction_pub.publish(
            prediction_message
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetKalmanFilterNode()

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
