"""ROS 2 wrapper for bounded target prediction."""

import math
import json
import os
from contextlib import nullcontext
from uav_control.guidance.follow_profile import CycleProfile, stage
import time

import rclpy
from builtin_interfaces.msg import Duration, Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from uav_usv_interfaces.msg import MissionState, PredictedTargetPoint
from uav_usv_interfaces.msg import TargetPrediction, TargetState

from .maneuvering_target_predictor import ManeuveringTargetPredictor
from .target_prediction import PredictionEngine, TargetKinematicState


def _split_nanoseconds(seconds):
    nanoseconds = round(float(seconds) * 1e9)
    return nanoseconds // 1_000_000_000, nanoseconds % 1_000_000_000


def seconds_to_time(seconds):
    """Convert floating-point seconds to a normalized ROS time message."""
    seconds_part, nanoseconds_part = _split_nanoseconds(seconds)
    message = Time()
    message.sec = int(seconds_part)
    message.nanosec = int(nanoseconds_part)
    return message


def seconds_to_duration(seconds):
    """Convert floating-point seconds to a normalized ROS duration."""
    seconds_part, nanoseconds_part = _split_nanoseconds(seconds)
    message = Duration()
    message.sec = int(seconds_part)
    message.nanosec = int(nanoseconds_part)
    return message


def state_from_message(message):
    """Keep the KF evaluation epoch and its image acquisition time."""
    stamp = message.stamp.sec + message.stamp.nanosec * 1e-9
    return TargetKinematicState(
        stamp=stamp,
        observation_stamp=(
            message.source_stamp.sec + message.source_stamp.nanosec * 1e-9
        ),
        position=(
            float(message.position.x),
            float(message.position.y),
            float(message.position.z),
        ),
        velocity=(
            float(message.velocity.x),
            float(message.velocity.y),
            float(message.velocity.z),
        ),
    )


def prediction_to_message(result, compute_time, frame_id='local_ned'):
    """Convert a transport-neutral prediction to its ROS interface."""
    message = TargetPrediction()
    message.mission_id = result.mission_id
    message.sequence_id = result.sequence_id
    message.source_stamp = seconds_to_time(result.source_stamp)
    message.observation_stamp = seconds_to_time(result.observation_stamp)
    message.generated_stamp = seconds_to_time(result.generated_stamp)
    message.valid_until = seconds_to_time(result.valid_until)
    message.frame_id = str(frame_id)
    message.source = result.source
    message.model = result.model
    message.prediction_horizon = result.horizon
    message.turn_rate = result.turn_rate
    message.turn_acceleration = result.turn_acceleration
    message.longitudinal_acceleration = result.longitudinal_acceleration
    message.compute_time = float(compute_time)
    message.valid = result.valid
    message.invalid_reason = result.invalid_reason

    samples = []
    for sample in result.samples:
        sample_message = PredictedTargetPoint()
        sample_message.relative_time = seconds_to_duration(
            sample.relative_time
        )
        sample_message.position.x = sample.position[0]
        sample_message.position.y = sample.position[1]
        sample_message.position.z = sample.position[2]
        sample_message.velocity.x = sample.velocity[0]
        sample_message.velocity.y = sample.velocity[1]
        sample_message.velocity.z = sample.velocity[2]
        sample_message.acceleration.x = sample.acceleration[0]
        sample_message.acceleration.y = sample.acceleration[1]
        sample_message.acceleration.z = sample.acceleration[2]
        position_std = 0.02 + 0.10 * sample.relative_time
        variance = position_std * position_std
        sample_message.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        samples.append(sample_message)
    message.samples = samples
    return message


class TargetPredictorNode(Node):
    """Publish latest-input-only BCTRA predictions at 20 Hz."""

    def __init__(self):
        super().__init__('target_predictor_node')
        self.declare_parameter('p43_profile_enabled',
                               os.environ.get('UAV_USV_PROFILE_PREDICTOR') == '1')
        self.p43_profile_enabled = self.get_parameter('p43_profile_enabled').value
        self.declare_parameter('target_state_source', 'tracking')
        self.declare_parameter('tracking_topic', '/tracking/target_state')
        self.declare_parameter(
            'prediction_topic',
            '/planning/target_prediction',
        )
        self.declare_parameter('frame_id', 'local_ned')
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('prediction_horizon', 4.0)
        self.declare_parameter('prediction_sample_period', 0.1)
        self.declare_parameter('input_timeout', 0.125)
        self.declare_parameter('turn_rate_filter_alpha', 0.25)
        self.declare_parameter('max_target_turn_rate', 0.7)
        self.declare_parameter('maneuver_prediction_horizon', 2.0)
        self.declare_parameter('minimum_target_speed', 0.2)
        self.declare_parameter('minimum_turn_updates', 3)
        self.declare_parameter('turn_acceleration_filter_alpha', 0.2)
        self.declare_parameter('max_target_turn_acceleration', 1.5)
        self.declare_parameter('speed_acceleration_filter_alpha', 0.2)
        self.declare_parameter('max_target_longitudinal_acceleration', 2.0)
        self.declare_parameter('maneuver_acceleration_decay_time', 1.0)
        self.declare_parameter('vertical_velocity_decay_time', 0.75)
        self.declare_parameter('maximum_vertical_displacement', 0.5)
        self.declare_parameter('vertical_observation_margin', 0.25)
        self.declare_parameter('vertical_history_window', 1.0)

        source = str(
            self.get_parameter('target_state_source').value
        ).strip().lower()
        if source != 'tracking':
            raise ValueError('target_state_source must be tracking')
        input_topic = str(self.get_parameter('tracking_topic').value).strip()
        resolved_topic = self.resolve_topic_name(input_topic)
        forbidden_topics = {
            '/target/state', '/target/position', '/target/velocity',
        }
        if (
            '/' + input_topic.strip('/') in forbidden_topics
            or resolved_topic in forbidden_topics
        ):
            raise ValueError(
                'tracking_topic cannot be a simulation truth topic'
            )
        output_topic = str(self.get_parameter('prediction_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        if not math.isfinite(update_rate_hz) or update_rate_hz <= 0.0:
            raise ValueError('update_rate_hz must be finite and positive')

        predictor = ManeuveringTargetPredictor(
            turn_rate_filter_alpha=self.get_parameter(
                'turn_rate_filter_alpha'
            ).value,
            max_turn_rate=self.get_parameter(
                'max_target_turn_rate'
            ).value,
            maneuver_horizon=self.get_parameter(
                'maneuver_prediction_horizon'
            ).value,
            minimum_speed=self.get_parameter(
                'minimum_target_speed'
            ).value,
            minimum_updates=self.get_parameter(
                'minimum_turn_updates'
            ).value,
            turn_acceleration_filter_alpha=self.get_parameter(
                'turn_acceleration_filter_alpha'
            ).value,
            max_turn_acceleration=self.get_parameter(
                'max_target_turn_acceleration'
            ).value,
            speed_acceleration_filter_alpha=self.get_parameter(
                'speed_acceleration_filter_alpha'
            ).value,
            max_longitudinal_acceleration=self.get_parameter(
                'max_target_longitudinal_acceleration'
            ).value,
            acceleration_decay_time=self.get_parameter(
                'maneuver_acceleration_decay_time'
            ).value,
            vertical_velocity_decay_time=self.get_parameter(
                'vertical_velocity_decay_time'
            ).value,
            maximum_vertical_displacement=self.get_parameter(
                'maximum_vertical_displacement'
            ).value,
            vertical_observation_margin=self.get_parameter(
                'vertical_observation_margin'
            ).value,
            vertical_history_window=self.get_parameter(
                'vertical_history_window'
            ).value,
        )
        self.engine = PredictionEngine(
            predictor=predictor,
            horizon=self.get_parameter('prediction_horizon').value,
            sample_period=self.get_parameter(
                'prediction_sample_period'
            ).value,
            input_timeout=self.get_parameter('input_timeout').value,
            source=source,
        )
        self.mission_id = 0
        self.sequence_id = 0
        self.last_prediction_observation_stamp = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.state_sub = self.create_subscription(
            TargetState,
            input_topic,
            self.state_callback,
            sensor_qos,
        )
        self.mission_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_callback,
            1,
        )
        self.prediction_pub = self.create_publisher(
            TargetPrediction,
            output_topic,
            sensor_qos,
        )
        self.timer = self.create_timer(
            1.0 / update_rate_hz,
            self.timer_callback,
        )
        self.get_logger().info(
            f'Target predictor ready | source={source} | input={input_topic} '
            f'| output={output_topic} | rate={update_rate_hz:.1f} Hz'
        )

    def state_callback(self, message):
        if not message.valid:
            self.engine.invalidate('INVALID_TARGET_STATE')
            return
        if message.frame_id != self.frame_id:
            self.engine.invalidate('FRAME_MISMATCH')
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.engine.update(state_from_message(message), now=now):
            self.timer_callback()

    def mission_callback(self, message):
        self.mission_id = int(message.mission_id)

    def timer_callback(self):
        start = time.perf_counter()
        now = self.get_clock().now().nanoseconds * 1e-9
        state = self.engine.latest_state
        if (
            state is not None
            and state.observation_stamp == self.last_prediction_observation_stamp
            and state.stamp <= now
            and 0.0 <= now - state.observation_stamp <= self.engine.input_timeout
        ):
            # A KF projection or heartbeat is not another image. Publish
            # fresh observations immediately, once each; timers still report
            # invalid/stale input without resetting acquisition deadlines.
            return
        self.sequence_id += 1
        profiling = bool(getattr(self, 'p43_profile_enabled', False))
        profile = CycleProfile(self.sequence_id) if profiling else nullcontext()
        with profile:
            result = self.engine.generate(now=now, mission_id=self.mission_id,
                                          sequence_id=self.sequence_id)
            with stage('prediction_message_conversion'):
                message = prediction_to_message(result, compute_time=0., frame_id=self.frame_id)
            message.compute_time = time.perf_counter()-start
            with stage('prediction_publication'):
                self.prediction_pub.publish(message)
        if profiling:
            report = dict(profile.report(), producer_process='target_predictor_node',
                          sequence=self.sequence_id, source_stamp=result.source_stamp)
            self.get_logger().info('P43_PREDICTOR_PROFILE '+json.dumps(report))
        if result.valid:
            self.last_prediction_observation_stamp = result.observation_stamp


def main(args=None):
    rclpy.init(args=args)
    node = TargetPredictorNode()
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
