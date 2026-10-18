"""ROS raw-observation IMM wrapper with acquisition-bounded prediction leases."""

from dataclasses import dataclass
import json
import math
import time

from builtin_interfaces.msg import Duration, Time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from uav_usv_interfaces.msg import MissionState, PredictedTargetPoint
from uav_usv_interfaces.msg import TargetObservation, TargetPrediction
from uav_control.common.navigation_frame import install_navigation_frame_guard

from .imm_predictor import ImmConfig, ImmPredictor, MODEL_NAMES, UpdateStatus
from .target_prediction import PredictionResult


@dataclass(frozen=True)
class RawObservation:
    stamp: float
    position: tuple
    covariance: object
    frame_id: str
    source: str
    valid: bool
    rejection_reason: str
    clock_identity: tuple
    received_stamp: float
    image_measurement_stamp: float


@dataclass(frozen=True)
class ImmSample:
    relative_time: float
    position: tuple
    velocity: tuple
    acceleration: tuple
    position_covariance: tuple


@dataclass(frozen=True)
class ImmPredictionResult(PredictionResult):
    forecast: object = None
    observation_source: str = ''


def _stamp_seconds(stamp):
    return int(stamp.sec) + int(stamp.nanosec) * 1e-9


def _fractional_nanoseconds(seconds):
    """Round the fractional second without multiplying a large absolute epoch."""
    value = float(seconds)
    if not math.isfinite(value):
        raise ValueError('INVALID_IMM_TIME')
    second = math.floor(value)
    carry, nanosecond = divmod(round((value - second) * 1e9), 1_000_000_000)
    return second + carry, nanosecond


def _seconds_to_time(seconds):
    second, nanosecond = _fractional_nanoseconds(seconds)
    return Time(sec=second, nanosec=nanosecond)


def _seconds_to_duration(seconds):
    second, nanosecond = _fractional_nanoseconds(seconds)
    return Duration(sec=second, nanosec=nanosecond)


def observation_from_message(message):
    """Read the original image epoch; never substitute receipt/publication time."""
    return RawObservation(
        stamp=_stamp_seconds(message.stamp),
        position=(float(message.position.x), float(message.position.y),
                  float(message.position.z)),
        covariance=tuple(message.covariance),
        frame_id=str(message.frame_id), source=str(message.source),
        valid=bool(message.valid), rejection_reason=str(message.rejection_reason),
        clock_identity=(int(message.image_clock_reset_count),
                        int(message.px4_clock_reset_count)),
        received_stamp=_stamp_seconds(message.received_stamp),
        image_measurement_stamp=float(message.image_measurement_stamp))


class ImmPredictionEngine:
    """Transport-neutral time gate around the independent raw-position IMM."""

    def __init__(self, predictor=None, horizon=4., sample_period=.1,
                 input_timeout=.125, frame_id='local_ned', target_time_profile='baseline_125'):
        self.predictor = predictor or ImmPredictor()
        for name, value in (('horizon', horizon), ('sample_period', sample_period),
                            ('input_timeout', input_timeout)):
            value = float(value)
            if not math.isfinite(value) or value <= 0.:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)
        from uav_control.guidance.follow_time_contract import input_time_contract
        self.time_contract = input_time_contract(target_time_profile)
        if self.input_timeout != self.time_contract.observation_age:
            raise ValueError('125ms baseline input_timeout requires an explicit matching '
                             'target_time_profile for experiments')
        self.frame_id = str(frame_id)
        self.last_clock = None
        self.clock_identity = None
        self.latest_observation = None
        self.latest_input = None
        self.invalid_reason = 'NO_OBSERVATION'
        self.last_rejection_reason = ''
        count = int(math.floor(self.horizon / self.sample_period))
        self.horizons = np.arange(count + 1) * self.sample_period
        if self.horizon - self.horizons[-1] > 1e-9:
            self.horizons = np.append(self.horizons, self.horizon)
        else:
            self.horizons[-1] = self.horizon

    def reset(self, reason='MISSION_RESET'):
        self.predictor.reset()
        self.latest_observation = None
        self.latest_input = None
        self.clock_identity = None
        self.last_clock = None
        self.invalid_reason = str(reason)
        self.last_rejection_reason = str(reason)

    def invalidate(self, reason):
        self.invalid_reason = str(reason)
        self.last_rejection_reason = str(reason)

    def _observe_clock(self, now):
        if not math.isfinite(now):
            self.invalidate('INVALID_CLOCK')
            return False
        if self.last_clock is not None and now < self.last_clock - 1e-9:
            self.reset('CLOCK_RESET')
            self.last_clock = now
            return False
        self.last_clock = now
        return True

    def update(self, observation, now):
        """One unique acquisition updates the EKFs; old projections never do."""
        now = float(now)
        self.latest_input = observation
        if not self._observe_clock(now):
            return UpdateStatus(False, self.invalid_reason)
        if self.clock_identity is not None and observation.clock_identity != self.clock_identity:
            identity = observation.clock_identity
            self.reset('CLOCK_RESET')
            self.clock_identity = identity
            self.last_clock = now
            self.latest_input = observation
            return UpdateStatus(False, 'CLOCK_RESET')
        self.clock_identity = observation.clock_identity
        reason = ''
        if not observation.valid:
            reason = 'INVALID_OBSERVATION'
        elif observation.frame_id != self.frame_id:
            reason = 'FRAME_MISMATCH'
        elif any(token in observation.source.lower() for token in ('truth', 'ground_truth')):
            reason = 'FORBIDDEN_OBSERVATION_SOURCE'
        elif not math.isfinite(observation.stamp) or observation.stamp <= 0.:
            reason = 'INVALID_OBSERVATION'
        elif (observation.image_measurement_stamp != 0.
              and (not math.isfinite(observation.image_measurement_stamp)
                   or abs(observation.image_measurement_stamp - observation.stamp) > 1e-8)):
            reason = 'ACQUISITION_STAMP_MISMATCH'
        elif observation.stamp > now + 1e-9:
            reason = 'OBSERVATION_TIME_IN_FUTURE'
        elif now - observation.stamp > self.input_timeout + 1e-12:
            reason = 'OBSERVATION_STALE'
        if reason:
            self.invalidate(reason)
            return UpdateStatus(False, reason)
        status = self.predictor.update(observation.stamp, observation.position,
                                       observation.covariance)
        if status.accepted:
            self.latest_observation = observation
            self.invalid_reason = ''
            self.last_rejection_reason = ''
        elif status.reason in ('DUPLICATE_OBSERVATION', 'OUT_OF_ORDER_OBSERVATION'):
            # They are not new information, but cannot invalidate an already
            # accepted newer observation. Rejection remains in diagnostics.
            self.last_rejection_reason = status.reason
        else:
            self.invalidate(status.reason)
        return status

    def _invalid(self, now, mission_id, sequence_id, reason):
        observation_stamp = self.predictor.stamp or 0.
        source = self.latest_observation.source if self.latest_observation else ''
        return ImmPredictionResult(
            mission_id=int(mission_id), sequence_id=int(sequence_id),
            source_stamp=observation_stamp, observation_stamp=observation_stamp,
            generated_stamp=now, valid_until=observation_stamp + self.input_timeout,
            source='tracking', model='IMM_CV_CTRV_CTRA', horizon=self.horizon,
            samples=(), turn_rate=0., turn_acceleration=0.,
            longitudinal_acceleration=0., valid=False, invalid_reason=str(reason),
            observation_source=source)

    def generate(self, now, mission_id, sequence_id):
        """Explicitly propagate to now while retaining the original expiry."""
        now = float(now)
        if not self._observe_clock(now):
            return self._invalid(now, mission_id, sequence_id, self.invalid_reason)
        if self.invalid_reason or self.predictor.stamp is None:
            return self._invalid(now, mission_id, sequence_id,
                                 self.invalid_reason or 'NO_OBSERVATION')
        age = now - self.predictor.stamp
        if age < 0.:
            return self._invalid(now, mission_id, sequence_id, 'OBSERVATION_TIME_IN_FUTURE')
        if age > self.input_timeout + 1e-12:
            return self._invalid(now, mission_id, sequence_id, 'OBSERVATION_STALE')
        try:
            forecast = self.predictor.forecast_at(now, self.horizons)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            self.invalidate('IMM_NUMERICAL_FAILURE')
            return self._invalid(now, mission_id, sequence_id, self.invalid_reason)
        samples = tuple(ImmSample(float(t), tuple(position), tuple(velocity),
                                  tuple(acceleration), tuple(covariance.ravel()))
                        for t, position, velocity, acceleration, covariance in zip(
                            forecast.horizons, forecast.position, forecast.velocity,
                            forecast.acceleration, forecast.position_covariance))
        return ImmPredictionResult(
            mission_id=int(mission_id), sequence_id=int(sequence_id),
            source_stamp=now, observation_stamp=forecast.observation_stamp,
            generated_stamp=now, valid_until=forecast.observation_stamp + self.input_timeout,
            source='tracking', model='IMM_CV_CTRV_CTRA', horizon=self.horizon,
            samples=samples, turn_rate=float(forecast.state[4]),
            turn_acceleration=float(forecast.state[6]),
            longitudinal_acceleration=float(forecast.state[5]),
            valid=True, invalid_reason='', forecast=forecast,
            observation_source=self.latest_observation.source)

    def diagnostics(self, result, compute_time=0.):
        """Read-only model distribution with the exact message sequence key."""
        record = dict(
            schema_version='p6_imm_v1', sequence_id=result.sequence_id,
            mission_id=result.mission_id, source=result.source, model=result.model,
            frame_id=self.frame_id, source_stamp=result.source_stamp,
            observation_stamp=result.observation_stamp, generated_stamp=result.generated_stamp,
            valid_until=result.valid_until, valid=result.valid,
            invalid_reason=result.invalid_reason, observation_source=result.observation_source,
            input_age=result.generated_stamp - result.observation_stamp,
            last_rejection_reason=self.last_rejection_reason, compute_time=float(compute_time),
            timebase='source: propagated evaluation epoch; observation: image acquisition',
            covariance_status='engineering_prior_requires_independent_calibration',
            future_model_semantics='fixed_current_model_conditionals',
            posterior_projection_count=self.predictor.projection_count,
            posterior_projection_covariance='P_plus_outer_mean_shift',
            maximum_horizontal_speed_prior=self.predictor.config.maximum_horizontal_speed,
            relative_times=[], model_probabilities=[], state=[], state_covariance=[],
            model_disagreement=[], model_predictions=[])
        if self.latest_input is not None:
            record['raw_input_stamp'] = self.latest_input.stamp
            record['raw_input_age'] = result.generated_stamp - self.latest_input.stamp
            record['raw_image_measurement_stamp'] = self.latest_input.image_measurement_stamp
            record['raw_received_stamp'] = self.latest_input.received_stamp
            record['clock_identity'] = list(self.latest_input.clock_identity)
        if result.forecast is not None:
            forecast = result.forecast
            record.update(
                relative_times=forecast.horizons.tolist(),
                model_probabilities=forecast.model_probabilities.tolist(),
                state=forecast.state.tolist(), state_covariance=forecast.state_covariance.tolist(),
                model_disagreement=forecast.model_disagreement.tolist(),
                model_predictions=[dict(
                    model=model, probability=float(forecast.model_probabilities[index]),
                    positions=forecast.conditional_position[index].tolist(),
                    velocities=forecast.conditional_velocity[index].tolist(),
                    accelerations=forecast.conditional_acceleration[index].tolist(),
                    position_covariances=forecast.conditional_position_covariance[index].tolist())
                    for index, model in enumerate(MODEL_NAMES)])
        if result.valid:
            # Forecast arrays were checked in the numerical layer. Walking
            # every finite scalar again adds avoidable hot-path latency.
            return record
        # Invalid input diagnostics must remain legal JSON even if a rejected
        # transport carried NaN. No nonfinite value becomes a valid forecast.

        def finite_json(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {key: finite_json(item) for key, item in value.items()}
            if isinstance(value, list):
                return [finite_json(item) for item in value]
            return value
        return finite_json(record)


def prediction_to_message(result, compute_time, frame_id='local_ned'):
    """Reuse TargetPrediction with propagated covariance, including its z block."""
    message = TargetPrediction()
    for name in ('mission_id', 'sequence_id', 'source', 'model', 'turn_rate',
                 'turn_acceleration', 'longitudinal_acceleration', 'valid', 'invalid_reason'):
        setattr(message, name, getattr(result, name))
    for name in ('source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until'):
        setattr(message, name, _seconds_to_time(getattr(result, name)))
    message.frame_id = str(frame_id)
    message.prediction_horizon = result.horizon
    message.compute_time = float(compute_time)
    for sample in result.samples:
        output = PredictedTargetPoint()
        output.relative_time = _seconds_to_duration(sample.relative_time)
        for name in ('position', 'velocity', 'acceleration'):
            target = getattr(output, name)
            target.x, target.y, target.z = map(float, getattr(sample, name))
        output.position_covariance = list(sample.position_covariance)
        message.samples.append(output)
    return message


def node_parameter_defaults():
    """Fresh ROS-compatible defaults shared by the node and full-config snapshots."""
    def plain(value):
        if isinstance(value, np.ndarray):
            return plain(value.tolist())
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, (tuple, list)):
            return [plain(item) for item in value]
        return value

    defaults = dict(observation_topic='/perception/front/target_observation',
                    prediction_topic='/planning/imm_target_prediction',
                    diagnostics_topic='/planning/imm_diagnostics',
                    mission_topic='/mission/state', frame_id='local_ned',
                    update_rate_hz=20., prediction_horizon=4.,
                    prediction_sample_period=.1, input_timeout=.125,
                    target_time_profile='baseline_125')
    config_defaults = ImmConfig()
    for name in config_defaults.__dataclass_fields__:
        value = getattr(config_defaults, name)
        if isinstance(value, np.ndarray):
            value = value.ravel().tolist()
        elif isinstance(value, tuple):
            value = list(value)
        defaults['imm_' + name] = plain(value)
    return defaults


class ImmTargetPredictorNode(Node):
    """Only raw observations and mission identity enter this shadow-safe node."""

    def __init__(self):
        super().__init__('imm_target_predictor_node')
        defaults = node_parameter_defaults()
        config_defaults = ImmConfig()
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def parameter(name):
            return self.get_parameter(name).value
        input_topic = str(parameter('observation_topic')).strip()
        forbidden = {'/target/state', '/target/position', '/target/velocity',
                     '/tracking/target_state'}
        if ('/' + input_topic.strip('/') in forbidden
                or self.resolve_topic_name(input_topic) in forbidden):
            raise ValueError('observation_topic must be a raw observation, never truth or KF')
        rate = float(parameter('update_rate_hz'))
        if not math.isfinite(rate) or rate <= 0.:
            raise ValueError('update_rate_hz must be finite and positive')
        config_values = {name: parameter('imm_' + name)
                         for name in config_defaults.__dataclass_fields__}
        config_values['transition_matrix'] = np.asarray(
            config_values['transition_matrix']).reshape(3, 3)
        config = ImmConfig(**config_values)
        self.frame_id = str(parameter('frame_id'))
        self.engine = ImmPredictionEngine(
            ImmPredictor(config), horizon=parameter('prediction_horizon'),
            sample_period=parameter('prediction_sample_period'),
            input_timeout=parameter('input_timeout'), frame_id=self.frame_id,
            target_time_profile=parameter('target_time_profile'))
        self.mission_id = 0
        self.sequence_id = 0
        self.last_published_observation_stamp = None
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.observation_sub = self.create_subscription(
            TargetObservation, input_topic, self.observation_callback, sensor_qos)
        self.mission_sub = self.create_subscription(
            MissionState, str(parameter('mission_topic')), self.mission_callback, 1)
        self.prediction_pub = self.create_publisher(
            TargetPrediction, str(parameter('prediction_topic')), sensor_qos)
        self.diagnostics_pub = self.create_publisher(
            String, str(parameter('diagnostics_topic')), sensor_qos)
        self.timer = self.create_timer(1. / rate, self.timer_callback)
        install_navigation_frame_guard(self, self.navigation_frame_failed)
        self.get_logger().info(
            f'IMM raw observation input={input_topic} output={parameter("prediction_topic")} '
            f'with original {self.engine.input_timeout:.3f}s acquisition lease')

    def navigation_frame_failed(self, reason):
        self.engine.reset(reason)
        self.last_published_observation_stamp = None
        self.timer_callback(force=True)

    def observation_callback(self, message):
        if getattr(self, 'navigation_frame_fault', ''):
            return
        started = time.perf_counter()
        now = self.get_clock().now().nanoseconds * 1e-9
        status = self.engine.update(observation_from_message(message), now)
        if status.reason not in ('DUPLICATE_OBSERVATION', 'OUT_OF_ORDER_OBSERVATION'):
            self.timer_callback(force=True, observation_compute_time=time.perf_counter() - started)

    def mission_callback(self, message):
        mission_id = int(message.mission_id)
        if mission_id != self.mission_id:
            self.engine.reset('MISSION_RESET')
            self.last_published_observation_stamp = None
        self.mission_id = mission_id

    def timer_callback(self, force=False, observation_compute_time=0.):
        started = time.perf_counter()
        if getattr(self, 'navigation_frame_fault', ''):
            self.engine.invalidate(self.navigation_frame_fault)
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = self.engine.predictor.stamp
        # Checking the clock before suppression ensures simulation reset is
        # observable even when no new image is available.
        clock_ok = self.engine._observe_clock(now)
        if (not force and clock_ok and not self.engine.invalid_reason and stamp is not None
                and stamp == self.last_published_observation_stamp
                and 0. <= now - stamp <= self.engine.input_timeout):
            return
        self.sequence_id += 1
        result = self.engine.generate(now, self.mission_id, self.sequence_id)
        message = prediction_to_message(result, 0., self.frame_id)
        forecast_message_time = time.perf_counter() - started
        compute_time = forecast_message_time + observation_compute_time
        message.compute_time = compute_time
        self.prediction_pub.publish(message)
        record = self.engine.diagnostics(result, compute_time)
        record['observation_update_compute_time'] = observation_compute_time
        record['forecast_message_compute_time'] = forecast_message_time
        record['compute_time_scope'] = 'observation_update_plus_forecast_message_excludes_json_dds'
        diagnostic = String()
        diagnostic.data = json.dumps(record, allow_nan=False, separators=(',', ':'))
        self.diagnostics_pub.publish(diagnostic)
        if result.valid:
            self.last_published_observation_stamp = result.observation_stamp


def main(args=None):
    rclpy.init(args=args)
    node = ImmTargetPredictorNode()
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
