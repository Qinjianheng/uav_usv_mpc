"""Real IMM math with ROS conversion and timestamp boundary checks."""

import json
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.node import Node
from uav_usv_interfaces.msg import TargetObservation

from uav_control.tracking.imm_target_predictor_node import ImmPredictionEngine
from uav_control.tracking.imm_target_predictor_node import ImmTargetPredictorNode
from uav_control.tracking.imm_target_predictor_node import observation_from_message
from uav_control.tracking.imm_target_predictor_node import prediction_to_message
from uav_control.tracking.imm_target_predictor_node import node_parameter_defaults


def observation(stamp=10., **overrides):
    message = TargetObservation()
    message.stamp.sec = int(stamp)
    message.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    message.image_measurement_stamp = stamp
    message.frame_id = 'local_ned'
    message.source = 'front_rgbd_red_sphere'
    message.position.x = 1.
    message.position.y = 2.
    message.position.z = -.15
    message.covariance = list(np.diag((.04, .09, .01)).ravel())
    message.valid = True
    for name, value in overrides.items():
        setattr(message, name, value)
    return message


def test_materialized_node_defaults_are_ros_compatible_and_independent_copies():
    defaults = node_parameter_defaults()
    assert isinstance(defaults['imm_transition_matrix'], list)
    assert len(defaults['imm_transition_matrix']) == 9
    assert all(isinstance(item, float) for item in defaults['imm_transition_matrix'])
    defaults['imm_transition_matrix'][0] = -1.
    fresh = node_parameter_defaults()
    from uav_control.tracking.imm_predictor import ImmConfig
    values = {name: fresh['imm_' + name] for name in ImmConfig.__dataclass_fields__}
    values['transition_matrix'] = np.asarray(values['transition_matrix']).reshape(3, 3)
    config = ImmConfig(**values)
    assert np.all(config.transition_matrix >= 0.)


def test_node_defaults_roundtrip_through_yaml_full_parameter_snapshot():
    import yaml
    defaults = node_parameter_defaults()
    encoded = yaml.safe_dump({'imm_target_predictor_node': {'ros__parameters': defaults}})
    decoded = yaml.safe_load(encoded)['imm_target_predictor_node']['ros__parameters']
    assert decoded == defaults


def test_raw_message_conversion_keeps_acquisition_and_clock_identity():
    message = observation(10.02, image_clock_reset_count=4, px4_clock_reset_count=2)
    message.received_stamp.sec = 10
    message.received_stamp.nanosec = 30_000_000
    result = observation_from_message(message)
    assert result.stamp == pytest.approx(10.02)
    assert result.position == (1., 2., -.15)
    assert result.clock_identity == (4, 2)
    assert result.received_stamp == pytest.approx(10.03)
    assert result.image_measurement_stamp == pytest.approx(10.02)


def test_generation_propagates_to_source_epoch_but_cannot_refresh_acquisition_lease():
    engine = ImmPredictionEngine(horizon=.2, sample_period=.1)
    assert engine.update(observation_from_message(observation()), now=10.02).accepted
    first = engine.generate(10.02, 9, 4)
    message = prediction_to_message(first, .001)
    assert message.valid
    assert message.source == 'tracking'
    assert message.model == 'IMM_CV_CTRV_CTRA'
    assert message.source_stamp.nanosec == 20_000_000
    assert message.observation_stamp.sec == 10
    assert message.observation_stamp.nanosec == 0
    assert message.valid_until.nanosec == 125_000_000
    assert len(message.samples) == 3
    assert message.samples[-1].relative_time.nanosec == 200_000_000
    assert message.samples[-1].position_covariance[0] > .04
    assert message.samples[-1].position_covariance[4] > .09
    expired = engine.generate(10.126, 9, 5)
    assert not expired.valid
    assert expired.invalid_reason == 'OBSERVATION_STALE'
    assert expired.valid_until == pytest.approx(10.125)
    assert not expired.samples


def test_timeout_configuration_cannot_extend_original_125ms_observation_lease():
    with pytest.raises(ValueError, match='125'):
        ImmPredictionEngine(input_timeout=.126)


@pytest.mark.parametrize('kwargs,now,reason', (
    ({'valid': False}, 10.02, 'INVALID_OBSERVATION'),
    ({'frame_id': 'map'}, 10.02, 'FRAME_MISMATCH'),
    ({'source': 'simulation_truth'}, 10.02, 'FORBIDDEN_OBSERVATION_SOURCE'),
    ({'image_measurement_stamp': 9.}, 10.02, 'ACQUISITION_STAMP_MISMATCH'),
    ({}, 9.99, 'OBSERVATION_TIME_IN_FUTURE'),
    ({}, 10.126, 'OBSERVATION_STALE'),
))
def test_invalid_transport_revokes_current_output_without_retiming(kwargs, now, reason):
    engine = ImmPredictionEngine()
    status = engine.update(observation_from_message(observation(**kwargs)), now=now)
    assert not status.accepted
    assert status.reason == reason
    result = engine.generate(max(now, 10.02), 1, 1)
    assert not result.valid
    assert result.invalid_reason == reason


def test_duplicate_raw_frames_are_not_independent_measurements():
    engine = ImmPredictionEngine()
    engine.update(observation_from_message(observation()), now=10.01)
    before = engine.predictor.predict((0.,))
    status = engine.update(observation_from_message(observation()), now=10.03)
    assert status.reason == 'DUPLICATE_OBSERVATION'
    assert engine.predictor.update_count == 1
    np.testing.assert_array_equal(engine.predictor.predict((0.,)).state_covariance,
                                  before.state_covariance)
    assert engine.generate(10.03, 1, 2).valid


def test_clock_backwards_clears_posterior_and_requires_new_observation():
    engine = ImmPredictionEngine()
    engine.update(observation_from_message(observation()), now=10.01)
    assert not engine.generate(5., 1, 2).valid
    assert engine.predictor.stamp is None
    assert engine.generate(5.01, 1, 3).invalid_reason == 'CLOCK_RESET'
    assert engine.update(observation_from_message(observation(5.02)), now=5.03).accepted
    assert engine.generate(5.03, 1, 4).valid


def test_image_or_px4_clock_reset_rejects_trigger_and_clears_old_track():
    engine = ImmPredictionEngine()
    engine.update(observation_from_message(observation()), now=10.01)
    status = engine.update(observation_from_message(observation(
        10.05, image_clock_reset_count=1)), now=10.06)
    assert status.reason == 'CLOCK_RESET'
    assert engine.predictor.stamp is None
    assert engine.update(observation_from_message(observation(
        10.1, image_clock_reset_count=1)), now=10.11).accepted
    assert engine.predictor.update_count == 1


def test_diagnostics_binds_conditional_covariance_to_prediction_sequence_and_source():
    engine = ImmPredictionEngine(horizon=.2, sample_period=.1)
    engine.update(observation_from_message(observation()), now=10.02)
    result = engine.generate(10.02, 9, 7)
    diagnostics = engine.diagnostics(result, compute_time=.001)
    encoded = json.dumps(diagnostics, allow_nan=False)
    record = json.loads(encoded)
    assert record['sequence_id'] == 7
    assert record['mission_id'] == 9
    assert record['source_stamp'] == pytest.approx(10.02)
    assert record['observation_stamp'] == 10.
    assert record['input_age'] == pytest.approx(.02)
    assert record['relative_times'] == pytest.approx((0., .1, .2))
    assert len(record['state_covariance']) == 7
    assert len(record['model_predictions']) == 3
    assert record['model_predictions'][0]['model'] == 'CV'
    assert len(record['model_predictions'][0]['position_covariances']) == 3
    assert record['valid_until'] == pytest.approx(10.125)


@pytest.mark.parametrize('now,acquisition', (
    (10.0200000004, 10.), (1_791_956_000.0200004, 1_791_956_000.)))
def test_real_engine_ros_follow_parser_accepts_same_prediction_diagnostics(now, acquisition):
    from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
    from uav_control.guidance.p6_follow import attach_model_diagnostics
    engine = ImmPredictionEngine(horizon=.4, sample_period=.1)
    # A non-nanosecond-grid float and 3*.1 exercise actual ROS quantization.
    engine.update(observation_from_message(observation(acquisition)), now=now)
    result = engine.generate(now, 9, 7)
    transport = prediction_to_message(result, compute_time=.001)
    parsed = prediction_from_message(transport)
    attached = attach_model_diagnostics(parsed, engine.diagnostics(result))
    assert attached.model_probabilities == pytest.approx(result.forecast.model_probabilities)
    assert np.asarray(attached.model_positions).shape == (3, 5, 3)
    assert parsed.observation_stamp == acquisition
    assert parsed.prediction_times[3] == pytest.approx(.3)


def real_sequence_780():
    engine = ImmPredictionEngine()
    acquired = observation(1791992166.966803)
    original_stamp = (acquired.stamp.sec, acquired.stamp.nanosec)
    now = 1791992166.9974535
    assert engine.update(observation_from_message(acquired), now=now).accepted
    result = engine.generate(now, 1, 780)
    return engine, acquired, original_stamp, result


def test_real_sequence_780_preserves_all_float_epochs_and_full_grid_on_ros_roundtrip():
    from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
    from uav_control.guidance.p6_follow import attach_model_diagnostics
    engine, acquired, original_stamp, result = real_sequence_780()
    parsed = prediction_from_message(prediction_to_message(result, .001))
    attached = attach_model_diagnostics(parsed, engine.diagnostics(result))
    for name in ('source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until'):
        assert getattr(parsed, name) == getattr(result, name)
    assert len(parsed.prediction_times) == 41
    assert np.allclose(parsed.prediction_times, engine.horizons, atol=.5e-9, rtol=0.)
    assert np.asarray(attached.model_positions).shape == (3, 41, 3)
    assert (acquired.stamp.sec, acquired.stamp.nanosec) == original_stamp
    assert result.observation_stamp == acquired.image_measurement_stamp
    assert result.valid_until == result.observation_stamp + .125


@pytest.mark.parametrize('field', (
    'source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until', 'relative_time'))
def test_real_sequence_780_rejects_a_239_nanosecond_wire_identity_change(field):
    from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
    from uav_control.guidance.p6_follow import attach_model_diagnostics
    engine, _, _, result = real_sequence_780()
    message = prediction_to_message(result, .001)
    stamp = (message.samples[3].relative_time if field == 'relative_time'
             else getattr(message, field))
    # Keep source <= generated so this specifically exercises identity checking.
    shift = -239 if field == 'source_stamp' else 239
    carry, nanosecond = divmod(stamp.nanosec + shift, 1_000_000_000)
    stamp.sec += carry
    stamp.nanosec = nanosecond
    with pytest.raises(ValueError, match='P6_MODEL_IDENTITY_MISMATCH'):
        attach_model_diagnostics(prediction_from_message(message), engine.diagnostics(result))


@pytest.mark.parametrize('value,expected', (
    (0., (0, 0)), (10.9999999996, (11, 0)), (-.1, (-1, 900_000_000)),
    (-1.0000000006, (-2, 999_999_999)), (-.0000000004, (0, 0)),
    (1791992166.966803, (1791992166, 966_803_074))))
def test_imm_fractional_seconds_normalizes_carry_and_negative_time(value, expected):
    from uav_control.tracking.imm_target_predictor_node import _seconds_to_duration
    from uav_control.tracking.imm_target_predictor_node import _seconds_to_time
    for convert in (_seconds_to_time, _seconds_to_duration):
        stamp = convert(value)
        assert (stamp.sec, stamp.nanosec) == expected
        assert 0 <= stamp.nanosec < 1_000_000_000


@pytest.mark.parametrize('value', (np.nan, np.inf, -np.inf))
def test_imm_time_conversion_rejects_nonfinite_epoch(value):
    from uav_control.tracking.imm_target_predictor_node import _seconds_to_time
    with pytest.raises(ValueError, match='INVALID_IMM_TIME'):
        _seconds_to_time(value)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_node(clock):
    node = object.__new__(ImmTargetPredictorNode)
    node.engine = ImmPredictionEngine(horizon=.2)
    node.frame_id = 'local_ned'
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=round(clock[0] * 1e9)))
    node.mission_id = 1
    node.sequence_id = 0
    node.last_published_observation_stamp = None
    node.prediction_pub = Publisher()
    node.diagnostics_pub = Publisher()
    return node


def test_new_raw_observation_publishes_immediately_timer_only_reports_expiry():
    clock = [10.02]
    node = make_node(clock)
    node.observation_callback(observation())
    assert len(node.prediction_pub.messages) == 1
    clock[0] = 10.04
    node.observation_callback(observation())
    node.timer_callback()
    assert len(node.prediction_pub.messages) == 1
    assert node.engine.predictor.update_count == 1
    clock[0] = 10.13
    node.timer_callback()
    assert len(node.prediction_pub.messages) == 2
    assert not node.prediction_pub.messages[-1].valid
    record = json.loads(node.diagnostics_pub.messages[-1].data)
    assert not record['valid']
    assert record['invalid_reason'] == 'OBSERVATION_STALE'


def test_mission_change_cannot_relabel_old_observation_as_new_mission():
    clock = [10.02]
    node = make_node(clock)
    node.observation_callback(observation())
    node.mission_callback(SimpleNamespace(mission_id=2))
    node.timer_callback()
    assert node.engine.predictor.stamp is None
    assert not node.prediction_pub.messages[-1].valid
    assert node.prediction_pub.messages[-1].mission_id == 2


@pytest.fixture
def unsafe_configuration(monkeypatch):
    parameters = {}
    monkeypatch.setattr(Node, '__init__', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda _node, name, value:
                        parameters.setdefault(name, value))
    monkeypatch.setattr(Node, 'get_parameter', lambda _node, name:
                        SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, 'resolve_topic_name', lambda _node, name: name)
    monkeypatch.setattr(Node, 'create_subscription', lambda *_args, **_kwargs:
                        pytest.fail('unsafe input reached DDS creation'))
    return parameters


@pytest.mark.parametrize('topic', (
    '/target/state', '/target/position', '/target/velocity', '/tracking/target_state'))
def test_node_forbids_truth_and_filtered_state_topics(topic, unsafe_configuration):
    unsafe_configuration['observation_topic'] = topic
    with pytest.raises(ValueError, match='raw observation'):
        ImmTargetPredictorNode()


def test_node_checks_resolved_remap_not_only_requested_topic(
        monkeypatch, unsafe_configuration):
    monkeypatch.setattr(Node, 'resolve_topic_name', lambda *_args: '/target/state')
    with pytest.raises(ValueError, match='raw observation'):
        ImmTargetPredictorNode()
