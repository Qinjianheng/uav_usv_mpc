"""ROS message boundary tests for the target predictor node."""

import pytest
from types import SimpleNamespace

from rclpy.node import Node
from uav_usv_interfaces.msg import TargetState

from uav_control.tracking.target_prediction import PredictionEngine
from uav_control.tracking.target_predictor_node import prediction_to_message
from uav_control.tracking.target_predictor_node import state_from_message
from uav_control.tracking.target_predictor_node import TargetPredictorNode


def test_target_state_conversion_preserves_measurement_time():
    message = TargetState()
    message.stamp.sec = 12
    message.stamp.nanosec = 250_000_000
    message.source_stamp.sec = 12
    message.frame_id = 'local_ned'
    message.position.x = 1.0
    message.position.y = 2.0
    message.position.z = 0.1
    message.velocity.x = 3.0
    message.velocity.y = 4.0
    message.velocity.z = 0.2
    message.valid = True

    state = state_from_message(message)

    assert state.stamp == pytest.approx(12.25)
    assert state.observation_stamp == pytest.approx(12.0)
    assert state.position == pytest.approx((1.0, 2.0, 0.1))
    assert state.velocity == pytest.approx((3.0, 4.0, 0.2))


def test_prediction_message_keeps_source_age_and_samples():
    state_message = TargetState()
    state_message.stamp.sec = 20
    state_message.source_stamp.sec = 20
    state_message.frame_id = 'local_ned'
    state_message.position.x = 1.0
    state_message.position.y = 2.0
    state_message.position.z = 0.0
    state_message.velocity.x = 4.0
    state_message.velocity.z = 0.2
    state_message.valid = True
    engine = PredictionEngine(horizon=0.2, sample_period=0.1)
    engine.update(state_from_message(state_message))
    result = engine.generate(now=20.05, mission_id=9, sequence_id=4)

    message = prediction_to_message(result, compute_time=0.001)

    assert message.mission_id == 9
    assert message.sequence_id == 4
    assert message.source_stamp.sec == 20
    assert message.observation_stamp.sec == 20
    assert message.generated_stamp.sec == 20
    assert message.generated_stamp.nanosec == 50_000_000
    assert message.compute_time == pytest.approx(0.001)
    assert len(message.samples) == 3
    assert message.samples[-1].relative_time.nanosec == 200_000_000
    assert message.samples[-1].position.x == pytest.approx(1.8)
    assert len(message.samples[-1].position_covariance) == 9


def make_tracking_message(stamp=10.1, source_stamp=10.0, valid=True):
    message = TargetState()
    message.stamp.sec = int(stamp)
    message.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    message.source_stamp.sec = int(source_stamp)
    message.source_stamp.nanosec = round(
        (source_stamp - int(source_stamp)) * 1e9
    )
    message.frame_id = 'local_ned'
    message.position.x = 4.0
    message.velocity.x = 2.0
    message.valid = valid
    return message


def make_callback_node(now=10.11):
    return SimpleNamespace(
        engine=PredictionEngine(source='tracking'),
        frame_id='local_ned',
        timer_callback=lambda: None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=round(now * 1e9)),
        ),
    )


@pytest.mark.parametrize(
    'failure', ('invalid', 'frame', 'future', 'zero', 'stale'),
)
def test_invalid_tracking_transport_immediately_revokes_cached_state(failure):
    node = make_callback_node()
    TargetPredictorNode.state_callback(node, make_tracking_message())
    assert node.engine.generate(10.11, 1, 1).valid
    bad = make_tracking_message(stamp=10.11)
    if failure == 'invalid':
        bad.valid = False
    elif failure == 'frame':
        bad.frame_id = 'world_enu'
    elif failure == 'future':
        bad.source_stamp.nanosec = 120_000_000
    elif failure == 'zero':
        bad.source_stamp.sec = 0
    elif failure == 'stale':
        bad.source_stamp.sec = 9

    TargetPredictorNode.state_callback(node, bad)

    assert node.engine.latest_state is None
    assert not node.engine.generate(10.11, 1, 2).valid


def test_out_of_order_tracking_packet_cannot_poison_newer_valid_input():
    node = make_callback_node()
    TargetPredictorNode.state_callback(node, make_tracking_message())
    TargetPredictorNode.state_callback(
        node, make_tracking_message(stamp=9.5, source_stamp=9.4),
    )
    assert node.engine.generate(10.11, 1, 2).valid
    assert node.engine.latest_state.stamp == pytest.approx(10.1)


@pytest.fixture
def initialize_without_ros_transport(monkeypatch):
    """Isolate configuration validation from DDS and disk logging."""
    parameters = {}
    monkeypatch.setattr(Node, '__init__', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        Node, 'declare_parameter',
        lambda _node, name, value: parameters.setdefault(name, value),
    )
    monkeypatch.setattr(
        Node, 'get_parameter',
        lambda _node, name: SimpleNamespace(value=parameters[name]),
    )
    monkeypatch.setattr(
        Node, 'resolve_topic_name', lambda _node, name: name,
    )
    monkeypatch.setattr(
        Node, 'create_subscription',
        lambda *_args, **_kwargs: pytest.fail(
            'unsafe configuration reached subscription creation',
        ),
    )
    return parameters


@pytest.mark.parametrize(
    'topic', ('/target/state', '/target/position', '/target/velocity'),
)
def test_online_predictor_refuses_truth_topics(
    topic, initialize_without_ros_transport,
):
    initialize_without_ros_transport['tracking_topic'] = topic
    with pytest.raises(ValueError, match='truth'):
        TargetPredictorNode()


def test_online_predictor_refuses_simulation_truth_source(
    initialize_without_ros_transport,
):
    initialize_without_ros_transport['target_state_source'] = (
        'simulation_truth'
    )
    with pytest.raises(ValueError, match='tracking'):
        TargetPredictorNode()


def test_online_predictor_refuses_tracking_topic_remapped_to_truth(
    monkeypatch, initialize_without_ros_transport,
):
    monkeypatch.setattr(
        Node, 'resolve_topic_name', lambda _node, _name: '/target/state',
    )
    with pytest.raises(ValueError, match='truth'):
        TargetPredictorNode()


class PredictionPublisher:
    """Collect actual predictor output messages at the transport boundary."""

    def __init__(self):
        """Create an empty output sequence."""
        self.messages = []

    def publish(self, message):
        """Store one timestamp-preserving forecast."""
        self.messages.append(message)


def make_publication_node(clock):
    """Use the real callbacks without starting ROS middleware."""
    node = object.__new__(TargetPredictorNode)
    node.engine = PredictionEngine(source='tracking', horizon=0.2)
    node.frame_id = 'local_ned'
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock[0] * 1e9)),
    )
    node.mission_id = 1
    node.sequence_id = 0
    node.last_prediction_observation_stamp = None
    node.prediction_pub = PredictionPublisher()
    return node


def test_new_kf_observation_publishes_before_next_periodic_timer():
    """Do not spend another 50 ms of acquisition age waiting to predict."""
    clock = [10.02]
    node = make_publication_node(clock)
    node.state_callback(make_tracking_message(stamp=10.02))
    assert len(node.prediction_pub.messages) == 1
    message = node.prediction_pub.messages[0]
    assert message.valid
    assert message.source_stamp.nanosec == 20_000_000
    assert message.observation_stamp.sec == 10
    assert message.observation_stamp.nanosec == 0
    assert message.valid_until.nanosec == 125_000_000


def test_duplicate_kf_projection_and_timer_do_not_double_prediction_rate():
    """Publish once per new image while maintaining timeout heartbeats."""
    clock = [10.02]
    node = make_publication_node(clock)
    node.state_callback(make_tracking_message(stamp=10.02))
    clock[0] = 10.035
    node.state_callback(make_tracking_message(stamp=10.035))
    clock[0] = 10.055
    node.timer_callback()
    assert len(node.prediction_pub.messages) == 1
    clock[0] = 10.07
    node.state_callback(make_tracking_message(stamp=10.07, source_stamp=10.05))
    assert len(node.prediction_pub.messages) == 2
    clock[0] = 10.20
    node.timer_callback()
    assert len(node.prediction_pub.messages) == 3
    message = node.prediction_pub.messages[-1]
    assert not message.valid
    assert message.invalid_reason == 'STATE_STALE'
    assert message.observation_stamp.nanosec == 50_000_000
    assert message.valid_until.nanosec == 175_000_000


def test_optin_producer_profile_measures_actual_bctra_batch_without_epoch_changes():
    import json
    clock = [10.02]
    node = make_publication_node(clock)
    records = []
    node.p43_profile_enabled = True
    node.get_logger = lambda: SimpleNamespace(info=records.append)
    node.state_callback(make_tracking_message(stamp=10.02))
    assert len(records) == 1
    record = json.loads(records[0].split('P43_PREDICTOR_PROFILE ', 1)[1])
    assert record['producer_process'] != 'planner'
    assert record['stages']['bctra_batch']['calls'] == 1
    assert node.prediction_pub.messages[0].observation_stamp.nanosec == 0
