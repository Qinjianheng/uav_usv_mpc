"""Preserve legacy guidance limits when terminal limits are configured."""

from types import SimpleNamespace

import pytest
from rclpy.node import Node

from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode


@pytest.mark.parametrize('overrides, expected', [
    ({'maximum_horizontal_acceleration': 2.5}, 2.5),
    ({'maximum_horizontal_acceleration': 3.5,
      'guidance_maximum_horizontal_acceleration': 3.0}, 3.0),
])
def test_legacy_follow_limit_is_inherited_unless_explicitly_overridden(
    monkeypatch, overrides, expected,
):
    # Exercise the real node's parameter declaration and core construction.
    # ROS transports are replaced because this is a constructor unit test.
    values = {}
    monkeypatch.setattr(Node, '__init__', lambda *args, **kwargs: None)

    def declare(self, name, default, descriptor=None):
        values[name] = overrides.get(name, default)
        return SimpleNamespace(value=values[name])

    monkeypatch.setattr(Node, 'declare_parameter', declare)
    monkeypatch.setattr(Node, 'get_parameter',
                        lambda self, name: SimpleNamespace(value=values[name]))
    monkeypatch.setattr(Node, 'resolve_topic_name', lambda self, name: name)
    for method in ('create_subscription', 'create_publisher', 'create_timer'):
        monkeypatch.setattr(Node, method, lambda *args, **kwargs: None)
    monkeypatch.setattr(Node, 'get_logger',
                        lambda self: SimpleNamespace(info=lambda *args: None,
                                                     warning=lambda *args: None))
    node = TrajectoryTrackerNode()
    assert node.flight_guidance.maximum_horizontal_acceleration == expected
    assert node.tracker.maximum_horizontal_acceleration == overrides[
        'maximum_horizontal_acceleration']
