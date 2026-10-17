import xml.etree.ElementTree as ElementTree

import pytest

from uav_control.gazebo_target_visualizer import (
    GazeboTargetVisualizer,
    ned_to_gazebo_enu,
    red_sphere_sdf,
)


def test_visualizer_reports_native_pause_and_resume():
    from types import SimpleNamespace

    visualizer = GazeboTargetVisualizer.__new__(GazeboTargetVisualizer)
    visualizer._world_paused = None
    assert visualizer.world_paused is None
    visualizer._world_stats_callback(SimpleNamespace(paused=True))
    assert visualizer.world_paused is True
    visualizer._world_stats_callback(SimpleNamespace(paused=False))
    assert visualizer.world_paused is False


def test_ned_to_gazebo_enu_swaps_horizontal_axes_and_flips_z():
    assert ned_to_gazebo_enu(10.0, 4.0, -3.0) == pytest.approx(
        (4.0, 10.0, 3.0)
    )


def test_red_sphere_is_visual_only_and_has_requested_size():
    model = ElementTree.fromstring(
        red_sphere_sdf('usv_target', 0.5)
    ).find('model')

    assert model is not None
    assert model.attrib['name'] == 'usv_target'
    assert model.findtext('static') == 'true'
    assert model.find('.//collision') is None
    radius = float(model.findtext('.//sphere/radius'))
    assert radius == pytest.approx(0.25)
    assert model.findtext('.//material/diffuse') == '1 0 0 1'


@pytest.mark.parametrize(
    'entity_name',
    ['', '1target', 'bad name', 'bad/name'],
)
def test_red_sphere_rejects_unsafe_entity_names(entity_name):
    with pytest.raises(ValueError):
        red_sphere_sdf(entity_name, 0.5)


def visualizer_with_transport(monkeypatch, outcomes):
    """Keep real update/pose conversion; replace only the external Gazebo RPC."""
    from types import SimpleNamespace
    import uav_control.gazebo_target_visualizer as module
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    visualizer = GazeboTargetVisualizer.__new__(GazeboTargetVisualizer)
    visualizer._created = True
    visualizer._last_create_attempt = 10.
    visualizer._create_retry_period = 1.
    visualizer._entity_name = 'usv_target'
    visualizer._model_sdf = red_sphere_sdf('usv_target', .5)
    visualizer._set_pose_service = '/world/default/set_pose'
    visualizer._create_service = '/world/default/create'
    visualizer._pose_type, visualizer._entity_factory_type = Pose, EntityFactory
    visualizer._pose_failures = 0
    visualizer.last_error = ''
    transport = SimpleNamespace(now=10.1, calls=[], results=iter(outcomes))

    def request(service, value, request_type):
        transport.calls.append((service, value))
        return next(transport.results)

    monkeypatch.setattr(module.time, 'monotonic', lambda: transport.now)
    monkeypatch.setattr(visualizer, '_request', request)
    return visualizer, transport


def test_single_pose_timeout_retries_latest_pose_next_tick_without_creation_wait(monkeypatch):
    visualizer, transport = visualizer_with_transport(monkeypatch, [False, True])
    assert not visualizer.update(2., 3., -4.)
    transport.now += .05
    assert visualizer.update(2.1, 3., -4.)
    assert [c[0] for c in transport.calls] == ['/world/default/set_pose']*2
    assert transport.calls[-1][1].position.y == pytest.approx(2.1)
    assert visualizer.last_error == ''


def test_persistently_missing_entity_still_attempts_bounded_recreation(monkeypatch):
    visualizer, transport = visualizer_with_transport(monkeypatch, [False]*3+[True, True])
    for i in range(3):
        transport.now = 10.1+i*.1
        assert not visualizer.update(2.+i*.2, 3., -4.)
    transport.now = 10.4
    assert visualizer.update(2.6, 3., -4.)
    assert [c[0] for c in transport.calls] == (
        ['/world/default/set_pose']*3+['/world/default/create', '/world/default/set_pose'])


def test_success_between_timeouts_resets_consecutive_missing_entity_evidence(monkeypatch):
    visualizer, transport = visualizer_with_transport(monkeypatch, [False, False, True, False])
    for i, expected in enumerate([False, False, True, False]):
        transport.now = 10.1+i*.1
        assert visualizer.update(2.+i*.2, 3., -4.) == expected
    assert visualizer.created
    assert [c[0] for c in transport.calls] == ['/world/default/set_pose']*4
