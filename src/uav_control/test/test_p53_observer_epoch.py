"""Atomic health snapshots retain bounded transport age, without double-counting it."""
import pytest
from test_p5_sitl import health
from p5_minco_sitl import safety_reason


def test_snapshot_delivery_delay_is_not_recounted_as_each_input_age():
    snapshot = health()
    for key in ('control', 'navigation', 'prediction', 'attitude'):
        snapshot[key]['receipt'] = 99.95
    assert safety_reason(snapshot, 100.214) is None


@pytest.mark.parametrize('section,patch,reason', [
    ('control', {'receipt': 99.7}, 'HEALTH_INPUT_LOST'),
    ('control', {'receipt': 100.01}, 'HEALTH_INPUT_LOST'),
    ('navigation', {'stamp': 99.8}, 'NAVIGATION_INPUT_STALE'),
    ('navigation', {'p': [0., 0., -7.]}, 'HEIGHT_ERROR_OVER_1M'),
])
def test_delayed_snapshot_keeps_input_and_physical_faults(section, patch, reason):
    snapshot = health()
    snapshot[section].update(patch)
    assert safety_reason(snapshot, 100.214) == reason


def test_snapshot_transport_age_remains_bounded():
    assert safety_reason(health(), 100.251) == 'HEALTH_MONITOR_STALE'
    assert safety_reason(health(), 99.99) == 'HEALTH_MONITOR_STALE'
