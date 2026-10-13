"""Do not issue X while the independent health recorder still drains startup history."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
from p4_sitl_session import pre_x_health_ready  # noqa: E402


def ground():
    return dict(written=100., ros_now=100.,
                control=dict(status='GROUND_HOLD', receipt=99.99),
                navigation=dict(valid=True, receipt=99.99, stamp=99.98),
                attitude=dict(receipt=99.99),
                vehicle=dict(nav_state=14, armed=1, receipt=99.8))


def test_fresh_ground_offboard_recorder_allows_x():
    assert pre_x_health_ready(ground(), 100.05)


@pytest.mark.parametrize('section,patch', [
    ('control', dict(status='INIT')), ('control', dict(receipt=99.7)),
    ('navigation', dict(valid=False)), ('vehicle', dict(armed=2)),
    ('vehicle', dict(nav_state=4)), ('attitude', dict(receipt=100.01)),
])
def test_initial_backlog_or_wrong_flight_state_cannot_issue_x(section, patch):
    snapshot = deepcopy(ground())
    snapshot[section].update(patch)
    assert not pre_x_health_ready(snapshot, 100.05)


def test_missing_stale_or_future_recorder_cannot_issue_x():
    assert not pre_x_health_ready({}, 100.)
    assert not pre_x_health_ready(ground(), 100.251)
    assert not pre_x_health_ready(ground(), 99.99)


@pytest.mark.parametrize('native_stamp', (99.8, 100.01))
def test_recent_receipt_cannot_hide_old_or_future_native_navigation(native_stamp):
    snapshot = ground()
    snapshot['navigation']['stamp'] = native_stamp
    assert not pre_x_health_ready(snapshot, 100.05)
