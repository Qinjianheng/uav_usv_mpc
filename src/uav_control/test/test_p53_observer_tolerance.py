"""Read-only experiment monitoring tolerates queue jitter; execution TTL remains125ms."""
from test_p53_sitl import healthy
import pytest
import p53_minco_sitl as entry
from p51_minco_sitl import health_decision


@pytest.mark.parametrize('owner', ('FOLLOW', 'MINCO_FOLLOW'))
def test_research_observer_tolerates173ms_queue_age_without_rewriting_samples(owner):
    snapshot = healthy(10., owner=owner)
    snapshot['navigation']['stamp'] = 9.827
    snapshot['prediction'].update(stamp=9.827, observation=9.8)
    assert health_decision(snapshot, 10.)['fatal'] == 'NAVIGATION_INPUT_STALE'
    assert entry.FollowHealthGuard()(snapshot, 10.) is None
    assert snapshot['navigation']['stamp'] == 9.827
    assert snapshot['prediction']['observation'] == 9.8


@pytest.mark.parametrize('fault,expected', [
    ('old', 'NAVIGATION_INPUT_STALE'), ('future', 'NAVIGATION_INPUT_STALE'),
    ('height', 'HEIGHT_ERROR_OVER_1M'), ('offboard', 'OFFBOARD_OR_ARMING_LOST'),
])
def test_moderate_observer_window_retains_real_input_and_physical_rejections(fault, expected):
    snapshot = healthy(10.)
    if fault == 'old':
        snapshot['navigation']['stamp'] = 9.69
    elif fault == 'future':
        snapshot['navigation']['stamp'] = 10.01
    elif fault == 'height':
        snapshot['navigation']['p'][2] = -7.
    else:
        snapshot['vehicle']['nav_state'] = 3
    assert entry.FollowHealthGuard()(snapshot, 10.) == expected
