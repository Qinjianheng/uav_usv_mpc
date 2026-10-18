"""P7 preparation retains real scene parameters and requires current healthy F2."""

import json
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def test_p7_cases_preserve_full_r11_motion_and_nominal_policy(tmp_path):
    import p7_follow_sitl as entry
    for scene, speed, mode in (('f2', 2., 'linear'), ('f3', 4., 'figure_eight')):
        output = tmp_path/scene
        entry.prepare(output, scene)
        flight = yaml.safe_load((output/'config/flight.yaml').read_text())
        target = flight['moving_target']['ros__parameters']
        assert target['horizontal_speed'] == speed
        assert target['trajectory_type'] == mode
        planner = yaml.safe_load((output/'config/research.yaml').read_text())
        params = planner['p4_follow_planner_node']['ros__parameters']
        assert params['minco_engine'] == 'p47_follow_guided'
        assert params['short_horizon'] == 1.6
        assert params['p6_visibility_mode'] == 'nominal'
        assert params['target_time_profile'] == 'baseline_125'
        assert flight['trajectory_tracker_node']['ros__parameters'][
            'follow_minco_acceleration_feedforward']
        with pytest.raises(FileExistsError):
            entry.prepare(output, scene)


def test_f3_rejects_failed_or_different_source_f2(tmp_path):
    import p7_follow_sitl as entry
    path = tmp_path/'summary.json'
    for summary in [
        {'healthy': False},
        {'healthy': True, 'completed': True, 'f3_eligible': True, 'source_fingerprint': 'old'},
    ]:
        path.write_text(json.dumps(summary))
        with pytest.raises(ValueError, match='CURRENT_HEALTHY_F2_REQUIRED'):
            entry.require_f2(path)


def test_motion_diagnostic_uses_existing_profile_and_preserves_f2_speed(tmp_path):
    import p7_follow_sitl as entry
    output = tmp_path/'turning'
    entry.prepare(output, 'f2', motion='figure_eight')
    flight = yaml.safe_load((output/'config/flight.yaml').read_text())
    assert flight['moving_target']['ros__parameters']['trajectory_type'] == 'figure_eight'
    assert flight['moving_target']['ros__parameters']['horizontal_speed'] == 2.
    assert json.loads((output/'condition.json').read_text())['motion_mode'] == 'figure_eight'


def test_capture_is_explicit_and_does_not_change_evaluation_radius(tmp_path):
    import p7_follow_sitl as entry
    entry.prepare(tmp_path/'capture', 'f2', motion='variable_speed_z', capture=True)
    flight = yaml.safe_load((tmp_path/'capture/config/flight.yaml').read_text())
    assert flight['trajectory_tracker_node']['ros__parameters']['p7_capture_enabled']
    evaluator = flight['intercept_evaluator_node']['ros__parameters']
    assert evaluator['evaluation_capture_radius'] == .5
    assert evaluator['enable_sea_contact_failure']


def test_capture_observer_uses_descent_envelope_and_keeps_body_clear_of_sea():
    from p7_follow_sitl import guidance_health
    from test_p53_sitl import healthy
    health = healthy(10.)
    health['navigation']['p'][2] = -.33
    assert guidance_health(health, 10.) == 'HEIGHT_ERROR_OVER_1M'
    assert guidance_health(health, 10., capture=True) is None
    health['navigation']['p'][2] = -.2
    assert guidance_health(health, 10., capture=True) == 'CAPTURE_HEIGHT_OUTSIDE_ENVELOPE'


def test_capture_result_precedes_paused_navigation_staleness_without_hiding_failure():
    from p7_follow_sitl import capture_health
    result = dict(success=True, reason='CAPTURE_RADIUS_REACHED')
    assert capture_health({}, 10., result, None) == (10., None, True)
    assert capture_health({}, 13.1, result, 10.) == (10., None, True)
    assert capture_health({}, 15.1, result, 10.) == (10., 'P7_CAPTURE_COMPLETE', True)
    assert capture_health({}, 10., dict(success=False, reason='SEA_CONTACT'), None) == (
        None, 'CAPTURE_SEA_CONTACT', True)
    assert capture_health({}, 10., None, None) == (None, 'HEALTH_MONITOR_STALE', False)


def test_inner_follow_watchdog_uses_explicit_capture_floor_after_guidance_only(tmp_path):
    from p7_follow_sitl import prepare
    from p4_sitl_session import capture_stop_policy
    prepare(tmp_path/'normal', 'f2')
    prepare(tmp_path/'capture', 'f2', capture=True)
    assert capture_stop_policy(tmp_path/'normal') == (False, .5)
    assert capture_stop_policy(tmp_path/'capture') == (True, .5)
    assert capture_stop_policy(tmp_path/'capture', guidance_seen=True) == (True, .25)


def test_p7_observer_distinguishes_existing_input_wait_from_visual_loss():
    from p7_follow_sitl import P7FollowHealthGuard
    from test_p53_sitl import healthy
    observer = P7FollowHealthGuard(initial_acquisition=True)
    assert observer(healthy(10., owner='MINCO_FOLLOW'), 10.) is None
    for now in (10.05, 10.30, 10.45):
        state = healthy(now, locked=False)
        state['control'].update(visible=True, search='INPUT_WAIT')
        assert observer(state, now) is None
    assert observer(healthy(10.49, owner='FOLLOW'), 10.49) is None


def test_p7_observer_does_not_extend_input_wait_or_hide_physical_failure():
    from p7_follow_sitl import P7FollowHealthGuard
    from test_p53_sitl import healthy
    observer = P7FollowHealthGuard(initial_acquisition=True)
    observer(healthy(10., owner='MINCO_FOLLOW'), 10.)
    for now in (10.05, 10.51):
        state = healthy(now, locked=False)
        state['control'].update(visible=True, search='INPUT_WAIT')
        reason = observer(state, now)
    assert reason == 'FOLLOW_INPUT_WAIT_TIMEOUT'
    state['vehicle']['nav_state'] = 0
    assert observer(state, 10.51) == 'OFFBOARD_OR_ARMING_LOST'


@pytest.mark.parametrize('group,field', [
    ('outer', 'errors'), ('outer', 'survivors'), ('late', 'errors'), ('late', 'survivors'),
])
def test_manual_and_automatic_runs_cannot_hide_cleanup_failure(group, field):
    from p7_follow_sitl import clean_shutdown
    cleanup = dict(errors=[], survivors=[], late_components=dict(errors=[], survivors=[]))
    assert clean_shutdown(cleanup)
    target = cleanup if group == 'outer' else cleanup['late_components']
    target[field].append('task-owned process failure')
    assert not clean_shutdown(cleanup)
