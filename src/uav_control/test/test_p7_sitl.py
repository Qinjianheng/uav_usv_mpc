"""P7 preparation applies current speed and requires current healthy F2."""

import json
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def test_p7_cases_preserve_current_f2_and_explicit_f3_geometry(tmp_path, monkeypatch):
    import p7_follow_sitl as entry
    # Unit input only: never rebuild or relabel an absent historical qualification artifact.
    current = yaml.safe_load(entry.CONFIG.read_text())
    historical = yaml.safe_load(entry.CONFIG.read_text())
    historical['moving_target']['ros__parameters'].update(
        trajectory_type='figure_eight', initial_x=11., horizontal_speed=2.)
    source = tmp_path/'unit_scene_input/f3_h16_01/config/flight.yaml'
    source.parent.mkdir(parents=True)
    source.write_text(yaml.safe_dump(historical))
    monkeypatch.setattr(entry, 'R11', tmp_path/'unit_scene_input')
    for scene, mode in (('f2', 'linear'), ('f3', 'figure_eight')):
        output = tmp_path/scene
        entry.prepare(output, scene)
        flight = yaml.safe_load((output/'config/flight.yaml').read_text())
        target = flight['moving_target']['ros__parameters']
        assert target['horizontal_speed'] == 4.
        original = current if scene == 'f2' else historical
        original_target = original['moving_target']['ros__parameters']
        assert {k: v for k, v in target.items() if k != 'horizontal_speed'} == {
            k: v for k, v in original_target.items() if k != 'horizontal_speed'}
        assert target['trajectory_type'] == mode
        assert target['start_on_command'] is True
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


@pytest.mark.parametrize('mode', ['linear', 'figure_eight'])
def test_prepared_constant_modes_reach_four_mps_with_existing_acceleration(tmp_path, mode):
    from dataclasses import fields
    import math
    import p7_follow_sitl as entry
    from uav_control.usv_motion_profiles import MotionParameters, UsvMotionProfile
    output = tmp_path/'turning'
    entry.prepare(output, 'f2', motion=mode)
    flight = yaml.safe_load((output/'config/flight.yaml').read_text())
    target = flight['moving_target']['ros__parameters']
    assert target['trajectory_type'] == mode
    parameters = {field.name: target[field.name] for field in fields(MotionParameters)
                  if field.name in target}
    parameters.update(linear_vx=target['velocity_x'], linear_vy=target['velocity_y'],
                      velocity_z=target['velocity_z'])
    profile = UsvMotionProfile(mode, MotionParameters(**parameters))
    previous = 0.
    for _ in range(120):
        sample = profile.advance(.05)
        speed = math.hypot(*sample.velocity[:2])
        assert -1e-10 <= speed-previous <= target['horizontal_acceleration_limit']*.05+1e-10
        previous = speed
    assert speed == pytest.approx(4.)
    assert json.loads((output/'condition.json').read_text())['motion_mode'] == mode


def test_capture_is_explicit_and_does_not_change_evaluation_radius(tmp_path):
    import p7_follow_sitl as entry
    entry.prepare(tmp_path/'capture', 'f2', motion='variable_speed_z', capture=True)
    flight = yaml.safe_load((tmp_path/'capture/config/flight.yaml').read_text())
    assert flight['trajectory_tracker_node']['ros__parameters']['p7_capture_enabled']
    assert flight['moving_target']['ros__parameters']['start_on_command'] is True
    evaluator = flight['intercept_evaluator_node']['ros__parameters']
    assert evaluator['evaluation_capture_radius'] == .25
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


def test_new_invalid_prediction_cannot_be_joined_to_an_older_minco_diagnostic():
    from p7_follow_sitl import P7FollowHealthGuard
    from test_p53_sitl import healthy
    observer = P7FollowHealthGuard()
    assert observer(healthy(10., owner='MINCO_FOLLOW'), 10.) is None
    state = healthy(10.05, owner='MINCO_FOLLOW')
    state['prediction'].update(valid=False, receipt=10.04)
    state['control']['receipt'] = 10.03
    assert observer(state, 10.05) is None
    # The next control sample has actually relinquished MINCO authority.
    state = healthy(10.10, owner='FOLLOW')
    assert observer(state, 10.10) is None


def test_prediction_response_wait_is_one_cycle_and_cannot_hide_unsafe_minco_or_physical_fault():
    from p7_follow_sitl import P7FollowHealthGuard
    from test_p53_sitl import healthy
    observer = P7FollowHealthGuard()
    state = healthy(10., owner='MINCO_FOLLOW')
    state['prediction'].update(valid=False, receipt=9.99)
    state['control']['receipt'] = 9.98
    assert observer(state, 10.) is None
    # A newer MINCO diagnostic would prove that it did not relinquish control.
    state = healthy(10.02, owner='MINCO_FOLLOW')
    state['prediction'].update(valid=False, receipt=9.99)
    state['control']['receipt'] = 10.01
    assert observer(state, 10.02) == 'PREDICTION_INPUT_STALE'
    state = healthy(10.051, owner='MINCO_FOLLOW')
    state['control']['receipt'] = 9.98
    state['prediction'].update(valid=False, receipt=10.03)
    assert observer(state, 10.051) == 'PREDICTION_INPUT_STALE'
    state = healthy(10.04, owner='MINCO_FOLLOW')
    state['prediction'].update(valid=False, receipt=10.03)
    state['control']['receipt'] = 10.02
    state['vehicle']['nav_state'] = 0
    assert observer(state, 10.04) == 'OFFBOARD_OR_ARMING_LOST'


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
