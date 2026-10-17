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
