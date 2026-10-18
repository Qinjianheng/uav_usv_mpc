"""P7 pursuit geometry: reuse bounded takeoff and causal FOLLOW yaw feedforward."""

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from uav_control.mission.target_visibility import VisibilityDecision
from uav_usv_interfaces.msg import MissionState


def test_p7_enables_existing_safe_takeoff_follow_without_changing_depth_or_lock_limits():
    path = Path(__file__).parents[2]/'uav_usv_bringup/config/p7_guidance.yaml'
    config = yaml.safe_load(path.read_text())
    tracker = config['trajectory_tracker_node']['ros__parameters']
    assert tracker.get('takeoff_visual_tracking') is True
    assert tracker['takeoff_horizontal_start_height'] == .5
    assert tracker['takeoff_maximum_horizontal_acceleration'] == 1.5
    assert tracker['maximum_state_age'] == .125
    assert tracker['target_lock_max_bearing'] == .15
    assert config['rgbd_target_localizer']['ros__parameters']['maximum_depth'] == 25.


def yaw_node(locked=True, active=True):
    from test_strict_visual_control import control_node
    node, state = control_node()
    node.p7_guidance_enabled = True
    node.follow_reference_mode = 'target'
    node.mission_state = MissionState.FOLLOW
    node.follow_receiver = SimpleNamespace(active=object() if active else None)
    node.visibility_decision = VisibilityDecision('TRACKING' if locked else 'REACQUIRE',
                                                  locked, True, 0., 0)
    node.visibility.last_valid_image_bearing = .21
    node.follow_metrics = dict(epoch=10.18, source_stamp=10.16, los_yaw_rate=.30)
    return node, state


@pytest.mark.parametrize('locked,active', [(True, True), (True, False), (False, False)])
def test_p7_target_reference_follow_reuses_causal_los_rate_during_turn_and_reacquisition(
        locked, active):
    node, state = yaw_node(locked, active)
    out = SimpleNamespace()
    node._final_yaw(out, state, .05)
    assert out.yawspeed == pytest.approx(.51)


@pytest.mark.parametrize('field,value', [
    ('epoch', 10.), ('source_stamp', 10.), ('epoch', 10.21), ('source_stamp', 10.21)])
def test_follow_yaw_cannot_reuse_old_or_future_common_epoch_pair(field, value):
    node, state = yaw_node()
    node.follow_metrics[field] = value
    out = SimpleNamespace()
    node._final_yaw(out, state, .05)
    assert out.yawspeed == pytest.approx(.21)


def test_reacquisition_yaw_keeps_original_search_rate_cap():
    node, state = yaw_node(locked=False, active=False)
    node.follow_metrics['los_yaw_rate'] = 10.
    out = SimpleNamespace()
    node._final_yaw(out, state, .05)
    assert out.yawspeed == node.visibility.config.maximum_search_yaw_rate


def test_y_keeps_exclusive_ibvs_yaw_without_follow_feedforward():
    node, state = yaw_node()
    node.mission_state = MissionState.FAR_GUIDANCE
    node.p7_result = SimpleNamespace(valid=True, yaw_rate=.12)
    out = SimpleNamespace()
    node._final_yaw(out, state, .05)
    assert out.yawspeed == pytest.approx(.12)
    assert node.yaw_owner == 'IBVS'
