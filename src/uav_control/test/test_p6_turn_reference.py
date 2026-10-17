"""Independent circular-motion and causal-frame cases for a turning rear viewpoint."""
from dataclasses import replace
import math

import numpy as np
import pytest

from uav_control.guidance.follow_admission import follow_settling
from uav_control.guidance.follow_problem import FollowProblem, future_request
from uav_control.guidance.follow_rollout import (
    FollowRolloutConfig, rollout_follow, rollout_follow_fast,
)
from test_follow_research_algorithms import request
from test_progress_follow import model


def circular_histories(offset=0.):
    tracking = []
    for t in (100., 100.05, 100.1, 100.15, 100.2):
        theta = offset + .4*(t-100.)
        tracking.append(dict(valid=True, stamp=t, source=t-.02,
                             p=(10*math.cos(theta), 10*math.sin(theta), 0.),
                             v=(-4*math.sin(theta), 4*math.cos(theta), 0.)))
    theta = offset + .08
    p = (10*math.cos(theta)+5*math.sin(theta),
         10*math.sin(theta)-5*math.cos(theta), -5.)
    v = (-4*math.sin(theta)+2*math.cos(theta),
         4*math.cos(theta)+2*math.sin(theta), 0.)
    nav = [dict(valid=True, stamp=t, p=p, v=v) for t in (100.19, 100.21)]
    return nav, tracking


def test_exact_circular_rearpoint_motion_is_not_a_two_metre_per_second_error():
    nav, tracking = circular_histories()
    old = follow_settling(nav, tracking, 100.22)
    assert old['relative_speed'] == pytest.approx(2.)
    corrected = follow_settling(nav, tracking, 100.22, reference_mode='rear_point')
    assert corrected['settled']
    assert corrected['horizontal_error'] == pytest.approx(0., abs=1e-12)
    assert corrected['relative_speed'] == pytest.approx(0., abs=1e-10)
    assert corrected['target_relative_speed'] == pytest.approx(2.)
    assert corrected['turn_rate'] == pytest.approx(.4)
    assert corrected['los_yaw_rate'] == pytest.approx(.4)


def test_turn_derivative_unwraps_heading_without_a_fixed_axis_or_yaw_bias():
    nav, tracking = circular_histories(math.pi/2-.04)
    corrected = follow_settling(nav, tracking, 100.22, reference_mode='rear_point')
    assert corrected['turn_rate'] == pytest.approx(.4)
    assert corrected['relative_speed'] == pytest.approx(0., abs=1e-10)


@pytest.mark.parametrize('change', ['duplicate', 'short', 'old', 'invalid'])
def test_missing_distinct_causal_turn_history_fails_closed(change):
    nav, tracking = circular_histories()
    if change == 'duplicate':
        tracking = [dict(r, source=100.) for r in tracking]
    elif change == 'short':
        tracking = tracking[-2:]
    elif change == 'old':
        tracking = [dict(r, stamp=r['stamp']-1., source=r['source']-1.)
                    for r in tracking[:-1]]+tracking[-1:]
    else:
        tracking[-1]['valid'] = False
    result = follow_settling(nav, tracking, 100.22, reference_mode='rear_point')
    assert not result['settled']


def test_bracket_wait_and_expiry_stay_independent_of_turn_compensation():
    nav, tracking = circular_histories()
    nav = nav[:1]
    result = follow_settling(nav, tracking, 100.22, reference_mode='rear_point')
    assert result.get('awaiting_native_bracket')
    assert not result['settled']
    assert not follow_settling(nav, tracking, 100.4, reference_mode='rear_point')['settled']


def test_unrecognised_reference_mode_cannot_silently_select_a_weaker_gate():
    nav, tracking = circular_histories()
    with pytest.raises(ValueError, match='REFERENCE_MODE'):
        follow_settling(nav, tracking, 100.22, reference_mode='unknown')
    with pytest.raises(ValueError, match='REFERENCE_MODE'):
        FollowRolloutConfig(reference_mode='unknown').validate()


def test_rear_point_rollouts_share_the_existing_shaping_and_constant_jerk_response():
    raw = request(4.)
    times = np.asarray(raw.prediction_times)
    theta = .4*times
    positions = np.c_[10*np.cos(theta), 10*np.sin(theta), times*0.]
    velocities = np.c_[-4*np.sin(theta), 4*np.cos(theta), times*0.]
    raw = replace(raw, target_positions=tuple(map(tuple, positions)),
                  target_velocities=tuple(map(tuple, velocities)),
                  state=(10., -5., -5., 2., 4., 0., -1.6, .8, 0., math.pi/2))
    problem = FollowProblem(future_request(raw, .15), model(), 1.6)
    config = FollowRolloutConfig(reference_mode='rear_point')
    slow, fast = rollout_follow(problem, config), rollout_follow_fast(problem, config)
    np.testing.assert_allclose(fast.positions, slow.positions, atol=1e-12, rtol=0.)
    assert fast.metrics['reference_mode'] == 'rear_point'
    assert max(np.linalg.norm(np.asarray(fast.accelerations)[:, :2], axis=1)) <= 3.+1e-9
    assert all(abs(p[2]+5.) < 1e-9 for p in fast.positions)


def test_minco_visual_yaw_adds_only_fresh_common_epoch_los_rate():
    from types import SimpleNamespace
    from test_strict_visual_control import control_node
    from uav_control.mission.target_visibility import VisibilityDecision
    from uav_usv_interfaces.msg import MissionState
    n, state = control_node()
    n.mission_state = MissionState.FOLLOW
    n.follow_reference_mode = 'rear_point'
    n.follow_receiver = SimpleNamespace(active=object())
    n.visibility_decision = VisibilityDecision('TRACKING', True, True, 0., 0)
    n.visibility.last_valid_image_bearing = .2
    n.follow_metrics = dict(epoch=10.18, source_stamp=10.16, los_yaw_rate=.4)
    command = SimpleNamespace()
    n._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.6)
    n.follow_metrics['source_stamp'] = 10.
    n._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.2)
    n.follow_receiver.active = None
    n.follow_metrics['source_stamp'] = 10.16
    n._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.2)


def test_opt_in_yaw_envelope_preserves_the_base_contract_and_rejects_above_cap():
    from uav_control.controllers.follow_mpc_seed import MpcConfig
    from uav_control.guidance.follow_limits import ResearchConfig
    ResearchConfig(maximum_yaw_rate=2.5).validate()
    with pytest.raises(ValueError):
        ResearchConfig(maximum_yaw_rate=2.5001).validate()
    with pytest.raises(ValueError):
        MpcConfig(maximum_yaw_rate=2.5).validate()


def test_turn_fit_uses_acquisition_epochs_instead_of_kf_publication_jitter():
    nav, tracking = circular_histories()
    for row, delay in zip(tracking[:-1], (.019, .003, .017, .002)):
        row['stamp'] += delay
    result = follow_settling(nav, tracking, 100.22, reference_mode='rear_point')
    assert result['turn_rate'] == pytest.approx(.4)
    assert result['relative_speed'] == pytest.approx(0., abs=1e-10)


@pytest.mark.parametrize('motion', ['straight', 'turn', 'wrap', 'near_stop'])
def test_batched_rear_reference_matches_independent_scalar_fit_at_each_epoch(motion):
    from uav_control.guidance import follow_reference
    batch = getattr(follow_reference, 'forecast_rear_point_batch', None)
    assert callable(batch), 'The rollout needs an equivalent batched rear-point fit'
    raw = request(4.)
    ts = np.asarray(raw.prediction_times)
    speed = .1 if motion == 'near_stop' else 4.
    omega = 0. if motion == 'straight' else .4
    heading = (math.pi-.1 if motion == 'wrap' else .3)+omega*ts
    v = np.c_[speed*np.cos(heading), speed*np.sin(heading), ts*0.]
    raw = replace(raw, target_velocities=tuple(map(tuple, v)))
    problem = FollowProblem(future_request(raw, .15), model(), 1.6)
    query = np.r_[0., np.arange(.05, 1.6, .05), 1.6]
    p, velocity = batch(problem, query)
    scalar = [follow_reference.forecast_viewpoint(problem, float(t)) for t in query]
    np.testing.assert_allclose(p, [r[0] for r in scalar], atol=5e-11, rtol=0.)
    np.testing.assert_allclose(velocity, [r[1] for r in scalar], atol=5e-11, rtol=0.)
