"""P7.3 pairing is bounded, causal, and in the P7 operational NED frame."""


import numpy as np
import pytest


def module():
    from uav_control.analysis import observation_pairing
    return observation_pairing


def test_interpolation_requires_two_close_samples():
    interpolate = module().bounded_interpolate
    assert interpolate(1.05, [1.0, 1.1], [[0, 0, 0], [1, 0, 0]]) == (
        pytest.approx([0.5, 0, 0]))
    assert interpolate(1.2, [1.0, 1.1], [[0, 0, 0], [1, 0, 0]]) is None
    assert interpolate(1.05, [0.9, 1.1], [[0, 0, 0], [1, 0, 0]]) is None


def test_kf_pair_uses_only_previously_published_causal_state():
    states = [
        {'stamp': 1.0, 'source_stamp': 0.9, 'valid': True,
         'frame_id': 'local_ned', 'position_ned': [1, 2, 3],
         'velocity_ned': [2, 0, 0]},
        {'stamp': 1.1, 'source_stamp': 1.08, 'valid': True,
         'frame_id': 'local_ned', 'position_ned': [4, 2, 3],
         'velocity_ned': [2, 0, 0]},
    ]
    result, reason = module().causal_kf_at_image(states, 1.05)
    assert reason == ''
    assert result['position'] == pytest.approx([1.1, 2, 3])
    assert result['source_age_s'] == pytest.approx(0.15)
    assert result['status'] == 'PREDICT_ONLY_SINCE_PREVIOUS_PUBLISH'
    result, reason = module().causal_kf_at_image(states, 1.12)
    assert reason == ''
    assert result['status'] == 'UPDATED_SINCE_PREVIOUS_PUBLISH'
    states[1]['source_stamp'] = 1.2
    assert module().causal_kf_at_image(states, 1.12)[1] == 'KF_FUTURE_SOURCE'
    assert module().causal_kf_at_image(states, 1.3)[1] == 'KF_PUBLISH_GAP'


def test_raw_velocity_respects_reset_and_positive_image_dt():
    rows = [
        {'timestamp': 1.0, 'frame_valid': True, 'reset_epoch': 0,
         'raw_position_ned': [0, 0, 0]},
        {'timestamp': 1.1, 'frame_valid': True, 'reset_epoch': 0,
         'raw_position_ned': [0, 0.4, 0]},
        {'timestamp': 1.2, 'frame_valid': True, 'reset_epoch': 1,
         'raw_position_ned': [0, 0.8, 0]},
    ]
    values = module().finite_difference_velocity(rows, 'raw_position_ned')
    assert values[1.1] == pytest.approx([0, 4, 0])
    assert 1.2 not in values


def test_operational_target_anchor_and_velocity_construction():
    px4 = np.array([1.1, 2.2, -5.1])
    model = np.array([1.0, 2.0, -5.0])
    entity = np.array([8.0, 4.0, -0.42])
    operational = px4 + entity - model
    assert operational == pytest.approx([8.1, 4.2, -0.52])
    physical = entity
    assert operational - physical == pytest.approx(px4 - model)
    target_v = np.array([0.0, 4.0, 0.0])
    ekf_v = np.array([0.3, 0.2, 0.0])
    model_v = np.array([0.2, 0.0, 0.0])
    assert target_v + ekf_v - model_v == pytest.approx([0.1, 4.2, 0])


def test_lag_sweep_keeps_same_pairs_and_finds_known_shift():
    rows = []
    for index in range(120):
        stamp = 0.05 * index
        truth = [0.0, 4.0 * stamp, 0.0]
        rows.append({
            'timestamp': stamp,
            'truth': truth,
            'estimate': [0.0, 4.0 * (stamp + 0.05), 0.0],
        })
    sweep = module().lag_sweep(rows, 'estimate', 'truth')
    assert sweep['best']['lag_s'] == pytest.approx(0.05)
    assert sweep['best']['rmse_m'] < 1e-9
    assert len({item['n'] for item in sweep['curve']}) == 1
