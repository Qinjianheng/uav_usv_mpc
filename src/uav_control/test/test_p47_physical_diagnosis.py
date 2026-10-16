"""Offline epoch and vector attribution safeguards, independent of ROS/SITL."""

import numpy as np
import pytest


def module():
    from uav_control.analysis import offline_geometry
    return offline_geometry


def test_interpolation_requires_brackets_and_rejects_gaps_and_resets():
    m = module()
    assert m.interpolate([1., 1.1], [[2.], [4.]], 1.05, .15)[0] == pytest.approx(3.)
    for query in (.99, 1.11):
        with pytest.raises(ValueError, match='OUTSIDE_SUPPORT'):
            m.interpolate([1., 1.1], [[2.], [4.]], query, .15)
    with pytest.raises(ValueError, match='GAP'):
        m.interpolate([1., 2.], [[2.], [4.]], 1.5, .15)
    with pytest.raises(ValueError, match='MONOTONIC'):
        m.interpolate([1., 1.], [[2.], [4.]], 1., .15)


def test_native_epoch_mapping_uses_original_sample_pairs_and_never_receipt():
    m = module()
    nav = [dict(native=1000000, stamp=100., receipt=800., valid=True),
           dict(native=1020000, stamp=100.021, receipt=900., valid=True)]
    mapping = m.navigation_mapping(nav)
    assert m.map_native(mapping, 1010000) == pytest.approx(100.0105)
    with pytest.raises(ValueError, match='OUTSIDE_SUPPORT'):
        m.map_native(mapping, 999999)
    nav[1]['native'] = 900000
    with pytest.raises(ValueError, match='MONOTONIC'):
        m.navigation_mapping(nav)


def test_vector_decomposition_includes_cross_term_and_covariance():
    m = module()
    reference = np.array([[1., 0., 2.], [-1., 0., 0.]])
    tracking = -reference / 2
    actual = reference + tracking
    report = m.decompose(reference, tracking, actual)
    for group in ('xy', 'z', 'xyz'):
        r = report[group]
        assert r['actual_mse'] == pytest.approx(
            r['reference_mse'] + r['tracking_mse'] + r['cross_term'])
        assert r['closure_max_abs'] == pytest.approx(0.)
        assert r['cross_term'] < 0
    assert report['z']['centered_cross_covariance'] == pytest.approx(-.5)


def test_desired_is_rear_five_metres_with_fixed_ned_altitude():
    m = module()
    assert m.desired_point([10., 20., .15], [0., 2., .1]).tolist() == [10., 15., -5.]
    with pytest.raises(ValueError, match='HEADING_UNDEFINED'):
        m.desired_point([10., 20., 0.], [0., 0., 0.])


def test_polynomial_sampler_checks_support_and_derivatives():
    m = module()
    xyz = np.zeros((1, 6, 3))
    xyz[0, 0] = [1., 2., -5.]
    xyz[0, 1] = [2., 0., .5]
    xyz[0, 2, 2] = .1
    proposal = dict(start=10., end=11., durations=[1.], xyz=xyz.ravel().tolist())
    sample = m.sample_curve(proposal, 10.5)
    assert sample[:3].tolist() == pytest.approx([2., 2., -4.725])
    assert sample[3:6].tolist() == pytest.approx([2., 0., .6])
    for epoch in (9.99, 11.01):
        with pytest.raises(ValueError, match='OUTSIDE_SUPPORT'):
            m.sample_curve(proposal, epoch)


def test_callback_pairing_uses_header_epochs_and_rejects_cross_cycle():
    m = module()
    diag = [dict(stamp=10., compute=.02, status='MINCO_FOLLOW', plan=7),
            dict(stamp=10.05, compute=.02, status='FOLLOW', plan=0)]
    commands = [dict(stamp=10.0001, v=[1., 0., .2], receipt=99.),
                dict(stamp=10.0501, v=[2., 0., 0.], receipt=1.)]
    debug = dict(stamp=10.01, plan=7)
    tick, command, index = m.pair_callback(debug, diag, commands)
    assert index == 0 and tick is diag[0] and command is commands[0]
    with pytest.raises(ValueError, match='CALLBACK'):
        m.pair_callback(dict(stamp=10.04, plan=7), diag, commands)


def test_native_command_correspondence_requires_causal_header_and_bounds():
    m = module()
    commands = [dict(stamp=10., v=[1., 0., .2]), dict(stamp=10.05, v=[1., 0., .3])]
    observed = [dict(dataset='trajectory_setpoint', epoch=10.04, vx=1., vy=0., vz=.2),
                dict(dataset='trajectory_setpoint', epoch=10.04, vx=1., vy=0., vz=.3),
                dict(dataset='trajectory_setpoint', epoch=10.30, vx=1., vy=0., vz=.2)]
    result = m.native_correspondence(observed, commands)
    assert result[0]['exact_vector_matches'] == 1
    assert result[1]['exact_vector_matches'] == 0  # future command cannot explain output
    assert result[2]['causal_candidates'] == 0  # stale vector not accepted
