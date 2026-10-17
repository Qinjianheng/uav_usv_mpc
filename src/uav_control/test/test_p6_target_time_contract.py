"""Experimental target time never renews native pose or receiver authority."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from uav_control.guidance.follow_time_contract import input_time_contract, context_deadline
from uav_control.guidance.follow_limits import ResearchConfig
from uav_control.tracking.imm_target_predictor_node import ImmPredictionEngine


def context(profile='baseline_125'):
    return SimpleNamespace(navigation_stamp=100.1, attitude_stamp=100.1,
                           observation_stamp=100., prediction_source_stamp=100.05,
                           prediction_valid_until=100.2, target_time_profile=profile)


def test_explicit_target_profiles_preserve_independent_native_bounds():
    old = input_time_contract('baseline_125')
    exp = input_time_contract('experiment_200')
    assert old.observation_age == old.prediction_age == .125
    assert exp.observation_age == exp.prediction_age == .2
    assert exp.navigation_age == exp.attitude_age == .125
    assert context_deadline(context()) == 100.125
    assert context_deadline(context('experiment_200')) == 100.2
    c = context('experiment_200')
    c.navigation_stamp = c.attitude_stamp = 100.
    assert context_deadline(c) == 100.125
    c = context('experiment_200')
    c.prediction_valid_until = 100.12
    assert context_deadline(c) == 100.12


def test_prediction_state_age_and_acquisition_age_have_distinct_deadlines():
    c = context('experiment_200')
    c.navigation_stamp = c.attitude_stamp = 100.2
    c.prediction_source_stamp = 99.98
    assert context_deadline(c) == pytest.approx(100.18)


def test_legacy_context_shape_and_experimental_policy_roundtrip():
    from dataclasses import asdict
    from test_follow_research_algorithms import request
    from uav_control.controllers.follow_mpc_seed import planning_context_from_payload
    original = asdict(request(2.).context)
    assert 'target_time_profile' not in original
    assert asdict(planning_context_from_payload(original)) == original
    experimental = dict(original, target_time_profile='experiment_200')
    assert asdict(planning_context_from_payload(experimental)) == experimental


def test_unknown_profile_and_implicit_imm_timeout_extension_are_rejected():
    with pytest.raises(ValueError):
        input_time_contract('unreviewed')
    with pytest.raises(ValueError):
        ImmPredictionEngine(input_timeout=.2)
    with pytest.raises(ValueError):
        ImmPredictionEngine(input_timeout=.125, target_time_profile='experiment_200')
    with pytest.raises(ValueError):
        replace(ResearchConfig(), target_time_profile='unreviewed').validate()
    replace(ResearchConfig(), target_time_profile='experiment_200').validate()
    with pytest.raises(ValueError):
        replace(ResearchConfig(), target_time_profile='experiment_200',
                maximum_input_age=.2).validate()


def test_200ms_imm_propagates_covariance_without_retiming_or_renewal():
    import numpy as np
    from test_p6_imm_node import observation
    from uav_control.tracking.imm_target_predictor_node import (
        observation_from_message, prediction_to_message,
    )
    from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
    engine = ImmPredictionEngine(input_timeout=.2, target_time_profile='experiment_200')
    assert engine.update(observation_from_message(observation()), 10.02).accepted
    early = engine.generate(10.02, 1, 1)
    late = engine.generate(10.199, 1, 2)
    assert late.valid and len(late.samples) == 41
    assert late.observation_stamp == 10.
    assert late.source_stamp == late.generated_stamp == 10.199
    assert late.valid_until == 10.2
    assert np.trace(np.asarray(late.samples[0].position_covariance).reshape(3, 3)) > np.trace(
        np.asarray(early.samples[0].position_covariance).reshape(3, 3))
    assert all(np.min(np.linalg.eigvalsh(np.asarray(s.position_covariance).reshape(3, 3)))
               >= -1e-9 for s in late.samples)
    parsed = prediction_from_message(prediction_to_message(late, 0.))
    assert parsed.source_stamp == late.source_stamp and parsed.observation_stamp == 10.
    expired = engine.generate(10.201, 1, 3)
    assert not expired.valid and not expired.samples
    assert expired.invalid_reason == 'OBSERVATION_STALE'


def test_receiver_uses_local_time_policy_and_keeps_native_expiry():
    from test_follow_contract import curve
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_limits import constraint_snapshot
    from uav_control.guidance.follow_contract import curve_errors
    limits = constraint_snapshot(FollowMpcSeed(ResearchConfig(
        target_time_profile='experiment_200')))
    c = replace(curve(start=100.19), navigation_stamp=100.1, attitude_stamp=100.1,
                observation_stamp=100., source_stamp=100.1, input_until=100.2,
                constraint_snapshot=limits.payload, constraint_fingerprint=limits.fingerprint)
    assert 'TTL_EXTENSION' in curve_errors(c)
    assert 'TTL_EXTENSION' not in curve_errors(c, limits)
    stale_nav = replace(c, navigation_stamp=100., attitude_stamp=100.)
    assert 'TTL_EXTENSION' in curve_errors(stale_nav, limits)
    baseline = constraint_snapshot(FollowMpcSeed(ResearchConfig()))
    assert 'CONSTRAINT_MISMATCH' in curve_errors(c, baseline)


def test_exact_case_requires_three_way_time_agreement():
    from test_p6_sitl_driver import driver
    p = dict(target_time_profile='experiment_200', maximum_input_age=.125)
    t = dict(follow_target_time_profile='experiment_200',
             follow_admission_target_observation_age=.2)
    i = dict(target_time_profile='experiment_200', input_timeout=.2)
    assert driver().time_contract_for_configs(p, t, i).navigation_age == .125
    for bad in (dict(i, input_timeout=.125), dict(i, target_time_profile='baseline_125')):
        with pytest.raises(ValueError, match='CONTRACT_MISMATCH'):
            driver().time_contract_for_configs(p, t, bad)
    with pytest.raises(ValueError, match='CONTRACT_MISMATCH'):
        driver().time_contract_for_configs(
            p, dict(t, follow_target_time_profile='baseline_125'), i)


def test_kf_source_can_age_200_but_propagated_state_cannot_age_200():
    from uav_control.control.trajectory_tracker_node import fresh_flight_target
    from uav_control.controllers.follow_transport import stamp
    from uav_usv_interfaces.msg import TargetState
    m = TargetState()
    m.valid, m.frame_id = True, 'local_ned'
    m.source_stamp, m.stamp = stamp(100.), stamp(100.1)
    assert fresh_flight_target(m, 100.18, .125) is None
    assert fresh_flight_target(m, 100.18, .125, source_max_age=.2) is not None
    m.stamp = stamp(100.)
    assert fresh_flight_target(m, 100.18, .125, source_max_age=.2) is None


def experimental_tracker():
    from test_p53_input_roles import node_context
    from uav_control.controllers.follow_transport import stamp
    n = node_context()
    n.follow_time_contract = input_time_contract('experiment_200')
    n._ros_seconds = lambda: 100.18
    n.latest_state.stamp = 100.17
    n.latest_kf_message.stamp, n.latest_kf_message.source_stamp = stamp(100.17), stamp(100.)
    p = n.latest_prediction
    p.observation_stamp, p.source_stamp = stamp(100.), stamp(100.1)
    p.generated_stamp, p.valid_until = stamp(100.1), stamp(100.2)
    n.visibility.last_valid_observation_stamp = 100.1
    return n


def test_actual_tracker_context_revokes_target_after_200ms_without_changing_lease():
    from test_follow_sitl_execution import receiver, candidate
    n = experimental_tracker()
    c = n._follow_context()
    assert c.navigation_fresh and c.target_fresh and c.prediction_fresh and c.observation_fresh
    r = receiver()
    r.active, r.deadline = candidate(), 100.5
    n._ros_seconds = lambda: 100.201
    c = n._follow_context()
    assert c.navigation_fresh and c.observation_fresh
    assert not c.target_fresh and not c.prediction_fresh
    ack = r.tick(c, nominal_valid=True)
    assert ack.state == 'REVOKED'
    assert r.active is None and r.pending is None and r.deadline == 100.5


def test_actual_tracker_native_and_visual_gates_stay_independent_of_target_experiment():
    n = experimental_tracker()
    n.latest_state.stamp = 100.
    assert not n._follow_context().navigation_fresh
    n.latest_state.stamp = 100.17
    n.visibility.last_valid_observation_stamp = 100.
    c = n._follow_context()
    assert c.navigation_fresh and c.prediction_fresh
    assert not c.observation_fresh
