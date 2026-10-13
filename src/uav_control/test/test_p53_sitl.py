"""P53 has a fresh one-shot namespace and cannot inherit stale flight readiness."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
import p53_minco_sitl as entry  # noqa: E402


def test_new_attempt_namespace_and_explicit_permission(tmp_path):
    assert entry.ROOT.name == '20261013_p53_closed_loop_01'
    with pytest.raises(ValueError, match='EXPLICIT_P53_SITL_AUTHORIZATION_REQUIRED'):
        entry.require_readiness(tmp_path/'absent.json', False)


def test_old_readiness_kind_is_not_p53_qualification(tmp_path):
    path = tmp_path/'readiness.json'
    path.write_text(json.dumps(dict(kind='P52_TESTED_PROTOCOL_CANDIDATE')))
    with pytest.raises(ValueError, match='P53_A_E_NOT_PASSED'):
        entry.require_readiness(path, True)


def test_source_edit_invalidates_new_readiness_before_any_attempt(tmp_path):
    source = tmp_path/'source.py'
    source.write_text('original')

    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    verification, qualification = tmp_path/'verification.json', tmp_path/'qualification.json'
    verification.write_text(json.dumps(dict(passed=True, independent_review_pass=True)))
    qualification.write_text('{}')
    path = tmp_path/'readiness.json'
    path.write_text(json.dumps(dict(
        kind=entry.KIND, a_e_pass=True,
        verification_sha256=digest(verification), qualification_sha256=digest(qualification),
        source_checks=[dict(path=str(source), sha256=digest(source))])))
    assert entry.require_readiness(path, True)
    source.write_text('changed')
    with pytest.raises(ValueError, match='READINESS_SOURCE_CHANGED'):
        entry.require_readiness(path, True)


def healthy(now=10., locked=True, owner='FOLLOW'):
    return dict(written=now, ros_now=now,
                control=dict(receipt=now, status=owner, locked=locked, compute=.001),
                navigation=dict(receipt=now, stamp=now-.01, valid=True,
                                p=[0., 0., -5.], v=[2., 0., 0.]),
                attitude=dict(receipt=now, q=[1., 0., 0., 0.]),
                vehicle=dict(receipt=now, nav_state=14, armed=2),
                prediction=dict(receipt=now, stamp=now-.02, observation=now-.05, valid=True))


def test_original_follow_one_frame_unlock_can_recover_without_stopping():
    guard = entry.FollowHealthGuard()
    assert guard(healthy(10., locked=False), 10.) is None
    assert guard(healthy(10.05), 10.05) is None
    assert guard(healthy(11., locked=False), 11.) is None


def test_unlocked_follow_wait_is_bounded_and_clock_reversal_fails_closed():
    guard = entry.FollowHealthGuard()
    assert guard(healthy(10., locked=False), 10.) is None
    assert guard(healthy(10.201, locked=False), 10.201) == 'FOLLOW_LOCK_RECOVERY_TIMEOUT'
    guard = entry.FollowHealthGuard()
    assert guard(healthy(10., locked=False), 10.) is None
    assert guard(healthy(9.9, locked=False), 9.9) == 'FOLLOW_LOCK_RECOVERY_CLOCK_REVERSED'


def test_wait_does_not_mask_physical_or_offboard_faults():
    for mutate, reason in (
            (lambda h: h['vehicle'].update(nav_state=3), 'OFFBOARD_OR_ARMING_LOST'),
            (lambda h: h['navigation'].update(valid=False), 'NAVIGATION_INVALID'),
            (lambda h: h['navigation'].update(p=[0., 0., -7.]), 'HEIGHT_ERROR_OVER_1M'),
            (lambda h: h['control'].update(receipt=9.), 'HEALTH_INPUT_LOST')):
        h = healthy(10., locked=False)
        mutate(h)
        assert entry.FollowHealthGuard()(h, 10.) == reason


def test_minco_unlock_and_wrong_owner_have_no_observer_grace():
    assert entry.FollowHealthGuard()(healthy(10., False, 'MINCO_FOLLOW'), 10.) == (
        'NO_LAWFUL_FOLLOW_PATH')
    assert entry.FollowHealthGuard()(healthy(10., False, 'STATE_STALE'), 10.) == (
        'NAVIGATION_INVALID')


def test_diagnostic_rounds_preserve_claims_and_reject_unbounded_runs(tmp_path, monkeypatch):
    monkeypatch.setattr(entry, 'ROOT', tmp_path/'20261013_p53_closed_loop_01')
    assert entry.round_root(1).name == '20261013_p53_closed_loop_01'
    assert entry.round_root(2).name == '20261013_p53_closed_loop_02'
    for bad in (0, 10, True, 1.5):
        with pytest.raises(ValueError, match='BOUNDED_P53_ROUND_REQUIRED'):
            entry.round_root(bad)


@pytest.mark.parametrize('recovered_at,reason', [
    (10.201, 'FOLLOW_LOCK_RECOVERY_TIMEOUT'),
    (9.9, 'FOLLOW_LOCK_RECOVERY_CLOCK_REVERSED'),
])
def test_late_or_backwards_locked_sample_cannot_erase_prior_wait(recovered_at, reason):
    guard = entry.FollowHealthGuard()
    assert guard(healthy(10., locked=False), 10.) is None
    assert guard(healthy(recovered_at), recovered_at) == reason


def test_initial_unlocked_follow_can_approach_until_original_settling_bound():
    guard = entry.FollowHealthGuard(initial_acquisition=True)
    h = healthy(10., locked=False)
    h['prediction']['valid'] = False
    h['follow_settling'] = dict(settled=True)
    assert guard(h, 10.) is None
    assert h['follow_settling']['settled'] is False
    assert guard(healthy(30., locked=False), 30.) is None
    assert guard(healthy(40.), 40.) is None
    assert guard(healthy(41., locked=False), 41.) is None
    assert guard(healthy(41.201, locked=False), 41.201) == 'FOLLOW_LOCK_RECOVERY_TIMEOUT'


@pytest.mark.parametrize('now,reason', [
    (70.001, 'INITIAL_FOLLOW_ACQUISITION_TIMEOUT'),
    (9.9, 'INITIAL_FOLLOW_ACQUISITION_CLOCK_REVERSED'),
])
def test_initial_wait_checks_deadline_before_accepting_late_lock(now, reason):
    guard = entry.FollowHealthGuard(initial_acquisition=True)
    assert guard(healthy(10., locked=False), 10.) is None
    assert guard(healthy(now), now) == reason


def test_initial_acquisition_keeps_all_other_safety_checks_and_minco_strict():
    guard = entry.FollowHealthGuard(initial_acquisition=True)
    h = healthy(10., locked=False)
    h['vehicle']['nav_state'] = 3
    assert guard(h, 10.) == 'OFFBOARD_OR_ARMING_LOST'
    assert entry.FollowHealthGuard(initial_acquisition=True)(
        healthy(10., False, 'MINCO_FOLLOW'), 10.) == 'NO_LAWFUL_FOLLOW_PATH'


def test_bounded_f3_retry_requires_original_actual_f2_pass_and_prior_attempt(
        tmp_path, monkeypatch):
    monkeypatch.setattr(entry, 'ROOT', tmp_path/'20261013_p53_closed_loop_01')
    base = entry.round_root(3)
    base.mkdir()
    with pytest.raises(ValueError, match='PRIOR_F3_ATTEMPT_REQUIRED'):
        entry.f3_retry_root(3)
    (base/'attempt_f3.json').write_text('{}')
    (base/'f2_summary.json').write_text(json.dumps(dict(completed=False)))
    with pytest.raises(ValueError, match='F2_SUSTAINED_HEALTHY_EXECUTION_REQUIRED'):
        entry.f3_retry_root(3)
    good = dict(
        completed=True, healthy=True, minco_fraction=1., longest_minco_seconds=25.,
        observation_seconds=25., diagnostic_coverage=1.,
        p51_audit=dict(passed=True, checks=dict(
            reference_height=True, entry_recovery=True, execution_gates=True,
            evidence_coverage=True, switching=True, handovers=True)))
    (base/'f2_summary.json').write_text(json.dumps(good))
    retry = entry.f3_retry_root(3)
    assert retry.name == 'f3_retry_02'
    assert json.loads((retry/'f2_summary.json').read_text()) == good
    (retry/'attempt_f3.json').write_text('{}')
    with pytest.raises(ValueError, match='F3_RETRY_ALREADY_CONSUMED'):
        entry.f3_retry_root(3)


def test_ground_recorder_startup_grace_expires_and_ends_before_x(tmp_path):
    guard = entry.GroundStartupHealth(tmp_path)
    h = dict(written=100., control=dict(status='INIT'), vehicle=dict(armed=1))
    assert guard(None, h, 100.3, 99.) is None
    assert guard(None, h, 105.301, 99.) == 'HEALTH_MONITOR_STALLED'
    guard = entry.GroundStartupHealth(tmp_path)
    assert guard(None, h, 100.3, 99.) is None
    (tmp_path/'X_command.txt').touch()
    assert guard(None, h, 100.4, 99.) == 'HEALTH_MONITOR_STALLED'


def test_startup_grace_never_masks_exit_armed_flight_or_future_record(tmp_path):
    for patch in (dict(vehicle=dict(armed=2)), dict(control=dict(status='TAKEOFF')),
                  dict(control=dict(status='FOLLOW')), dict(written=100.4)):
        h = dict(written=100., control=dict(status='INIT'), vehicle=dict(armed=1))
        h.update(patch)
        assert entry.GroundStartupHealth(tmp_path)(None, h, 100.3, 99.) == (
            'HEALTH_MONITOR_STALLED')
    assert entry.GroundStartupHealth(tmp_path)(1, dict(written=100.), 100.3, 99.) == (
        'HEALTH_MONITOR_EXITED')
    assert entry.GroundStartupHealth(tmp_path)(None, {}, 102., 99.) == (
        'HEALTH_MONITOR_NO_FIRST_SAMPLE')


def test_only_a_verified_pre_x_stall_allows_one_ground_retry(tmp_path, monkeypatch):
    monkeypatch.setattr(entry, 'ROOT', tmp_path/'20261013_p53_closed_loop_01')
    base = entry.round_root(3)
    base.mkdir()
    (base/'attempt_f3.json').write_text('{}')
    # Preserve the real F2 gate; use the same helper-tested complete audit shape.
    checks = dict(reference_height=True, entry_recovery=True, execution_gates=True,
                  evidence_coverage=True, switching=True, handovers=True)
    good = dict(completed=True, healthy=True, minco_fraction=1., longest_minco_seconds=25.,
                observation_seconds=25., diagnostic_coverage=1.,
                p51_audit=dict(passed=True, checks=checks))
    (base/'f2_summary.json').write_text(json.dumps(good))
    retry2 = entry.f3_retry_root(3)
    (retry2/'attempt_f3.json').write_text('{}')
    source = retry2/'f3_h16_01'
    source.mkdir()
    (source/'bounded_stop.json').write_text(json.dumps(dict(reason='HEALTH_MONITOR_STALLED')))
    (source/'watchdog_end.json').write_text(json.dumps(dict(x_time=10.)))
    with pytest.raises(ValueError, match='PRE_X_RECORDER_STALL_REQUIRED'):
        entry.f3_retry_root(3, 3)
    (source/'watchdog_end.json').write_text('{}')
    with pytest.raises(ValueError, match='PRE_X_RECORDER_STALL_REQUIRED'):
        entry.f3_retry_root(3, 3)
    (source/'watchdog_end.json').write_text(json.dumps(dict(x_time=None)))
    (source/'X_command.txt').touch()
    with pytest.raises(ValueError, match='PRE_X_RECORDER_STALL_REQUIRED'):
        entry.f3_retry_root(3, 3)
    (source/'X_command.txt').unlink()
    retry3 = entry.f3_retry_root(3, 3)
    assert retry3.name == 'f3_retry_03'
    (retry3/'attempt_f3.json').write_text('{}')
    source3 = retry3/'f3_h16_01'
    source3.mkdir()
    (source3/'bounded_stop.json').write_text(json.dumps(dict(reason='HEALTH_MONITOR_STALLED')))
    (source3/'watchdog_end.json').write_text(json.dumps(dict(x_time=None)))
    (source3/'X_command.txt').touch()
    with pytest.raises(ValueError, match='PRE_X_RECORDER_STALL_REQUIRED'):
        entry.f3_retry_root(3, 4)
    (source3/'X_command.txt').unlink()
    assert entry.f3_retry_root(3, 4).name == 'f3_retry_04'
    with pytest.raises(ValueError, match='BOUNDED_F3_ATTEMPT_REQUIRED'):
        entry.f3_retry_root(3, 5)


def test_health_recorder_waits_for_actual_experiment_start_marker(tmp_path):
    log = tmp_path/'launcher.txt'
    marker = 'Starting UAV-USV experiment...'
    assert not entry.harness.health_start_allowed(tmp_path, marker)
    log.write_text('Starting Gazebo...\nStarting PX4 SITL...\n')
    assert not entry.harness.health_start_allowed(tmp_path, marker)
    log.write_text('Starting Gazebo...\nStarting UAV-USV experiment...\n')
    assert entry.harness.health_start_allowed(tmp_path, marker)


def test_legacy_health_recorder_start_policy_remains_immediate(tmp_path):
    assert entry.harness.health_start_allowed(tmp_path, None)
