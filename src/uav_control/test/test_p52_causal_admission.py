"""First actual P52 F2: fresh KF awaits a native right bracket, not a lost input."""
from uav_control.guidance.follow_admission import FollowAdmission, follow_settling


def histories():
    nav = [dict(valid=True, stamp=t, p=[0., 0., -5.], v=[2., 0., 0.])
           for t in [100.084, 100.104]]
    kf = [dict(valid=True, stamp=100.095, source=100., p=[5., 0., 0.], v=[2., 0., 0.]),
          dict(valid=True, stamp=100.106, source=100.05, p=[5., 0., 0.], v=[2., 0., 0.])]
    return nav, kf


def healthy():
    return dict(horizontal_error=0., relative_speed=0., height_error=0., settled=True)


def test_missing_right_bracket_defers_without_grant_or_false_loss_then_resumes():
    nav, kf = histories()
    gate = FollowAdmission()
    gate.stable_since, gate.last_now = 95., 100.1
    gate.probes, gate.last_probe = [1, 2, 3], 100.1
    pending = follow_settling(nav, kf, 100.126)
    assert pending.get('awaiting_native_bracket') and not pending['settled']
    assert gate.update(100.126, pending, True) == ''
    assert gate.stable_since == 95.
    assert not gate.converged(100.126) and not gate.allowed(100.126)
    nav.append(dict(nav[-1], stamp=100.128))
    matched = follow_settling(nav, kf, 100.13)
    assert matched['settled']
    gate.update(100.13, matched, True)
    assert gate.allowed(100.13)


def test_causal_wait_does_not_revoke_active_but_actual_expiry_does():
    nav, kf = histories()
    gate = FollowAdmission()
    gate.update(100.1, healthy(), True, active=True)
    pending = follow_settling(nav, kf, 100.126)
    assert gate.update(100.126, pending, True, active=True) == ''
    expired = follow_settling(nav, kf, 100.18)
    assert not expired.get('awaiting_native_bracket')
    assert gate.update(100.18, expired, True, active=True) == 'EXECUTION_CONTEXT_INVALID'


def test_actual_invalid_input_during_bracket_wait_resets_the_full_five_seconds():
    nav, kf = histories()
    gate = FollowAdmission()
    gate.stable_since, gate.last_now = 95., 100.1
    gate.update(100.126, follow_settling(nav, kf, 100.126), True)
    kf[-1]['valid'] = False
    gate.update(100.13, follow_settling(nav, kf, 100.13), True)
    assert gate.stable_since is None and not gate.converged(100.13)


def test_fresh_rolling_unbracketable_epochs_cannot_preserve_steady_qualification():
    gate = FollowAdmission()
    gate.stable_since, gate.last_now = 95., 100.
    gate.probes, gate.last_probe = [1, 2, 3], 100.
    nav, kf = [], []
    for step in range(101):
        now = 100.+step*.06
        nav.append(dict(valid=True, stamp=now-.02, p=[0., 0., -5.], v=[2., 0., 0.]))
        kf.append(dict(valid=True, stamp=now-.01, source=now-.03,
                       p=[5., 0., 0.], v=[2., 0., 0.]))
        pending = follow_settling(nav, kf, now)
        assert pending.get('awaiting_native_bracket')
        reason = gate.update(now, pending, True)
        if step:
            assert reason == 'NATIVE_BRACKET_TIMEOUT'
            assert gate.stable_since is None and not gate.probes
    nav.append(dict(nav[-1], stamp=106.001))
    matched = follow_settling(nav, kf, 106.002)
    assert matched['settled']
    gate.update(106.002, matched, True)
    assert gate.stable_since == 106.002
    assert not gate.converged(106.002)


def test_active_bracket_wait_has_fixed_fifty_ms_deadline_despite_new_epochs():
    gate = FollowAdmission()
    for step in range(7):
        now = 100.+step*.01
        pending = dict(settled=False, awaiting_native_bracket=True, epoch=now-.001)
        reason = gate.update(now, pending, True, active=True)
        assert reason == ('' if step <= 5 else 'NATIVE_BRACKET_TIMEOUT')


def test_successful_geometry_clears_wait_origin_for_a_later_short_wait():
    gate = FollowAdmission()
    pending = dict(settled=False, awaiting_native_bracket=True)
    gate.update(100., pending, True)
    gate.update(100.04, healthy(), True)
    assert gate.update(100.08, pending, True) == ''
    assert gate.update(100.12, pending, True) == ''


def test_late_successful_bracket_restarts_qualification_instead_of_hiding_timeout():
    gate = FollowAdmission()
    gate.stable_since = 95.
    gate.probes, gate.last_probe = [1, 2, 3], 100.
    gate.update(100., dict(awaiting_native_bracket=True), True)
    gate.update(100.06, healthy(), True)
    assert gate.stable_since == 100.06
    assert not gate.probes and not gate.allowed(100.06)


def test_late_successful_bracket_cannot_hide_active_observation_timeout():
    gate = FollowAdmission()
    gate.update(100., dict(awaiting_native_bracket=True), True, active=True)
    assert gate.update(100.06, healthy(), True, active=True) == 'NATIVE_BRACKET_TIMEOUT'
