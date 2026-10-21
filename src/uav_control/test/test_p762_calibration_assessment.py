"""Completed actions alone cannot grant measured dynamics qualification."""
from test_p761_calibration import script


def test_completed_unqualified_stage_is_not_success():
    analysis = script('p76_calibration_analysis')
    result = analysis.stage_verdict('A1', {'completed': True},
                                    {'checks': {}, 'A1_passed': False}, [], True, True)
    assert result['completed']
    assert not result['qualified']
    assert not result['success']


def test_qualification_requires_actual_landing_and_current_complete_evidence():
    analysis = script('p76_calibration_analysis')
    checks = {key: True for key in analysis.A1_REQUIRED}
    outcome = {'A1_passed': True, 'checks': checks}
    rows = [dict(kind='wire', phase='MANEUVER', topic='vehicle_status',
                 values={'arming_state': 2}),
            dict(kind='wire', phase='LAND', topic='vehicle_status', values={'arming_state': 1}),
            dict(kind='wire', phase='LAND', topic='vehicle_land_detected',
                 values={'landed': True})]
    good = analysis.stage_verdict('A1', {'completed': True}, outcome, rows, True, True)
    assert good['success'] and good['qualified']
    assert not analysis.stage_verdict('A1', {'completed': True}, outcome, rows[:-1],
                                      True, True)['success']
    assert not analysis.stage_verdict('A1', {'completed': True}, outcome, rows,
                                      False, True)['success']


def test_session_exit_rejects_completed_but_unqualified():
    session = script('p761_session')
    assert session.stage_exit_code(dict(completed=True, qualified=False, success=False)) == 2
    assert session.stage_exit_code(dict(completed=True, qualified=True, success=True)) == 0


def test_clock_reset_discards_old_native_epoch():
    from collections import deque
    import pytest
    from uav_control.common.p76_native import NativePoseHistory, NativeReferenceHistory
    calibration = script('p76_bodyrate_calibration')
    states = deque([(10., None, None, None)])
    poses, refs = NativePoseHistory(), NativeReferenceHistory()
    poses.add(10., [1., 0., 0., 0.], [0., 0., 0.], [0., 0., 0.])
    refs.add(10., 0.)
    with pytest.raises(ValueError, match='CLOCK_RESET'):
        calibration.check_native_epoch(9.99, 10., states, poses, refs)
    assert not states and not poses.samples and not refs.samples
