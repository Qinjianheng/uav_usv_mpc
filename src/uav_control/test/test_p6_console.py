"""Motion selection is independent of mission commands and requires an ACK."""
import pytest


def test_console_mode_aliases_and_invalid_input():
    from uav_control.motion_mode_client import canonical_mode
    assert canonical_mode('2') == 'figure_eight'
    assert canonical_mode('1') == 'linear'
    assert canonical_mode('VARIABLE_SPEED_Z') == 'variable_speed_z'
    with pytest.raises(ValueError, match='INVALID_MOTION_MODE'):
        canonical_mode('6')


def test_mode_ack_requires_exact_success_and_requested_mode():
    from uav_control.motion_mode_client import validate_ack
    result = validate_ack(True, '{"mode":"figure_eight","mode_id":2}', '2')
    assert result['mode'] == 'figure_eight'
    with pytest.raises(ValueError, match='MODE_ACK_MISMATCH'):
        validate_ack(True, '{"mode":"linear"}', '2')
    with pytest.raises(ValueError, match='MODE_SELECTION_REJECTED'):
        validate_ack(False, 'MOTION_ALREADY_STARTED', '2')
