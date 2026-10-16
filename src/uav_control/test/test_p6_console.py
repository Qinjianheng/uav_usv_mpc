"""Motion ACK and actual console X gating remain required with IMM-only prediction."""

from pathlib import Path
import subprocess

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


@pytest.mark.parametrize('reason', ('{}', '[]', 'not-json'))
def test_malformed_mode_ack_cannot_enable_x(reason):
    from uav_control.motion_mode_client import validate_ack
    with pytest.raises(ValueError, match='INVALID_MODE_ACK'):
        validate_ack(True, reason, '2')


@pytest.mark.parametrize('ack_exit,expected', ((0, True), (1, False)))
def test_actual_console_publishes_x_only_after_mode_ack(ack_exit, expected):
    script = Path(__file__).resolve().parents[3] / 'scripts/start_px4_ros2.sh'
    console = script.read_text().split('motion_reset_pending=0\n', 1)[1]
    # Run the real console case logic, with only the external service process replaced.
    prefix = ('motion_reset_pending=0\n'
              f'timeout() {{ return {ack_exit}; }}\n'
              'publish_command() { echo "PUBLISHED_$1"; }\n')
    result = subprocess.run(['bash', '-c', prefix + console], input='X\nQ\n',
                            text=True, capture_output=True, timeout=2.)
    assert result.returncode == 0
    assert ('PUBLISHED_X' in result.stdout) is expected
    if not expected:
        assert 'X blocked: USV motion mode not acknowledged.' in result.stderr
