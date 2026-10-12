"""Real solver/transport/receiver chain on explicitly hypothetical co-moving geometry."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


@pytest.mark.parametrize('fault', [
    'none', 'one_solve_failure', 'no_successor', 'bad_parent', 'prefix_invalid',
    'offboard_lost', 'clock_reversed', 'start_missed'])
def test_receiver_replay_has_no_expired_active_and_keeps_real_failure_causes(tmp_path, fault):
    from p51_receiver_replay import replay
    result = replay(tmp_path/fault, fault)
    assert result['hypothetical_only']
    assert result['expired_active_count'] == 0
    assert result['active_events'] > 0
    assert result['max_parent_pva_step'] < 1e-6
    assert result['max_reference_height_error'] < 1e-6
    if fault == 'none':
        assert result['fallback_count'] == 0
        assert result['active_events'] >= 5
    elif fault == 'one_solve_failure':
        assert result['fallback_count'] == 0
        assert result['failed_solve_kept_old']
    else:
        assert result['injected']
        assert result['recovery_immediate_reentry_count'] == 0
