"""Extended production solver/receiver protocol, explicitly without flight or real-time proof."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


@pytest.mark.parametrize('fault', [
    'none', 'two_solve_failures', 'input_interruption', 'performance_deviation',
    'prefix_loss_reentry', 'prediction_change'])
def test_extended_replay_and_recovery(tmp_path, fault):
    from p52_receiver_replay import replay
    s = replay(tmp_path/fault, fault)
    assert s['not_sitl'] and s['hypothetical_only']
    assert s['expired_active_count'] == 0
    assert s['illegal_parent_activations'] == 0
    assert s['recovery_immediate_reentry_count'] == 0
    assert s['max_parent_pva_step'] < 1e-6
    assert s['max_reference_height_error'] < 1e-6
    assert s['maximum_command_step'] <= .15+1e-8
    assert s['maximum_command_speed'] <= 6.2+1e-8
    if fault in ('none', 'prediction_change'):
        assert s['successor_activations'] > 20
        assert s['fallback_count'] == 0
        assert s['successor_activations']+s['pending_successors'] == s['accepted_successors']
    elif fault == 'two_solve_failures':
        # Whether two misses fit depends on their phase within the unchanged lease.
        assert s['injected'] and s['fallback_count'] <= 1
        assert s['initial_entries'] == 1+s['fallback_count']
    else:
        assert s['injected'] and s['fallback_count'] == 1
        assert s['initial_entries'] == 2
        assert s['minimum_reentry_delay'] >= 7.-1e-8
    if fault == 'two_solve_failures':
        assert s['failed_solve_count'] == 2 and s['all_failures_kept_legal_old']
    if fault == 'prediction_change':
        assert s['changed_prediction_checks'] > 0
