"""Real event epochs and coefficients; simulated disturbances remain explicitly labelled."""
import json
import gzip
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
from p53_event_replay import FAULTS, replay, replay_ack_delivery  # noqa: E402


def recorded():
    with gzip.open(Path(__file__).parent/'fixtures/p53_round2_events.json.gz', 'rt') as stream:
        return json.load(stream)


def test_recorded_lifecycle_keeps_start_missed_and_more_than_twenty_successors():
    value = replay(recorded())
    assert value['observed_sequence_matches']
    assert value['successor_activations'] > 20
    assert any('START_MISSED' in r['reasons'] for r in value['events'])


def test_real_ack_payloads_with_late_duplicates_do_not_roll_back_planner_owner():
    original = replay_ack_delivery(recorded())
    delayed = replay_ack_delivery(recorded(), late_duplicates=True)
    assert original['processed'] > 20
    assert original['final_active'] == delayed['final_active']
    assert delayed['injected_count'] > 0


@pytest.mark.parametrize('fault', FAULTS)
def test_recorded_curves_with_labelled_faults_never_execute_expired_or_illegal_parent(fault):
    value = replay(recorded(), fault)
    assert value['expired_active_count'] == value['illegal_parent_activations'] == 0
    assert value['hypothetical_only']
