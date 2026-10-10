"""Arrival metrics use the real ACK callback epoch; a new forecast cannot refresh navigation."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def test_real_callback_age_keeps_original_raw_navigation_deadline():
    from p44_follow_analysis import tracker_arrival
    context = dict(cycle_id=3, navigation_stamp=10., attitude_stamp=10.,
                   observation_stamp=9.99, prediction_source_stamp=9.99)
    rows = [dict(event='completion', request=dict(context=context)),
            dict(event='prediction_revalidation', cycle_id=3, report=dict(valid=True),
                 new_prediction=dict(observation_stamp=10.1, source_stamp=10.1))]
    ack = dict(plan=3, stamp=10.12, receiver_compute_seconds=.01, state='REJECTED', reasons=[])
    report = tracker_arrival(rows, [ack])
    record = report['records'][0]
    assert abs(record['maximum_input_age']-.12) < 1e-9
    assert abs(record['remaining_input_ttl']-.005) < 1e-9
    assert record['remaining_after_compute'] < 0


def test_rejected_parent_output_without_metrics_is_retained():
    from p44_follow_analysis import candidate_summary
    row = dict(result=dict(solver_status='INPUT_REJECTED'), profile={})
    result = candidate_summary([row])
    assert result['count'] == 1 and result['strict_mean'] == 0.
    assert result['final_statuses'] == {'INPUT_REJECTED': 1}


def test_real_ack_union_preserves_missing_compute_measurement():
    from p44_follow_analysis import merge_acks
    identity = ['receiver', 'planner', 1, 0, 3]
    planner = dict(identity=identity, stamp=10.12, state='REJECTED', reasons=['HOLDING'])
    merged = merge_acks([], [planner])
    assert len(merged) == 1 and merged[0]['plan'] == 3
    assert merged[0]['receiver_compute_seconds'] is None
