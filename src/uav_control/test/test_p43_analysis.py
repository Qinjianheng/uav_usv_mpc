"""Publication attempts and legal deadline-compliant proposals are separate populations."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))


def test_publication_summary_does_not_count_crossed_deadline_as_legal():
    from p43_follow_analysis import publication_summary
    rows = [dict(published=True, whole_cycle_time=.05, remaining_raw_ttl=.04),
            dict(published=True, publication_crossed_deadline=True, whole_cycle_time=.13,
                 remaining_raw_ttl=-.01),
            dict(published=False, whole_cycle_time=.08, remaining_raw_ttl=.01)]
    s = publication_summary(rows)
    assert s['proposals'] == 2 and s['legal_proposals'] == 1
    assert s['crossed_deadline_proposals'] == 1
    assert s['final_legal_seconds']['max'] == .05
    assert s['final_published_seconds']['max'] == .13
    assert s['legal_publication_fraction'] == 1/3


def test_new_prediction_does_not_extend_original_navigation_deadline():
    from p43_follow_analysis import publication_input_ttl
    context = dict(cycle_id=4, navigation_stamp=100., attitude_stamp=100.,
                   observation_stamp=99.98, prediction_source_stamp=99.98,
                   prediction_valid_until=104., execution_start_stamp=100.15)
    rows = [dict(event='completion', request={'context': context}),
            dict(event='prediction_revalidation', cycle_id=4, report={'valid': True},
                 new_prediction=dict(observation_stamp=100.02, source_stamp=100.02,
                                     valid_until=104.)),
            dict(event='follow_proposal', identity=['receiver', 'planner', 1, 0, 4], stamp=100.11)]
    report = publication_input_ttl(rows)
    assert abs(report['p50']-.015) < 1e-12
    # Even a much newer target prediction cannot extend raw navigation/attitude freshness.
    rows[1]['new_prediction']['observation_stamp'] = 100.1
    rows[1]['new_prediction']['source_stamp'] = 100.1
    assert abs(publication_input_ttl(rows)['p50']-.015) < 1e-12
