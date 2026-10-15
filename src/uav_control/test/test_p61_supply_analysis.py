"""Clock-domain arithmetic and missing fields in actual supply diagnostics."""
import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))


def api():
    return importlib.import_module('p61_supply_analysis')


def test_cycle_separates_ros_epochs_from_monotonic_worker_cost():
    row = dict(event='completion', event_stamp=100.07, cycle_started_monotonic=9000.,
               request=dict(now_stamp=100.02, context=dict(
                   cycle_id=26, navigation_stamp=100.01, attitude_stamp=100.01,
                   observation_stamp=100., prediction_source_stamp=100.015,
                   prediction_valid_until=100.125, execution_start_stamp=100.17)),
               candidate_output=dict(solver_status='SAMPLED_FEASIBLE', solve_time=.04,
                                     timing=dict(validation=.02), metrics=dict(
                                         worker_timeline=dict(
                                             solve_start_ros=100.025, solve_end_ros=100.065,
                                             solve_start_monotonic=9000.005,
                                             solve_end_monotonic=9000.045))),
               output=dict(valid=True, solver_status='SAMPLED_FEASIBLE'),
               input_provenance=dict(prediction=dict(generated_stamp=100.016,
                                                     receipt_ros_stamp=100.019)))
    final = dict(stamp=100.08, wire_finish_ros=100.075, wire_cycle_seconds=.055,
                 published=True, reason='', whole_cycle_time=.06,
                 wire_finish_input_margin=.062)
    result = api().cycle_metrics(row, final, [])
    assert result['durations']['worker_wall_monotonic'] == pytest.approx(.04)
    assert result['durations']['request_to_worker_ros'] == pytest.approx(.005)
    assert result['durations']['acquisition_to_wire_ros'] == pytest.approx(.075)
    assert result['margins']['original_request_ttl_at_wire_ros'] == pytest.approx(.05)
    assert result['margins']['final_wire_input_margin_ros'] == pytest.approx(.062)
    assert result['durations']['prediction_transport_ros'] == pytest.approx(.003)
    assert result['clock_domains']['worker_wall_monotonic'] == 'monotonic'


def test_unknown_wire_or_cpu_time_stays_absent_and_expiry_is_a_failure():
    row = dict(request=dict(now_stamp=10., context=dict(cycle_id=80)),
               candidate_output=dict(metrics={}),
               output=dict(valid=False, solver_status='DEADLINE_EXCEEDED'), input_provenance={})
    result = api().cycle_metrics(row, None, [])
    assert result['durations']['worker_cpu_seconds'] is None
    assert result['durations']['acquisition_to_wire_ros'] is None
    assert result['failure'] == 'DEADLINE_EXCEEDED'
    assert api().statistics([.01, .02, None])['missing'] == 1
    assert api().statistics([.01, .02, None])['p95'] == pytest.approx(.0195)
