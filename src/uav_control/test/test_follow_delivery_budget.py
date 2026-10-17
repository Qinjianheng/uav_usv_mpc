"""Real delivery must price the executed path and preserve original acquisition deadlines."""
from dataclasses import FrozenInstanceError

import pytest

from uav_control.guidance.follow_delivery_budget import DeliveryCosts, make_plan, tail_reserve


def plan(**changes):
    values = dict(identity=(1, 0, 23), now=1792204362.0632136,
                  input_until=1792204362.1310284, start=1792204362.2108872,
                  parent_id=22, parent_deadline=1792204362.4520855,
                  costs={'follow': .015, 'fallback': .0349580928}, queue=.020,
                  publication=.010, receiver=.020)
    values.update(changes)
    return make_plan(**values)


def test_recorded23_locks_only_affordable_follow_path():
    p = plan()
    assert p.path == 'follow'
    assert p.worker_hint == .015
    assert p.worker_until == pytest.approx(1792204362.1010284)
    assert p.wire_until == pytest.approx(1792204362.1110284)
    assert p.accept_until == 1792204362.1310284
    assert p.available_worker == pytest.approx(.0178148, abs=1e-6)
    with pytest.raises(FrozenInstanceError):
        p.path = 'fallback'


def test_larger_internal_remaining_does_not_authorize_another_path():
    p = plan()
    assert p.accepts('follow', p.worker_until-.001)
    assert not p.accepts('fallback', p.worker_until-.001)
    assert not p.accepts('follow', p.worker_until)


def test_no_fitting_path_rejects_before_worker_and_retry_consumption():
    with pytest.raises(ValueError, match='SUCCESSOR_AWAITING_FRESH_INPUT'):
        plan(costs={'follow': .023, 'fallback': .035})


def test_handover_and_parent_cutoff_cannot_be_bypassed_by_larger_ttl():
    with pytest.raises(ValueError, match='NO_SUCCESSOR_ACTIVATION_MARGIN'):
        plan(start=1792204362.4020855)
    p = plan(input_until=1792204362.300, start=1792204362.2108872)
    assert p.accept_until == p.start


@pytest.mark.parametrize('key,value', [
    ('queue', -.1), ('receiver', float('nan')), ('input_until', float('inf'))])
def test_invalid_or_nonfinite_budget_is_rejected(key, value):
    with pytest.raises(ValueError):
        plan(**{key: value})


def test_failed_deadline_and_fallback_costs_affect_delivery_risk():
    costs = DeliveryCosts()
    costs.observe('fallback', .027, 100., valid=True, status='SAMPLED_FEASIBLE')
    before = costs.hints(100.05)['fallback']
    costs.observe('fallback', .055, 100.05, valid=False, status='DEADLINE_EXCEEDED')
    assert costs.hints(100.1)['fallback'] > before
    assert costs.report(100.1)['fallback']['failed_count'] == 1
    assert costs.report(100.1)['fallback']['censored_count'] == 1
    assert costs.hints(104.)['fallback'] == .012


def test_clock_reversal_clears_both_cost_populations():
    costs = DeliveryCosts()
    costs.observe('follow', .060, 100., valid=False, status='DEADLINE_EXCEEDED')
    assert costs.hints(100.1)['follow'] > .060
    assert costs.hints(99.) == {'follow': .015, 'fallback': .012}


def test_receiver_and_completion_tail_reserves_include_failures_without_freezing_forever():
    samples = [(100., .055)]
    assert tail_reserve(samples, 100.1, .020, .005) == pytest.approx(.071)
    assert tail_reserve(samples, 104., .020, .005) == .020
    assert tail_reserve(samples, 99., .020, .005) == .020
