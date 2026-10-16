"""Dispatch on newly received predictions without changing raw epochs or authority."""
from concurrent.futures import Future
from dataclasses import replace
from functools import partial

import pytest

from test_follow_mpc_shadow_node import _mission, _pose, _prediction, _Pool, _time
from uav_control.controllers.follow_mpc_seed import MpcConfig, MpcSeedResult
from uav_control.controllers.follow_mpc_shadow_node import make_shadow_request
from uav_control.controllers.mpc_shadow_inputs import PairResult
from uav_control.controllers.p4_follow_planner_node import P4ResearchRunner


class PairedNavigation:
    """Supply deterministic native navigation at the test's explicit acquired epoch."""

    def __init__(self):
        self.clock_generation = 2
        self.pose = _pose()

    def pair_latest(self, now, mono, observation_stamp):
        return PairResult('PAIRED', self.pose)


def runner():
    pool, events, adapter = _Pool(), [], PairedNavigation()
    r = P4ResearchRunner(MpcConfig(), adapter, pool, events.append, rate_hz=5.,
                         request_factory=partial(make_shadow_request, execution_lead=.15))
    r.deadline_scheduling = True
    r.execution_status = None
    r.update_mission(_mission(), 100.01, 10.01)
    receive(r, 100., 100.035, 10.035, 1)
    return r, pool, events


def receive(r, acquired, receipt_ros, receipt_mono, sequence):
    r.adapter.pose = replace(r.adapter.pose, stamp=acquired+.02,
                             observation_stamp=acquired,
                             clock_generation=r.adapter.clock_generation)
    r.update_prediction(_prediction(model='imm', sequence_id=sequence,
                        observation_stamp=_time(acquired), source_stamp=_time(acquired+.03),
                        generated_stamp=_time(acquired+.03), valid_until=_time(acquired+.125)),
                        receipt_ros, receipt_mono)


def finish(r, pool, ros=100.09, mono=10.09, valid=True):
    pool.future.set_result(MpcSeedResult(pool.requests[-1].context, valid=valid))
    r.tick(ros, mono)
    pool.future = Future()


def test_regular_dispatch_uses_callback_phase_instead_of_aged_timer_phase():
    r, pool, _ = runner()
    # A 77ms-old request is the early phase seen in the second real F2.
    r.tick(100.077, 10.077)
    finish(r, pool)
    receive(r, 100.2, 100.235, 10.235, 2)
    r.tick(100.24, 10.24)
    assert len(pool.requests) == 2
    req = pool.requests[-1]
    assert req.now_stamp == 100.24
    assert req.context.observation_stamp == 100.2
    assert req.context.prediction_valid_until == 100.325
    assert req.now_stamp-req.context.observation_stamp == pytest.approx(.04)


def test_elapsed_cooldown_waits_for_input_received_in_the_new_period():
    r, pool, _ = runner()
    r.tick(100.077, 10.077)
    finish(r, pool)
    receive(r, 100.16, 100.195, 10.195, 2)
    # Old code prepares and submits at 10.28 despite no new-period callback.
    r.tick(100.28, 10.28)
    assert len(pool.requests) == 1
    receive(r, 100.25, 100.285, 10.285, 3)
    r.tick(100.29, 10.29)
    assert len(pool.requests) == 2
    assert pool.requests[-1].context.observation_stamp == 100.25


def test_dispatch_audit_keeps_prepared_receipt_when_latest_prediction_changes():
    r, pool, events = runner()
    r.tick(100.077, 10.077)
    receive(r, 100.05, 100.085, 10.085, 2)
    finish(r, pool, ros=100.10, mono=10.10)
    event = events[-1]
    timing = event['dispatch_timing']
    assert timing['policy'] == 'input_aligned'
    assert timing['prediction_sequence_id'] == 1
    assert timing['prediction_observation_stamp'] == 100.
    assert timing['prediction_receipt_ros_stamp'] == 100.035
    assert timing['prediction_receipt_monotonic'] == 10.035
    assert timing['input_receipt_to_dispatch_seconds'] == pytest.approx(.042)
    assert event['input_provenance']['prediction']['sequence_id'] == 2


def test_new_callback_cannot_refresh_stale_acquisition():
    r, pool, events = runner()
    r.tick(100.077, 10.077)
    finish(r, pool)
    receive(r, 100., 100.285, 10.285, 2)
    r.tick(100.29, 10.29)
    assert len(pool.requests) == 1
    assert events[-1]['output']['reason'] == 'STALE_INPUT'


def test_completed_worker_is_consumed_without_a_new_period_input():
    r, pool, events = runner()
    r.tick(100.077, 10.077)
    finish(r, pool)
    assert r.pending is None
    assert events[-1]['event'] == 'completion'


def test_shadow_runner_keeps_existing_timer_cooldown():
    r, pool, _ = runner()
    r.deadline_scheduling = False
    r.tick(100.077, 10.077)
    finish(r, pool)
    receive(r, 100.2, 100.235, 10.235, 2)
    r.tick(100.24, 10.24)
    assert len(pool.requests) == 1
    r.tick(100.28, 10.28)
    assert len(pool.requests) == 2


def test_missing_inputs_do_not_create_catchup_work_after_a_long_pause():
    r, pool, _ = runner()
    r.tick(100.077, 10.077)
    finish(r, pool)
    for t in (100.28, 100.5, 101., 102.):
        r.tick(t, t-90.)
    assert len(pool.requests) == 1
    receive(r, 102.05, 102.085, 12.085, 2)
    r.tick(102.09, 12.09)
    assert len(pool.requests) == 2
    finish(r, pool, 102.10, 12.10)
    r.tick(102.11, 12.11)
    assert len(pool.requests) == 2


def test_new_inputs_never_queue_behind_an_unfinished_worker():
    r, pool, _ = runner()
    r.tick(100.077, 10.077)
    for n in range(1, 8):
        acquired = 100.+.05*n
        receive(r, acquired, acquired+.035, 10.+.05*n+.035, n+1)
        r.tick(acquired+.04, 10.+.05*n+.04)
    assert len(pool.requests) == 1


def test_aligned_dispatch_still_rejects_a_late_completed_candidate():
    r, pool, events = runner()
    r.tick(100.077, 10.077)
    finish(r, pool, 100.13, 10.13)
    event = events[-1]
    assert event['candidate_output']['valid']
    assert not event['output']['valid']
    assert event['output']['reason'] == 'STALE_INPUT'


def test_bounded_parent_retry_keeps_its_original_early_dispatch_policy():
    r, pool, _ = runner()
    r.execution_status = dict(stamp=100.077, active=83, deadline=100.40)
    r.tick(100.077, 10.077)
    finish(r, pool, valid=False)
    assert r.retry_waiting_parent == 83
    r.tick(100.105, 10.105)
    assert len(pool.requests) == 2
    assert r.pending['dispatch_timing']['policy'] == 'bounded_retry'
    assert r.next_solve == pytest.approx(10.305)
    finish(r, pool, 100.11, 10.11, valid=False)
    assert r.retry_waiting_parent is None
    r.tick(100.12, 10.12)
    assert len(pool.requests) == 2
