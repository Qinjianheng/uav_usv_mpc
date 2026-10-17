"""Do not repeat heavyweight request pairing during the normal 5 Hz cooldown."""
from concurrent.futures import Future

from uav_control.controllers.follow_mpc_shadow_node import ShadowResearchRunner
from uav_control.controllers.p4_follow_planner_node import P4ResearchRunner


def test_no_prepare_during_idle_dispatch_cooldown(monkeypatch):
    runner = object.__new__(P4ResearchRunner)
    runner.pending = None
    runner.next_solve = 10.2
    calls = []
    monkeypatch.setattr(ShadowResearchRunner, 'tick',
                        lambda self, now, mono: calls.append((now, mono)))
    for mono in (10.01, 10.10, 10.199):
        runner.tick(100. + mono, mono)
    assert calls == []
    runner.tick(110.2, 10.2)
    assert calls == [(110.2, 10.2)]


def test_completed_worker_is_serviced_even_in_dispatch_cooldown(monkeypatch):
    runner = object.__new__(P4ResearchRunner)
    complete = Future()
    complete.set_result(None)
    runner.pending = {'future': complete}
    runner.next_solve = 10.2
    runner.deadline_scheduling = False
    runner.execution_status = None
    calls = []
    monkeypatch.setattr(ShadowResearchRunner, 'tick',
                        lambda self, now, mono: calls.append((now, mono)))
    runner.tick(110.01, 10.01)
    assert calls == [(110.01, 10.01)]


def test_running_worker_is_never_duplicated(monkeypatch):
    runner = object.__new__(P4ResearchRunner)
    runner.pending = {'future': Future()}
    runner.next_solve = 10.2
    calls = []
    monkeypatch.setattr(ShadowResearchRunner, 'tick',
                        lambda self, now, mono: calls.append((now, mono)))
    runner.tick(110.01, 10.01)
    assert calls == []
