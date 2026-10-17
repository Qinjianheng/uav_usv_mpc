"""Observe the real solver's thread CPU separately from its wall-clock service time."""
from concurrent.futures import ThreadPoolExecutor
import time

from test_follow_revalidation import inputs
from test_progress_follow import model
from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
from uav_control.controllers.p4_follow_planner_node import P4ResearchRunner


def test_real_minco_worker_reports_same_thread_cpu_and_wall_without_retiming_request():
    request, _, _ = inputs()
    original_context = request.context
    started = time.perf_counter()
    runner = P4ResearchRunner.__new__(P4ResearchRunner)
    runner.solver = P47FollowGuidedMinco(
        model(), duration=1.6, refinement=False, scheduling='adaptive',
        online_budget_guard=True, wall_clock=lambda: request.now_stamp
        + time.perf_counter()-started)
    with ThreadPoolExecutor(max_workers=1) as pool:
        result = pool.submit(runner._solve_request, request).result(timeout=2.)
    assert result.valid
    assert result.context == original_context
    timeline = result.metrics['worker_timeline']
    cpu = timeline['worker_thread_cpu_seconds']
    assert cpu == timeline['solve_end_thread_cpu']-timeline['solve_start_thread_cpu']
    assert 0. < cpu <= timeline['solve_end_monotonic']-timeline['solve_start_monotonic']+.001
    assert timeline['solve_end_ros'] >= timeline['solve_start_ros'] >= request.now_stamp
