"""Exercise the actual evaluator callback from radius entry to bounded world pause."""

from types import SimpleNamespace

import pytest

from uav_control.evaluation.gazebo_terminal import GazeboTerminalPauser
from uav_control.evaluation.intercept_evaluator import (
    InterceptEvaluatorCore, KinematicState, TimestampedStateHistory,
)
from uav_control.evaluation.intercept_evaluator_node import InterceptEvaluatorNode


@pytest.mark.parametrize('horizontal,expected', ((.4, False), (.3, True)))
@pytest.mark.parametrize('native', (False, True))
def test_original_three_dimensional_radius_drives_result_and_pause(horizontal, expected, native):
    evaluator = InterceptEvaluatorCore(capture_radius=.5)
    evaluator.begin(1, 10.)
    uav, target = TimestampedStateHistory(), TimestampedStateHistory()
    uav.add(10.05, KinematicState((.7 if native else horizontal, 0., -.35), (4., 0., 0.)))
    target.add(10.05, KinematicState((0., 0., 0.), (4., 0., 0.)))
    results, hits, pauses, finalized = [], [], [], []
    node = SimpleNamespace(
        writer=SimpleNamespace(append_detail_event=lambda *a: None,
                               paths=SimpleNamespace(csv_path='test.csv')),
        evaluator=evaluator, uav_history=uav, truth_history=target,
        intercept_started_at=10., result_published=False, last_synchronized_stamp=None,
        evaluation_capture_radius=.5, pause_gazebo_on_success=True,
        gazebo_pauser=GazeboTerminalPauser(lambda: pauses.append(True) or True),
        result_pub=SimpleNamespace(publish=results.append),
        hit_pub=SimpleNamespace(publish=hits.append),
        runtime_performance=SimpleNamespace(summary=lambda: {}),
        _now=lambda: 10.05, _drain_visual_observations=lambda: None,
        _write_run_sample=lambda *a, **kw: None, _prediction_summary=lambda: {},
        _finalize_run=lambda *a: finalized.append(a),
        get_logger=lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None))
    if native:
        from uav_control.evaluation.uav_heading_diagnostics import GazeboUavPoseTracker
        node.capture_geometry_source = 'gazebo_native'
        node.gazebo_visual_height_offset = .42
        node.gazebo_uav_tracker = GazeboUavPoseTracker()
        node.gazebo_capture_target_tracker = GazeboUavPoseTracker(model_name='usv_target')
        for tracker, offset, height in ((node.gazebo_uav_tracker, horizontal, .35),
                                        (node.gazebo_capture_target_tracker, 0., .42)):
            for elapsed in (0., .025, .05):
                tracker.history.add(1.+elapsed, 10.+elapsed,
                                    (0., 4.*elapsed+offset, height, 1., 0., 0., 0.),
                                    source_bracket=(1.+elapsed, 1.+elapsed,
                                                    10.+elapsed, 10.+elapsed))
            tracker.last_mapped_stamp = 10.05
    InterceptEvaluatorNode.timer_callback(node)
    InterceptEvaluatorNode.timer_callback(node)
    assert bool(results) is expected
    assert bool(pauses) is expected
    assert len(pauses) == int(expected)  # One terminal action, never a control correction.
    if expected:
        assert results[0].success and hits[0].data
        assert results[0].capture_radius == .5
        assert node.terminal_pause_result.gazebo_pause_succeeded
        assert finalized[0][0] == 'CAPTURE_RADIUS_REACHED'
