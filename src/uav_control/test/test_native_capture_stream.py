"""Real evaluator: transient entry, first position, and set_pose discontinuity."""

import math
from types import SimpleNamespace

import pytest

from uav_control.evaluation.gazebo_terminal import GazeboTerminalPauser
from uav_control.evaluation.intercept_evaluator import (
    ExperimentArtifactWriter, InterceptEvaluatorCore, KinematicState, TimestampedStateHistory,
)
from uav_control.evaluation.intercept_evaluator_node import InterceptEvaluatorNode
from uav_control.evaluation.uav_heading_diagnostics import GazeboUavPoseTracker


def native_node():
    evaluator = InterceptEvaluatorCore(capture_radius=.5)
    evaluator.begin(1, 10.)
    results, pauses = [], []
    node = SimpleNamespace(
        writer=SimpleNamespace(append_detail_event=lambda *a: None,
                               paths=SimpleNamespace(csv_path='test.csv')),
        evaluator=evaluator, uav_history=TimestampedStateHistory(),
        truth_history=TimestampedStateHistory(),
        capture_geometry_source='gazebo_native', gazebo_visual_height_offset=.42,
        gazebo_uav_tracker=GazeboUavPoseTracker(),
        gazebo_capture_target_tracker=GazeboUavPoseTracker(model_name='usv_target'),
        intercept_started_at=10., result_published=False, last_synchronized_stamp=None,
        evaluation_capture_radius=.5, pause_gazebo_on_success=True,
        gazebo_pauser=GazeboTerminalPauser(lambda: pauses.append(True) or True),
        result_pub=SimpleNamespace(publish=results.append),
        hit_pub=SimpleNamespace(publish=lambda message: None),
        runtime_performance=SimpleNamespace(summary=lambda: {}), now=10.06,
        _drain_visual_observations=lambda: None,
        _write_run_sample=lambda *a, **kw: None, _prediction_summary=lambda: {},
        _finalize_run=lambda *a: None,
        get_logger=lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None))
    node._now = lambda: node.now
    node.results, node.pauses = results, pauses
    return node


def pose(node, elapsed, horizontal, target_x=0., target=True):
    # Stored Gazebo ENU: NED X is ENU Y; marker is .42 m above USV origin.
    node.gazebo_uav_tracker.history.add(
        1.+elapsed, 10.+elapsed, (0., target_x+horizontal, .35, 1., 0., 0., 0.),
        source_bracket=(1.+elapsed, 1.+elapsed, 10.+elapsed, 10.+elapsed))
    node.gazebo_uav_tracker.last_mapped_stamp = 10.+elapsed
    if target:
        node.gazebo_capture_target_tracker.history.add(
            1.+elapsed, 10.+elapsed, (0., target_x, .42, 1., 0., 0., 0.),
            source_bracket=(1.+elapsed, 1.+elapsed, 10.+elapsed, 10.+elapsed))
        node.gazebo_capture_target_tracker.last_mapped_stamp = 10.+elapsed


def test_first_valid_native_position_hits_without_velocity_history():
    node = native_node()
    pose(node, .04, .3)
    InterceptEvaluatorNode.timer_callback(node)
    assert len(node.results) == 1
    assert node.results[0].success
    assert node.last_native_capture_sample.stamp == pytest.approx(10.04)
    assert not node.last_native_capture_sample.velocity_valid
    assert math.isnan(node.results[0].relative_speed)
    assert node.pauses == [True]


def test_entry_then_exit_between_evaluator_ticks_latches_first_entry_once():
    node = native_node()
    pose(node, .00, .7)
    pose(node, .02, .7)
    pose(node, .04, .3)
    pose(node, .06, .7)
    InterceptEvaluatorNode.timer_callback(node)
    InterceptEvaluatorNode.timer_callback(node)
    assert len(node.results) == 1
    assert node.results[0].success
    assert node.last_native_capture_sample.stamp == pytest.approx(10.04)
    assert node.pauses == [True]


def test_set_pose_jump_across_capture_ball_does_not_invent_a_continuous_hit():
    node = native_node()
    pose(node, .00, .7, target_x=-.7)
    pose(node, .02, .7, target_x=-.7)
    node.now = 10.02
    InterceptEvaluatorNode.timer_callback(node)
    # UAV stays at NED X=0; target teleports -0.7 -> +0.7, never inside.
    pose(node, .04, -.7, target_x=.7)
    node.now = 10.04
    InterceptEvaluatorNode.timer_callback(node)
    assert not node.results
    assert not node.pauses


def test_missing_target_epoch_never_uses_estimate_or_other_epoch():
    node = native_node()
    pose(node, .02, .7)
    pose(node, .04, .3, target=False)
    InterceptEvaluatorNode.timer_callback(node)
    assert not node.results


def test_old_native_entry_is_rejected_not_backdated_to_current_time():
    node = native_node()
    pose(node, .02, .3)
    node.now = 10.146
    InterceptEvaluatorNode.timer_callback(node)
    assert not node.results


def test_rejected_history_gap_does_not_prevent_fresh_inside_position_hit():
    node = native_node()
    pose(node, .00, .7)
    node.now = 10.
    InterceptEvaluatorNode.timer_callback(node)
    pose(node, .08, .3)
    node.now = 10.08
    InterceptEvaluatorNode.timer_callback(node)
    assert len(node.results) == 1 and node.results[0].success
    assert not node.last_native_capture_sample.velocity_valid


def test_geometry_before_y_does_not_complete_new_capture():
    node = native_node()
    pose(node, .02, .3)
    node.intercept_started_at = 10.03
    InterceptEvaluatorNode.timer_callback(node)
    assert not node.results


def test_native_capture_queries_are_persisted_even_when_optional_diagnostics_are_off(tmp_path):
    writer = ExperimentArtifactWriter(tmp_path, 1, {'capture_geometry_source': 'gazebo_native'})
    writer.append_capture_event({'kind': 'QUERY', 'decision_stamp': 10.04,
                                 'rejected': [{'reason': 'NATIVE_POSITION_AGE'}]})
    writer.finalize({'success': False})
    import json
    path, = tmp_path.glob('*_capture.jsonl')
    event, = [json.loads(line) for line in path.read_text().splitlines()]
    assert event['rejected'][0]['reason'] == 'NATIVE_POSITION_AGE'


@pytest.mark.parametrize('native_width,system_width', ((.08, .04), (.04, .08)))
def test_wide_clock_mapping_cannot_qualify_an_exact_native_pose(native_width, system_width):
    node = native_node()
    for tracker, xy, height in ((node.gazebo_uav_tracker, .3, .35),
                                (node.gazebo_capture_target_tracker, 0., .42)):
        tracker.add_clock_anchor(1., 10., 10., 100.)
        tracker.add_clock_anchor(1.+native_width, 10.+system_width,
                                 10.+system_width, 100.+system_width)
        tracker.add_pose(1.+native_width/2, (0., xy, height), (1., 0., 0., 0.),
                         10.+system_width)
    node.now = 10.+system_width
    InterceptEvaluatorNode.timer_callback(node)
    assert not node.results


def test_pose_newer_than_current_timer_epoch_is_deferred_not_discarded():
    node = native_node()
    pose(node, .03, .3)
    node.now = 10.02
    InterceptEvaluatorNode.timer_callback(node)
    assert not node.results
    node.now = 10.05
    InterceptEvaluatorNode.timer_callback(node)
    assert len(node.results) == 1 and node.results[0].success


def test_target_jump_cannot_hide_an_earlier_uav_sea_contact():
    node = native_node()
    for elapsed, target_x, height in ((0., 0., .232), (.02, .08, .227)):
        entries = ((node.gazebo_uav_tracker, .525, height),
                   (node.gazebo_capture_target_tracker, target_x, .42))
        for tracker, x, z in entries:
            tracker.history.add(1.+elapsed, 10.+elapsed, (0., x, z, 1., 0., 0., 0.),
                                source_bracket=(1.+elapsed, 1.+elapsed,
                                                10.+elapsed, 10.+elapsed))
        node.now = 10.+elapsed
        InterceptEvaluatorNode.timer_callback(node)
    assert len(node.results) == 1
    assert not node.results[0].success
    assert node.results[0].reason == 'SEA_CONTACT'
    assert not node.pauses


def test_capture_segment_provenance_distinguishes_entry_from_right_endpoint_time():
    core = InterceptEvaluatorCore(capture_radius=.5)
    core.begin(1, 10.)
    target = KinematicState((0., 0., 0.), (0., 0., 0.))
    core.update(10., KinematicState((.5, 0., -.35), (-4., 0., 0.)), target)
    result = core.update(10.05, KinematicState((.3, 0., -.35), (-4., 0., 0.)), target)
    assert result.success
    # The 0.5 m sphere meets z=-0.35 at x=sqrt(0.1275), before the right sample.
    fraction = (.5-math.sqrt(.1275))/.2
    assert core.event_provenance['fraction'] == pytest.approx(fraction)
    assert core.event_provenance['event_stamp'] == pytest.approx(10.+.05*fraction)
    assert core.event_provenance['source'] == 'CAPTURE_SEGMENT'
