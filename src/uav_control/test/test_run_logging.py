"""Run artifacts begin on X, independently of truth interception evaluation."""

import csv
import json
from collections import deque
from types import SimpleNamespace

import pytest
from uav_usv_interfaces.msg import MissionState, TargetObservation

from uav_control.common.runtime_performance import RateMeter
from uav_control.evaluation.intercept_evaluator import InterceptEvaluatorCore
from uav_control.evaluation.intercept_evaluator import KinematicState
from uav_control.evaluation.intercept_evaluator import TimestampedStateHistory
from uav_control.evaluation.intercept_evaluator_node import InterceptEvaluatorNode
from uav_control.evaluation.intercept_evaluator_node import _seconds_to_time
from uav_control.tracking.prediction_error_tracker import PredictionErrorTracker


def node_fixture(tmp_path):
    node = object.__new__(InterceptEvaluatorNode)
    node.clock_value = 10.
    node._now = lambda: node.clock_value
    node.get_logger = lambda: SimpleNamespace(info=lambda value: None)
    node.log_directory = str(tmp_path)
    node._config_snapshot = lambda: {'truth_role': 'evaluation_only'}
    node.evaluator = InterceptEvaluatorCore(maximum_duration=30.)
    node.evaluation_capture_radius = .5
    node.planned_capture_radius = .5
    node.truth_topic = '/target/state'
    node.truth_role = 'evaluation_only'
    node.capture_geometry_source = 'navigation_estimate'
    node.detailed_diagnostics_enabled = False
    node.visual_evaluation_enabled = True
    node.gazebo_entity_tracker = None
    node.gazebo_uav_tracker = None
    node.gazebo_uav_diagnostics_enabled = False
    node.gazebo_uav_entity_name = 'x500_mono_cam_0'
    node.gazebo_visual_height_offset = .42
    node.latest_mission = None
    node.latest_controller = None
    node.latest_planner_diagnostic = None
    node.latest_prediction = None
    node.latest_uav = KinematicState((0., 0., -5.), (0., 0., 0.))
    node.latest_truth = KinematicState((20., 0., 0.), (0., 0., 0.))
    node.uav_history = TimestampedStateHistory()
    node.truth_history = TimestampedStateHistory()
    node.pending_visual_observations = deque(maxlen=100)
    node.prediction_tracker = PredictionErrorTracker((.5, 1., 2.))
    node.prediction_sequences = set()
    node.shadow_prediction_sequences = set()
    node.prediction_errors = {}
    node.latest_prediction_error = {}
    node.controller_compute_times = []
    node.tracker_rate = RateMeter()
    node.performance_values = {}
    node.writer = None
    node.run_started_at = None
    node.run_mission_id = None
    node.intercept_started_at = None
    node.intercept_result = None
    node.last_run_paths = None
    node.last_run_summary = None
    node.last_run_sample_stamp = None
    node.result_published = False
    node.result_pub = SimpleNamespace(publish=lambda value: None)
    node.hit_pub = SimpleNamespace(publish=lambda value: None)
    return node


def phase(node, name, at, mission_id=1, intercept=False, completed=False):
    node.clock_value = at
    message = MissionState()
    message.mission_id = mission_id
    message.stamp = _seconds_to_time(at)
    message.state_name = name
    message.state = getattr(MissionState, name)
    message.intercept_requested = intercept
    message.completed = completed
    node.mission_callback(message)


def csv_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def test_accepted_x_creates_all_run_artifacts_before_y(tmp_path):
    node = node_fixture(tmp_path)
    phase(node, 'GROUND_HOLD', 9.)
    assert node.writer is None
    phase(node, 'TAKEOFF', 10.)
    assert node.writer is not None
    assert node.evaluator.started_at is None
    paths = node.writer.paths
    assert paths.csv_path.exists()
    assert paths.visual_path.exists()
    assert paths.config_path.exists()
    node._finalize_run('NODE_SHUTDOWN', 11.)
    summary = json.loads(paths.summary_path.read_text())
    assert summary['outcome'] == 'ABORTED'
    assert summary['failure_reason'] == 'NODE_SHUTDOWN'
    assert summary['intercept_started'] is False
    assert summary['run_elapsed_time'] == pytest.approx(1.)


def test_x_follow_shutdown_preserves_transient_phases_and_vision(tmp_path):
    node = node_fixture(tmp_path)
    for name, at in (('TAKEOFF', 10.), ('TARGET_ACQUIRE', 13.),
                     ('TARGET_LOCK', 14.), ('FOLLOW', 14.05)):
        phase(node, name, at)
    observation = TargetObservation()
    observation.stamp = _seconds_to_time(14.05)
    observation.received_stamp = observation.stamp
    observation.valid = True
    observation.source = 'front_rgbd_red_sphere'
    node.visual_observation_callback(observation)
    paths = node.writer.paths
    node.clock_value = 34.05
    node.timer_callback()
    node._finalize_run('NODE_SHUTDOWN', 34.05)
    assert [row['phase'] for row in csv_rows(paths.csv_path)][:4] == [
        'TAKEOFF', 'TARGET_ACQUIRE', 'TARGET_LOCK', 'FOLLOW',
    ]
    assert len(csv_rows(paths.visual_path)) == 1
    summary = json.loads(paths.summary_path.read_text())
    assert summary['run_metrics']['takeoff_duration'] == pytest.approx(3.)
    assert summary['run_metrics']['follow_duration'] == pytest.approx(20.)
    assert summary['intercept_started'] is False


def test_y_starts_only_interception_clock_and_preserves_run_writer(tmp_path):
    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    writer = node.writer
    phase(node, 'FOLLOW', 20.)
    assert node.evaluator.started_at is None
    phase(node, 'FAR_GUIDANCE', 70., intercept=True)
    assert node.writer is writer
    assert node.run_started_at == 10.
    assert node.evaluator.started_at == node.intercept_started_at == 70.
    result = node.evaluator.update(70.1, node.latest_uav, node.latest_truth)
    assert result is None  # Pre-Y follow/search did not consume the limit.
    row = node._sample_row(71.)
    assert row['run_elapsed_time'] == pytest.approx(61.)
    assert row['intercept_elapsed_time'] == pytest.approx(1.)
    node._finalize_run('NODE_SHUTDOWN', 71.)


def test_reset_finalizes_old_run_once_before_new_mission(tmp_path):
    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    paths = node.writer.paths
    phase(node, 'GROUND_HOLD', 12., mission_id=2)
    assert node.writer is None
    document = paths.summary_path.read_text()
    assert json.loads(document)['failure_reason'] == 'MISSION_RESET'
    node._finalize_run('NODE_SHUTDOWN', 12.5)
    assert paths.summary_path.read_text() == document
    phase(node, 'TAKEOFF', 13., mission_id=2)
    assert node.writer.paths != paths
    assert node.run_mission_id == 2
    node._finalize_run('NODE_SHUTDOWN', 14.)


def test_x_only_logging_does_not_require_truth_or_uav_messages(tmp_path):
    node = node_fixture(tmp_path)
    node.latest_truth = node.latest_uav = None
    phase(node, 'TAKEOFF', 10.)
    paths = node.writer.paths
    node.clock_value = 11.
    node.timer_callback()
    node._finalize_run('NODE_SHUTDOWN', 12.)
    rows = csv_rows(paths.csv_path)
    assert len(rows) >= 2
    assert all(row['truth_available'] == 'False' for row in rows)
    assert rows[-1]['run_elapsed_time'] == '1.0'
    assert node.evaluator.started_at is None


def test_run_metrics_track_loss_reacquisition_follow_and_search_drift():
    from uav_control.evaluation.run_metrics import RunMetricAccumulator

    metrics = RunMetricAccumulator(10.)
    metrics.observe(10., 'TAKEOFF')
    metrics.observe(13., 'TARGET_ACQUIRE', visible=True, position=(0., 0., -5.))
    metrics.observe(14., 'TARGET_LOCK', visible=True, locked=True)
    metrics.observe(14.05, 'FOLLOW', locked=True, estimated_distance=5.)
    metrics.observe(15., 'FOLLOW', locked=True, estimated_distance=7.)
    metrics.observe(16., 'REACQUIRE', locked=False, position=(1., 2., -5.))
    metrics.observe(17., 'SAFE_WAIT', locked=False, position=(1.3, 2.4, -5.))
    metrics.observe(18., 'TARGET_LOCK', locked=True)
    metrics.observe(18.05, 'FOLLOW', locked=True)
    summary = metrics.summary(20.)
    assert summary['takeoff_duration'] == 3.
    assert summary['time_to_first_visible'] == 3.
    assert summary['time_to_first_lock'] == 4.
    assert summary['lock_loss_count'] == 1
    assert summary['reacquire_count'] == 1
    assert summary['reacquire_duration']['mean'] == 2.
    assert summary['follow_duration'] == pytest.approx(3.9)
    assert summary['follow_estimated_distance']['mean'] == 6.
    assert summary['follow_evaluation_distance']['count'] == 0
    assert summary['search_xy_drift_max'] == pytest.approx(.5)
    assert metrics.summary(20.) == summary  # Summary is not a state mutation.


def test_evaluator_pose_callback_records_only_exact_uav_model(tmp_path):
    from uav_control.evaluation.uav_heading_diagnostics import GazeboUavPoseTracker

    node = node_fixture(tmp_path)
    node.gazebo_uav_tracker = GazeboUavPoseTracker()
    tracker = node.gazebo_uav_tracker
    for raw, mapped in ((1., 10.), (1.1, 10.1)):
        tracker.add_clock_anchor(raw, mapped, mapped, raw)
    node.clock_value = 10.1

    def pose(name):
        return SimpleNamespace(
            name=name, position=SimpleNamespace(x=0., y=0., z=5.),
            orientation=SimpleNamespace(w=1., x=0., y=0., z=0.),
        )
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nsec=50_000_000)),
        pose=[pose('x500_mono_cam_0::base_link'), pose('x500_mono_cam_1')],
    )
    node.gazebo_entity_pose_callback(message)
    assert tracker.query_at(10.05).value is None
    message.pose.append(pose('x500_mono_cam_0'))
    node.gazebo_entity_pose_callback(message)
    assert tracker.query_at(10.05).value is not None


def test_wrong_mission_diagnostics_do_not_contaminate_run(tmp_path):
    from uav_usv_interfaces.msg import ControllerDiagnostic

    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    diagnostic = ControllerDiagnostic()
    diagnostic.mission_id = 0
    diagnostic.target_visible = diagnostic.target_locked = True
    diagnostic.stamp = _seconds_to_time(10.)
    node.controller_callback(diagnostic)
    assert node.latest_controller is None
    assert node.run_metrics.first_lock_at is None
    node._finalize_run('NODE_SHUTDOWN', 11.)


def test_stale_controller_cannot_pollute_follow_statistics_or_csv(tmp_path):
    from uav_usv_interfaces.msg import ControllerDiagnostic

    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    phase(node, 'FOLLOW', 11.)
    diagnostic = ControllerDiagnostic()
    diagnostic.mission_id = 1
    diagnostic.stamp = _seconds_to_time(11.)
    diagnostic.target_distance = 99.
    diagnostic.target_visible = diagnostic.target_locked = True
    node.controller_callback(diagnostic)
    node.clock_value = 12.
    node.timer_callback()  # A once-fresh cached diagnostic is now expired.
    assert node.run_metrics.summary(12.)['follow_estimated_distance']['count'] == 0
    assert node._sample_row(12.)['controller_status'] == ''
    node.controller_callback(diagnostic)  # A late callback remains expired.
    assert node.latest_controller is None
    node._finalize_run('NODE_SHUTDOWN', 12.)


def test_late_older_mission_cannot_reopen_run_after_finalize(tmp_path):
    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    node._finalize_run('NODE_SHUTDOWN', 11.)
    phase(node, 'TAKEOFF', 12., mission_id=2)
    node._finalize_run('NODE_SHUTDOWN', 13.)
    phase(node, 'TAKEOFF', 15., mission_id=1)
    assert node.writer is None
    assert node.run_mission_id == 2
    assert node.latest_mission.mission_id == 2


def test_enabled_heading_diagnostics_wait_for_acquisition_bracket(tmp_path):
    from uav_control.evaluation.uav_heading_diagnostics import GazeboUavPoseTracker

    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    node.gazebo_uav_diagnostics_enabled = True
    node.gazebo_uav_tracker = GazeboUavPoseTracker()
    tracker = node.gazebo_uav_tracker
    for raw, mapped in ((1., 10.), (1.1, 10.1)):
        tracker.add_clock_anchor(raw, mapped, mapped, raw)
    tracker.add_pose(1., (0., 0., 5.), (1., 0., 0., 0.), 10.1)
    for at in (10., 10.1):
        node.truth_history.add(at, node.latest_truth)
    node.clock_value = 10.1
    observation = TargetObservation()
    observation.stamp = _seconds_to_time(10.05)
    observation.received_stamp = _seconds_to_time(10.1)
    observation.valid = observation.geometry_diagnostics_enabled = True
    observation.interpolated_attitude_w = 1.
    observation.target_range = 20.
    observation.camera_fx = observation.camera_fy = 100.
    paths = node.writer.paths
    node.visual_observation_callback(observation)
    assert len(node.pending_visual_observations) == 1
    tracker.add_pose(1.1, (0., 0., 5.), (1., 0., 0., 0.), 10.11)
    node.clock_value = 10.11
    node._drain_visual_observations()
    assert not node.pending_visual_observations
    assert csv_rows(paths.visual_path)[0]['heading_diagnostics_status'] == 'VALID'
    node._finalize_run('NODE_SHUTDOWN', 10.12)


def test_pending_valid_vision_is_logged_after_missing_truth_timeout(tmp_path):
    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    observation = TargetObservation()
    observation.stamp = _seconds_to_time(10.)
    observation.received_stamp = _seconds_to_time(10.)
    observation.valid = True
    node.visual_observation_callback(observation)
    assert len(node.pending_visual_observations) == 1
    node.clock_value = 10.6
    node.timer_callback()
    assert not node.pending_visual_observations
    paths = node.writer.paths
    rows = csv_rows(paths.visual_path)
    assert len(rows) == 1
    assert rows[0]['measurement_stamp'] == '10.0'
    assert rows[0]['observation_valid'] == 'True'
    assert rows[0]['truth_available'] == 'False'
    node._finalize_run('NODE_SHUTDOWN', 10.7)


def test_record_only_evaluation_result_does_not_end_run_logging(tmp_path):
    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    phase(node, 'FAR_GUIDANCE', 11., intercept=True)
    node.evaluator.maximum_duration = .2
    results = []
    node.result_pub = SimpleNamespace(publish=results.append)
    for at in (11., 11.21):
        node.uav_history.add(at, node.latest_uav)
        node.truth_history.add(at, node.latest_truth)
    node.clock_value = 11.21
    node.timer_callback()
    assert node.result_published
    assert len(results) == 1
    assert node.writer is not None
    paths = node.writer.paths
    before = len(csv_rows(paths.csv_path))
    node.clock_value = 11.5
    node.timer_callback()
    assert len(results) == 1
    assert len(csv_rows(paths.csv_path)) == before + 1
    node._finalize_run('NODE_SHUTDOWN', 11.6)
    summary = json.loads(paths.summary_path.read_text())
    assert summary['intercept_started'] is True
    assert summary['outcome'] == 'FAILURE'
    assert summary['failure_reason'] == 'TIMEOUT'
    assert summary['run_end_reason'] == 'NODE_SHUTDOWN'


@pytest.mark.parametrize('transport_succeeds', [True, False])
def test_capture_pauses_world_and_finalizes_once_even_if_transport_fails(
        tmp_path, transport_succeeds):
    """A successful capture must not leave physics/logging running forever."""
    from uav_control.evaluation.gazebo_terminal import GazeboTerminalPauser

    node = node_fixture(tmp_path)
    node.evaluator.capture_radius = .5
    phase(node, 'TAKEOFF', 10.)
    phase(node, 'FAR_GUIDANCE', 11., intercept=True)
    paths = node.writer.paths
    node.pause_gazebo_on_success = True
    requests = []
    node.gazebo_pauser = GazeboTerminalPauser(
        lambda: requests.append(True) or transport_succeeds,
        retry_delay=0.,
    )
    node.get_logger = lambda: SimpleNamespace(
        info=lambda _: None, warn=lambda _: None)
    node.latest_uav = KinematicState((0., 0., -.33), (5.5, 0., 0.))
    node.latest_truth = KinematicState((0., 0., 0.), (4., 0., 0.))
    node.uav_history.add(11.01, node.latest_uav)
    node.truth_history.add(11.01, node.latest_truth)
    node.clock_value = 11.01
    results = []
    node.result_pub = SimpleNamespace(publish=results.append)
    node.timer_callback()

    assert node.writer is None
    summary_text = paths.summary_path.read_text()
    summary = json.loads(summary_text)
    assert summary['outcome'] == 'SUCCESS'
    assert summary['run_end_reason'] == 'CAPTURE_RADIUS_REACHED'
    assert summary['gazebo_pause_requested'] is True
    assert summary['gazebo_pause_succeeded'] is transport_succeeds
    assert summary['gazebo_pause_attempts'] == (1 if transport_succeeds else 2)
    assert requests
    assert len(results) == 1
    node.timer_callback()
    node._finalize_run('NODE_SHUTDOWN', 12.)
    assert len(results) == 1
    assert paths.summary_path.read_text() == summary_text


def test_safe_recovery_cannot_be_reported_as_a_clean_first_terminal_hit(tmp_path):
    """The visual safety recovery observed in SITL belongs in capture metrics."""
    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10.)
    phase(node, 'FAR_GUIDANCE', 11., intercept=True)
    phase(node, 'TERMINAL_MINCO', 11.5, intercept=True)
    phase(node, 'SAFE_RECOVERY', 12., intercept=True)
    phase(node, 'REACQUIRE', 12.1, intercept=True)
    assert node.recovery_count == 1
    assert node.first_terminal_approach_result == 'RECOVERY'
    phase(node, 'FAR_GUIDANCE', 13., intercept=True)
    phase(node, 'TERMINAL_MINCO', 13.5, intercept=True)
    assert node.terminal_approach_count == 2
    node._finalize_run('NODE_SHUTDOWN', 14.)


def test_old_worker_plan_cannot_enter_new_run_artifacts(tmp_path):
    from uav_usv_interfaces.msg import PlannerDiagnostic

    node = node_fixture(tmp_path)
    phase(node, 'TAKEOFF', 10., mission_id=2)
    message = PlannerDiagnostic()
    message.mission_id = 1
    message.plan_id = 123
    message.result = PlannerDiagnostic.RESULT_SUCCESS
    node.planner_callback(message)
    assert node.latest_planner_diagnostic is None
    assert node.event_metrics.summary(0.)['planner_succeeded'] == 0
    node._finalize_run('NODE_SHUTDOWN', 11.)


def test_old_prediction_cannot_enter_new_run_artifacts(tmp_path):
    from uav_usv_interfaces.msg import TargetPrediction

    node = node_fixture(tmp_path)
    old = TargetPrediction()
    old.mission_id = 1
    old.sequence_id = 123
    old.valid = True
    node.latest_prediction = old
    phase(node, 'TAKEOFF', 10., mission_id=2)
    assert node.latest_prediction is None
    node.prediction_callback(old)
    node.shadow_prediction_callback(old)
    assert node.latest_prediction is None
    assert not node.prediction_sequences
    assert not node.shadow_prediction_sequences
    node._finalize_run('NODE_SHUTDOWN', 11.)
