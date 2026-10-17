"""Visual lock, state readiness and nominal execution permission are independent."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
from uav_control.mission.target_visibility import TargetVisibilityState
from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
from uav_control.control.flight_guidance import FlightGuidanceCore
from uav_usv_interfaces.msg import MissionState, TargetState, TargetPrediction
from builtin_interfaces.msg import Time
from test_target_visibility import lock, lose
from test_follow_sitl_execution import candidate, ctx, receiver
from test_p51_execution_audit import rows
from p51_execution_audit import audit_rows
from test_bearing_approach import approach_node


def follow_lock(core):
    lock(core)
    assert core.update(1.04, 0., 5., .05, True, separate_input_wait=True).locked


def test_visible_target_with_late_kf_waits_for_input_without_visual_scan():
    core = TargetVisibilityState()
    follow_lock(core)
    decision = core.update(1.05, 0., 5., .05, False, separate_input_wait=True)
    assert decision.state == 'INPUT_WAIT' and decision.visual_locked
    assert decision.visible and not decision.locked
    assert decision.search_direction == 0 and core._search_started_at is None
    assert core.last_valid_bearing_stamp == 1.04
    assert core.update(1.06, 0., 5., .05, True, separate_input_wait=True).locked


def test_fresh_rgb_with_stale_depth_does_not_grant_combined_lock():
    core = TargetVisibilityState()
    follow_lock(core)
    for stamp in (1.25, 1.27, 1.29):
        core.observe(stamp, .05, True, stamp)
    decision = core.update(1.29, 0., 5., .05, True, separate_input_wait=True)
    assert decision.state == 'INPUT_WAIT' and not decision.locked and decision.visual_locked


def test_real_rgb_loss_keeps_the_existing_finite_reacquisition_path():
    core = TargetVisibilityState()
    lock(core)
    now = lose(core)
    decision = core.update(now, 0., 5., .05, True, separate_input_wait=True)
    assert decision.state == 'REACQUIRE' and not decision.visual_locked
    assert decision.search_direction != 0


def test_initial_rgb_only_acquisition_is_not_a_previous_lock_input_wait():
    core = TargetVisibilityState()
    for stamp in (1., 1.02, 1.04):
        core.observe(stamp, .05, True, stamp)
    decision = core.update(1.04, 0., 5., .05, False, separate_input_wait=True)
    assert decision.state == 'TARGET_ACQUIRE' and not decision.locked
    assert core.bearing_approach_ready(1.04, .125)


def test_takeoff_lock_does_not_block_initial_follow_rgb_approach_when_depth_is_late():
    core = TargetVisibilityState()
    lock(core)  # Legacy takeoff/acquisition lock, before FOLLOW has controlled the target.
    for stamp in (1.25, 1.27, 1.29):
        core.observe(stamp, .05, True, stamp)
    decision = core.update(1.29, 0., 5., .05, False, separate_input_wait=True)
    assert decision.state != 'INPUT_WAIT' and not decision.locked
    assert core.bearing_approach_ready(1.29, .125)


def test_first_follow_tick_can_approach_rgb_after_a_takeoff_only_depth_lock():
    n, _ = approach_node()
    n.visibility.mark_observation(10.2, True, 10.2)
    assert n.visibility.update(10.2, 0., 5., .05, True).locked
    n.mission_state, n.mission_state_name = MissionState.FOLLOW, 'FOLLOW'
    n.timer_callback()  # KF is absent, fresh RGB can still guide the initial approach.
    assert n.bearing_approach_active
    assert n.visibility_decision.state != 'INPUT_WAIT'
    assert n.diagnostics[-1][0].velocity[0] > 0.
    assert n.latest_target_state is None and not n.visibility_decision.locked


@pytest.mark.parametrize('invalid_dt', [False, True])
def test_non_follow_early_return_clears_previous_follow_acquisition(invalid_dt):
    core = TargetVisibilityState()
    follow_lock(core)
    lose(core)
    core.update(1.5, 0., 5., 0. if invalid_dt else .05, False, terminal=True)
    for stamp in (1.51, 1.53, 1.55):
        core.observe(stamp, .05, True, stamp)
    decision = core.update(1.55, 0., 5., .05, False, separate_input_wait=True)
    assert decision.state != 'INPUT_WAIT' and core.bearing_approach_ready(1.55, .125)


def test_input_wait_is_bounded_before_rgb_only_reacquisition_can_resume():
    core = TargetVisibilityState()
    follow_lock(core)
    for stamp in (1.25, 1.27, 1.29):
        core.observe(stamp, .05, True, stamp)
    assert core.update(1.29, 0., 5., .05, False, separate_input_wait=True).state == 'INPUT_WAIT'
    for stamp in (1.6, 1.62, 1.64):
        core.observe(stamp, .05, True, stamp)
    decision = core.update(1.64, 0., 5., .05, False, separate_input_wait=True)
    assert decision.state == 'REACQUIRE' and not decision.locked
    assert core.bearing_approach_ready(1.64, .125)


@pytest.mark.parametrize('field,reason', [
    ('navigation_fresh', 'NAVIGATION_STALE'), ('target_fresh', 'TARGET_STATE_STALE'),
    ('prediction_fresh', 'PREDICTION_STALE'), ('recovering', 'SAFETY_RECOVERY'),
    ('observation_fresh', 'OBSERVATION_STALE'),
])
def test_ready_rgb_never_keeps_expired_execution_or_pending_successor(field, reason):
    r, curve = receiver(), candidate()
    r.propose(curve, ctx())
    r.tick(ctx(curve.start), nominal_valid=True)
    deadline = r.deadline
    bad = replace(ctx(curve.start+.01), **{field: field == 'recovering'})
    ack = r.tick(bad, nominal_valid=True)
    assert ack.state == 'REVOKED' and reason in ack.reasons
    assert 'TARGET_NOT_LOCKED' not in ack.reasons
    assert r.active is None and r.pending is None and r.deadline == deadline


def node_context(prediction_generated=100.01):
    n = object.__new__(TrajectoryTrackerNode)
    n._ros_seconds = lambda: 100.03
    n.follow_clock = SimpleNamespace(observe=lambda now: 0, boot_id='receiver')
    n.mission_id, n.mission_state = 1, MissionState.FOLLOW
    n.offboard_active = n.vehicle_armed = True
    n.safe_recovery_latched = False
    n.maximum_state_age, n.expected_frame_id = .125, 'local_ned'
    n.latest_state = SimpleNamespace(stamp=100.01)
    kf = TargetState(valid=True, frame_id='local_ned')
    kf.stamp, kf.source_stamp = Time(sec=100, nanosec=20000000), Time(sec=100)
    n.latest_kf_message = kf
    pred = TargetPrediction(valid=True, source='tracking', frame_id='local_ned', mission_id=1,
                            sequence_id=688)
    pred.observation_stamp = Time(sec=99, nanosec=900000000)
    pred.source_stamp, pred.valid_until = Time(sec=100), Time(sec=100, nanosec=100000000)
    pred.generated_stamp = Time(sec=100, nanosec=round((prediction_generated-100)*1e9))
    n.latest_prediction = pred
    n.visibility = TargetVisibilityState()
    n.visibility.mark_observation(100., True, 100.03)
    n.visibility_decision = SimpleNamespace(locked=True, visual_locked=True, visible=True)
    return n


def test_actual128ms_event_is_reported_as_prediction_expiry_with_rgb_still_locked():
    n = node_context()
    context = n._follow_context()
    assert context.visible and context.navigation_fresh and context.target_fresh
    assert not context.prediction_fresh
    ack = receiver().propose(candidate(), context)
    assert 'PREDICTION_STALE' in ack.reasons and 'TARGET_NOT_LOCKED' not in ack.reasons


def test_future_prediction_generation_cannot_pass_nominal_context():
    n = node_context(100.04)
    n.latest_prediction.observation_stamp = Time(sec=100)
    assert not n._follow_context().prediction_fresh


def test_late_direct_depth_never_grants_execution_from_fresh_kf_and_rgb():
    n = node_context()
    n.latest_prediction.observation_stamp = Time(sec=100)
    n.visibility.last_valid_observation_stamp = 99.8
    n.visibility_decision = SimpleNamespace(state='INPUT_WAIT', locked=False,
                                            visual_locked=True, visible=True)
    context = n._follow_context()
    assert context.visible and context.target_fresh and context.prediction_fresh
    assert not context.observation_fresh
    ack = receiver().propose(candidate(), context)
    assert 'OBSERVATION_STALE' in ack.reasons


def test_new_context_audit_requires_explicit_input_flags():
    events = rows()
    for row in events:
        if row.get('kind') == 'execution_debug':
            row['execution_context'].update(schema_version=2)
    audit = audit_rows(events, 100., 'f2')
    assert any(v['check'] == 'execution_gates' for v in audit['violations'])
    for row in events:
        if row.get('kind') == 'execution_debug':
            row['execution_context'].update(navigation_fresh=True, target_fresh=True,
                                            prediction_fresh=True, observation_fresh=True,
                                            recovering=False)
    assert audit_rows(events, 100.04, 'f2')['passed']


def test_late_state_after_a_visual_lock_brakes_instead_of_launching_rgb_approach():
    n = object.__new__(TrajectoryTrackerNode)
    n.latest_target_state = None
    n.visibility_decision = SimpleNamespace(state='INPUT_WAIT', visible=True, locked=False,
                                            visual_locked=True)
    n.flight_guidance = FlightGuidanceCore()
    n.flight_guidance.previous_velocity = (4., 0., 0.)
    n.safe_recovery_latched = n.terminal_mode_latched = n.bearing_approach_active = False
    n.mission_state, n.current_heading = MissionState.FOLLOW, 0.
    n.search_hold_position = None
    n.visibility = TargetVisibilityState()
    n.tracker = SimpleNamespace(sea_surface_z=0., recovery_clearance=1.5, control_dt=.05)
    n._bearing_approach_available = lambda current, now: True
    n._bearing_approach_direction = lambda: 0.
    n.bearing_approach_speed = 6.
    current = SimpleNamespace(position=(0., 0., -5.), velocity=(4., 0., 0.))
    command = n._continuous_follow_command(current, 100., .05)
    assert 3.8 <= command.velocity[0] < 4.
    assert not n.bearing_approach_active


def test_input_wait_keeps_visual_yaw_authority_and_rate_limit():
    n = object.__new__(TrajectoryTrackerNode)
    n.visibility = TargetVisibilityState()
    n.visibility.last_valid_image_bearing = .8
    n.visibility_decision = SimpleNamespace(state='INPUT_WAIT', visible=True, locked=False,
                                            visual_locked=True, yaw_rate=0.)
    n.mission_state, n.current_heading = MissionState.FOLLOW, 0.
    n.safe_recovery_latched, n.intercept_requested = False, False
    n.max_observation_yaw_rate, n.last_target_yaw = 1.2, None
    n.tracker = SimpleNamespace(sea_surface_z=0.)
    current = SimpleNamespace(stamp=100., position=(0., 0., -5.))
    setpoint = SimpleNamespace()
    n._final_yaw(setpoint, current, .05)
    assert n.yaw_owner == 'VISION' and setpoint.yawspeed == pytest.approx(.8)
