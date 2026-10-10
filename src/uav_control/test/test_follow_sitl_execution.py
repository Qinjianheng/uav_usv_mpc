"""Actual nominal SITL lifecycle is separate from qualified flight authorization."""
from dataclasses import replace

import numpy as np
import pytest

from test_follow_contract import curve, context


def receiver():
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    return NominalFollowReceiver()


def candidate(plan=1, start=100.04, parent=0):
    return replace(curve(plan, start, parent), holding_until=0., holding_model='',
                   receiver_boot_id='receiver', planner_boot_id='planner')


def ctx(now=100.01, **changes):
    return context(now, receiver_boot_id='receiver', **changes)


def test_nominal_pending_activation_has_finite_nonrenewable_lease():
    r, c = receiver(), candidate()
    assert r.propose(c, ctx()).state == 'ACCEPTED'
    assert r.active is None
    assert r.tick(ctx(c.start+.02)).state == 'ACTIVE'
    assert r.active == c
    assert r.deadline == pytest.approx(c.start+.45)
    until = r.deadline
    assert r.propose(candidate(2), ctx(100.2)).state == 'REJECTED'
    assert r.deadline == until
    assert r.tick(ctx(until)).state == 'EXPIRED'
    assert r.active is None


@pytest.mark.parametrize('change,reason', [
    ({'now': 100.13}, 'INPUT_EXPIRED'), ({'generation': 2}, 'CLOCK_GENERATION_CHANGED'),
    ({'mission_id': 2}, 'MISSION_CHANGED'), ({'visible': False}, 'TARGET_NOT_LOCKED'),
    ({'offboard': False}, 'OFFBOARD_UNAVAILABLE'),
])
def test_admission_retains_original_fail_closed_checks(change, reason):
    r = receiver()
    assert reason in r.propose(candidate(), ctx(**change)).reasons
    assert r.active is None and r.pending is None


def test_late_activation_and_reset_cannot_leave_stale_reference():
    r, c = receiver(), candidate()
    r.propose(c, ctx())
    assert r.tick(ctx(c.start+.051)).state == 'REJECTED'
    assert r.active is None
    r = receiver()
    r.propose(c, ctx())
    r.tick(ctx(c.start))
    assert r.tick(ctx(c.start+.01, generation=2)).state == 'REVOKED'
    assert r.active is None


def test_parent_boundary_uses_actual_handover_instead_of_old_endpoint():
    r, c = receiver(), candidate()
    r.propose(c, ctx())
    r.tick(ctx(c.start))
    new = replace(candidate(2, 100.1, 1), xyz=((.06, 0., -5.), *c.xyz[1:]))
    assert r.propose(new, ctx(100.06)).state == 'ACCEPTED'
    bad = replace(new, plan_id=3, xyz=((1.2, 0., -5.), *new.xyz[1:]))
    assert 'PVA_YAW_DISCONTINUITY' in r.propose(bad, ctx(100.07)).reasons
    assert r.pending.plan_id == 2


def test_latest_prediction_requires_fresh_prefix_check_and_never_extends_lease():
    r, c = receiver(), candidate()
    r.propose(c, ctx())
    r.tick(ctx(c.start))
    until = r.deadline
    # Caller supplies the result of its receiver-local full remaining-prefix check.
    assert r.tick(ctx(c.start+.1, prediction_id=8), nominal_valid=True) is None
    assert r.deadline == until
    assert r.tick(ctx(c.start+.15, prediction_id=9), nominal_valid=False).state == 'REVOKED'
    assert r.active is None


def test_invalid_pending_prefix_keeps_valid_active_and_original_deadline():
    r, c = receiver(), candidate()
    r.propose(c, ctx())
    r.tick(ctx(c.start))
    new = replace(candidate(2, 100.1, 1), xyz=((.06, 0., -5.), *c.xyz[1:]))
    r.propose(new, ctx(100.06))
    until = r.deadline
    r.tick(ctx(100.08), nominal_valid={1: True, 2: False})
    assert r.active == c and r.pending is None and r.deadline == until


def test_independent_nominal_geometry_rejects_target_behind_and_sea_curve():
    from types import SimpleNamespace
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_minco_execution import nominal_curve
    c = candidate()
    # Known level straight flight, camera is tilted 28 degrees down, target 8 m ahead.
    prediction = SimpleNamespace(source_stamp=100., prediction_times=(0., 4.),
                                 target_positions=((8., 0., 0.), (12., 0., 0.)))
    assert nominal_curve(c, prediction, FollowMpcSeed(), c.start, c.end)
    prediction.target_positions = ((-8., 0., 0.), (-4., 0., 0.))
    assert not nominal_curve(c, prediction, FollowMpcSeed(), c.start, c.end)
    prediction.target_positions = ((8., 0., 0.), (12., 0., 0.))
    bad = replace(c, xyz=((0., 0., -.1), *c.xyz[1:]))
    assert not nominal_curve(bad, prediction, FollowMpcSeed(), bad.start, bad.end)


def test_accepted_request_retains_measurement_epochs_and_roundtrips():
    from dataclasses import asdict
    from test_hypothetical_follow import scenario
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.follow_minco_execution import accepted_request, request_from_payload
    from uav_control.guidance.follow_problem import future_request
    req, c = future_request(scenario('constant_velocity')), candidate()
    req = replace(req, context=replace(req.context, execution_start_stamp=c.start+.1))
    actual = accepted_request(req, c)
    assert actual.context == req.context
    assert actual.state == c.sample(actual.context.execution_start_stamp)[:10]
    assert actual.measurement_state == req.measurement_state
    restored = request_from_payload(asdict(actual))
    assert restored.context == actual.context and restored.state == actual.state
    assert restored.prior_curve == actual.prior_curve
    # This checks boundary provenance only; geometrically unsuitable requests can still reject.
    with pytest.raises(ValueError):
        FollowProblem(replace(actual, state=(99.,)*10))


def test_velocity_shaping_preserves_emitted_command_on_entry_and_fallback():
    from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState
    from uav_control.guidance.follow_minco_execution import reference_velocity
    r, c = receiver(), candidate()
    r.propose(c, ctx())
    r.tick(ctx(c.start))
    guidance = FlightGuidanceCore()
    actual = FlightKinematicState((0., 0., -5.), (3., 0., 0.))
    guidance.previous_velocity = (3., 0., 0.)
    desired = reference_velocity(c, c.start, c.start, actual.position, .8)
    command = guidance._velocity_command(actual, desired, .05, False, True)
    assert np.linalg.norm(np.asarray(command.velocity[:2])-(3., 0.)) <= 3.*.05+1e-12
    assert command.safety_margin > 0


def test_nominal_end_at_large_ros_epoch_has_no_roundoff_extrapolation():
    from types import SimpleNamespace
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_minco_execution import nominal_curve
    offset = 1791599000.
    c = candidate()
    c = replace(c, start=c.start+offset, end=c.end+offset)
    prediction = SimpleNamespace(source_stamp=100.+offset, prediction_times=(0., 4.),
                                 target_positions=((8., 0., 0.), (12., 0., 0.)))
    assert nominal_curve(c, prediction, FollowMpcSeed(), c.start, c.end)


def test_analysis_includes_minco_control_cycles_in_follow_windows(tmp_path):
    import json
    from pathlib import Path
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
    from p4_follow_analysis import analyze
    (tmp_path/'offline_evidence.jsonl').write_text(json.dumps(dict(
        kind='diagnostic', stamp=100., receipt=100., status='MINCO_FOLLOW',
        visible=True, locked=True))+'\n')
    (tmp_path/'watchdog_end.json').write_text('{}')
    result = analyze(tmp_path, follow_statuses=('FOLLOW', 'MINCO_FOLLOW'),
                     actual_controller='NOMINAL_MINCO')
    assert result['minco_closed_loop']
    assert result['windows']['all_follow']['visible_fraction'] == 1.
