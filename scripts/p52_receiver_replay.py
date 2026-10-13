#!/usr/bin/env python3
"""Extended hypothetical production solver/receiver replay; no ROS nodes or PX4 publisher."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np

from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState
from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcRequest, PlanningContext
from uav_control.controllers.follow_transport import curve_from_message, proposal_from_event
from uav_control.guidance.follow_admission import FollowAdmission
from uav_control.guidance.follow_contract import ReceiverContext
from uav_control.guidance.follow_limits import ResearchConfig, constraint_snapshot
from uav_control.guidance.follow_minco_execution import (
    NominalFollowReceiver, accepted_request, nominal_attitude, nominal_curve, reference_velocity)
from uav_control.guidance.follow_problem import future_request

FAULTS = ('none', 'two_solve_failures', 'input_interruption', 'performance_deviation',
          'prefix_loss_reentry', 'prediction_change')


def replay(output, fault='none'):
    if fault not in FAULTS:
        raise ValueError('UNKNOWN_FAULT')
    output.mkdir(parents=True, exist_ok=False)
    model = FollowMpcSeed(ResearchConfig(allow_synthetic_predictions=True))
    current = [100.]
    solver = P47FollowGuidedMinco(model, duration=1.6, refinement=False,
                                  online_budget_guard=True, clock=lambda: 0.,
                                  wall_clock=lambda: current[0])
    gate = FollowAdmission()
    receiver = NominalFollowReceiver(limits=constraint_snapshot(model), admission=gate)
    # Call the actual Tracker revocation helper when admission observes a context/performance loss.
    tracker = object.__new__(TrajectoryTrackerNode)
    tracker.follow_receiver, tracker.follow_minco_requested = receiver, True
    rows = []
    tracker.follow_ack_pub = SimpleNamespace(publish=lambda m: rows.append(dict(
        event='tracker_revocation', state=m.state, reasons=list(m.reasons))))
    flight = FlightGuidanceCore()
    sequence, due, first_active, injected_at = 0, 100., None, None
    failed, kept, steps, heights, commands, losses = 0, [], [], [], [], []
    expired = illegal = immediate = fallbacks = initial = successors = accepted = changed = 0
    last_active = 0
    for tick in range(401):
        now = 100.+tick*.05
        current[0] = now
        if first_active is not None and now-first_active >= .4 and injected_at is None:
            if fault != 'none':
                injected_at = now
                rows.append(dict(event='fault_start', stamp=now, fault=fault))
        elapsed = now-injected_at if injected_at is not None else -1.
        interrupted = fault == 'input_interruption' and 0 <= elapsed < .15-1e-8
        bad = fault == 'performance_deviation' and 0 <= elapsed < .6-1e-8
        metrics = dict(horizontal_error=1.6 if bad else 0., relative_speed=.8 if bad else 0.,
                       height_error=0., settled=not bad)
        context = ReceiverContext(now, 1, 0, tick+1, True, True, True, not interrupted,
                                  receiver_boot_id='offline-receiver')
        tracker._follow_context = lambda: context
        old = receiver.active
        reason = gate.update(now, metrics, not interrupted, active=bool(old))
        if reason and (receiver.active or receiver.pending):
            tracker._revoke_nominal_follow(reason)
        prediction = SimpleNamespace(source_stamp=now, prediction_times=(0., 4.),
                                     target_positions=((2*(now-100), 0., 0.),
                                                       (2*(now-100)+8, 0., 0.)))
        eligible = gate.converged(now) or receiver.active is not None
        if eligible and now >= due-1e-8 and receiver.pending is None and not interrupted:
            due = now+.2
            if fault == 'two_solve_failures' and injected_at is not None and failed < 2:
                failed += 1
                kept.append(bool(receiver.active and now < receiver.deadline))
                # Exactly one early retry. After its failure return to the 5 Hz schedule.
                if failed == 1:
                    due = now+.05
                rows.append(dict(event='injected_no_candidate', stamp=now,
                                 old_deadline=receiver.deadline, failure=failed))
            else:
                sequence += 1
                ts = tuple(np.arange(0., 4.01, .1))
                c = PlanningContext(1, sequence, now, now, now, now, now-.01, tick+1,
                                    now+.115)
                r = MpcRequest(c, (-5.+2*(now-100), 0., -5., 2., 0., 0., 0., 0., 0., 0.),
                               ts, tuple((2*(now-100+t), 0., 0.) for t in ts),
                               ((2., 0., 0.),)*len(ts), tuple(map(tuple, np.eye(3))), now)
                r = future_request(r, .15)
                if receiver.active:
                    r = accepted_request(r, receiver.active)
                started = time.perf_counter()
                value = solver.solve(r)
                measured = time.perf_counter()-started
                assert value.valid, value.solver_status
                msg = proposal_from_event(dict(request=asdict(r), output=asdict(value)), now)
                msg.receiver_boot_id, msg.planner_boot_id = 'offline-receiver', 'offline-planner'
                msg.parent_plan_id = receiver.active.plan_id if receiver.active else 0
                curve = curve_from_message(msg)
                assert nominal_attitude(curve, model)
                assert nominal_curve(curve, prediction, model, curve.start, curve.end)
                probe = receiver.active is None and not gate.allowed(now)
                ack = receiver.propose(curve, context, probe=probe)
                rows.append(dict(event='proposal', stamp=now, plan=sequence,
                                 parent=curve.parent_id, probe=probe, state=ack.state,
                                 reasons=ack.reasons, solve_wall_seconds=measured))
                if ack.state == 'ACCEPTED' and receiver.active:
                    accepted += 1
                    steps.append(float(np.max(np.abs(np.subtract(
                        curve.sample(curve.start), receiver.active.sample(curve.start))))))
        checks = {}
        for curve in (receiver.active, receiver.pending):
            if curve and now < min(curve.end, curve.start+.45):
                checks[curve.plan_id] = nominal_curve(
                    curve, prediction, model, max(now, curve.start),
                    min(curve.end, curve.start+.45))
                if fault == 'prediction_change' and injected_at is not None:
                    changed += 1  # New sequence every tick, full current geometry revalidation.
        if fault == 'prefix_loss_reentry' and elapsed == 0 and receiver.active:
            checks[receiver.active.plan_id] = False
        receiver.tick(context, checks)
        for ack in receiver.drain_events():
            rows.append(dict(event='receiver_ack', **asdict(ack)))
            if ack.state == 'ACTIVE':
                curve = receiver.active
                first_active = now if first_active is None else first_active
                if curve.parent_id:
                    successors += 1
                    illegal += curve.parent_id != last_active
                else:
                    initial += 1
                if gate.loss:
                    losses.append(now-gate.loss['stamp'])
                    immediate += now < gate.loss['stamp']+7.-1e-8
                last_active = curve.plan_id
        if old and receiver.active is None:
            fallbacks += 1
            rows.append(dict(event='fallback', **gate.loss))
        position = (-5.+2*(now-100), 0., -5.)
        desired = (reference_velocity(receiver.active, now, now, position, .8)
                   if receiver.active else (2., 0., 0.))
        command = flight._velocity_command(FlightKinematicState(position, (2., 0., 0.)),
                                           desired, .05, True, True)
        commands.append(command.velocity)
        if receiver.active:
            expired += now >= receiver.deadline
            heights.append(abs(receiver.active.sample(now)[2]+5.))
        rows.append(dict(event='owner', stamp=now, state=gate.state,
                         active=receiver.active.plan_id if receiver.active else 0,
                         deadline=receiver.deadline, command=command.velocity))
    counts = Counter(r['state'] for r in rows if r['event'] == 'receiver_ack')
    summary = dict(hypothetical_only=True, not_sitl=True, fault=fault,
                   injected=injected_at is not None, active_events=counts['ACTIVE'],
                   initial_entries=initial, successor_activations=successors,
                   accepted_successors=accepted, fallback_count=fallbacks,
                   pending_successors=int(bool(receiver.pending
                                               and receiver.pending.parent_id)),
                   failed_solve_count=failed, all_failures_kept_legal_old=bool(kept) and all(kept),
                   expired_active_count=expired, illegal_parent_activations=illegal,
                   max_parent_pva_step=max(steps, default=0.),
                   max_reference_height_error=max(heights, default=0.),
                   minimum_reentry_delay=min(losses, default=None),
                   recovery_immediate_reentry_count=immediate,
                   changed_prediction_checks=changed,
                   maximum_command_step=float(np.max(np.linalg.norm(
                       np.diff(commands, axis=0), axis=1))),
                   maximum_command_speed=float(np.max(np.linalg.norm(commands, axis=1))),
                   solver_clock='virtual clock; logged wall cost is not online proof',
                   command_scope='production shaping on equilibrium; no PX4 publication',
                   ack_states=dict(counts), fallback_reasons=dict(Counter(
                       r['reason'] for r in rows if r['event'] == 'fallback')))
    (output/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    for fault in FAULTS:
        print(json.dumps(replay(args.output/fault, fault)))
