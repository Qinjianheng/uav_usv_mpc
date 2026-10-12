#!/usr/bin/env python3
"""Hypothetical solver→wire→receiver fault replay. Never publishes ROS or grants SITL."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np

from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcRequest, PlanningContext
from uav_control.controllers.follow_transport import curve_from_message, proposal_from_event
from uav_control.guidance.follow_admission import FollowAdmission
from uav_control.guidance.follow_contract import ReceiverContext
from uav_control.guidance.follow_limits import ResearchConfig, constraint_snapshot
from uav_control.guidance.follow_minco_execution import (
    NominalFollowReceiver, accepted_request, nominal_attitude, nominal_curve)
from uav_control.guidance.follow_problem import future_request


FAULTS = ('none', 'one_solve_failure', 'no_successor', 'bad_parent', 'prefix_invalid',
          'offboard_lost', 'clock_reversed', 'start_missed')


def replay(output, fault='none'):
    """Known straight 2m/s equilibrium; virtual epochs and zero solver clock are explicit."""
    output.mkdir(parents=True, exist_ok=False)
    model = FollowMpcSeed(ResearchConfig(allow_synthetic_predictions=True))
    current = [100.]
    solver = P47FollowGuidedMinco(model, duration=1.6, refinement=False,
                                  online_budget_guard=True, clock=lambda: 0.,
                                  wall_clock=lambda: current[0])
    gate = FollowAdmission()
    receiver = NominalFollowReceiver(limits=constraint_snapshot(model), admission=gate)
    rows, sequence, due, first_active = [], 0, 100., None
    injected = failed_kept = suppress = False
    pva_steps, heights, expired_active, fallbacks, immediate = [], [], 0, 0, 0
    for tick in range(181):
        now = 100.+tick*.05
        current[0] = now
        metrics = dict(horizontal_error=0., relative_speed=0., height_error=0., settled=True)
        gate.update(now, metrics, True, active=receiver.active is not None)
        context = ReceiverContext(now, 1, 0, tick+1, True, True, True, True,
                                  receiver_boot_id='offline-receiver')
        prediction = SimpleNamespace(source_stamp=now, prediction_times=(0., 4.),
                                     target_positions=((2*(now-100), 0., 0.),
                                                       (2*(now-100)+8, 0., 0.)))
        eligible = gate.converged(now) or receiver.active is not None
        trigger = first_active is not None and now-first_active >= .4 and not injected
        if trigger and fault in ('prefix_invalid', 'offboard_lost', 'clock_reversed'):
            injected = suppress = True
        if (eligible and now >= due-1e-8 and receiver.pending is None and not suppress):
            due = now+.2
            if trigger and fault in ('one_solve_failure', 'no_successor'):
                injected = True
                if fault == 'no_successor':
                    suppress = True
                else:
                    failed_kept = receiver.active is not None and now < receiver.deadline
                    due = now+.05  # matches the separately tested bounded deadline retry
                rows.append(dict(event='injected_no_candidate', stamp=now, fault=fault,
                                 old_deadline=receiver.deadline))
            else:
                sequence += 1
                times = tuple(np.arange(0., 4.01, .1))
                c = PlanningContext(1, sequence, now, now, now, now, now-.01, tick+1,
                                    now+.115)
                r = MpcRequest(c, (-5.+2*(now-100), 0., -5., 2., 0., 0., 0., 0., 0., 0.),
                               times, tuple((2*(now-100+t), 0., 0.) for t in times),
                               ((2., 0., 0.),)*len(times), tuple(map(tuple, np.eye(3))), now)
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
                if trigger and fault == 'bad_parent':
                    curve = replace(curve, parent_id=99999)
                    injected = suppress = True
                probe = receiver.active is None and not gate.allowed(now)
                ack = receiver.propose(curve, context, probe=probe)
                rows.append(dict(event='proposal', stamp=now, plan=sequence,
                                 parent=curve.parent_id, probe=probe, state=ack.state,
                                 reasons=ack.reasons, solve_wall_seconds=measured,
                                 old_deadline=receiver.deadline, start=curve.start))
                if ack.state == 'ACCEPTED' and receiver.active:
                    pva_steps.append(float(np.max(np.abs(np.subtract(
                        curve.sample(curve.start), receiver.active.sample(curve.start))))))
                if trigger and fault == 'start_missed' and not probe:
                    injected = suppress = True
        old = receiver.active
        checks = {}
        for curve in (receiver.active, receiver.pending):
            if curve and now < min(curve.end, curve.start+.45):
                checks[curve.plan_id] = nominal_curve(
                    curve, prediction, model, max(now, curve.start),
                    min(curve.end, curve.start+.45))
        if trigger and fault == 'prefix_invalid' and old:
            checks[old.plan_id] = False
        if trigger and fault == 'offboard_lost':
            context = replace(context, offboard=False)
        if trigger and fault == 'clock_reversed':
            context = replace(context, now=now-.1)
        skip = (fault == 'start_missed' and injected and receiver.pending
                and receiver.pending.start <= now < receiver.pending.start+.051)
        if not skip:
            receiver.tick(context, checks)
            for ack in receiver.drain_events():
                rows.append(dict(event='receiver_ack', **asdict(ack)))
                if ack.state == 'ACTIVE':
                    if first_active is None:
                        first_active = now
                    if gate.loss and now < gate.loss['stamp']+7.-1e-8:
                        immediate += 1
        if old and receiver.active is None:
            fallbacks += 1
        if receiver.active:
            expired_active += now >= receiver.deadline
            heights.append(abs(receiver.active.sample(max(now, receiver.active.start))[2]+5.))
        rows.append(dict(event='owner', stamp=now, state=gate.state,
                         active=receiver.active.plan_id if receiver.active else 0,
                         deadline=receiver.deadline, reference_z=(
                             receiver.active.sample(max(now, receiver.active.start))[2]
                             if receiver.active else None)))
    counts = Counter(r['state'] for r in rows if r['event'] == 'receiver_ack')
    summary = dict(hypothetical_only=True, not_sitl=True, fault=fault, injected=injected,
                   solver_clock='zero-duration deterministic fixture; measured wall cost logged '
                   'separately and is not online timing proof',
                   active_events=counts['ACTIVE'], ack_states=dict(counts),
                   fallback_count=fallbacks,
                   failed_solve_kept_old=failed_kept, expired_active_count=expired_active,
                   max_parent_pva_step=max(pva_steps, default=0.),
                   max_reference_height_error=max(heights, default=0.),
                   recovery_immediate_reentry_count=immediate,
                   rejection_reasons=dict(Counter(
                       reason for r in rows if r['event'] in ('proposal', 'receiver_ack')
                       for reason in r['reasons'])))
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
