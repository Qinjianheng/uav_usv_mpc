#!/usr/bin/env python3
"""Replay real successor requests through current math/wire/receiver code, without ROS flight."""
import argparse
from collections import Counter, deque
from dataclasses import asdict, replace
import hashlib
from io import StringIO
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from uav_usv_interfaces.msg import PredictedTargetPoint, TargetPrediction
from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, planning_context_from_payload
from uav_control.controllers.follow_mpc_shadow_node import ShadowPrediction
from uav_control.controllers.follow_transport import curve_from_message, stamp
from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode, P4ResearchRunner
from uav_control.guidance.camera_visibility import (
    CameraExtrinsics, CameraIntrinsics, TargetBoundingSphere, VisibilityConstraints,
)
from uav_control.guidance.follow_contract import ReceiverContext
from uav_control.guidance.follow_limits import ResearchConfig, constraint_snapshot
from uav_control.guidance.follow_time_contract import input_time_contract
from uav_control.guidance.follow_math_runtime import configure_numerical_threads
from uav_control.guidance.follow_minco_execution import (
    NominalFollowReceiver, nominal_curve, request_from_payload,
)
from uav_control.guidance.follow_rollout import FollowRolloutConfig
from uav_control.guidance.p6_follow import P6Policy, P6Runtime
from uav_control.guidance.planned_attitude import AttitudeConfig


class ReplayClock:
    """Original ROS origin plus measured compute and explicitly recorded elapsed delays."""

    def __init__(self, origin):
        """Do not change any input acquisition/source/sample timestamp."""
        self.origin, self.started, self.delay = origin, time.perf_counter(), 0.

    def __call__(self):
        """A counterfactual process clock; never presented as new raw evidence."""
        return self.origin+self.delay+time.perf_counter()-self.started


def sha(path):
    """Retain original source identity."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def model_from_record(row, target_time_profile='baseline_125'):
    """Restore the actual recorded constraints and camera, without nominal defaults."""
    m = row['input_provenance']['model_config']
    result = FollowMpcSeed(
        ResearchConfig(**dict(m['mpc'], target_time_profile=target_time_profile)),
        intrinsics=CameraIntrinsics(**m['intrinsics']),
        extrinsics=CameraExtrinsics(**m['extrinsics']),
        target=TargetBoundingSphere(**m['target']),
        visibility=VisibilityConstraints(**m['visibility']),
        attitude_config=AttitudeConfig(**m['attitude']))
    result.p6_policy = P6Policy()
    return result


def prediction_message(prediction, accelerations=()):
    """Convert real forecast values through the same ROS message layout."""
    message = TargetPrediction()
    message.mission_id, message.sequence_id = prediction.mission_id, prediction.sequence_id
    for name in ('source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until'):
        setattr(message, name, stamp(getattr(prediction, name)))
    message.frame_id, message.source, message.model = (
        prediction.frame_id, prediction.source, prediction.model)
    message.prediction_horizon, message.valid = prediction.prediction_horizon, True
    for i, relative in enumerate(prediction.prediction_times):
        point = PredictedTargetPoint()
        encoded = stamp(relative)
        point.relative_time.sec, point.relative_time.nanosec = encoded.sec, encoded.nanosec
        for field, values in (('position', prediction.target_positions),
                              ('velocity', prediction.target_velocities)):
            for axis, value in zip('xyz', values[i]):
                setattr(getattr(point, field), axis, float(value))
        if accelerations:
            for axis, value in zip('xyz', accelerations[i]):
                setattr(point.acceleration, axis, float(value))
        if prediction.target_covariances:
            point.position_covariance = list(prediction.target_covariances[i])
        message.samples.append(point)
    return message


def raw_prediction(record, target_time_profile='baseline_125'):
    """Preserve all original epochs and identity of an actual recorded IMM forecast."""
    result = ShadowPrediction(
        record['mission'], record['sequence'], record['source_stamp'],
        record['observation_stamp'], record['generated_stamp'], record['valid_until'],
        record['frame'], record['source'], record['model'], record['times'][-1],
        tuple(record['times']), tuple(map(tuple, record['p'])), tuple(map(tuple, record['v'])),
        tuple(map(tuple, record['covariance'])))
    return experimental_prediction(result, target_time_profile)


def experimental_prediction(prediction, profile):
    """Counterfactual current expiry policy; original mean/covariance/source epochs retained."""
    if profile == 'baseline_125':
        return prediction
    return replace(prediction, valid_until=prediction.observation_stamp
                   + input_time_contract(profile).observation_age)


def summary(values):
    """SI seconds; absent timing remains unknown."""
    values = [v for v in values if v is not None]
    if not values:
        return dict(count=0)
    return dict(count=len(values), **dict(zip(
        ('min', 'p50', 'p95', 'max'), map(float, np.percentile(values, (0, 50, 95, 100))))))


def replay(flight, output, target_time_profile='baseline_125'):
    """All36 actual successor inputs; independent receiver models never grant flight authority."""
    output.mkdir(parents=True, exist_ok=False)
    paths = sorted((flight/'shadow').glob('mpc_seed_shadow_*.jsonl'))
    records = [(p, line, text, json.loads(text)) for p in paths
               for line, text in enumerate(p.open(), 1)]
    cases = [r for r in records if r[3]['event'] == 'completion'
             and (r[3].get('dispatch_timing') or {}).get('policy') == 'successor_delivery']
    if not cases:
        raise ValueError('NO_RECORDED_SUCCESSOR_REQUESTS')
    model_config = cases[0][3]['input_provenance']['model_config']
    if any(r[3]['input_provenance']['model_config'] != model_config for r in cases):
        raise ValueError('MIXED_RECORDED_MODEL_CONFIGURATIONS')
    raw_path = flight/'offline_evidence.jsonl'
    raw = [json.loads(line) for line in raw_path.open()]
    predictions = [r for r in raw if r['kind'] == 'p6_prediction' and r['valid']]
    prediction_by_id = {(r['mission'], r['sequence']): r for r in predictions}
    diagnostics = [r for r in raw if r['kind'] == 'p6_imm' and r['valid']]
    debug = [r for r in raw if r['kind'] == 'execution_debug' and r.get('execution_context')]
    receives = {r['plan']: r for r in raw if r['kind'] == 'follow_receive'}
    finals = {r['cycle_id']: r for _, _, _, r in records
              if r['event'] == 'follow_publication_final'}
    transports = []
    for _, _, _, row in cases:
        cid = row['request']['context']['cycle_id']
        final, receive = finals[cid], receives.get(cid)
        if receive and final.get('wire_cycle_seconds') is not None:
            transports.append(receive['receive_monotonic']-row['cycle_started_monotonic']
                              - final['wire_cycle_seconds'])
    unmeasured_transport = max(transports)
    math_runtime = configure_numerical_threads(1)
    contract = input_time_contract(target_time_profile)
    model = model_from_record(cases[0][3], target_time_profile)
    solver = P47FollowGuidedMinco(
        model, duration=1.6, refinement=False, rolling=True,
        online_budget_guard=True, rollout_config=FollowRolloutConfig(reference_mode='rear_point'))
    outputs = []
    receiver_samples, publication_samples = deque(maxlen=16), deque(maxlen=16)
    for path, line, text, row in cases:
        request = request_from_payload(row['request'])
        if target_time_profile != 'baseline_125':
            context = planning_context_from_payload(dict(
                asdict(request.context), target_time_profile=target_time_profile,
                prediction_valid_until=request.context.observation_stamp+contract.observation_age))
            limits = constraint_snapshot(model)
            parent_model = replace(request.prior_curve, constraint_snapshot=limits.payload,
                                   constraint_fingerprint=limits.fingerprint)
            request = replace(request, context=context, prior_curve=parent_model)
        c, parent = request.context, request.prior_curve
        old = row['candidate_output']['metrics']['worker_timeline']
        clock = ReplayClock(request.now_stamp)
        clock.delay = row['preparation_profile']['total_seconds']
        solver.wall_clock = clock
        runner = P4ResearchRunner(
            model.config, SimpleNamespace(clock_generation=c.clock_generation),
            None, lambda _: None, rate_hz=5., solver=solver)
        runner.deadline_scheduling, runner.successor_scheduling = True, 'delivery'
        runner.execution_status = dict(
            stamp=request.now_stamp, active=parent.plan_id, pending=0,
            deadline=min(parent.end, parent.start+.45), allowed=False, state='FOLLOW_STABLE')
        runner.current_receiver_epoch = tuple(row['receiver_epoch_at_dispatch'])
        runner.planner_boot_id, runner.math_runtime = parent.planner_boot_id, math_runtime
        runner.mission = row['input_provenance']['mission']
        runner.prediction = experimental_prediction(
            ShadowPrediction(**row['input_provenance']['prepared_prediction']),
            target_time_profile)
        runner.prediction_provenance = row['input_provenance']['prediction']
        runner.receiver_cost_samples, runner.publication_cost_samples = (
            receiver_samples, publication_samples)
        item = dict(
            cycle=c.cycle_id, parent=parent.plan_id, source=str(path), source_line=line,
            source_line_sha256=hashlib.sha256(text.encode()).hexdigest(),
            original_context=row['request']['context'], model_context=asdict(c),
            target_time_profile=target_time_profile, real_tracker_accepted=False,
            original_worker=old, current_source_model='recorded-state one-step successor')
        try:
            locked, _ = runner._prepare_delivery((request, None), request.now_stamp)
        except ValueError as error:
            outputs.append(dict(item, failure=str(error), phase='dispatch', published=False))
            continue
        clock.delay += old['solve_start_monotonic']-row['submitted_monotonic']
        solved = runner._solve_request(locked)
        item.update(plan=asdict(locked.delivery_plan), worker=solved.metrics['worker_timeline'],
                    worker_valid=solved.valid, worker_status=solved.solver_status)
        item['physical_checks'] = {key: solved.metrics.get(key) for key in (
            'full_projection_validated', 'maximum_horizontal_speed', 'maximum_vertical_speed',
            'maximum_horizontal_acceleration', 'maximum_vertical_acceleration',
            'maximum_horizontal_jerk', 'maximum_vertical_jerk', 'maximum_yaw_rate',
            'maximum_tilt_rad', 'maximum_specific_thrust', 'minimum_horizontal_margin',
            'minimum_vertical_margin', 'constraint_fingerprint')}
        if not solved.valid:
            outputs.append(dict(item, failure=solved.solver_status,
                                phase='worker', published=False))
            continue
        clock.delay += (row['completion_timeline']['future_observed_monotonic']
                        - old['solve_end_monotonic'])
        planner = object.__new__(P4FollowPlannerNode)
        planner.runner, planner.adapter = runner, runner.adapter
        planner._ros_seconds, planner.log_file = clock, StringIO()
        planner.receiver_epoch = (runner.current_receiver_epoch, request.now_stamp)
        planner.receiver_status = runner.execution_status
        planner.follow_execution_enabled = planner.follow_admission_required = True
        planner.prediction_policy, planner.planner_boot_id = 'full', parent.planner_boot_id
        planner.execution_curves, planner.expected_plans = {}, {}
        messages = []
        planner.follow_proposal_pub = SimpleNamespace(publish=messages.append)
        record = dict(started=clock.started, receiver_epoch=runner.current_receiver_epoch)
        outcome = planner._publish_typed_completion(locked, solved, record)
        if publication_samples:
            # ROS model delay includes recorded poll; live monotonic compute does not.
            finish_ros = outcome.get('wire_finish_ros', clock())
            publication_samples[-1] = (
                c.observation_stamp,
                max(0., finish_ros-solved.metrics['worker_timeline']['solve_end_ros']))
        item.update(publication=outcome, published=outcome['published'])
        if not outcome['published'] or outcome.get('reason'):
            outputs.append(dict(item, failure=outcome.get('reason'), phase='publication'))
            continue
        message = messages[0]
        receive = receives.get(c.cycle_id)
        transport = (receive['receive_monotonic']-row['cycle_started_monotonic']
                     - finals[c.cycle_id]['wire_cycle_seconds'] if receive
                     else unmeasured_transport)
        receiver_clock = ReplayClock(outcome['wire_finish_ros']+transport)
        tracker = object.__new__(TrajectoryTrackerNode)
        tracker.follow_model, tracker.p6_runtime = model, P6Runtime(model)
        tracker.follow_receiver = NominalFollowReceiver(limits=constraint_snapshot(model))
        tracker.follow_receiver.active = parent
        tracker.follow_receiver.deadline = runner.execution_status['deadline']
        tracker.follow_minco_requested, tracker.follow_stable_ticks = True, 10
        tracker.follow_math_runtime = math_runtime
        original_wire = prediction_by_id[(c.mission_id, c.prediction_sequence_id)]
        assert original_wire['source_stamp'] == runner.prediction.source_stamp
        assert original_wire['observation_stamp'] == runner.prediction.observation_stamp
        tracker.latest_prediction = prediction_message(runner.prediction, original_wire['a'])
        tracker._ros_seconds = receiver_clock
        tracker.get_publishers_info_by_topic = lambda _: [SimpleNamespace(
            node_name='p4_follow_planner_node')]
        source_context = [d for d in debug if d['stamp'] <= request.now_stamp][-1][
            'execution_context']

        def context_at(now, prediction, navigation_stamp):
            return ReceiverContext(
                now, c.mission_id, c.clock_generation, prediction.sequence_id,
                source_context['following'], source_context['offboard'], source_context['armed'],
                source_context['visible'], receiver_boot_id=parent.receiver_boot_id,
                navigation_fresh=0 <= now-navigation_stamp <= .125,
                target_fresh=0 <= now-prediction.observation_stamp <= contract.observation_age,
                prediction_fresh=(prediction.source_stamp <= now < prediction.valid_until
                                  and now-prediction.source_stamp <= contract.prediction_age),
                observation_fresh=0 <= now-prediction.observation_stamp <= .150)
        tracker._follow_context = lambda: context_at(
            receiver_clock(), runner.prediction, c.navigation_stamp)
        tracker._update_follow_admission = lambda _: ''
        tracker._schedule_follow_activation = lambda: None
        ack, receiver_debug = [], []
        tracker.follow_ack_pub = SimpleNamespace(publish=ack.append)
        tracker.follow_debug_pub = SimpleNamespace(publish=receiver_debug.append)
        backlog = [d for d in diagnostics if d['sequence_id'] <= c.prediction_sequence_id][-16:]
        for d in backlog:
            tracker.p6_runtime.diagnostic(SimpleNamespace(data=json.dumps(d)))
        # An independently receiving process does not wait for planner post-wire audit
        # or the probe's construction of a historical diagnostic backlog.
        receiver_clock = ReplayClock(outcome['wire_finish_ros']+transport)
        tracker._ros_seconds = receiver_clock
        tracker.follow_trajectory_callback(message)
        received_report = json.loads(receiver_debug[-1].data)
        receiver_samples.append((received_report['ack_publish_end_ros'],
                                 received_report['receiver_compute_seconds']))
        item.update(
            receiver=received_report, receiver_transport_seconds=transport,
            transport_source=('actual recorded DDS elapsed' if receive
                              else 'maximum measured DDS scenario'),
            model_ack=ack[-1].state, model_ack_reasons=list(ack[-1].reasons),
            original_input_margin_at_ack=(locked.delivery_plan.accept_until
                                          - received_report['ack_publish_end_ros']))
        if ack[-1].state != 'ACCEPTED':
            outputs.append(dict(item, failure=','.join(ack[-1].reasons), phase='receiver'))
            continue
        curve = curve_from_message(message)
        continuity = np.asarray(curve.sample(curve.start))-parent.sample(curve.start)
        item['maximum_handover_pva_yaw_rate_delta'] = float(np.max(np.abs(continuity)))
        latest = [p for p in predictions if p['receipt'] <= curve.start]
        prediction = raw_prediction(latest[-1], target_time_profile)
        nav = [d for d in debug if d['stamp'] <= curve.start][-1]['state_stamp']
        detail = {}
        valid_prefix = nominal_curve(curve, prediction, model, curve.start,
                                     min(curve.end, curve.start+.45), detail)
        activated = tracker.follow_receiver.tick(
            context_at(curve.start, prediction, nav), nominal_valid=valid_prefix)
        item.update(
            model_activation=activated.state if activated else None,
            activation_prefix=detail,
            original_activation_prediction_sequence=prediction.sequence_id,
            failure='' if activated and activated.state == 'ACTIVE' else 'MODEL_ACTIVATION_FAILED')
        outputs.append(item)
    (output/'cycles.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in outputs))
    live = [r for r in outputs if r.get('worker')]
    original_common_wall = [r['original_worker']['solve_end_monotonic']
                            - r['original_worker']['solve_start_monotonic'] for r in live]
    report = dict(
        schema='p61-delivery-replay-v1', inputs=len(cases), outputs=len(outputs),
        target_time_profile=target_time_profile, input_time_contract=asdict(contract),
        published=sum(r['published'] for r in outputs),
        model_accepted=sum(r.get('model_ack') == 'ACCEPTED' for r in outputs),
        model_active=sum(r.get('model_activation') == 'ACTIVE' for r in outputs),
        failures=dict(Counter(r['failure'] for r in outputs if r.get('failure'))),
        worker_wall=summary(r['worker']['solve_end_monotonic']
                            - r['worker']['solve_start_monotonic'] for r in live),
        original_common_worker_wall=summary(original_common_wall),
        receiver_wall=summary(r.get('receiver', {}).get('receiver_compute_seconds')
                              for r in outputs),
        ack_input_margin=summary(r.get('original_input_margin_at_ack') for r in outputs),
        math_runtime=math_runtime, source_sha256={str(p): sha(p) for p in (*paths, raw_path)},
        script_sha256=sha(Path(__file__)), real_tracker_accepted=False, sitl_started=False,
        limitations=[
            'Current measured compute on recorded-state one-step requests, not real ROS/PX4.',
            'Recorded preparation/queue/poll/DDS elapsed reused; original input epochs unchanged.',
            'Unpublished original cases use the declared maximum observed DDS scenario.',
            'Original ACTIVE parent bootstrap; live admission/performance history not simulated.',
            'Experimental profiles rebuild expiry and parent policy as counterfactual models; '
            'old actual ACK/TTL is not new-policy authority. Mean/covariance math is unchanged.',
            'Model activation uses causal monitor receipt as a proxy for latest IMM callback.',
            'No inference of60s ownership or continuous closed-loop physical success.'])
    (output/'analysis.json').write_text(json.dumps(report, indent=2)+'\n')
    (output/'SHA256SUMS').write_text(''.join(
        sha(p)+'  '+p.name+'\n' for p in (output/'cycles.jsonl', output/'analysis.json')))
    return report


def main():
    """Read immutable evidence and create a new isolated result directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('flight', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target-time-profile', default='baseline_125',
                        choices=('baseline_125', 'experiment_150', 'experiment_200'))
    args = parser.parse_args()
    print(json.dumps(replay(args.flight.resolve(), args.output.resolve(),
                            args.target_time_profile), indent=2))


if __name__ == '__main__':
    main()
