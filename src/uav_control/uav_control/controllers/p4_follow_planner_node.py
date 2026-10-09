"""P4 nominal FOLLOW proposals and actual rejection ACK recording; no control publisher."""
from dataclasses import asdict
import json
import time
import secrets
from types import SimpleNamespace

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from uav_usv_interfaces.msg import FollowTrajectory, FollowPlanAck, FollowReceiverState

from uav_control.controllers.follow_research_shadow_node import FollowResearchShadowNode
from uav_control.controllers.follow_transport import proposal_from_event, seconds
from uav_control.controllers.follow_mpc_shadow_node import (
    ShadowResearchRunner, completion_rejection,
)


class P4ResearchRunner(ShadowResearchRunner):
    """Math hints may survive mere input waiting; never an accepted reference or permission."""

    def _invalidate_warm(self):
        hint = getattr(self.solver, 'hint', None)
        if (hint is not None and self._follow()
                and hint[0].mission_id == self.mission['mission_id']
                and hint[0].clock_generation == self.adapter.clock_generation):
            return
        super()._invalidate_warm()

    def event(self, *args, **kwargs):
        """Bind each completed solve to the receiver identity observed at dispatch."""
        result = super().event(*args, **kwargs)
        result['receiver_epoch_at_dispatch'] = (self.pending.get('receiver_epoch')
                                                if self.pending else None)
        return result

    def tick(self, now, monotonic_now):
        """Busy work has no queued request; completion alone rebuilds from latest inputs."""
        if self.pending is not None and not self.pending['future'].done():
            return
        previous = self.pending
        super().tick(now, monotonic_now)
        if self.pending is not None and self.pending is not previous:
            self.pending['receiver_epoch'] = getattr(self, 'current_receiver_epoch', None)


class P4FollowPlannerNode(FollowResearchShadowNode):
    """Keep one worker/latest input and all existing completion/serialization TTL checks."""

    node_name = 'p4_follow_planner_node'

    def __init__(self):
        """Publish a distinct proposal topic; ACK is subscribed only, never synthesized."""
        super().__init__()
        self.follow_proposal_pub = self.create_publisher(
            FollowTrajectory, '/planning/follow_trajectory', 1)
        self.follow_ack_sub = self.create_subscription(
            FollowPlanAck, '/control/follow_ack', self.ack_callback, 10)
        self.ack_file = self.log_path.with_name('follow_ack_'+self.log_path.name).open('x')
        self.expected_plans = {}
        self.declare_parameter('prediction_revalidation', 'latest_only')
        self.prediction_policy = self.get_parameter('prediction_revalidation').value
        if self.prediction_policy not in ('latest_only', 'full'):
            raise ValueError('INVALID_PREDICTION_POLICY')
        self.planner_boot_id, self.receiver_epoch = secrets.token_hex(16), None
        epoch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.receiver_epoch_sub = self.create_subscription(
            FollowReceiverState, '/control/follow_receiver_state', self.epoch_callback, epoch_qos)
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(.01, self.tick)

    def epoch_callback(self, message):
        """Trust the unique actual receiver graph endpoint, never a proposal counter."""
        endpoints = self.get_publishers_info_by_topic('/control/follow_receiver_state')
        now, stamp = self._ros_seconds(), seconds(message.stamp)
        valid = (len(endpoints) == 1 and endpoints[0].node_name == 'trajectory_tracker_node'
                 and message.receiver == 'trajectory_tracker_node' and message.receiver_boot_id
                 and 0 <= now-stamp <= .2)
        if not valid:
            self.receiver_epoch = None
            self.runner.current_receiver_epoch = None
            return
        identity = (message.receiver_boot_id, int(message.clock_generation),
                    int(message.mission_id), self.adapter.clock_generation)
        if (self.receiver_epoch and identity[0] == self.receiver_epoch[0][0]
                and identity[1] < self.receiver_epoch[0][1]):
            return
        self.receiver_epoch = (identity, stamp)
        self.runner.current_receiver_epoch = identity

    def make_runner(self, config):
        """Retain the original synchronization factory, with explicitly separate cache policy."""
        old = super().make_runner(config)
        return P4ResearchRunner(old.config, old.adapter, old.executor, self._publish_event,
                                1/old.period, old.solver, old.request_factory)

    def ack_callback(self, message):
        """Log wrong receiver/identity as invalid ACK; REJECTED never grants execution."""
        key = (message.receiver_boot_id, message.planner_boot_id, int(message.mission_id),
               int(message.clock_generation), int(message.plan_id))
        publication = self.expected_plans.get(key)
        now = self._ros_seconds()
        ack_stamp = seconds(message.stamp)
        endpoints = self.get_publishers_info_by_topic('/control/follow_ack')
        one_receiver = len(endpoints) == 1 and endpoints[0].node_name == 'trajectory_tracker_node'
        valid = bool(one_receiver and message.receiver == 'trajectory_tracker_node'
                     and publication is not None and publication <= ack_stamp <= now
                     and now-ack_stamp <= .125
                     and message.mission_id == self.runner.mission['mission_id']
                     and self.receiver_epoch is not None
                     and key[:1] == self.receiver_epoch[0][:1]
                     and key[3] == self.receiver_epoch[0][1]
                     and key[1] == self.planner_boot_id)
        unexpected_accept = message.state in ('ACCEPTED', 'ACTIVE')
        self.ack_file.write(json.dumps(dict(
            receipt=self._ros_seconds(), stamp=seconds(message.stamp),
            identity=key, state=message.state, reasons=list(message.reasons),
            active_plan_id=message.active_plan_id, pending_plan_id=message.pending_plan_id,
            control_owner=message.control_owner, replaced=message.replaced,
            identity_valid=valid,
            publish_to_ack_seconds=seconds(message.stamp)-publication if valid else None,
            accepted_by_tracker=False, unexpected_accept=unexpected_accept))+'\n')
        self.ack_file.flush()

    def _proposal_rejection(self, event, now):
        """Check the complete dispatch-to-proposal budget including new-version validation."""
        start = event.get('cycle_started_monotonic')
        if start is None:
            return 'CYCLE_ORIGIN_MISSING'
        payload = event['request']
        request = SimpleNamespace(context=SimpleNamespace(**payload['context']),
                                  now_stamp=payload['now_stamp'])
        return completion_rejection(request, self.runner.config, now,
                                    self.runner.mission['mission_id'],
                                    self.adapter.clock_generation, time.monotonic()-start,
                                    self.runner._follow())

    def _publish_event(self, event):
        """Include prediction revalidation and wire publication in the final cycle audit."""
        super()._publish_event(event)
        if not event['output'].get('valid') or not event.get('request'):
            return
        outcome = self._publish_proposal(event)
        finished = time.monotonic()
        self.log_file.write(json.dumps(dict(
            event='follow_publication_final', cycle_id=event['request']['context']['cycle_id'],
            stamp=self._ros_seconds(), whole_cycle_time=finished-event['cycle_started_monotonic'],
            **outcome))+'\n')
        self.log_file.flush()

    def _publish_proposal(self, event):
        """Recheck all epochs after allocation; a published nominal curve grants no holding."""
        before = time.perf_counter()
        reason = self._proposal_rejection(event, self._ros_seconds())
        if reason:
            return dict(published=False, reason=reason)
        proposal_event = self._prediction_event(event)
        if proposal_event is None:
            return dict(published=False, reason='PREDICTION_CHANGED_OR_REVALIDATION_FAILED')
        message = proposal_from_event(proposal_event, self._ros_seconds())
        if message is None:
            return dict(published=False, reason='INPUT_OR_START_EXPIRED')
        now = self._ros_seconds()
        reason = self._proposal_rejection(proposal_event, now)
        if reason:
            return dict(published=False, reason=reason)
        if self.receiver_epoch is None:
            return dict(published=False, reason='RECEIVER_EPOCH_UNAVAILABLE')
        epoch, epoch_stamp = self.receiver_epoch
        dispatched = event.get('receiver_epoch_at_dispatch')
        if (dispatched is None or tuple(dispatched) != epoch or not 0 <= now-epoch_stamp <= .2
                or epoch[2] != message.mission_id or epoch[3] != self.adapter.clock_generation):
            return dict(published=False, reason='RECEIVER_EPOCH_CHANGED')
        message.receiver_boot_id, message.clock_generation = epoch[:2]
        message.planner_boot_id = self.planner_boot_id
        key = (message.receiver_boot_id, message.planner_boot_id, message.mission_id,
               message.clock_generation, message.plan_id)
        self.expected_plans[key] = now
        if len(self.expected_plans) > 1024:
            self.expected_plans.pop(next(iter(self.expected_plans)))
        self.follow_proposal_pub.publish(message)
        crossed = self._proposal_rejection(proposal_event, self._ros_seconds())
        self.log_file.write(json.dumps(dict(event='follow_proposal', identity=key,
                                            stamp=now, holding_valid_until=0., authority=False,
                                            conversion_publish_seconds=time.perf_counter()-before))
                            + '\n')
        self.log_file.flush()
        return dict(published=True, reason=crossed, publication_crossed_deadline=bool(crossed))

    def _prediction_event(self, event):
        """Reject superseded output (A) or fully reassess unchanged coefficients (B)."""
        prediction = self.runner.prediction
        request_data = event.get('request')
        if prediction is None or not request_data or not event['output'].get('valid'):
            return None
        if prediction.sequence_id == request_data['context']['prediction_sequence_id']:
            return event
        report = dict(valid=False, reason='PREDICTION_CHANGED', policy=self.prediction_policy)
        proposed = None
        if self.prediction_policy == 'full':
            from uav_control.controllers.follow_mpc_seed import MpcSeedResult, PlanningContext
            from uav_control.guidance.follow_problem import FutureRequest
            from uav_control.guidance.follow_revalidation import revalidate_prediction
            payload = dict(request_data)
            payload['context'] = PlanningContext(**payload['context'])
            request = FutureRequest(**payload)
            until = min(request.context.navigation_stamp+.125,
                        request.context.attitude_stamp+.125,
                        prediction.observation_stamp+.125, prediction.source_stamp+.125,
                        prediction.valid_until, request.context.execution_start_stamp)
            remaining = until-self._ros_seconds()-.005
            if event.get('cycle_started_monotonic') is not None:
                remaining = min(remaining, self.runner.config.solve_budget
                                - (time.monotonic()-event['cycle_started_monotonic'])-.005)
            if remaining <= 0:
                return None
            updated, report = revalidate_prediction(
                request, MpcSeedResult(**event['output']), prediction, self._ros_seconds(),
                self.runner.solver.model, budget=remaining)
            if report['valid'] and self.runner.prediction is prediction:
                proposed = dict(event, request=asdict(updated))
                proposed['output'] = dict(
                    event['output'], context=asdict(updated.context), metrics=dict(
                        event['output']['metrics'],
                        minimum_horizontal_margin=report['minimum_horizontal_margin'],
                        minimum_vertical_margin=report['minimum_vertical_margin'],
                        prediction_revalidation=report))
        self.log_file.write(json.dumps(dict(event='prediction_revalidation',
                                            cycle_id=request_data['context']['cycle_id'],
                                            stamp=self._ros_seconds(), report=report,
                                            new_prediction=asdict(prediction)))+'\n')
        self.log_file.flush()
        return proposed

    def destroy_node(self):
        """Close the independent receiver log before the existing worker shutdown."""
        if hasattr(self, 'ack_file'):
            self.ack_file.close()
        return super().destroy_node()


def main(args=None):
    """Run proposals; final PX4 authority stays exclusively with original Tracker."""
    rclpy.init(args=args)
    node = None
    try:
        node = P4FollowPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
