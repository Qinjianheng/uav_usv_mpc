"""P4 nominal FOLLOW proposals and actual rejection ACK recording; no control publisher."""
import json
import time

import rclpy
from uav_usv_interfaces.msg import FollowTrajectory, FollowPlanAck

from uav_control.controllers.follow_research_shadow_node import FollowResearchShadowNode
from uav_control.controllers.follow_transport import proposal_from_event, seconds
from uav_control.controllers.follow_mpc_shadow_node import ShadowResearchRunner


class P4ResearchRunner(ShadowResearchRunner):
    """Math hints may survive mere input waiting; never an accepted reference or permission."""

    def _invalidate_warm(self):
        hint = getattr(self.solver, 'hint', None)
        if (hint is not None and self._follow()
                and hint[0].mission_id == self.mission['mission_id']
                and hint[0].clock_generation == self.adapter.clock_generation):
            return
        super()._invalidate_warm()

    def tick(self, now, monotonic_now):
        """Busy work has no queued request; completion alone rebuilds from latest inputs."""
        if self.pending is not None and not self.pending['future'].done():
            return
        super().tick(now, monotonic_now)


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
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(.01, self.tick)

    def make_runner(self, config):
        """Retain the original synchronization factory, with explicitly separate cache policy."""
        old = super().make_runner(config)
        return P4ResearchRunner(old.config, old.adapter, old.executor, self._publish_event,
                                1/old.period, old.solver, old.request_factory)

    def ack_callback(self, message):
        """Log wrong receiver/identity as invalid ACK; REJECTED never grants execution."""
        key = (int(message.mission_id), int(message.clock_generation), int(message.plan_id))
        publication = self.expected_plans.get(key)
        now = self._ros_seconds()
        ack_stamp = seconds(message.stamp)
        endpoints = self.get_publishers_info_by_topic('/control/follow_ack')
        one_receiver = len(endpoints) == 1 and endpoints[0].node_name == 'trajectory_tracker_node'
        valid = bool(one_receiver and message.receiver == 'trajectory_tracker_node'
                     and publication is not None and publication <= ack_stamp <= now
                     and now-ack_stamp <= .125
                     and message.mission_id == self.runner.mission['mission_id']
                     and message.clock_generation == self.adapter.clock_generation)
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

    def _publish_event(self, event):
        """Existing JSON gates first, then current input/start gate immediately before proposal."""
        super()._publish_event(event)
        before = time.perf_counter()
        message = proposal_from_event(event, self._ros_seconds())
        if message is None:
            return
        # Recheck after message allocation, without changing any original epoch.
        now = self._ros_seconds()
        if (now >= seconds(message.input_valid_until)
                or now >= seconds(message.execution_start_stamp)):
            return
        key = (message.mission_id, message.clock_generation, message.plan_id)
        self.expected_plans[key] = now
        if len(self.expected_plans) > 1024:
            self.expected_plans.pop(next(iter(self.expected_plans)))
        self.follow_proposal_pub.publish(message)
        self.log_file.write(json.dumps(dict(event='follow_proposal', identity=key,
                                            stamp=now, holding_valid_until=0.,
                                            authority=False,
                                            conversion_publish_seconds=time.perf_counter()-before))
                            + '\n')
        self.log_file.flush()

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
