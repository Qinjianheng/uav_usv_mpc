"""Explicit P76 phase subscriptions and estimator acknowledgements."""
import json

from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def phase_qos():
    """Retain the sole Tracker's current phase for late-starting consumers."""
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def install_phase_gate(node, on_change=None):
    """Opt in through an explicit topic parameter; legacy nodes remain active."""
    if not node.has_parameter('p76_mode_topic'):
        node.declare_parameter('p76_mode_topic', '')
    topic = str(node.get_parameter('p76_mode_topic').value)
    node.p76_phase = 'FOLLOW_ACTIVE'
    node.p76_epoch = 0
    node.p76_y_stamp = 0.
    if not topic:
        return

    def receive(message):
        try:
            data = json.loads(message.data)
            epoch = int(data['epoch'])
            phase = str(data['phase'])
            if epoch < node.p76_epoch:
                return
            changed = epoch != node.p76_epoch or phase != node.p76_phase
            node.p76_phase, node.p76_epoch = phase, epoch
            node.p76_y_stamp = float(data.get('y_stamp', 0.))
            if changed and on_change is not None:
                on_change()
        except (ValueError, TypeError, KeyError):
            node.p76_phase = 'SAFE_RECOVERY'
    node.p76_phase_sub = node.create_subscription(String, topic, receive, phase_qos())


def follow_active(node):
    """Keep legacy nodes active and freeze an explicit P76 non-FOLLOW phase."""
    return getattr(node, 'p76_phase', 'FOLLOW_ACTIVE') == 'FOLLOW_ACTIVE'


def publish_ack(node, estimator, active, ready=False, source_stamp=0., reason=''):
    """Expose estimator activity and original measurement authority per mode epoch."""
    node.ack_pub.publish(String(data=json.dumps(dict(
        estimator=estimator, phase=node.p76_phase, epoch=node.p76_epoch,
        stamp=node.get_clock().now().nanoseconds*1e-9, active=active, ready=ready,
        source_stamp=source_stamp, reason=reason))))
