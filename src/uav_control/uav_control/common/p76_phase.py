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
    """Authorize FOLLOW consumers only; this does not describe estimator activity."""
    return getattr(node, 'p76_phase', 'FOLLOW_ACTIVE') == 'FOLLOW_ACTIVE'


def estimator_active(node):
    """Run both DKFs throughout normal guidance and its Y authority handover."""
    return getattr(node, 'p76_phase', 'FOLLOW_ACTIVE') in (
        'FOLLOW_ACTIVE', 'Y_HANDOVER', 'PAPER_INTERCEPT_ACTIVE')


def publish_ack(node, estimator, active, ready=False, source_stamp=0., reason=''):
    """Separate live estimation, source freshness and phase-specific consumption rights."""
    permitted = (follow_active(node) if estimator == 'spatial' else
                 node.p76_phase in ('Y_HANDOVER', 'PAPER_INTERCEPT_ACTIVE'))
    node.ack_pub.publish(String(data=json.dumps(dict(
        schema_version=2,
        estimator=estimator, phase=node.p76_phase, epoch=node.p76_epoch,
        stamp=node.get_clock().now().nanoseconds*1e-9, active=active, ready=ready,
        source_stamp=source_stamp, reason=reason,
        control_authorized=bool(active and ready and permitted)))))
