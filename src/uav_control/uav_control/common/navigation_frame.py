"""Latched fail-closed event for a discontinuous PX4 local coordinate frame."""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


FRAME_FAULT_TOPIC = '/navigation/frame_fault'


def navigation_frame_fault_qos():
    """Late subscribers must observe the same fault; empty events cannot clear it."""
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def install_navigation_frame_guard(node, on_fault=None):
    """Require a full graph restart after a spatial reset, never transform old references."""
    node.navigation_frame_fault = ''

    def receive(message):
        reason = message.data.strip()
        if not reason or node.navigation_frame_fault:
            return
        node.navigation_frame_fault = reason
        if on_fault is not None:
            on_fault(reason)

    node.navigation_frame_fault_sub = node.create_subscription(
        String, FRAME_FAULT_TOPIC, receive, navigation_frame_fault_qos())
