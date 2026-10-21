"""Continuous delayed six-state estimation with separate FOLLOW consumption authority."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from uav_usv_interfaces.msg import TargetObservation, TargetState

from uav_control.common.navigation_frame import install_navigation_frame_guard
from uav_control.common.p76_phase import estimator_active, install_phase_gate, publish_ack
from .delayed_spatial_kalman import DelayedSpatialKalman
from .target_kalman_filter import measurement_from_observation


class SpatialDkfNode(Node):
    """Keep estimating after Y; only FOLLOW consumers may use this target interface."""

    def __init__(self):
        super().__init__('spatial_dkf_node')
        defaults = dict(input_topic='/perception/front/target_observation',
                        state_topic='/tracking/target_state', update_rate_hz=50.,
                        acceleration_std=.5, initial_velocity_std=5., history_seconds=1.,
                        maximum_source_age=.125, p76_mode_topic='/control/p76_mode')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.filter = DelayedSpatialKalman(**{k: self.get_parameter(k).value for k in (
            'acceleration_std', 'initial_velocity_std', 'history_seconds', 'maximum_source_age')})
        self.state_pub = self.create_publisher(
            TargetState, self.get_parameter('state_topic').value, 10)
        self.ack_pub = self.create_publisher(String, '/tracking/p76_filter_ack', 10)
        self.last_now = None
        install_phase_gate(self, self.mode_changed)
        install_navigation_frame_guard(self, self.frame_failed)
        self.subscription = self.create_subscription(TargetObservation,
                                                     self.get_parameter('input_topic').value,
                                                     self.receive, 10)
        self.timer = self.create_timer(1/self.get_parameter('update_rate_hz').value, self.tick)

    def frame_failed(self, reason):
        """Remove every posterior across a physical frame or clock discontinuity."""
        self.filter.reset()

    def mode_changed(self):
        """Keep estimation history unchanged across normal phase and authority labels."""
        if not estimator_active(self):
            self.filter.reset()

    def check_clock(self, now):
        """Detect rollback before accepting the first measurement in a new clock epoch."""
        if self.last_now is not None and now < self.last_now:
            self.frame_failed('SPATIAL_DKF_CLOCK_RESET')
        self.last_now = now

    def receive(self, message):
        """Update at the original RGB-D acquisition; reject invalid covariance atomically."""
        now = self.get_clock().now().nanoseconds*1e-9
        self.check_clock(now)
        if not estimator_active(self) or getattr(self, 'navigation_frame_fault', ''):
            return
        try:
            position, covariance, ns = measurement_from_observation(message, 'local_ned')
            self.filter.update(position, covariance, ns*1e-9, now)
            self.tick()
        except (ValueError, np.linalg.LinAlgError):
            return

    def tick(self):
        """Project to now without renewing the original source authorization."""
        now = self.get_clock().now().nanoseconds*1e-9
        self.check_clock(now)
        active = estimator_active(self) and not getattr(self, 'navigation_frame_fault', '')
        if not active:
            publish_ack(self, 'spatial', False)
            return
        estimate = self.filter.predict(now)
        result = TargetState()
        result.stamp = Time(nanoseconds=round(now*1e9)).to_msg()
        result.source_stamp = Time(nanoseconds=max(0, round(estimate.source_stamp*1e9))).to_msg()
        result.frame_id = 'local_ned'
        for field, values in ((result.position, estimate.state[:3]),
                              (result.velocity, estimate.state[3:])):
            field.x, field.y, field.z = map(float, values)
        result.covariance = estimate.covariance.reshape(-1).tolist()
        result.valid = estimate.valid
        self.state_pub.publish(result)
        publish_ack(self, 'spatial', True, result.valid, estimate.source_stamp, estimate.reason)


def main(args=None):
    """Run only this main target-state publisher in a P76 launch."""
    rclpy.init(args=args)
    node = SpatialDkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
