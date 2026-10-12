import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition


class PositionListener(Node):
    def __init__(self):
        super().__init__('position_listener')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos_profile
        )

    def position_callback(self, msg):

        x = msg.x
        y = msg.y
        z = msg.z

        self.get_logger().info(
            f"Position -> x:{x:.2f}, y:{y:.2f}, z:{z:.2f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = PositionListener()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
