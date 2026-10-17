"""Publish a functional ToF depth stream with original image acquisition stamps."""

from array import array

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .rgbd_target_localizer import aligned_camera_qos
from .tof_depth_model import TofDepthModel, decode_depth_image


class TofDepthNode(Node):
    """Apply range-domain errors without consuming pose, target state or truth."""

    def __init__(self):
        super().__init__('front_tof_depth_model')
        defaults = {
            'input_topic': '/camera/front/depth/ideal',
            'output_topic': '/camera/front/depth/image_raw',
            'horizontal_fov': 1.74, 'minimum_range': .25,
            'maximum_range': 25., 'noise_std': .01,
            'range_noise_scale': .001, 'dropout_probability': .01,
            'quantization': .001, 'seed': 0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        values = {key: self.get_parameter(key).value for key in defaults}
        input_topic, output_topic = values.pop('input_topic'), values.pop('output_topic')
        if input_topic == output_topic:
            raise ValueError('ToF input/output topics must be different')
        self.model = TofDepthModel(**values)
        qos = aligned_camera_qos()
        self.depth_pub = self.create_publisher(Image, output_topic, qos)
        self.depth_sub = self.create_subscription(
            Image, input_topic, self.on_depth, qos,
        )
        self.get_logger().info(
            'FUNCTIONAL TOF SIMULATION | uncalibrated test profile | '
            f'radial range {values["minimum_range"]}-{values["maximum_range"]} m | '
            'RGB/depth alignment retained; no optical/water physics model',
        )

    def on_depth(self, message):
        """Publish every validly encoded frame, including all-NaN missing returns."""
        try:
            depth = self.model.measure(decode_depth_image(message))
        except ValueError as error:
            self.get_logger().warning(f'ToF depth frame rejected: {error}',
                                      throttle_duration_sec=5.)
            return
        output = Image()
        output.header = message.header
        output.height, output.width = message.height, message.width
        output.encoding, output.is_bigendian = '32FC1', 0
        output.step = output.width * 4
        # rclpy's bytes setter validates every element in Python (~0.1 s for
        # 640x480 float depth). A typed byte array avoids that frame backlog.
        output.data = array('B', depth.astype('<f4', copy=False).tobytes())
        self.depth_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TofDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
