"""Opt-in shared-detection adapters; legacy color processing is never a P76 fallback."""
from collections import OrderedDict
import math

import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from uav_usv_interfaces.msg import TargetBearing, TargetImageDetection

from .yolo_detection_contract import DetectionCache, detection_mask, raw_key, stamp_seconds


def _parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def detection_source(node):
    """Read the explicit mode without allowing a missing P76 model to select legacy detection."""
    source = str(_parameter(node, 'detection_source', 'color'))
    if source not in ('color', 'yolo'):
        raise ValueError('DETECTION_SOURCE_REQUIRED')
    return source


def install_localizer_detection(node):
    """Add a bounded packet cache while leaving the native pose relay alive during Y."""
    node.detection_source = detection_source(node)
    if node.detection_source != 'yolo':
        return
    sha = str(_parameter(node, 'detection_model_sha256', ''))
    node.yolo_cache = DetectionCache(sha)
    node.p76_phase = 'FOLLOW_ACTIVE'

    def receive(message):
        node.yolo_cache.add(message)
        node._request_localization()

    def mode(message):
        import json
        try:
            node.p76_phase = json.loads(message.data)['phase']
        except (ValueError, KeyError, TypeError):
            node.p76_phase = 'SAFE_RECOVERY'
    node.yolo_subscription = node.create_subscription(
        TargetImageDetection, str(_parameter(node, 'detection_topic',
                                  '/perception/target_detection')),
        receive, 10)
    node.p76_mode_subscription = node.create_subscription(
        String, str(_parameter(node, 'p76_mode_topic', '/control/p76_mode')), mode,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))


class YoloBearingConsumer:
    """Only shared detection feeds bearing; no RGB subscription or color extraction exists."""

    def __init__(self, node):
        self.node, self.latest = node, None
        self.sha = str(_parameter(node, 'detection_model_sha256', ''))
        self.fov = float(_parameter(node, 'horizontal_fov', 1.74))
        self.pitch = float(_parameter(node, 'camera_pitch_down', 0.))
        self.publisher = node.create_publisher(TargetBearing, str(_parameter(
            node, 'bearing_topic', '/perception/front/target_bearing')), 10)
        self.subscription = node.create_subscription(TargetImageDetection, str(_parameter(
            node, 'detection_topic', '/perception/target_detection')), self.receive, 10)
        self.timer = node.create_timer(.02, self.publish)

    def receive(self, message):
        """Cache explicit valid or invalid packets, preserving their original acquisition."""
        self.latest = message
        self.publish()

    def publish(self):
        """Retain source timestamps even when staleness invalidates the bearing."""
        if self.latest is None:
            return
        packet = self.latest
        result = TargetBearing()
        result.stamp, result.raw_stamp = packet.stamp, packet.raw_stamp
        now = self.node.get_clock().now().nanoseconds*1e-9
        try:
            detection_mask(packet, now, self.sha)
            focal = packet.image_width/(2*math.tan(self.fov/2))
            right = (packet.center_uv[0]-packet.image_width/2)/focal
            down = (packet.center_uv[1]-packet.image_height/2)/focal
            forward = math.cos(self.pitch)-math.sin(self.pitch)*down
            result.bearing = math.atan(right)
            result.body_bearing = math.atan2(right, forward)
            result.valid, result.confidence = True, packet.confidence
        except ValueError:
            result.valid, result.confidence = False, 0.
        self.publisher.publish(result)


class YoloDepthMonitor:
    """Observe the exact same acquisition mask and aligned depth, without truth inputs."""

    def __init__(self, node):
        self.node, self.latest = node, None
        self.sha = str(_parameter(node, 'detection_model_sha256', ''))
        self.depths = OrderedDict()
        prefix = str(_parameter(node, 'diagnostic_prefix', '/perception/front'))
        self.minimum = float(_parameter(node, 'minimum_depth', .05))
        self.maximum = float(_parameter(node, 'maximum_depth', 25.))
        self.minimum_ratio = float(_parameter(node, 'minimum_target_depth_ratio', .5))
        self.visible_pub = node.create_publisher(Bool, prefix+'/usv_visible', 10)
        self.valid_pub = node.create_publisher(Bool, prefix+'/usv_tof_valid', 10)
        self.ratio_pub = node.create_publisher(Float32, prefix+'/usv_depth_valid_ratio', 10)
        self.range_pub = node.create_publisher(Float32, prefix+'/usv_range', 10)
        self.status_pub = node.create_publisher(String, prefix+'/yolo_depth_status', 10)
        self.subscription = node.create_subscription(TargetImageDetection, str(_parameter(
            node, 'detection_topic', '/perception/target_detection')), self.receive, 10)
        from .rgbd_target_localizer import aligned_camera_qos
        topic = str(_parameter(node, 'depth_ros_topic', '/camera/front/depth/image_raw'))
        self.depth_sub = node.create_subscription(Image, topic, self.depth, aligned_camera_qos())
        self.timer = node.create_timer(.05, self.publish)

    def receive(self, message):
        """Store only the shared detector's current packet."""
        self.latest = message

    def depth(self, message):
        """Keep bounded raw acquisition identities; never pair merely by receipt order."""
        self.depths[raw_key(message.header.stamp)] = message
        while len(self.depths) > 12:
            self.depths.popitem(last=False)

    def publish(self):
        """Publish separate RGB-valid, depth-valid and exact-pair rejection categories."""
        if self.latest is None:
            return
        from .front_tof_monitor import decode_float32_depth
        import json
        now = self.node.get_clock().now().nanoseconds*1e-9
        rgb, depth_valid, ratio, distance, reason = False, False, 0., math.nan, 'NO_DEPTH_PAIR'
        try:
            mask = detection_mask(self.latest, now, self.sha)
            rgb = True
            message = self.depths.get(raw_key(self.latest.raw_stamp))
            if message is not None and message.encoding.lower() in ('32fc1', '32fc'):
                depth = decode_float32_depth(
                    message.data, message.width, message.height, message.step)
                valid = mask & np.isfinite(depth) & (
                    depth >= self.minimum) & (depth <= self.maximum)
                count = int(np.count_nonzero(valid))
                ratio = count/max(1, int(np.count_nonzero(mask)))
                depth_valid = count >= 12 and ratio >= self.minimum_ratio
                distance = float(np.median(depth[valid])) if count else math.nan
                reason = 'VALID' if depth_valid else 'DEPTH_UNUSABLE'
        except ValueError as exception:
            reason = str(exception)
        self.visible_pub.publish(Bool(data=rgb))
        self.valid_pub.publish(Bool(data=depth_valid))
        self.ratio_pub.publish(Float32(data=float(ratio)))
        self.range_pub.publish(Float32(data=distance))
        self.status_pub.publish(String(data=json.dumps(dict(
            source_stamp=stamp_seconds(self.latest.stamp), valid_rgb=rgb, valid_depth=depth_valid,
            depth_ratio=ratio, reason=reason))))
