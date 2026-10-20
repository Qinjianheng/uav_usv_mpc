"""Validate one shared detection and keep acquisition identity independent of receipt time."""
from collections import OrderedDict
import math

import numpy as np

from .yolo_inference import decode_mask


def stamp_seconds(stamp):
    """Read a ROS time without substituting a receipt timestamp."""
    return int(stamp.sec)+int(stamp.nanosec)*1e-9


def raw_key(stamp):
    """Use exact original acquisition nanoseconds as the RGB/depth/mask identity."""
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def detection_mask(message, now, expected_sha):
    """Reject model, class, source-age or pixel extent drift before consuming any geometry."""
    if not message.valid:
        raise ValueError('YOLO_'+message.rejection_reason)
    source = stamp_seconds(message.stamp)
    completed = stamp_seconds(message.detection_stamp)
    if not (0 < source <= completed <= now+1e-6 and 0 <= now-source <= .125+1e-9):
        raise ValueError('YOLO_SOURCE_STALE_OR_INVALID')
    if (len(expected_sha) != 64 or message.model_sha256 != expected_sha
            or message.class_name != 'red_ball' or message.frame_sequence <= 0):
        raise ValueError('YOLO_MODEL_IDENTITY')
    center = tuple(message.center_uv)
    if (len(center) != 2 or not all(math.isfinite(v) for v in center)
            or message.image_width != 640 or message.image_height != 480
            or not 0 <= message.confidence <= 1):
        raise ValueError('YOLO_GEOMETRY')
    mask = decode_mask(message.mask_counts, message.image_width, message.image_height)
    if np.count_nonzero(mask) < 3:
        raise ValueError('YOLO_MASK_EMPTY')
    return mask


class DetectionCache:
    """Bounded acquisition-keyed packets; never fall back to a different image's mask."""

    def __init__(self, expected_sha, capacity=12):
        self.expected_sha = expected_sha
        self.capacity = capacity
        self.packets = OrderedDict()

    def add(self, message):
        """Keep one authoritative packet per raw acquisition, including explicit rejection."""
        key = raw_key(message.raw_stamp)
        self.packets[key] = message
        while len(self.packets) > self.capacity:
            self.packets.popitem(last=False)

    def get(self, raw_stamp):
        """Lookup exact acquisition nanoseconds supplied by the original RGB header."""
        if not math.isfinite(raw_stamp) or raw_stamp <= 0:
            return None
        key = round(float(raw_stamp)*1e9)
        return self.packets.get(key)
