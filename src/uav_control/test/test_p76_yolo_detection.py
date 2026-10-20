"""YOLO mask representation and offline annotation preserve visible holes/occlusion."""
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

from uav_control.perception.yolo_inference import (
    encode_mask, decode_mask, letterbox_rgb, decode_segmentation,
)

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from p76_collect_yolo_dataset import polygon_label  # noqa: E402


def test_offline_polygon_does_not_fill_an_airframe_occlusion_with_target_pixels():
    mask = np.zeros((100, 120), dtype=bool)
    mask[10:90, 10:110] = True
    mask[25:75, 35:85] = False
    text = polygon_label(mask)
    values = np.asarray([float(v) for v in text.split()[1:]]).reshape(-1, 2)
    points = np.rint(values*np.array((120, 100))).astype(np.int32)
    restored = np.zeros(mask.shape, np.uint8)
    cv2.fillPoly(restored, [points], 1)
    intersection = np.count_nonzero(mask & restored.astype(bool))
    union = np.count_nonzero(mask | restored.astype(bool))
    assert intersection/union > .97


def test_shared_rle_mask_roundtrips_and_rejects_inconsistent_extent():
    mask = np.zeros((6, 8), bool)
    mask[1:4, 2:6] = True
    assert np.array_equal(mask, decode_mask(encode_mask(mask), 8, 6))
    with pytest.raises(ValueError, match='MASK_EXTENT'):
        decode_mask((0, 49), 8, 6)


def test_letterbox_uses_original640x480_geometry_and_exact_padding():
    tensor, geometry = letterbox_rgb(np.zeros((480, 640, 3), np.uint8), 416)
    assert tensor.shape == (1, 3, 416, 416)
    assert geometry == pytest.approx((.65, 0., 52., 640., 480.))


def test_real_yolo_output_geometry_decodes_original_pixels_without_color_input():
    predictions = np.zeros((1, 37, 1), np.float32)
    predictions[0, :5, 0] = (208., 208., 104., 104., .9)
    predictions[0, 5, 0] = 10.
    prototypes = np.ones((1, 32, 104, 104), np.float32)
    result = decode_segmentation(predictions, prototypes, (.65, 0., 52., 640., 480.))
    assert result.valid and result.confidence == pytest.approx(.9)
    assert result.center_uv == pytest.approx((320., 240.))
    assert result.bounding_box == pytest.approx((240., 160., 400., 320.))
    mask = decode_mask(result.mask_counts, 640, 480)
    assert mask[240, 320] and not mask[10, 10]
    predictions[0, 4, 0] = .1
    assert not decode_segmentation(predictions, prototypes,
                                   (.65, 0., 52., 640., 480.)).valid


def test_shared_detection_keeps_exact_acquisition_and_rejects_stale_or_wrong_model():
    from builtin_interfaces.msg import Time
    from uav_usv_interfaces.msg import TargetImageDetection
    from uav_control.perception.yolo_detection_contract import DetectionCache, detection_mask
    packet = TargetImageDetection()
    packet.stamp, packet.raw_stamp = Time(sec=10), Time(sec=2)
    packet.detection_stamp = Time(sec=10, nanosec=20_000_000)
    packet.image_width, packet.image_height = 640, 480
    packet.model_sha256, packet.class_name = 'a'*64, 'red_ball'
    packet.valid, packet.frame_sequence, packet.confidence = True, 1, .9
    packet.center_uv = [320., 240.]
    mask = np.zeros((480, 640), bool)
    mask[235:245, 315:325] = True
    packet.mask_counts = list(encode_mask(mask))
    cache = DetectionCache('a'*64)
    cache.add(packet)
    assert cache.get(2.) is packet and cache.get(2.05) is None
    assert np.array_equal(detection_mask(packet, 10.1, 'a'*64), mask)
    with pytest.raises(ValueError, match='SOURCE_STALE'):
        detection_mask(packet, 10.126, 'a'*64)
    with pytest.raises(ValueError, match='MODEL_IDENTITY'):
        detection_mask(packet, 10.1, 'b'*64)
