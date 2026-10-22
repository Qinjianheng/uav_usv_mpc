"""Strict single-class YOLO11 segmentation via ONNX Runtime; no color-detector fallback."""
from dataclasses import dataclass
import ast
import hashlib
import math
from pathlib import Path
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    """Original-image output shared by every P76 perception consumer."""

    valid: bool = False
    reason: str = 'NO_DETECTION'
    center_uv: tuple = (0., 0.)
    bounding_box: tuple = (0., 0., 0., 0.)
    mask_counts: tuple = ()
    confidence: float = 0.
    inference_seconds: float = 0.


def encode_mask(mask):
    """Encode a row-major boolean mask with an initial zero run, including an empty one."""
    flat = np.asarray(mask, dtype=bool).ravel()
    if not len(flat):
        return ()
    changes = np.flatnonzero(flat[1:] != flat[:-1])+1
    runs = np.diff(np.r_[0, changes, len(flat)])
    return tuple(int(v) for v in np.r_[0, runs]) if flat[0] else tuple(int(v) for v in runs)


def decode_mask(counts, width, height):
    """Reject malformed or oversized shared masks before allocating their pixel extent."""
    if not (0 < width <= 4096 and 0 < height <= 4096 and width*height <= 4_000_000):
        raise ValueError('MASK_EXTENT')
    values = np.asarray(counts)
    if (values.ndim != 1 or len(values) > 2*width*height+1 or not len(values)
            or not np.issubdtype(values.dtype, np.integer) or np.any(values < 0)
            or int(values.sum()) != width*height):
        raise ValueError('MASK_EXTENT')
    bits = np.repeat(np.arange(len(values)) % 2, values.astype(np.int64))
    return bits.astype(bool).reshape(height, width)


def letterbox_rgb(image, size):
    """Use centered YOLO letterboxing while retaining exact original-image scale/padding."""
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8 or size not in (416, 512):
        raise ValueError('YOLO_INPUT')
    height, width = image.shape[:2]
    scale = min(size/width, size/height)
    resized = cv2.resize(image, (round(width*scale), round(height*scale)),
                         interpolation=cv2.INTER_LINEAR)
    left = round((size-resized.shape[1])/2-.1)
    top = round((size-resized.shape[0])/2-.1)
    canvas = np.full((size, size, 3), 114, np.uint8)
    canvas[top:top+resized.shape[0], left:left+resized.shape[1]] = resized
    tensor = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None], dtype=np.float32)/255.
    return tensor, (scale, float(left), float(top), float(width), float(height))


def decode_segmentation(predictions, prototypes, geometry, confidence=.5, iou=.45):
    """Decode a single-class standard YOLO11-seg export into original-coordinate masks."""
    pred, proto = np.asarray(predictions), np.asarray(prototypes)
    if pred.ndim != 3 or proto.ndim != 4 or pred.shape[0] != 1 or proto.shape[0] != 1:
        return Detection(reason='YOLO_OUTPUT_SHAPE')
    if pred.shape[1] != 5+proto.shape[1] or not np.isfinite(pred).all():
        return Detection(reason='YOLO_SINGLE_CLASS_REQUIRED')
    candidates = pred[0].T
    candidates = candidates[candidates[:, 4] >= confidence]
    if not len(candidates):
        return Detection()
    boxes = candidates[:, :4].copy()
    boxes[:, :2] -= boxes[:, 2:]/2.
    indices = cv2.dnn.NMSBoxes(boxes.tolist(), candidates[:, 4].tolist(), confidence, iou)
    if not len(indices):
        return Detection()
    chosen = candidates[int(np.asarray(indices).ravel()[0])]
    scale, left, top, width, height = geometry
    width, height = int(width), int(height)
    xyxy = np.r_[chosen[:2]-chosen[2:4]/2., chosen[:2]+chosen[2:4]/2.]
    box = (xyxy-np.array((left, top, left, top)))/scale
    box[[0, 2]] = np.clip(box[[0, 2]], 0, width)
    box[[1, 3]] = np.clip(box[[1, 3]], 0, height)
    size = round(width*scale+2*left)
    if size <= 0 or proto.shape[2] != proto.shape[3]:
        return Detection(reason='YOLO_MASK_SHAPE')
    logits = chosen[5:]@proto[0].reshape(proto.shape[1], -1)
    mask = (1./(1.+np.exp(-np.clip(logits, -80, 80)))).reshape(proto.shape[2:])
    yy, xx = np.indices(mask.shape)
    crop = ((xx >= xyxy[0]*mask.shape[1]/size) & (xx < xyxy[2]*mask.shape[1]/size)
            & (yy >= xyxy[1]*mask.shape[0]/size) & (yy < xyxy[3]*mask.shape[0]/size))
    mask *= crop
    mask = cv2.resize(mask, (size, size), interpolation=cv2.INTER_LINEAR)
    mask = mask[int(top):int(top)+round(height*scale),
                int(left):int(left)+round(width*scale)]
    mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_LINEAR) > .5
    if np.count_nonzero(mask) < 3:
        return Detection(reason='YOLO_MASK_EMPTY')
    center = ((box[0]+box[2])/2., (box[1]+box[3])/2.)
    return Detection(True, 'VALID', tuple(center), tuple(box), encode_mask(mask), float(chosen[4]))


class YoloInference:
    """Pinned real ONNX model with a single red_ball class and bounded CPU threading."""

    def __init__(self, model_file, expected_sha256, size=416, confidence=.5, iou=.45, threads=1):
        path = Path(model_file)
        if (not path.is_file() or len(expected_sha256) != 64
                or hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256):
            raise ValueError('YOLO_MODEL_SHA256')
        import onnxruntime as ort
        cv2.setNumThreads(max(1, int(threads)))
        options = ort.SessionOptions()
        options.intra_op_num_threads = max(1, int(threads))
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
        names = ast.literal_eval(
            self.session.get_modelmeta().custom_metadata_map.get('names', '{}'))
        if list(names.values()) != ['red_ball']:
            raise ValueError('YOLO_RED_BALL_MODEL_REQUIRED')
        self.size, self.confidence, self.iou = size, confidence, iou
        self.input_name = self.session.get_inputs()[0].name
        shape = self.session.get_inputs()[0].shape
        if shape != [1, 3, size, size] or not math.isfinite(confidence):
            raise ValueError('YOLO_MODEL_RESOLUTION')

    def infer(self, rgb):
        """Execute actual model inference; callers retain source time independently."""
        began = time.perf_counter()
        tensor, geometry = letterbox_rgb(rgb, self.size)
        outputs = self.session.run(None, {self.input_name: tensor})
        result = decode_segmentation(outputs[0], outputs[1], geometry, self.confidence, self.iou)
        return Detection(**{**result.__dict__, 'inference_seconds': time.perf_counter()-began})
