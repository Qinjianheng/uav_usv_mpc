#!/usr/bin/env python3
"""Export trained red_ball YOLO11n-seg to a fixed CPU ONNX inference interface."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil


def export(weights, output, size):
    """Keep weight/model hashes and class metadata; export alone grants no flight permission."""
    output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('YOLO_CONFIG_DIR', str(output/'settings'))
    from ultralytics import YOLO
    model = YOLO(str(weights))
    if list(model.names.values()) != ['red_ball']:
        raise ValueError('YOLO_RED_BALL_MODEL_REQUIRED')
    path = Path(model.export(format='onnx', imgsz=size, dynamic=False, simplify=False,
                             opset=17, nms=False, device='cpu', batch=1))
    target = output/'red_ball_yolo11n_seg.onnx'
    shutil.copy2(path, target)
    from uav_control.perception.yolo_inference import YoloInference
    sha = hashlib.sha256(target.read_bytes()).hexdigest()
    YoloInference(target, sha, size=size)
    manifest = dict(model_file=str(target), model_sha256=sha,
                    input_size=size, architecture='yolo11n-seg', classes=['red_ball'],
                    source_weight_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
                    qualified=False, qualification_reason='HELD_OUT_REPLAY_TIMING_REQUIRED')
    (output/'model_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('weights', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--size', choices=(416, 512), type=int, default=416)
    args = parser.parse_args()
    export(args.weights.resolve(), args.output.resolve(), args.size)
