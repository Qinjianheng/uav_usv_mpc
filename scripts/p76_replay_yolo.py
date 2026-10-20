#!/usr/bin/env python3
"""Evaluate the frozen ONNX detector on held-out real Gazebo images; no flight qualification."""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.perception.yolo_inference import YoloInference, decode_mask  # noqa: E402


def replay(session, manifest_path, output):
    """Measure actual inference, masks, false positives and perspective-center error separately."""
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads(manifest_path.read_text())
    engine = YoloInference(manifest['model_file'], manifest['model_sha256'],
                           size=manifest['input_size'])
    metadata = {r['name']: r for r in map(
        json.loads, (session/'frames.jsonl').read_text().splitlines())}
    for _ in range(3):
        engine.infer(np.zeros((480, 640, 3), np.uint8))
    rows = []
    for image in sorted((session/'images').glob('*.png')):
        truth_positive = bool((session/'labels'/(image.stem+'.txt')).read_text().strip())
        rgb = cv2.cvtColor(cv2.imread(str(image)), cv2.COLOR_BGR2RGB)
        began = time.perf_counter()
        result = engine.infer(rgb)
        row = dict(file=image.name, positive=truth_positive, detected=result.valid,
                   reason=result.reason, confidence=result.confidence,
                   seconds=time.perf_counter()-began, inference_seconds=result.inference_seconds)
        if result.valid and truth_positive:
            mask = decode_mask(result.mask_counts, 640, 480)
            expected = np.load(session/'masks'/(image.stem+'.npz'))['mask']
            row['visible_mask_iou'] = float(np.count_nonzero(mask & expected)
                                            / max(1, np.count_nonzero(mask | expected)))
            frame = metadata[image.stem]
            projected = frame['projected_sphere_center_uv']
            row['sphere_center_error_pixels'] = float(np.linalg.norm(
                np.asarray(result.center_uv)-projected))
            row['camera_range'] = float(np.linalg.norm(frame['camera_relative_center']))
        rows.append(row)
    positive = sum(r['positive'] for r in rows)
    negatives = len(rows)-positive
    true_positive = sum(r['positive'] and r['detected'] for r in rows)
    false_positive = sum(not r['positive'] and r['detected'] for r in rows)
    times = [r['seconds'] for r in rows]
    errors = [r['sphere_center_error_pixels'] for r in rows
              if 'sphere_center_error_pixels' in r]
    ious = [r['visible_mask_iou'] for r in rows if 'visible_mask_iou' in r]
    summary = dict(scope='Held-out static-pose Gazebo images; no ROS end-to-end age or flight.',
                   images=len(rows), positive=positive, negative=negatives,
                   recall=true_positive/max(1, positive), false_positives=false_positive,
                   seconds_median=float(np.median(times)),
                   seconds_p95=float(np.percentile(times, 95)),
                   seconds_max=max(times), visible_mask_iou_median=float(np.median(ious)),
                   projected_sphere_center_error_median=float(np.median(errors)),
                   projected_sphere_center_error_p95=float(np.percentile(errors, 95)),
                   model_sha256=manifest['model_sha256'], qualified=False,
                   reason='ROS_TIMING_NEAR_CENTER_GEOMETRY_AND_FLIGHT_QUALIFICATION_REQUIRED')
    (output/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    replay(args.session.resolve(), args.manifest.resolve(), args.output.resolve())
