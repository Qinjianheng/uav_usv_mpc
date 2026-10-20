#!/usr/bin/env python3
"""Train actual single-class YOLO11n-seg using disjoint, hashed Gazebo sessions."""
import argparse
import hashlib
import json
import os
from pathlib import Path

import yaml


def session_dataset(folder, output):
    """Keep whole sessions in exactly one split and reject image-hash leakage."""
    groups = dict(train=[], val=[], test=[])
    seen = {}
    sessions = []
    for path in sorted(folder.glob('*/manifest.json')):
        manifest = json.loads(path.read_text())
        if not manifest['completed']:
            continue
        split = manifest['split']
        if split not in groups:
            raise ValueError('DATASET_SESSION_SPLIT')
        groups[split].append(str(path.parent/'images'))
        for name, digest in manifest['images'].items():
            if digest in seen and seen[digest] != split:
                raise ValueError('DATASET_IMAGE_LEAKAGE')
            seen[digest] = split
            image = path.parent/'images'/name
            if hashlib.sha256(image.read_bytes()).hexdigest() != digest:
                raise ValueError('DATASET_IMAGE_HASH')
        sessions.append(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                             split=split))
    if not all(groups.values()):
        raise ValueError('DATASET_REQUIRES_THREE_DISJOINT_SESSION_GROUPS')
    data = dict(path=str(folder), **groups, names={0: 'red_ball'})
    path = output/'dataset.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    (output/'dataset_sources.json').write_text(json.dumps(sessions, indent=2)+'\n')
    return path


def train(dataset, output, pretrained, epochs, size, threads):
    """Train only from actual labeled images; no detector substitute is generated."""
    output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('YOLO_CONFIG_DIR', str(output/'settings'))
    os.environ['OMP_NUM_THREADS'] = str(threads)
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(threads)
    data = session_dataset(dataset, output)
    model = YOLO(str(pretrained))
    model.train(data=str(data), epochs=epochs, imgsz=size, batch=8, device='cpu',
                workers=0, project=str(output), name='training', exist_ok=False,
                seed=76, deterministic=True, freeze=10, patience=7, plots=False,
                cache=False, amp=False, close_mosaic=5)
    best = output/'training/weights/best.pt'
    if not best.is_file():
        raise RuntimeError('YOLO_TRAINING_NO_WEIGHTS')
    manifest = dict(architecture='yolo11n-seg', classes=['red_ball'], input_size=size,
                    pretrained_sha256=hashlib.sha256(pretrained.read_bytes()).hexdigest(),
                    model_file=str(best), sha256=hashlib.sha256(best.read_bytes()).hexdigest(),
                    dataset_sha256=hashlib.sha256(data.read_bytes()).hexdigest(),
                    model_qualified=False, reason='HELD_OUT_REPLAY_AND_TIMING_REQUIRED')
    (output/'training_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--pretrained', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--size', type=int, choices=(416, 512), default=416)
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args()
    train(args.dataset.resolve(), args.output.resolve(), args.pretrained.resolve(),
          args.epochs, args.size, args.threads)
