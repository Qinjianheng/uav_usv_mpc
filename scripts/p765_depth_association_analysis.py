#!/usr/bin/env python3
"""Compare original and opt-in depth association using immutable captured ROI bytes."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import tempfile

import numpy as np

from uav_control.perception.p765_geometry import json_finite
from uav_control.perception.p765_roi_capture import replay_roi, runtime_identity
from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd
from uav_control.perception.yolo_inference import decode_mask

ROOT = Path(__file__).resolve().parents[1]


def compare(archive):
    """Hash-check and replay each original contract before testing the explicit candidate."""
    samples, pixel_hashes = [], set()
    with tarfile.open(archive) as stream, tempfile.TemporaryDirectory() as temporary:
        metadata_names = sorted(name for name in stream.getnames()
                                if '/roi/roi_' in name and name.endswith('.json'))
        if not metadata_names:
            raise ValueError('ROI_SAMPLES_MISSING')
        for name in metadata_names:
            metadata_bytes = stream.extractfile(name).read()
            metadata = json.loads(metadata_bytes)
            data_file = metadata['data_file']
            if Path(data_file).name != data_file:
                raise ValueError('ROI_DATA_FILENAME')
            payload = stream.extractfile(str(Path(name).parent/data_file)).read()
            metadata_path = Path(temporary)/Path(name).name
            metadata_path.write_bytes(metadata_bytes)
            (metadata_path.parent/data_file).write_bytes(payload)
            original = replay_roi(metadata_path)  # Includes original mapping and SHA checks.
            detection = metadata['detection']
            mask = decode_mask(detection['mask_counts'], detection['image_width'],
                               detection['image_height'])
            with np.load(metadata_path.parent/data_file, allow_pickle=False) as raw:
                depth = np.full(mask.shape, np.nan, dtype=raw['depth'].dtype)
                left, top, right, bottom = metadata['roi_xyxy_exclusive']
                depth[top:bottom, left:right] = raw['depth']
                pixel_hashes.add(hashlib.sha256(
                    raw['depth'].tobytes()+raw['mask'].tobytes()).hexdigest())
            contract = dict(metadata['geometry_contract'], sphere_depth_component_enabled=True,
                            minimum_depth_ratio=metadata['geometry_contract'].get(
                                'minimum_depth_ratio', .5))
            details = {}
            geometry = target_geometry_from_rgbd(mask, depth, diagnostics=details, **contract)
            original.pop('metadata_file')  # Temporary extraction path is not evidence identity.
            samples.append(dict(
                archive_metadata=name, metadata_sha256=hashlib.sha256(metadata_bytes).hexdigest(),
                rgb_raw_stamp=metadata['rgb_raw_stamp'],
                depth_raw_stamp=metadata['depth_raw_stamp'],
                original_contract=metadata['geometry_contract'], original=original,
                candidate_contract=contract, candidate=dict(
                    accepted=geometry is not None, diagnostics=details,
                    center_camera_flu=geometry.center_camera if geometry else None)))
    return json_finite(dict(
        qualification='OFFLINE_COUNTERFACTUAL_NO_NEW_REAL_OBSERVATIONS',
        archive_sha256=hashlib.sha256(Path(archive).read_bytes()).hexdigest(),
        runtime_identity=runtime_identity(), sample_count=len(samples),
        unique_depth_mask_payloads=len(pixel_hashes),
        original_accepted=sum(sample['original']['valid_sphere'] for sample in samples),
        candidate_accepted=sum(sample['candidate']['accepted'] for sample in samples),
        samples=samples))


def main():
    """Write a new offline report; never change or relabel the frozen experiment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, default=(
        ROOT/'data/baselines/20261022_p765_unarmed_01/raw_session.tar.gz'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.archive)
    with args.output.open('x') as target:
        json.dump(result, target, allow_nan=False, indent=2, sort_keys=True)
    print(json.dumps({key: result[key] for key in (
        'sample_count', 'unique_depth_mask_payloads', 'original_accepted', 'candidate_accepted',
        'qualification')}, indent=2))


if __name__ == '__main__':
    main()
