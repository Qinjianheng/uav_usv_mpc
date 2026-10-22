"""Offline only: known ray/sphere pixels + plane through actual bounded writer/replay."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'src/uav_control/test'))
from synthetic_sphere_fixture import render
from uav_control.perception.p765_roi_capture import BoundedRoiWriter, replay_roi, runtime_identity
from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd
from uav_control.perception.tof_depth_model import TofDepthModel
from uav_control.perception.yolo_inference import encode_mask

output = Path(__file__).resolve().parent
identity = runtime_identity()
identity['driver_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
identity['independent_ray_sphere_generator_sha256'] = hashlib.sha256(
    (ROOT/'src/uav_control/test/synthetic_sphere_fixture.py').read_bytes()).hexdigest()
writer = BoundedRoiWriter(output/'synthetic_roi', runtime_identity=identity)
contract = dict(horizontal_fov=1.74, minimum_depth=.05, maximum_depth=25., target_radius=.25,
                require_valid_sphere=True, sphere_fit_mode='ideal', sphere_noise_std=.01,
                sphere_range_noise_scale=.001)
results = []
for index, (center, mode) in enumerate([((3., .4, -.1), 'ideal'),
                                       ((8., -1.2, .3), 'tof'),
                                       ((5., 0., 0.), 'plane')]):
    mask, raw, _ = render(center)
    depth = raw.astype(np.float32)
    contract = dict(contract, sphere_fit_mode='tof' if mode == 'tof' else 'ideal')
    if mode == 'tof':
        depth = TofDepthModel(seed=4).measure(depth).astype(np.float32)
    if mode == 'plane':
        depth[mask] = 4.75
    detail = {}
    geometry = target_geometry_from_rgbd(mask, depth, diagnostics=detail, **contract)
    stamp = dict(sec=100+index, nanosec=7)
    metadata = dict(
        synthetic_only=True, known_center_flu=center,
        rgb_raw_stamp=stamp, depth_raw_stamp=stamp, detection_raw_stamp=stamp,
        rgb_receipt_ns=(200+index)*10**9, depth_receipt_ns=(200+index)*10**9+5,
        diagnostic_receipt_ns=(200+index)*10**9+10,
        localization_reason='' if geometry is not None else 'TARGET_VECTOR_INVALID',
        geometry=detail, geometry_contract=contract,
        detection=dict(raw_stamp=stamp, image_width=640, image_height=480,
                       mask_counts=list(encode_mask(mask)), model_sha256='0'*64),
        raw_key_ns=(100+index)*10**9+7,
    )
    result = writer.capture(rgb=np.zeros((480, 640, 3), np.uint8), depth=depth,
                            mask=mask, metadata=metadata)
    assert result['saved'], result
    results.append(replay_roi(writer.output/result['metadata_file']))
assert results[0]['valid_sphere'] and results[1]['valid_sphere']
assert not results[2]['valid_sphere']
assert results[2]['diagnostics']['explicit_fit_rejection_reason'] == 'FIT_RANK_DEFICIENT'
writer.close()
(output/'synthetic_capture_replay.json').write_text(json.dumps(
    dict(samples=results, runtime_identity=identity,
         qualification='SYNTHETIC_OFFLINE_ONLY_NO_NEW_REAL_OBSERVATIONS'),
    allow_nan=False, indent=2, sort_keys=True))
print(json.dumps(dict(samples=len(results), sphere_successes=2, plane_rejected=True)))
