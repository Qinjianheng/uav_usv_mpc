#!/usr/bin/env python3
"""Prepare one bounded unarmed P765 capture; execution requires separate human authorization."""
import argparse
import json
from pathlib import Path

from p4_sitl_session import WORKSPACE, preflight
from p764_nonflight_session import run
from uav_control.common.p76_readiness import source_fingerprint, sha256
from uav_control.common.p76_contract import load_contract

SCOPE = WORKSPACE/'data/experiments/p765_offline_validation'
APPROVAL = 'ONE_UNARMED_P765'


def prepare():
    """Read-only exclusion checks; this neither starts processes nor reserves a flight slot."""
    contract = load_contract(WORKSPACE/'src/uav_usv_bringup/config/p76_contract.yaml')
    detector = contract['detector']
    model_hash = sha256(detector['model_file'])
    if model_hash != detector['model_sha256'] or not Path(detector['runtime_python']).is_file():
        raise ValueError('P765_DETECTOR_RUNTIME_PREFLIGHT_FAILED')
    return dict(schema_version=1, kind='P765_UNARMED_CAPTURE_PLAN',
                detector_model_sha256_verified=model_hash,
                detector_python=detector['runtime_python'],
                preflight=preflight(), authorization_received=False, actual_sessions=0,
                sources=source_fingerprint(WORKSPACE), contract_sha256=sha256(
                    WORKSPACE/'src/uav_usv_bringup/config/p76_contract.yaml'),
                duration_seconds=30, watchdog_seconds=150,
                scene='Same P764 stationary target 8 m, ground, camera32deg, single YOLO',
                phases='FOLLOW/Y_HANDOVER/PAPER shadow labels only; no mission or flight',
                roi_limits=dict(max_samples=60, max_per_stratum=10, max_bytes=67108864,
                                retained_frames=24, lossless_depth='float32 npz'),
                command_publishers_allowed=[], arm_allowed=False,
                stop=['unexpected armed status', 'any PX4 command publisher',
                      'recorder overflow/failure', 'source drift', '30 s or150 s watchdog'],
                caveat='Extra ROI recording changes load. Compare component timing and '
                       'declare this difference; cannot isolate EOF fix or worker effect alone.',
                unique_next_experiment=True, a1_authorized=False)


def validate_capture_artifacts(root):
    """A recorder file's existence is insufficient: require completed, paired lossless data."""
    import numpy as np
    from uav_control.perception.p765_geometry import stamp_ns
    try:
        manifest = json.loads((root/'manifest.json').read_text())
        files = manifest['files']
        data_files = [name for name in files if name.endswith('.npz')]
        if not files or not data_files or len(data_files) != manifest['saved_samples']:
            raise ValueError('P765_ROI_NO_PAIRED_SAMPLES')
        for name, expected in files.items():
            if Path(name).name != name or sha256(root/name) != expected:
                raise ValueError('P765_ROI_HASH_OR_PATH_INVALID')
        for name in data_files:
            if str(Path(name).with_suffix('.json')) not in files:
                raise ValueError('P765_ROI_METADATA_HASH_REQUIRED')
            metadata = json.loads((root/Path(name).with_suffix('.json')).read_text())
            keys = [stamp_ns(metadata[k]) for k in (
                'rgb_raw_stamp', 'depth_raw_stamp', 'detection_raw_stamp')]
            if len(set(keys)) != 1 or keys[0] <= 0:
                raise ValueError('P765_ROI_RAW_IDENTITY_INVALID')
            with np.load(root/name, allow_pickle=False) as data:
                depth, rgb, mask = (data[k] for k in ('depth', 'rgb', 'mask'))
                if (depth.dtype.kind != 'f' or depth.dtype.itemsize != 4
                        or depth.ndim != 2 or mask.shape != depth.shape or mask.dtype != bool
                        or rgb.shape != (*depth.shape, 3) or rgb.dtype != np.uint8):
                    raise ValueError('P765_ROI_LOSSLESS_ARRAY_INVALID')
        events = [json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
        terminal = [r for r in events if r.get('reason') == 'SESSION_END']
        if (len(terminal) != 1 or events[-1] != terminal[0]
                or terminal[0]['metadata'].get('failure')
                or not 29. <= terminal[0]['metadata']['duration_s'] <= 40.):
            raise ValueError('P765_ROI_SESSION_NOT_COMPLETED')
        return dict(completed=True, paired_samples=len(data_files),
                    manifest_sha256=sha256(root/'manifest.json'), flight_qualified=False)
    except (OSError, KeyError, TypeError, IndexError, ValueError) as error:
        raise ValueError('P765_ROI_POSTFLIGHT_REJECTED:'+str(error)) from error


def execute(output, authorization):
    """Require an explicit grant and permanently claim the single attempt before launch."""
    if authorization != APPROVAL:
        raise ValueError('P765_SEPARATE_HUMAN_AUTHORIZATION_REQUIRED')
    output = output.resolve()
    if not output.is_relative_to(SCOPE) or output == SCOPE:
        raise ValueError('P765_NONFLIGHT_OUTPUT_SCOPE')
    preflight()
    SCOPE.mkdir(parents=True, exist_ok=True)
    with (SCOPE/'one_unarmed_attempt_claim.json').open('x') as stream:
        json.dump(dict(authorization=authorization, output=str(output)), stream)
    result = run(output, launch_name='p765_perception_probe.launch.py',
                 output_scope='p765_offline_validation', snapshot=True,
                 completion_files=('roi/manifest.json',),
                 authorization='Separately approved ONE_UNARMED_P765; no flight or A1-D')
    try:
        result['raw_capture'] = validate_capture_artifacts(output/'roi')
    except ValueError as error:
        result.update(success=False, raw_capture=dict(completed=False, reason=str(error)))
    (output/'p765_postflight.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute-once', action='store_true')
    parser.add_argument('--authorization', default='')
    args = parser.parse_args()
    if args.execute_once:
        result = execute(args.output, args.authorization)
    else:
        result = prepare()
        args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 2 if args.execute_once and not result.get('success') else 0


if __name__ == '__main__':
    raise SystemExit(main())
