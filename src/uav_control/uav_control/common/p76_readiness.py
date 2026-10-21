"""Bind physical qualification to exact current source, model and acquisition contracts."""
import hashlib
import json
from pathlib import Path

from .p76_model import verify_model


def sha256(path):
    """Hash saved bytes; missing inputs are explicit failures."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_fingerprint(workspace):
    """Include all online Python and launch sources, excluding reports and generated artifacts."""
    root = Path(workspace)
    paths = list((root/'src/uav_control/uav_control').rglob('*.py'))
    paths += list((root/'src/uav_usv_bringup/launch').glob('*.py'))
    paths += list((root/'src/uav_usv_interfaces/msg').glob('*.msg'))
    paths += [root/'scripts'/name for name in (
        'start_px4_ros2.sh', 'p76_config_audit.py', 'prepare_gz_camera_model.py')]
    return {str(p.relative_to(root)): sha256(p) for p in sorted(paths)}


def validate_calibration(contract, model_root, workspace):
    """Require measured force/rate/braking evidence, never promote model constants to flight."""
    path = contract['candidate']['bodyrate_calibration_manifest']
    if not path or not Path(path).is_file():
        raise ValueError('P76_ACTUAL_BODYRATE_CALIBRATION_MISSING')
    data = json.loads(Path(path).read_text())
    verify_model(model_root, contract)
    manifest = json.loads((Path(model_root)/'p76_model_manifest.json').read_text())
    if (data.get('schema_version') != 1 or data.get('qualified') is not True
            or data.get('kind') != 'PX4_GAZEBO_BODYRATE_CALIBRATION'
            or data.get('sources') != source_fingerprint(workspace)
            or data.get('model_files') != manifest['files']
            or data.get('contract_sha256') != sha256(contract['_path'])
            or data.get('detector_sha256') != contract['detector']['model_sha256']):
        raise ValueError('P76_CALIBRATION_IDENTITY_MISMATCH')
    logs = data.get('raw_evidence', {})
    if not logs or any(not Path(p).is_file() or sha256(p) != h for p, h in logs.items()):
        raise ValueError('P76_CALIBRATION_RAW_EVIDENCE_MISSING')
    required = ('hover', 'thrust_curve', 'rates_response', 'acceleration_5mps2',
                'descent_2mps_braking', 'exclusive_px4_bodyrate', 'fault_recovery',
                'navigation_reference_and_sea_datum')
    if any(data.get('checks', {}).get(key) is not True for key in required):
        raise ValueError('P76_BODYRATE_QUALIFICATION_FAILED')
    if (data.get('response_seconds', 0) <= 0 or data.get('braking_acceleration', 0) <= 0
            or data.get('mass') != manifest['total_mass']
            or data.get('collision_envelope') != manifest['collision_envelope']):
        raise ValueError('P76_CALIBRATED_DYNAMICS_REQUIRED')
    return data


def validate_follow_qualification(contract, workspace):
    """C cannot inherit P75 or another model's nominal planner feasibility."""
    path = contract['candidate'].get('follow_qualification_manifest', '')
    if not path or not Path(path).is_file():
        raise ValueError('P76_CURRENT_25S_FOLLOW_QUALIFICATION_MISSING')
    data = json.loads(Path(path).read_text())
    if (data.get('qualified') is not True or data.get('valid_follow_seconds', 0) < 25
            or data.get('sources') != source_fingerprint(workspace)
            or data.get('contract_sha256') != sha256(contract['_path'])
            or not data.get('actual_minco_ownership_verified')):
        raise ValueError('P76_CURRENT_FOLLOW_QUALIFICATION_INVALID')
    return data
