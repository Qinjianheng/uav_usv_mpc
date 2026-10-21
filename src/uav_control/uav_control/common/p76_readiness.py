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
        'start_px4_ros2.sh', 'p76_config_audit.py', 'prepare_gz_camera_model.py',
        'p76_bodyrate_calibration.py', 'p76_calibration_analysis.py',
        'p76_calibration_manifest.py', 'p761_session.py', 'p761_validation.py',
        'p763_exit_replay.py', 'p763_dkf_replay.py', 'p763_control_replay.py')]
    return {str(p.relative_to(root)): sha256(p) for p in sorted(paths)}


def validate_calibration(contract, model_root, workspace):
    """Require measured force/rate/braking evidence, never promote model constants to flight."""
    path = contract['candidate']['bodyrate_calibration_manifest']
    if not path or not Path(path).is_file():
        raise ValueError('P76_ACTUAL_BODYRATE_CALIBRATION_MISSING')
    data = json.loads(Path(path).read_text())
    verify_model(model_root, contract)
    manifest = json.loads((Path(model_root)/'p76_model_manifest.json').read_text())
    px4 = Path(contract['dynamics']['px4_model_root']).parents[3]
    px4_identity = dict(binary_sha256=sha256(px4/'build/px4_sitl_default/bin/px4'),
                        dds_sha256=sha256(px4/'src/modules/uxrce_dds_client/dds_topics.yaml'))
    if (data.get('schema_version') != 2
            or data.get('architecture') != 'parallel_dkf_unified_bodyrate'
            or contract.get('qualification_schema_version') != 2
            or data.get('qualified') is not True
            or data.get('kind') != 'PX4_GAZEBO_BODYRATE_CALIBRATION'
            or data.get('sources') != source_fingerprint(workspace)
            or data.get('model_files') != manifest['files']
            or data.get('contract_sha256') != sha256(contract['_path'])
            or data.get('detector_sha256') != contract['detector']['model_sha256']
            or data.get('px4') != px4_identity):
        raise ValueError('P76_CALIBRATION_IDENTITY_MISMATCH')
    logs = data.get('raw_evidence', {})
    if not logs or any(not Path(p).is_file() or sha256(p) != h for p, h in logs.items()):
        raise ValueError('P76_CALIBRATION_RAW_EVIDENCE_MISSING')
    required = ('hover', 'thrust_curve', 'rates_response', 'acceleration_5mps2',
                'descent_2mps_braking', 'exclusive_px4_bodyrate', 'bodyrate_recovery',
                'native_land_takeover',
                'navigation_reference_and_sea_datum')
    if any(data.get('checks', {}).get(key) is not True for key in required):
        raise ValueError('P76_BODYRATE_QUALIFICATION_FAILED')
    if (data.get('response_seconds', 0) <= 0 or data.get('braking_acceleration', 0) <= 0
            or data.get('mass') != manifest['total_mass']
            or data.get('collision_envelope') != manifest['collision_envelope']):
        raise ValueError('P76_CALIBRATED_DYNAMICS_REQUIRED')
    formal = data.get('formal_recovery', {})
    core = {str(Path('src/uav_control/uav_control/control')/name): sha256(
        Path(workspace)/'src/uav_control/uav_control/control'/name) for name in (
            'p76_minco_bodyrate_tracker.py', 'p76_recovery.py', 'p76_bodyrate_safety.py',
            'p76_paper_thrust.py')}
    if (formal.get('controller') != 'P763_BODYRATE_RECOVERY'
            or formal.get('core_sources') != core
            or formal.get('settings') != contract['bodyrate_follow']
            or formal.get('contract', {}).get('candidate') != contract['candidate']
            or formal.get('mass') != data['mass']
            or formal.get('maximum_thrust') != data['maximum_thrust']
            or not 0 < formal.get('braking_request', 0) <= 4.
            or formal.get('response_bound', 0) < .24
            or data['response_seconds'] < formal['response_bound']):
        raise ValueError('P763_FORMAL_RECOVERY_QUALIFICATION_REQUIRED')
    return data


def validate_follow_qualification(contract, workspace):
    """C cannot inherit P75 or another model's nominal planner feasibility."""
    path = contract['candidate'].get('follow_qualification_manifest', '')
    if not path or not Path(path).is_file():
        raise ValueError('P76_CURRENT_25S_FOLLOW_QUALIFICATION_MISSING')
    data = json.loads(Path(path).read_text())
    if (data.get('schema_version') != 2
            or data.get('architecture') != 'parallel_dkf_unified_bodyrate'
            or data.get('px4_interface') != 'BODY_RATE'
            or data.get('qualified') is not True or data.get('valid_follow_seconds', 0) < 25
            or data.get('sources') != source_fingerprint(workspace)
            or data.get('contract_sha256') != sha256(contract['_path'])
            or data.get('actual_minco_ownership_verified') is not True
            or data.get('body_rate_output_verified') is not True
            or data.get('native_body_rate_mode_verified') is not True
            or data.get('both_estimators_active_fresh') is not True
            or data.get('image_follow_control_authorized') is not False
            or data.get('native_physical_execution_verified') is not True
            or data.get('physical_acceptance', {}).get('count', 0) < 450
            or data.get('physical_acceptance', {}).get('source_seconds', 0) < 24.9):
        raise ValueError('P76_CURRENT_FOLLOW_QUALIFICATION_INVALID')
    logs = data.get('raw_evidence', {})
    if not logs or any(not Path(p).is_file() or sha256(p) != h for p, h in logs.items()):
        raise ValueError('P76_FOLLOW_RAW_EVIDENCE_MISSING')
    return data
