"""P764 runtime authority replay and current physical qualification identity boundaries."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_readiness import sha256, source_fingerprint

ROOT = Path(__file__).resolve().parents[3]


def old_positive_follow_manifest(tmp_path):
    """Build an adversarial unit fixture without granting physical or stage qualification."""
    evidence = tmp_path/'synthetic_nonflight_log.jsonl'
    evidence.write_text('{"synthetic_unit_fixture":true}\n')
    manifest = tmp_path/'synthetic_B.json'
    data = dict(schema_version=2, architecture='parallel_dkf_unified_bodyrate',
                px4_interface='BODY_RATE', qualified=True, valid_follow_seconds=25.,
                sources=source_fingerprint(ROOT),
                contract_sha256=sha256(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'),
                actual_minco_ownership_verified=True, body_rate_output_verified=True,
                native_body_rate_mode_verified=True, both_estimators_active_fresh=True,
                image_follow_control_authorized=False, native_physical_execution_verified=True,
                physical_acceptance=dict(count=500, source_seconds=25.),
                raw_evidence={str(evidence): sha256(evidence)})
    manifest.write_text(json.dumps(data))
    contract = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    contract['candidate']['follow_qualification_manifest'] = str(manifest)
    return contract, manifest, data


def test_positive_b_booleans_cannot_bypass_current_a_model_px4_and_dds_identity(tmp_path):
    from uav_control.common.p76_readiness import validate_follow_qualification
    contract, _, _ = old_positive_follow_manifest(tmp_path)
    with pytest.raises(ValueError, match='FOLLOW_CURRENT_IDENTITY_REQUIRED'):
        validate_follow_qualification(contract, ROOT)


@pytest.mark.parametrize('key', ['calibration_sha256', 'model_manifest_sha256', 'model_files',
                                 'px4', 'detector_sha256'])
def test_follow_identity_drift_is_rejected_even_with_pinned_raw_hashes(tmp_path, monkeypatch, key):
    from uav_control.common import p76_readiness as readiness
    contract, manifest, data = old_positive_follow_manifest(tmp_path)
    identity = dict(calibration_sha256='a'*64, model_manifest_sha256='b'*64,
                    model_files={'model.sdf': 'c'*64},
                    px4=dict(binary_sha256='d'*64, dds_sha256='e'*64),
                    detector_sha256='f'*64)
    current = deepcopy(identity)
    identity[key] = {'changed': True} if isinstance(identity[key], dict) else '0'*64
    data['physical_identity'] = identity
    manifest.write_text(json.dumps(data))
    monkeypatch.setattr(readiness, 'follow_physical_identity', lambda *args: current)
    with pytest.raises(ValueError, match='FOLLOW_PHYSICAL_IDENTITY_MISMATCH'):
        readiness.validate_follow_qualification(contract, ROOT)


def test_follow_identity_packet_must_be_pinned_in_raw_evidence(tmp_path, monkeypatch):
    from uav_control.common import p76_readiness as readiness
    contract, manifest, data = old_positive_follow_manifest(tmp_path)
    identity = dict(calibration_sha256='a'*64, model_manifest_sha256='b'*64,
                    model_files={'model.sdf': 'c'*64},
                    px4=dict(binary_sha256='d'*64, dds_sha256='e'*64),
                    detector_sha256='f'*64)
    data['physical_identity'] = identity
    manifest.write_text(json.dumps(data))
    monkeypatch.setattr(readiness, 'follow_physical_identity', lambda *args: identity)
    with pytest.raises(ValueError, match='FOLLOW_VALIDATION_IDENTITY_EVIDENCE_REQUIRED'):
        readiness.validate_follow_qualification(contract, ROOT)


def test_current_b_identity_is_accepted_and_a_changed_saved_identity_is_rejected(
        tmp_path, monkeypatch):
    from uav_control.common import p76_readiness as readiness
    contract, manifest, data = old_positive_follow_manifest(tmp_path)
    identity = dict(calibration_sha256='a'*64, model_manifest_sha256='b'*64,
                    model_files={'model.sdf': 'c'*64},
                    px4=dict(binary_sha256='d'*64, dds_sha256='e'*64),
                    detector_sha256='f'*64)
    identity_file = tmp_path/'validation_identity.json'
    saved = dict(sources=data['sources'], contract_sha256=data['contract_sha256'],
                 physical_identity=identity)
    identity_file.write_text(json.dumps(saved))
    data.update(physical_identity=identity, validation_identity_file=str(identity_file))
    data['raw_evidence'][str(identity_file)] = sha256(identity_file)
    manifest.write_text(json.dumps(data))
    monkeypatch.setattr(readiness, 'follow_physical_identity', lambda *args: identity)
    assert readiness.validate_follow_qualification(contract, ROOT)['physical_identity'] == identity
    saved['contract_sha256'] = '0'*64
    identity_file.write_text(json.dumps(saved))
    data['raw_evidence'][str(identity_file)] = sha256(identity_file)
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='FOLLOW_VALIDATION_IDENTITY_MISMATCH'):
        readiness.validate_follow_qualification(contract, ROOT)


def test_physical_identity_rejects_actual_detector_byte_drift(tmp_path, monkeypatch):
    from uav_control.common import p76_readiness as readiness
    contract = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    detector = tmp_path/'synthetic_detector.onnx'
    detector.write_bytes(b'explicit unit fixture')
    contract['detector']['model_file'] = str(detector)
    monkeypatch.setattr(readiness, 'validate_calibration', lambda *args: {})
    with pytest.raises(ValueError, match='FOLLOW_DETECTOR_IDENTITY_MISMATCH'):
        readiness.follow_physical_identity(contract, ROOT, tmp_path)


def test_actual_runtime_has_no_paper_steps_before_active_and_no_target_guidance_after_fault():
    from test_p761_calibration import script
    replay = script('p764_handover_replay')
    result = replay.handover_replay(load_contract(
        ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'))
    assert result['paper_step_calls_by_phase'].get('FOLLOW_ACTIVE', 0) == 0
    assert result['paper_step_calls_by_phase'].get('Y_HANDOVER', 0) == 0
    assert result['paper_step_calls_by_phase']['PAPER_INTERCEPT_ACTIVE'] >= 2
    assert result['paper_initialize_calls'] == result['png_initialize_calls'] == 1
    assert result['minco_revocations'] == 1
    assert result['old_minco_explicit_reentry_rejected']
    assert result['recovery_target_guidance_calls'] == 0
    assert result['first_y_output_continuous']
    assert result['target3d_reads'] == 0
    assert result['bodyrate_only'] and result['passed']
    assert not result['flight_qualified']


def test_fixed_offline_controller_grid_preserves_body_axis_signs_and_constraints():
    from test_p761_calibration import script
    replay = script('p764_handover_replay')
    result = replay.control_checks(load_contract(
        ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'))
    assert len(result['direction_cases']) == 12
    assert all(case['passed'] for case in result['direction_cases'])
    assert len(result['coordinate_transform_cases']) == 4
    assert all(case['passed'] for case in result['coordinate_transform_cases'])
    assert len(result['response_cases']) == 4
    assert all(case['final_error'] < .15 for case in result['response_cases'][:3])
    assert result['response_cases'][-1]['lag_seconds'] == .5
    assert result['response_cases'][-1]['final_error'] > .15
    assert result['same_epoch_pva_yaw_verified'] and result['constraints_passed']
    assert result['lease_expiry_rejected'] and result['invalid_reference_rejected']
    # A failed predeclared stress case is reported, never relabelled as an E2 pass.
    assert not result['passed'] and not result['flight_qualified']


def test_offline_b_audit_recomputes_native_samples_and_rejects_invented_25s_summary(tmp_path):
    from std_msgs.msg import String
    from test_p761_calibration import script
    from test_p763_b_physics import physical_fixture, feed
    audit = script('p764_qualification_audit')
    validation = script('p761_validation')
    gate, runtime, navigation, pose, collective = physical_fixture()
    feed(gate, runtime, navigation, pose, collective)
    passed, evidence = gate.evaluate(10.05)
    raw = tmp_path/'synthetic_native_fixture.jsonl'
    rows = [dict(kind=kind, receipt=10.05, message=validation.message_dict(message))
            for kind, message in (
                ('paper', String(data=json.dumps(runtime))), ('navigation', navigation),
                ('pose', pose), ('native_thrust', String(data=json.dumps(collective))))]
    rows.append(dict(kind='physical_acceptance', receipt=10.05, passed=passed, evidence=evidence))
    raw.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    advertised = dict(count=500, source_start=10.02, source_end=35.02, source_seconds=25.)
    result = audit.recompute_b_physics(raw, advertised)
    assert result['metrics']['count'] == 1
    assert result['all_recorded_decisions_reproduced']
    assert not result['advertised_metrics_match']
    assert not result['qualified_25s_native_window']
