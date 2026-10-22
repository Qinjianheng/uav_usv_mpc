#!/usr/bin/env python3
"""Audit P7.6 effective parameters and retain a source-level dependency inventory."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.common.p76_contract import (  # noqa: E402
    dump_ros_parameters, load_contract, parameter_bindings, ros_parameters, validate_profile,
)
from uav_control.common.p76_model import prepare_model, verify_model  # noqa: E402
from uav_control.common.p76_readiness import (  # noqa: E402
    validate_calibration, validate_follow_qualification, source_fingerprint, sha256,
)


def dependency_inventory(workspace):
    """Record literal and dynamic parameter/topic expressions, never guess resolved values."""
    results = []
    methods = {'declare_parameter', 'get_parameter', 'create_publisher', 'create_subscription'}
    for folder in ('src/uav_control/uav_control', 'src/uav_usv_bringup/launch', 'scripts'):
        for path in sorted((workspace/folder).rglob('*.py')):
            source = path.read_text()
            try:
                tree = ast.parse(source)
            except SyntaxError:
                continue
            for node in ast.walk(tree):
                if (not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute)
                        or node.func.attr not in methods):
                    continue
                args = [ast.get_source_segment(source, arg) for arg in node.args[:3]]
                results.append(dict(file=str(path.relative_to(workspace)), line=node.lineno,
                                    operation=node.func.attr, arguments=args,
                                    resolution='literal' if all(isinstance(a, ast.Constant)
                                                                for a in node.args[:2])
                                    else 'expression_requires_runtime_audit'))
    return results


def generate(contract_path, profile_path):
    """Generate only the P76 profile; never alter a legacy input file."""
    contract = load_contract(contract_path)
    if profile_path.name != 'p76_yan_paper.yaml':
        raise ValueError('P76_PROFILE_OUTPUT_REQUIRED')
    profile = ros_parameters(contract, ROOT/'src/uav_usv_bringup/config/p7_spatial_candidate.yaml')
    profile_path.write_text('# Generated from p76_contract.yaml; opt-in only.\n'
                            + dump_ros_parameters(profile))


def audit(contract_path, profile_path, model_root=None, require_flight=False,
          require_calibration=False):
    """Check all declared bindings; later qualification gates also require actual artifacts."""
    contract = load_contract(contract_path)
    profile = yaml.safe_load(profile_path.read_text())
    validate_profile(contract, profile)
    result = dict(parameter_contract_valid=True, flight_qualified=False,
                  qualification_reason='MODEL_DYNAMICS_AND_RUNTIME_QUALIFICATION_REQUIRED',
                  contract_sha256=hashlib.sha256(contract_path.read_bytes()).hexdigest(),
                  profile_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                  bindings=parameter_bindings(contract))
    if model_root is not None:
        result['actual_model'] = verify_model(model_root, contract)
        detector = contract['detector']
        path = Path(detector['model_file'])
        if (not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()
                != detector['model_sha256']):
            raise ValueError('P76_REAL_YOLO_MODEL_REQUIRED')
        result['weight_sha256_verified'] = True
    if require_flight or require_calibration:
        if model_root is None:
            raise ValueError('P76_ACTUAL_SESSION_MODEL_REQUIRED')
        validate_calibration(contract, model_root, ROOT)
    if require_flight:
        validate_follow_qualification(contract, ROOT, model_root)
        qualification = Path(contract['detector']['qualification_manifest'])
        if not qualification.is_file():
            raise ValueError('P76_ROS_DETECTOR_QUALIFICATION_MISSING')
        detector_data = json.loads(qualification.read_text())
        logs = detector_data.get('raw_evidence', {})
        if (detector_data.get('schema_version') != 2
                or detector_data.get('architecture') != 'parallel_dkf_unified_bodyrate'
                or detector_data.get('sources') != source_fingerprint(ROOT)
                or not logs or any(not Path(p).is_file() or sha256(p) != h
                                   for p, h in logs.items())
                or detector_data.get('qualified') is not True
                or detector_data.get('model_sha256') != contract['detector']['model_sha256']
                or detector_data.get('contract_sha256') != result['contract_sha256']):
            raise ValueError('P76_ROS_DETECTOR_QUALIFICATION_INVALID')
        result.update(flight_qualified=True, qualification_reason='CURRENT_ARTIFACTS_VERIFIED')
    return result


def main():
    """Generate or read-only check explicit candidate inputs and optional dependency inventory."""
    parser = argparse.ArgumentParser(description=__doc__)
    folder = ROOT/'src/uav_usv_bringup/config'
    parser.add_argument('--contract', type=Path, default=folder/'p76_contract.yaml')
    parser.add_argument('--profile', type=Path, default=folder/'p76_yan_paper.yaml')
    parser.add_argument('--generate', action='store_true')
    parser.add_argument('--inventory', type=Path)
    parser.add_argument('--model-root', type=Path)
    parser.add_argument('--prepare-model', type=Path)
    parser.add_argument('--require-flight', action='store_true')
    parser.add_argument('--require-calibration', action='store_true')
    options = parser.parse_args()
    if options.generate:
        generate(options.contract, options.profile)
    if options.prepare_model:
        contract = load_contract(options.contract)
        prepare_model(contract['dynamics']['source_camera_model'],
                      contract['dynamics']['px4_model_root'], options.prepare_model, contract)
        options.model_root = options.prepare_model
    result = audit(options.contract, options.profile, options.model_root, options.require_flight,
                   options.require_calibration)
    if options.inventory:
        options.inventory.mkdir(parents=True, exist_ok=False)
        (options.inventory/'dependencies.json').write_text(json.dumps(
            dependency_inventory(ROOT), indent=2)+'\n')
        (options.inventory/'contract_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'bindings'}, indent=2))


if __name__ == '__main__':
    main()
