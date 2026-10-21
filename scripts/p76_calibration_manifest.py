#!/usr/bin/env python3
"""Generate qualification only by recomputing checks from pinned actual stage logs."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.common.p76_readiness import sha256  # noqa: E402


def validate_stage_evidence(session, result):
    """A positive boolean is insufficient without physical samples and pinned identity."""
    required = [session/'calibration.jsonl', session/'identity.json', session/'run_result.json']
    if any(not p.is_file() for p in required):
        raise ValueError('P761_RAW_EVIDENCE_REQUIRED')
    from p76_calibration_analysis import analyze_rows
    identity = json.loads(required[1].read_text())
    rows = [json.loads(line) for line in required[0].read_text().splitlines()]
    actual = analyze_rows(rows, identity['model']['total_mass'], identity['model']['gravity'])
    if actual != result:
        raise ValueError('P761_RECOMPUTED_ANALYSIS_MISMATCH')
    if not json.loads(required[2].read_text()).get('completed'):
        raise ValueError('P761_INCOMPLETE_CALIBRATION_RUN')
    return identity, {str(p): sha256(p) for p in required}


def generate(a1, a2, output):
    """No --force/qualified option exists; current identities and measured checks decide."""
    from p76_calibration_analysis import analyze_rows
    from uav_control.common.p76_contract import load_contract
    from uav_control.common.p76_readiness import source_fingerprint
    stages = []
    for session in (a1, a2):
        identity = json.loads((session/'identity.json').read_text())
        rows = [json.loads(line)
                for line in (session/'calibration.jsonl').read_text().splitlines()]
        analysis = analyze_rows(rows, identity['model']
                                ['total_mass'], identity['model']['gravity'])
        validate_stage_evidence(session, analysis)
        stages.append((identity, analysis))
    identity, first = stages[0]
    if identity != stages[1][0] or identity['sources'] != source_fingerprint(ROOT):
        raise ValueError('P761_STALE_OR_MIXED_CALIBRATION_IDENTITY')
    contract = load_contract(identity['contract_file'])
    if identity['contract_sha256'] != sha256(contract['_path']):
        raise ValueError('P761_STALE_CONTRACT')
    checks = dict(first['checks'])
    second = stages[1][1]
    for key in ('descent_2mps_braking', 'fault_recovery'):
        checks[key] = second['checks'][key]
    passed = (all(checks.values()) and second['braking_acceleration'] > 0
              and second['response_seconds'] > 0)
    data = dict(
        schema_version=1,
        kind='PX4_GAZEBO_BODYRATE_CALIBRATION',
        qualified=passed,
        sources=identity['sources'],
        contract_sha256=identity['contract_sha256'],
        detector_sha256=identity['detector_sha256'],
        px4=identity['px4'],
        model_files=identity['model']['files'],
        collision_envelope=identity['model']['collision_envelope'],
        mass=identity['model']['total_mass'],
        maximum_thrust=first['maximum_effective_thrust'],
        force_newtons=[
            p['force'] for p in first['measured_curve']],
        normalized=[
            p['normalized'] for p in first['measured_curve']],
        response_seconds=second['response_seconds'],
        braking_acceleration=second['braking_acceleration'],
        checks=checks,
        raw_evidence={
            str(p): sha256(p) for s in (
                a1,
                a2) for p in (
                s/'calibration.jsonl',
                s/'identity.json',
                s/'run_result.json')})
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise ValueError('P761_NEVER_OVERWRITE_QUALIFICATION')
    output.write_text(json.dumps(data, indent=2)+'\n')
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a1', required=True, type=Path)
    parser.add_argument('--a2', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    return 0 if generate(args.a1, args.a2, args.output) else 2


if __name__ == '__main__':
    raise SystemExit(main())
