#!/usr/bin/env python3
"""Explicitly authorized single P5.1 F2, followed only by a qualified single F3."""
import argparse
import hashlib
import json
from pathlib import Path

import p5_minco_sitl as harness
from uav_control.guidance.follow_admission import AdmissionConfig

WORKSPACE = Path(__file__).resolve().parents[1]
ROOT = WORKSPACE/'data/experiments/20261012_p51_closed_loop_01'


def allow_f3(summary):
    """Sustained actual F2 plus complete P5.1 height, recovery and execution evidence."""
    from p51_execution_audit import CHECKS
    audit = summary.get('p51_audit') or {}
    return bool(harness.allow_f3(summary) and audit.get('passed')
                and all(audit.get('checks', {}).get(k) is True for k in CHECKS))


def validate_release(release):
    """Reuse named historical math evidence, never assert new wrappers are AST-equivalent."""
    source = Path(release['historical_h16_release'])
    if (release.get('kind') != 'P51_TESTED_PROTOCOL_CANDIDATE'
            or release.get('a_e_offline_pass') is not True
            or harness.sha(source) != release.get('historical_h16_sha256')):
        return False
    harness.verify_manifest(source.parent)
    historical = json.loads(source.read_text())
    return bool(release['candidate'] == historical['candidate']
                and release['formal_directory'] == historical['formal_directory']
                and harness.qualify(historical['evidence'])['h16_scoped_gate_pass'])


def health_decision(health, now):
    """Planning shortage inhibits entry only while original FOLLOW is independently lawful."""
    fatal = harness.safety_reason(health, now, planning_required=False)
    planning = harness.safety_reason(health, now)
    owner = health.get('control', {}).get('status')
    if fatal is None and owner == 'MINCO_FOLLOW':
        fatal = planning
    return dict(fatal=fatal, inhibit=planning if fatal is None else None)


def health_check(health, now):
    decision = health_decision(health, now)
    health['p51_decision'] = decision
    return decision['fatal']


def require_readiness(path, authorized):
    """Bind explicit flight permission to the tested sources, without consuming an attempt."""
    if not authorized:
        raise ValueError('EXPLICIT_P51_SITL_AUTHORIZATION_REQUIRED')
    readiness = json.loads(path.read_text())
    if not readiness.get('a_e_pass') or not readiness.get('source_checks'):
        raise ValueError('P51_A_E_NOT_PASSED')
    for row in readiness['source_checks']:
        source = Path(row['path'])
        if not source.is_absolute():
            source = WORKSPACE/source
        if hashlib.sha256(source.read_bytes()).hexdigest() != row['sha256']:
            raise ValueError('READINESS_SOURCE_CHANGED')
    return readiness


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--scene', choices=('f2', 'f3'), default='f2')
    parser.add_argument('--qualification', type=Path, required=True)
    parser.add_argument('--readiness', type=Path, required=True)
    parser.add_argument('--authorize-sitl', action='store_true')
    args = parser.parse_args()
    require_readiness(args.readiness.resolve(), args.authorize_sitl)
    from p51_execution_audit import audit
    raise SystemExit(harness.run(
        args.output.resolve(), args.qualification.resolve(), args.scene, task_root=ROOT,
        authorization='Explicit P5.1 human SITL permission; one F2 then qualified one F3',
        health_check=health_check,
        research_config=WORKSPACE/'src/uav_usv_bringup/config/p51_follow_research.yaml',
        extra_audit=audit, f3_gate=allow_f3, release_gate=validate_release,
        flight_parameters=dict(follow_admission_enabled=True, **{
            'follow_admission_'+key: value for key, value in vars(AdmissionConfig()).items()})))
