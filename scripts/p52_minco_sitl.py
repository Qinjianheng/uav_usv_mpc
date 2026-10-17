#!/usr/bin/env python3
"""P5.2 one-shot F2 and separately authorized, actual-F2-qualified F3."""
import argparse
import json
from pathlib import Path

import p5_minco_sitl as harness
import p51_minco_sitl as previous
from uav_control.guidance.follow_admission import AdmissionConfig

WORKSPACE = Path(__file__).resolve().parents[1]
ROOT = WORKSPACE/'data/experiments/20261013_p52_closed_loop_01'
KIND = 'P52_TESTED_PROTOCOL_CANDIDATE'
allow_f3 = previous.allow_f3


def round_root(number):
    """Each evidence-based diagnostic round has its own one-shot F2/F3 ledger."""
    if type(number) is not int or not 1 <= number <= 3:
        raise ValueError('BOUNDED_P52_ROUND_REQUIRED')
    return ROOT.with_name(f'20261013_p52_closed_loop_{number:02d}')


def validate_release(release):
    if release.get('kind') != KIND:
        return False
    # Same named historical H1.6 evidence; changed protocol/scheduling uses fresh P5.2 tests.
    return previous.validate_release(dict(release, kind='P51_TESTED_PROTOCOL_CANDIDATE'))


def require_readiness(path, authorized, qualification=None):
    """No attempt is consumed until permission, independent review and hashes all pass."""
    if not authorized:
        raise ValueError('EXPLICIT_P52_SITL_AUTHORIZATION_REQUIRED')
    readiness = json.loads(path.read_text())
    verification = path.parent/'verification.json'
    qualification = qualification or path.parent/'qualification.json'
    if (readiness.get('kind') != KIND or readiness.get('a_e_pass') is not True
            or not readiness.get('source_checks')
            or harness.sha(verification) != readiness.get('verification_sha256')
            or harness.sha(qualification) != readiness.get('qualification_sha256')):
        raise ValueError('P52_A_E_NOT_PASSED')
    verified = json.loads(verification.read_text())
    if verified.get('passed') is not True or verified.get('independent_review_pass') is not True:
        raise ValueError('P52_A_E_NOT_PASSED')
    for row in readiness['source_checks']:
        source = Path(row['path'])
        source = source if source.is_absolute() else WORKSPACE/source
        if harness.sha(source) != row['sha256']:
            raise ValueError('READINESS_SOURCE_CHANGED')
    return readiness


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--scene', choices=('f2', 'f3'), default='f2')
    parser.add_argument('--round', type=int, choices=(1, 2, 3), default=1)
    parser.add_argument('--qualification', type=Path, required=True)
    parser.add_argument('--readiness', type=Path, required=True)
    parser.add_argument('--authorize-f2', action='store_true')
    parser.add_argument('--authorize-f3', action='store_true')
    args = parser.parse_args()
    authorized = args.authorize_f2 if args.scene == 'f2' else args.authorize_f3
    require_readiness(args.readiness.resolve(), authorized, args.qualification.resolve())
    from p51_execution_audit import audit
    raise SystemExit(harness.run(
        args.output.resolve(), args.qualification.resolve(), args.scene,
        task_root=round_root(args.round),
        authorization='Explicit P5.2 human permission for this scene; maximum one attempt',
        health_check=previous.health_check,
        research_config=WORKSPACE/'src/uav_usv_bringup/config/p51_follow_research.yaml',
        extra_audit=audit, f3_gate=allow_f3, release_gate=validate_release,
        flight_parameters=dict(follow_admission_enabled=True, **{
            'follow_admission_'+key: value for key, value in vars(AdmissionConfig()).items()})))
