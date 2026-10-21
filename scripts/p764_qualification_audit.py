#!/usr/bin/env python3
"""Read-only P764 qualification/budget audit; never create or relabel flight qualification."""
import argparse
import json
from pathlib import Path

import numpy as np
from rosidl_runtime_py.set_message import set_message_fields
from std_msgs.msg import String
from uav_usv_interfaces.msg import NativeAttitude, UavState

from p761_validation import BPhysicalAcceptance
from p76_calibration_analysis import analyze_rows, stage_verdict
from uav_control.common.p76_contract import load_contract, parameter_bindings
from uav_control.common.p76_model import verify_model
from uav_control.common.p76_readiness import (
    sha256, source_fingerprint, validate_calibration, validate_follow_qualification,
)

ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT/'data/experiments/20261021_p761_01'


def recompute_b_physics(path, advertised):
    """Replay original receipt order and source epochs through native physical acceptance."""
    gate, comparisons = BPhysicalAcceptance(), []
    kinds = dict(paper=String, navigation=UavState, pose=NativeAttitude, native_thrust=String)
    rows, invalid = 0, []
    with Path(path).open() as stream:
        for index, line in enumerate(stream, 1):
            row = json.loads(line)
            rows += 1
            kind = row.get('kind')
            if kind in kinds:
                try:
                    message = kinds[kind]()
                    set_message_fields(message, row['message'])
                    gate.observe(kind, message, row['receipt'])
                except (ValueError, TypeError, AttributeError, KeyError) as error:
                    invalid.append(dict(line=index, reason=str(error)))
            elif kind == 'physical_acceptance':
                passed, evidence = gate.evaluate(row['receipt'])
                if evidence.get('navigation_stamp', 0.) < advertised.get('source_start', 0.):
                    gate.reset_window()
                    continue
                end = advertised.get('source_end', float('inf'))
                if evidence.get('navigation_stamp', 0.) > end:
                    continue
                comparisons.append(passed == row.get('passed') and evidence == row.get('evidence'))
    metrics = gate.metrics()
    keys = ('count', 'source_start', 'source_end', 'source_seconds', 'rate_rmse',
            'position_error_max', 'velocity_error_max', 'acceleration_error_max',
            'collective_error_max')
    match = True
    for key in keys:
        if key not in advertised or key not in metrics:
            match = False
        else:
            match &= bool(np.allclose(metrics[key], advertised[key], atol=1e-9, rtol=1e-9))
    return dict(status='RECOMPUTED', source=str(path), source_sha256=sha256(path), rows=rows,
                metrics=metrics, physical_decisions_compared=len(comparisons),
                all_recorded_decisions_reproduced=bool(comparisons) and all(comparisons),
                advertised_metrics_match=bool(match), invalid_raw_messages=invalid,
                qualified_25s_native_window=bool(gate.qualified_window() and match
                                                 and comparisons and all(comparisons)
                                                 and not invalid),
                scope='Native physical window only; graph/authority/capture are separate gates')


def measured_stage(session, current, model_check):
    """Recompute old A1 bytes without assess_stage's intentional assessment-file side effect."""
    names = ('identity.json', 'run_result.json', 'analysis.json', 'calibration.jsonl')
    hashes = {str(session/name): sha256(session/name) for name in names}
    identity = json.loads((session/'identity.json').read_text())
    run = json.loads((session/'run_result.json').read_text())
    rows = [json.loads(line) for line in (session/'calibration.jsonl').read_text().splitlines()]
    analysis = analyze_rows(rows, identity['model']['total_mass'], identity['model']['gravity'])
    matches = dict(sources=identity['sources'] == current['sources'],
                   contract=identity['contract_sha256'] == current['contract_sha256'],
                   detector=identity['detector_sha256'] == current['detector_sha256'],
                   model_files=identity['model']['files'] == current['model_files'],
                   px4=identity['px4'] == current['px4'], physical_model=model_check['consistent'])
    saved_matches = json.loads((session/'analysis.json').read_text()) == analysis
    verdict = stage_verdict(run['stage'], run, analysis, rows,
                            all(matches.values()), saved_matches)
    return dict(session=str(session), original_run=run, current_recomputation=verdict,
                identity_matches_current=matches, saved_analysis_matches_current=saved_matches,
                measured_twr=analysis['measured_twr'],
                hover_normalized=analysis['hover_normalized'],
                response_seconds=analysis['response_seconds'],
                measured_checks=analysis['checks'], raw_evidence=hashes,
                original_logs_modified=False, historical_only=True)


def audit(model_root):
    """The actual absence of A/B/C/D is explicit and cannot be supplied by replay booleans."""
    contract = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    model_root = Path(model_root).resolve()
    model_path = model_root/'p76_model_manifest.json'
    model = json.loads(model_path.read_text())
    px4 = Path(contract['dynamics']['px4_model_root']).parents[3]
    current = dict(sources=source_fingerprint(ROOT), contract_sha256=sha256(contract['_path']),
                   detector_sha256=sha256(contract['detector']['model_file']),
                   model_manifest_sha256=sha256(model_path), model_files=model['files'],
                   px4=dict(binary_sha256=sha256(px4/'build/px4_sitl_default/bin/px4'),
                            dds_sha256=sha256(px4/'src/modules/uxrce_dds_client/dds_topics.yaml')))
    model_check = verify_model(model_root, contract)
    stages = []
    sessions = sorted(path.parent for path in TASK.glob('session_*/environment.json'))
    inputs = {str(Path(__file__)): sha256(__file__), str(model_path): sha256(model_path),
              str(Path(contract['_path'])): current['contract_sha256']}
    for session in sessions:
        for name in ('environment.json', 'run_result.json', 'identity.json'):
            path = session/name
            if path.is_file():
                inputs[str(path)] = sha256(path)
        if all((session/name).is_file() for name in (
                'identity.json', 'run_result.json', 'analysis.json', 'calibration.jsonl')):
            result = measured_stage(session, current, model_check)
            inputs.update(result['raw_evidence'])
            stages.append(result)
    qualification = {}
    for name, key, validator in (
            ('A', 'bodyrate_calibration_manifest', validate_calibration),
            ('B', 'follow_qualification_manifest', validate_follow_qualification)):
        path = Path(contract['candidate'][key])
        if not path.is_file():
            qualification[name] = dict(status='MISSING', path=str(path), qualified=False)
            continue
        inputs[str(path)] = sha256(path)
        try:
            data = (validator(contract, model_root, ROOT) if name == 'A'
                    else validator(contract, ROOT, model_root))
            qualification[name] = dict(status='CURRENT_IDENTITY_ACCEPTED', path=str(path),
                                       qualified=True)
            inputs.update(data['raw_evidence'])
            if name == 'B':
                paths = [Path(p) for p in data['raw_evidence']
                         if Path(p).name == 'validation.jsonl']
                if len(paths) != 1:
                    qualification[name]['native_recomputation'] = dict(status='RAW_LOG_MISSING')
                    qualification[name]['qualified'] = False
                else:
                    recomputed = recompute_b_physics(paths[0], data['physical_acceptance'])
                    qualification[name]['native_recomputation'] = recomputed
                    qualification[name]['qualified'] &= recomputed['qualified_25s_native_window']
        except (ValueError, OSError, KeyError) as error:
            qualification[name] = dict(status='REJECTED', path=str(path), qualified=False,
                                       reason=str(error))
    for name in ('C', 'D'):
        runs = []
        for session in sessions:
            path = session/'run_result.json'
            if path.is_file() and json.loads(path.read_text()).get('stage') == name:
                runs.append(str(path))
        status = 'NOT_RUN' if not runs else 'REQUIRES_NATIVE_CAPTURE_AUDIT'
        qualification[name] = dict(status=status, run_files=runs, qualified=False)
    evaluator = parameter_bindings(contract)['intercept_evaluator_node']
    future_stages = ['A1', 'A2', 'B', 'C', 'D']
    budget = dict(original_task=str(TASK), counted_sessions=[str(p) for p in sessions],
                  primary_sessions_used=len(sessions), primary_session_limit=5,
                  primary_sessions_remaining=max(0, 5-len(sessions)),
                  minimum_separate_future_sessions=len(future_stages), future_stages=future_stages,
                  old_budget_sufficient=5-len(sessions) >= len(future_stages),
                  future_budget_status='REQUIRES_NEW_EXPLICIT_USER_APPROVAL',
                  future_evidence_directory='UNASSIGNED_UNTIL_APPROVED',
                  historical_sessions_must_not_be_deleted_or_relabelled=True)
    findings = [
        dict(id='B_IDENTITY', status='FIXED_WITH_DIRECTED_TESTS', severity='P0',
             source='common/p76_readiness.py:validate_follow_qualification; p761_validation.py',
             detail=('B now binds current A SHA, actual model files/manifest, detector, PX4 '
                     'binary/DDS, sources/contract, and hashes validation_identity.json.')),
        dict(id='B_RAW_RECOMPUTATION', status='OFFLINE_AUDIT_ONLY', severity='P1',
             source='common/p76_readiness.py:validate_follow_qualification',
             detail=('The online reader verifies pinned bytes and identity, but still trusts '
                     'native_physical_execution_verified and physical_acceptance summary. '
                     'This independent audit replays native samples when an actual B exists; '
                     'missing actual B remains MISSING. No raw parsing is added to Y dispatch.')),
        dict(id='D_C_DEPENDENCY', status='OPEN', severity='P1', source='p761_session.py:run(D)',
             detail=('D checks prior C success and current source/contract, but does not pin '
                     'the C result/evaluator logs or compare prior C model/PX4/DDS identity. '
                     'Current A/B gates remain necessary and do not make C native evidence.')),
        dict(id='E1_E2_TYPED_EVIDENCE', status='OPEN', severity='P1',
             source='p761_session.py:validate_offline_gates',
             detail=('E1/E2 typed reports are hashed but their asserted pass fields are not '
                     'fully recomputed at admission. Real E1 coverage and E2 scope must be '
                     'reviewed independently; synthetic fixtures cannot grant flight readiness.')),
        dict(id='B_BOUNDS_SCOPE', status='UNQUALIFIED_CANDIDATE', severity='P1',
             source='p761_validation.py:BPhysicalAcceptance.bounds',
             detail=('Physical tracking thresholds remain the existing engineering candidates; '
                     'this audit does not change or claim actual validation of those bounds.')),
        dict(id='SESSION_BUDGET', status='BLOCKED_FUTURE_FLIGHTS', severity='P0',
             source='p761_session.py:run',
             detail=(f'{len(sessions)} historical sessions consume the original '
                     'five-session budget. '
                     'A1/A2/B/C/D needs a separately named, user-approved new budget.')),
        dict(id='FOLLOW_MEASURED_SPEED_PROTECTION', status='FIXED_WITH_DIRECTED_TESTS',
             severity='P0', source='control/p76_tracker_runtime.py; p76_minco_bodyrate_tracker.py',
             detail=('The armed runtime checks native measured speed against the unchanged '
                     '7 m/s limit before target-guidance computation, revokes MINCO, and enters '
                     'independent bounded recovery or native LAND.')),
    ]
    return dict(schema_version=2, architecture=contract['architecture'],
                audit_kind='READ_ONLY_OFFLINE_QUALIFICATION_AUDIT',
                current_identity=current, model_check=model_check, historical_stages=stages,
                qualification=qualification, budget=budget, findings=findings,
                dependencies=dict(A1=['current E0 new protocol', 'real E1', 'bounded E2',
                                      'separate authorization', 'explicit new session budget'],
                                  A2=['current measured A1', 'Body-rate recovery and native LAND'],
                                  A=['measured A1+A2', 'schema2',
                                     'current full identity/raw hashes'],
                                  B=['current A', '25 s actual MINCO Body-rate/native physics'],
                                  C=['current A+B', 'Y image-only control',
                                     'native capture <=0.25 m',
                                     'no sea contact before capture'],
                                  D=['independent new session', 'current qualified C identity',
                                     'same native capture checks']),
                capture_contract=dict(radius=evaluator['evaluation_capture_radius'],
                                      geometry=evaluator['capture_geometry_source'],
                                      native_capture_verified=False),
                source_inputs=inputs, flight_qualified=False, A1_GO=False,
                verdict='NO_GO', old_evidence_modified=False,
                limitation='This audit launches no flight; missing stages remain missing.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--model-root', type=Path, default=(
        ROOT/'data/experiments/20261021_p763_01/frozen/models'))
    args = parser.parse_args()
    result = audit(args.model_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(dict(verdict=result['verdict'], qualification=result['qualification'],
                          budget=result['budget']), indent=2))
    return 0  # Successful audit execution is independent of its NO-GO flight verdict.


if __name__ == '__main__':
    raise SystemExit(main())
