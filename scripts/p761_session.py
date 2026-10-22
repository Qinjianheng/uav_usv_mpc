#!/usr/bin/env python3
"""Own one budgeted A1/A2/B/C/D session, including bounded startup and exact-identity cleanup."""
import argparse
import json
import os
import shlex
import subprocess
from pathlib import Path
import sys
import time

from p4_sitl_session import WORKSPACE, identity, preflight
from p5_minco_sitl import shell_process, stop_late_components, stop_owned
from p763_exit_replay import new_protocol, replay
from p765_qualification import (
    capture_evidence, capture_identity, validate_e1_raw, validate_e2_evidence,
    validate_experiment_budget, validate_previous_c,
)


def launcher_process(arguments, env, path):
    """Keep stdin open so PX4's console blocks rather than spinning on inherited EOF."""
    command = ('set -eo pipefail; source /opt/ros/humble/setup.bash; source '
               + shlex.quote(str(WORKSPACE/'install/setup.bash'))
               + '; exec '+shlex.join(arguments))
    with path.open('x') as log:
        process = subprocess.Popen(['bash', '-c', command], cwd=WORKSPACE, env=env,
                                   stdin=subprocess.PIPE, stdout=log, stderr=log,
                                   start_new_session=True)
    return process, identity(process.pid)


def validate_e0_evidence(historical, proposed):
    """A refused historical accident and an independently admissible action are distinct."""
    if not isinstance(proposed, dict):
        raise ValueError('P764_NEW_PROTOCOL_REQUIRED')
    if (not isinstance(historical, dict)
            or historical.get('evidence_kind') != 'historical_fault_regression'
            or historical != replay(Path(historical['source_session']))
            or historical.get('historical_unsafe_pulse_rejected') is not True
            or historical.get('passed') is not True):
        raise ValueError('P764_HISTORICAL_UNSAFE_PULSE_REGRESSION_REQUIRED')
    if (proposed.get('evidence_kind') != 'independent_proposed_protocol_analysis'
            or proposed.get('source_session') != historical.get('source_session')
            or proposed != new_protocol(Path(proposed['source_session']),
                                        proposed.get('physical_identity', {}).get(
                                            'actual_model_root'))):
        raise ValueError('P764_NEW_PROTOCOL_EVIDENCE_INVALID')
    if (proposed.get('new_protocol_defined') is not True
            or proposed.get('new_protocol_admissible') is not True
            or not proposed.get('new_protocol_source_sha256')
            or not proposed.get('executable_steps')):
        raise ValueError('P764_A1_PROTOCOL_BLOCKED')


def validate_e1_evidence(e1):
    """Require real synchronized inputs and actual executor timing, never a synthetic fixture."""
    from uav_control.common.p76_readiness import source_fingerprint
    if (e1.get('evidence_kind') != 'P764_REAL_SYNC_DKF_EXECUTOR'
            or e1.get('sources') != source_fingerprint(WORKSPACE)):
        raise ValueError('P764_REAL_E1_INPUT_OR_EXECUTOR_NOT_QUALIFIED')
    return validate_e1_raw(e1)


def validate_offline_gates(path):
    """An offline logic pass cannot masquerade as high-pulse capability or real coverage."""
    from uav_control.common.p76_readiness import source_fingerprint, sha256
    if path is None:
        raise ValueError('P763_OFFLINE_GATES_REQUIRED')
    gates = json.loads(Path(path).read_text())
    if (gates.get('schema_version') != 2
            or gates.get('architecture') != 'parallel_dkf_unified_bodyrate'
            or gates.get('sources') != source_fingerprint(WORKSPACE)
            or gates.get('contract_sha256') != sha256(
                WORKSPACE/'src/uav_usv_bringup/config/p76_contract.yaml')
            or not all(gates.get('gates', {}).get(k) is True for k in ('E0', 'E1', 'E2'))):
        raise ValueError('P763_OFFLINE_GATES_NOT_CURRENT_OR_NOT_SAFE')
    evidence = gates.get('raw_evidence', {})
    if not evidence or any(not Path(p).is_file() or sha256(p) != digest
                           for p, digest in evidence.items()):
        raise ValueError('P763_OFFLINE_GATES_EVIDENCE_INVALID')
    typed = gates.get('stage_evidence', {})
    if set(typed) != {'E0', 'E0_NEW_PROTOCOL', 'E1', 'E2'} or any(
            str(Path(p).resolve()) not in evidence for p in typed.values()):
        raise ValueError('P763_TYPED_OFFLINE_EVIDENCE_REQUIRED')
    e0, proposed, e1, e2 = (json.loads(Path(typed[k]).read_text()) for k in (
        'E0', 'E0_NEW_PROTOCOL', 'E1', 'E2'))
    validate_e0_evidence(e0, proposed)
    validate_e1_evidence(e1)
    if e2.get('schema_version') != 2:
        raise ValueError('P763_TYPED_OFFLINE_EVIDENCE_NOT_QUALIFIED')
    validate_e2_evidence(e2)


def run(output, stage, prior=None, *, authorized_stage=None, offline_gates=None,
        approved_budget=None, authorized_budget_id=None, prior_c=None, current_model_root=None):
    """No session starts without a separately approved new budget and explicit stage authority."""
    if authorized_stage != stage:
        raise ValueError('P763_SEPARATE_STAGE_AUTHORIZATION_REQUIRED')
    budget = validate_experiment_budget(approved_budget, authorized_budget_id, output, stage)
    if stage == 'A1':
        validate_offline_gates(offline_gates)
    output = output.resolve()
    c_binding, current_capture_identity = None, None
    if stage == 'D':
        from uav_control.common.p76_contract import load_contract
        if not current_model_root:
            raise ValueError('P765_CURRENT_PHYSICAL_MODEL_REQUIRED_FOR_D')
        contract = load_contract(WORKSPACE/'src/uav_usv_bringup/config/p76_contract.yaml')
        current_capture_identity = capture_identity(contract, current_model_root, WORKSPACE)
        c_binding = validate_previous_c(prior_c, current_capture_identity)
        if output == Path(prior_c).resolve():
            raise ValueError('P765_D_REQUIRES_INDEPENDENT_NEW_SESSION')
    output.mkdir(parents=True, exist_ok=False)
    (output/'approved_budget_binding.json').write_text(json.dumps(budget, indent=2)+'\n')
    if c_binding:
        (output/'previous_c_binding.json').write_text(json.dumps(c_binding, indent=2)+'\n')
    check = preflight()
    (output/'preflight.json').write_text(json.dumps(check, indent=2)+'\n')
    shim = output/'terminal_shim'
    shim.mkdir()
    wrapper = shim/'gnome-terminal'
    wrapper.write_text('''#!/usr/bin/env python3
import os, pathlib, subprocess, sys
root = pathlib.Path(os.environ['UAV_USV_RESEARCH_DIRECTORY'])
title = next(a.split('=', 1)[1] for a in sys.argv if a.startswith('--title='))
with (root/(title.replace(' ', '_')+'.txt')).open('x') as log:
    subprocess.Popen(sys.argv[sys.argv.index('--')+1:], stdout=log,
                     stderr=log, start_new_session=True)
''')
    wrapper.chmod(0o755)
    started = time.time()
    env = dict(os.environ, UAV_USV_WS=str(WORKSPACE), UAV_USV_RESEARCH_DIRECTORY=str(output),
               UAV_USV_EXPERIMENT_LAUNCH=('p761_calibration.launch.py' if stage.startswith('A')
                                          else 'p76_yan_paper.launch.py'),
               UAV_USV_P761_STAGE=stage,
               UAV_USV_P76_CONTRACT=str(WORKSPACE/'src/uav_usv_bringup/config/p76_contract.yaml'),
               UAV_USV_EXPERIMENT_CONFIG_FILE=str(
                   WORKSPACE/'src/uav_usv_bringup/config/p76_yan_paper.yaml'),
               UAV_USV_START_QGC='false', UAV_USV_HEADLESS_GCS='true', UAV_USV_GZ_SERVER_ONLY='1',
               ROS_DOMAIN_ID=str(check['domain']), GZ_PARTITION='p761_'+str(os.getpid()),
               ROS_LOG_DIR=str(output/'ros_logs'), ROS_LOCALHOST_ONLY='0',
               UAV_USV_TERMINAL_LOG_ROOT=str(output),
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               CAMERA_STARTUP_TIMEOUT='60', FLIGHT_READY_TIMEOUT='50',
               PATH=str(shim)+':'+os.environ['PATH'])
    if prior:
        env['UAV_USV_P761_PRIOR_A1'] = str(prior.resolve())
    grant = output/'sitl_execution_grant.json'
    grant.write_text(
        json.dumps(
            dict(
                pid=os.getpid(),
                process_start=identity(
                    os.getpid())['start'],
                issued=started,
                domain=check['domain'],
                partition=env['GZ_PARTITION'],
                preflight_clear=True,
                authorization='Separate operator authorization for P763 '+stage)))
    grant.chmod(0o600)
    env['UAV_USV_SITL_GRANT'] = str(grant)
    (output/'environment.json').write_text(json.dumps(
        {k: v for k, v in env.items() if k.startswith(('UAV_USV_', 'ROS_', 'GZ_'))},
        indent=2)+'\n')
    processes, observer_started, observer = [], False, None
    try:
        processes.append(shell_process([sys.executable, str(
            WORKSPACE/'scripts/p45_gcs_heartbeat.py')], env, output/'gcs.txt'))
        processes.append(launcher_process(
            [str(WORKSPACE/'scripts/start_px4_ros2.sh'), '--no-build'],
            env, output/'launcher.txt'))
        while time.time()-started < 240.:
            log = output/'UAV-USV_experiment.txt'
            if not stage.startswith('A') and log.exists() and not observer_started:
                import hashlib
                model_data = json.loads((output/'p76_session_model.json').read_text())
                model_root = Path(model_data['model_root'])
                actual_model_sha = hashlib.sha256(
                    (model_root/'p76_model_manifest.json').read_bytes()).hexdigest()
                if actual_model_sha != model_data['manifest_sha256']:
                    raise ValueError('P761_SESSION_MODEL_IDENTITY_CHANGED')
                env['UAV_USV_P76_MODEL_ROOT'] = str(model_root)
                if stage == 'D':
                    actual_identity = capture_identity(contract, model_root, WORKSPACE)
                    if actual_identity != current_capture_identity:
                        raise ValueError('P765_D_ACTUAL_MODEL_IDENTITY_CHANGED')
                    validate_previous_c(prior_c, actual_identity)
                observer_started = True
                for args, name in (([sys.executable, str(WORKSPACE/'scripts/p761_validation.py'),
                                     '--stage', stage, '--output', str(output)], 'observer.txt'),
                                   ([sys.executable, str(WORKSPACE/'scripts/p4_shadow_monitor.py'),
                                     str(output/'offline_evidence.jsonl')], 'monitor.txt')):
                    processes.append(shell_process(args, env, output/name))
                    if name == 'observer.txt':
                        observer = processes[-1][0]
            if (output/'run_result.json').exists():
                result = json.loads((output/'run_result.json').read_text())
                if observer is not None and observer.wait(timeout=5) != 0:
                    result.update(success=False, reason='OBSERVER_EXIT_FAILED')
                if stage.startswith('A'):
                    from p76_calibration_analysis import assess_stage
                    result = assess_stage(output)
                elif stage == 'C' and result.get('success') is True:
                    from uav_control.common.p76_contract import load_contract
                    current = capture_identity(load_contract(
                        WORKSPACE/'src/uav_usv_bringup/config/p76_contract.yaml'), model_root)
                    evidence = capture_evidence(output, current)
                    (output/'capture_qualification.json').write_text(
                        json.dumps(evidence, indent=2)+'\n')
                return result
            if processes[1][0].poll() is not None:
                break
            time.sleep(.2)
        failure = dict(completed=False, success=False,
                       reason='LAUNCH_OR_SESSION_WATCHDOG', stage=stage)
        (output/'harness_failure.json').write_text(json.dumps(failure, indent=2)+'\n')
        return failure
    finally:
        cleanup = stop_owned(processes)
        cleanup['late_components'] = stop_late_components(output)
        (output/'cleanup.json').write_text(json.dumps(cleanup, indent=2)+'\n')


def stage_exit_code(result):
    """A calibration process that finishes without qualified dynamics still fails."""
    return 0 if result.get('success') is True else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--stage', choices=('A1', 'A2', 'B', 'C', 'D'), required=True)
    parser.add_argument('--prior-a1', type=Path)
    parser.add_argument('--authorize-stage', choices=('A1', 'A2', 'B', 'C', 'D'), required=True,
                        help='Separate explicit authorization for exactly this bounded stage')
    parser.add_argument('--offline-gates', type=Path,
                        help='Frozen current E0/E1/E2 evidence manifest required before A1')
    parser.add_argument('--approved-budget', type=Path, required=True,
                        help='New separately approved budget; historical slots are closed')
    parser.add_argument('--authorize-budget-id', required=True,
                        help='Explicit approval for the exact independently approved budget id')
    parser.add_argument('--prior-c', type=Path, help='Bound current successful C evidence for D')
    parser.add_argument('--current-model-root', type=Path,
                        help='Actual current physical model to compare with previous C before D')
    args = parser.parse_args()
    result = run(args.output, args.stage, args.prior_a1,
                 authorized_stage=args.authorize_stage, offline_gates=args.offline_gates,
                 approved_budget=args.approved_budget,
                 authorized_budget_id=args.authorize_budget_id,
                 prior_c=args.prior_c, current_model_root=args.current_model_root)
    print(json.dumps(result, indent=2))
    return stage_exit_code(result)


if __name__ == '__main__':
    raise SystemExit(main())
