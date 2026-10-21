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
    from uav_control.common.p76_readiness import source_fingerprint, sha256
    coverage, timing, replayed = (e1.get(k, {}) for k in ('coverage', 'tracker_shadow', 'replay'))
    logs = e1.get('raw_evidence', {})
    if (e1.get('evidence_kind') != 'P764_REAL_SYNC_DKF_EXECUTOR'
            or e1.get('sources') != source_fingerprint(WORKSPACE)
            or not logs or any(not Path(p).is_file() or sha256(p) != h for p, h in logs.items())
            or coverage.get('evidence_class') != 'REAL_SYNCHRONIZED_UNARMED_SESSION'
            or coverage.get('qualified') is not True
            or coverage.get('causal_native_pose_navigation_brackets', 0) < 50
            or coverage.get('posterior_and_covariance_preserved') is not True
            or coverage.get('unarmed') is not True
            or timing.get('evidence_class') != 'ACTUAL_ROS_EXECUTOR_DISCONNECTED_SHADOW'
            or timing.get('qualified') is not True
            or timing.get('missed_cycles', 1) != 0
            or timing.get('compute_deadline_misses', 1) != 0
            or replayed.get('evidence_class') != 'REAL_MATCHED_CALLBACK_REPLAY'
            or replayed.get('original_acquisition_epochs_preserved') is not True
            or replayed.get('original_receipt_epochs_preserved') is not True
            or replayed.get('parallel', {}).get('spatial_distinct_updates_after_y', 0) <= 0
            or replayed.get('parallel', {}).get('illegal_authorizations', 1) != 0):
        raise ValueError('P764_REAL_E1_INPUT_OR_EXECUTOR_NOT_QUALIFIED')


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
    if (e2.get('schema_version') != 2 or e2.get('logic_passed') is not True
            or e2.get('recorded_replay', {}).get('output_bounds_and_continuity') is not True):
        raise ValueError('P763_TYPED_OFFLINE_EVIDENCE_NOT_QUALIFIED')


def run(output, stage, prior=None, *, authorized_stage=None, offline_gates=None):
    """No session starts without a free isolated simulator and a remaining budget slot."""
    if authorized_stage != stage:
        raise ValueError('P763_SEPARATE_STAGE_AUTHORIZATION_REQUIRED')
    if stage == 'A1':
        validate_offline_gates(offline_gates)
    output = output.resolve()
    task = WORKSPACE/'data/experiments/20261021_p761_01'
    if not output.is_relative_to(task):
        raise ValueError('P761_SCOPED_OUTPUT_REQUIRED')
    if not output.name.startswith('session_'):
        raise ValueError('P761_COUNTED_SESSION_NAME_REQUIRED')
    sessions = list(task.glob('session_*/environment.json'))
    if len(sessions) >= 5:
        raise ValueError('P761_FIVE_SESSION_BUDGET_EXHAUSTED')
    if stage == 'D':
        from uav_control.common.p76_readiness import source_fingerprint, sha256
        current_sources = source_fingerprint(WORKSPACE)
        contract_hash = sha256(WORKSPACE/'src/uav_usv_bringup/config/p76_contract.yaml')
        eligible = False
        for candidate in task.glob('session_*/run_result.json'):
            identity_file = candidate.parent/'validation_identity.json'
            result = json.loads(candidate.read_text())
            if (result.get('stage') != 'C' or not result.get('success')
                    or not identity_file.exists()):
                continue
            frozen = json.loads(identity_file.read_text())
            eligible |= (frozen['sources'] == current_sources
                         and frozen['contract_sha256'] == contract_hash)
        if not eligible:
            raise ValueError('P761_CURRENT_SUCCESSFUL_C_REQUIRED_FOR_D')
    output.mkdir(parents=True, exist_ok=False)
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
    args = parser.parse_args()
    result = run(args.output, args.stage, args.prior_a1,
                 authorized_stage=args.authorize_stage, offline_gates=args.offline_gates)
    print(json.dumps(result, indent=2))
    return stage_exit_code(result)


if __name__ == '__main__':
    raise SystemExit(main())
