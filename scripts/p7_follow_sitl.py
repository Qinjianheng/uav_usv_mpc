#!/usr/bin/env python3
"""One bounded P7 FOLLOW flight, using existing owned-session and evidence tools."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml

import p5_minco_sitl as harness
from p4_sitl_session import WORKSPACE, preflight
from p51_execution_audit import audit
from p53_minco_sitl import FollowHealthGuard, GroundStartupHealth
from uav_control.guidance.follow_sitl_gate import require_isolated_sitl

R11 = WORKSPACE/'data/experiments/20261013_p53_closed_loop_11'
CONFIG = WORKSPACE/'src/uav_usv_bringup/config/p7_guidance.yaml'


def runtime_hashes():
    """Current online code and configs; read-only observer revisions do not change flight code."""
    paths = list((WORKSPACE/'src/uav_control/uav_control').rglob('*.py'))
    for directory, pattern in (('launch', '*.py'), ('config', '*.yaml')):
        paths.extend((WORKSPACE/'src/uav_usv_bringup'/directory).glob(pattern))
    return {str(p.relative_to(WORKSPACE)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths)}


def source_fingerprint():
    """Bind runtime and this exact observer; each run also saves the complete source snapshot."""
    hashes = runtime_hashes()
    hashes[str(Path(__file__).resolve().relative_to(WORKSPACE))] = harness.sha(Path(__file__))
    return hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()


def require_f2(path):
    """A new source or failed F2 cannot inherit permission for F3."""
    summary = json.loads(Path(path).read_text())
    same_source = summary.get('source_fingerprint') == source_fingerprint()
    snapshot = Path(path).parent/'source_snapshot.json'
    if not same_source and snapshot.exists():
        # A diagnosed read-only observer change does not require repeating a
        # successful F2 with byte-identical online runtime/launch/configuration.
        previous = json.loads(snapshot.read_text())['files']
        previous_runtime = {
            k: v for k, v in previous.items()
            if (k.startswith('src/uav_control/uav_control/') and k.endswith('.py'))
            or (k.startswith('src/uav_usv_bringup/launch/') and k.endswith('.py'))
            or (k.startswith('src/uav_usv_bringup/config/') and k.endswith('.yaml'))}
        same_source = previous_runtime == runtime_hashes()
    if not (summary.get('healthy') and summary.get('completed') and summary.get('f3_eligible')
            and same_source):
        raise ValueError('CURRENT_HEALTHY_F2_REQUIRED')


class P7FollowHealthGuard(FollowHealthGuard):
    """Observe the controller's existing input-wait window separately from visual loss."""

    def __init__(self, initial_acquisition=False):
        super().__init__(initial_acquisition)
        self.input_wait_since = None

    def __call__(self, health, now):
        control = health.get('control', {})
        input_wait = (self.follow_lock_seen and control.get('status') == 'FOLLOW'
                      and control.get('visible') and control.get('search') == 'INPUT_WAIT')
        if not input_wait:
            self.input_wait_since = None
            return super().__call__(health, now)
        observed = dict(health, control=dict(control, locked=True))
        physical_fault = super().__call__(observed, now)
        if physical_fault:
            return physical_fault
        if self.input_wait_since is None:
            self.input_wait_since = now
        from uav_control.mission.target_visibility import VisibilityConfig
        config = VisibilityConfig()
        limit = config.target_lock_max_age*config.target_loss_frames
        elapsed = now-self.input_wait_since
        return 'FOLLOW_INPUT_WAIT_TIMEOUT' if not 0 <= elapsed <= limit else None


def prepare(output, scene):
    """Copy complete actual scene parameters into an exclusive new run directory."""
    if scene not in ('f2', 'f3'):
        raise ValueError('P7_F2_OR_F3_REQUIRED')
    flight = yaml.safe_load(CONFIG.read_text())
    source = R11/f'{scene}_h16_01/config/flight.yaml'
    original = yaml.safe_load(source.read_text())
    flight['moving_target'] = original['moving_target']
    planner = flight.pop('p4_follow_planner_node')
    output.mkdir(exist_ok=False)
    (output/'COLCON_IGNORE').touch()
    (output/'.gitignore').write_text('*\n')
    (output/'config').mkdir()
    (output/'config/flight.yaml').write_text(yaml.safe_dump(flight, sort_keys=False))
    (output/'config/research.yaml').write_text(yaml.safe_dump(
        {'p4_follow_planner_node': planner}, sort_keys=False))
    (output/'condition.json').write_text(json.dumps(dict(
        scene=scene, actual_controller='NOMINAL_MINCO', fallback='P44/D',
        prediction_model='bctra', maximum_input_age=.125, execution_lease_seconds=.45,
        source_fingerprint=source_fingerprint(),
        config_sources={str(p): harness.sha(p) for p in (CONFIG, source)},
        observation_seconds=25. if scene == 'f2' else 60.), indent=2)+'\n')


def run(output, scene, prior_f2=None):
    """Retain original admission warmup and strict health/lease checks, without P6 gates."""
    if not output.is_relative_to(WORKSPACE/'data/experiments'):
        raise ValueError('OUTPUT_MUST_BE_EXPERIMENT_DIRECTORY')
    if scene == 'f3':
        if prior_f2 is None:
            raise ValueError('CURRENT_HEALTHY_F2_REQUIRED')
        require_f2(prior_f2)
    preflight()
    prepare(output, scene)
    condition = json.loads((output/'condition.json').read_text())
    env = dict(os.environ, UAV_USV_HEADLESS_GCS='true', UAV_USV_MINCO_EXECUTE='true',
               UAV_USV_REQUIRE_PRE_X_HEALTH='true', UAV_USV_P6_PREDICTOR='bctra')
    processes, health = [], {}
    started = time.time()
    first_minco = follow_start = stable_start = planner_started = None
    health_process = planner = None
    health_started = None
    check = P7FollowHealthGuard(initial_acquisition=True)
    monitor_check = GroundStartupHealth(output)
    reason = 'SESSION_ENDED_BEFORE_OBSERVATION'
    try:
        session, owned = harness.shell_process([
            sys.executable, str(WORKSPACE/'scripts/p4_sitl_session.py'), str(output),
            '--follow-seconds', '180', '--watchdog', '280'], env, output/'session_driver.txt')
        processes.append((session, owned))
        while session.poll() is None and time.time()-started < 295.:
            environment = (harness.read_environment_snapshot(output)
                           if health_process is None else None)
            if (health_process is None and environment is not None
                    and harness.health_start_allowed(output, 'Starting UAV-USV experiment...')):
                env.update(environment)
                health_process, owned = harness.shell_process([
                    sys.executable, str(WORKSPACE/'scripts/p5_live_health.py'), str(output)],
                    env, output/'p5_health_monitor.txt')
                processes.append((health_process, owned))
                health_started = time.time()
            health, now = harness.read_health_snapshot(output/'p5_health.json')
            if health_process is not None:
                problem = monitor_check(health_process.poll(), health, now, health_started)
                if problem:
                    reason = problem
                    break
            if not health:
                time.sleep(.05)
                continue
            status = health.get('control', {}).get('status')
            if follow_start is None and status in ('FOLLOW', 'MINCO_FOLLOW'):
                follow_start = now
            if follow_start is not None:
                problem = check(health, now)
                if problem:
                    reason = problem
                    break
                if planner is None:
                    stable_start = harness.settled_since(
                        stable_start, health.get('follow_settling', {}), now)
                    if now-follow_start >= 60.:
                        reason = 'ORIGINAL_FOLLOW_NOT_SETTLED_WITHIN_60S'
                        break
                    if stable_start is not None and now-stable_start >= 5.:
                        require_isolated_sitl(env)
                        planner, owned = harness.shell_process([
                            'ros2', 'run', 'uav_control', 'p4_follow_planner_node', '--ros-args',
                            '--params-file', str(output/'config/research.yaml'),
                            '-p', 'log_directory:='+str(output/'shadow')],
                            env, output/'p5_planner.txt')
                        processes.append((planner, owned))
                        planner_started = now
                if planner is not None and planner.poll() is not None:
                    reason = 'PLANNER_EXITED'
                    break
                if first_minco is None and status == 'MINCO_FOLLOW':
                    first_minco = now
                if planner_started and first_minco is None and now-planner_started >= 10.:
                    reason = 'NO_REAL_MINCO_TAKEOVER_WITHIN_10S'
                    break
                if first_minco is not None and now-first_minco >= condition['observation_seconds']:
                    reason = 'MINCO_OBSERVATION_COMPLETE'
                    break
            time.sleep(.05)
        if time.time()-started >= 295.:
            reason = 'OUTER_WATCHDOG'
    finally:
        (output/'bounded_stop.json').write_text(json.dumps(dict(
            reason=reason, stamp=time.time(), follow_start=follow_start,
            first_minco=first_minco, health_snapshot=health), indent=2)+'\n')
        (output/'STOP').touch()
        if processes and processes[0][0].poll() is None:
            try:
                processes[0][0].wait(timeout=25.)
            except subprocess.TimeoutExpired:
                pass
        cleanup = harness.stop_owned(processes)
        late = harness.stop_late_components(output)
        cleanup['late_components'] = late
        cleanup['survivors'].extend(late['survivors'])
        (output/'p5_cleanup.json').write_text(json.dumps(cleanup, indent=2)+'\n')
    ended_path = output/'watchdog_end.json'
    ended = json.loads(ended_path.read_text()) if ended_path.exists() else {}
    if (output/'offline_evidence.jsonl').exists():
        summary = harness.assess(output, scene, reason, first_minco, ended,
                                 task_root=output, extra_audit=audit)
    else:
        summary = dict(completed=False, healthy=False, reason=reason, f3_eligible=False)
    summary.update(source_fingerprint=condition['source_fingerprint'], cleanup=cleanup)
    if cleanup['survivors'] or cleanup['errors'] or late['errors']:
        summary.update(healthy=False, f3_eligible=False)
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    manifest = {str(p.relative_to(output)): harness.sha(p)
                for p in sorted(output.rglob('*')) if p.is_file()}
    (output/'SHA256SUMS.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({k: summary.get(k) for k in (
        'completed', 'healthy', 'reason', 'minco_fraction', 'longest_minco_seconds',
        'diagnostic_coverage', 'f3_eligible')}, indent=2), flush=True)
    return 0 if summary['healthy'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--scene', choices=('f2', 'f3'), required=True)
    parser.add_argument('--prior-f2', type=Path)
    args = parser.parse_args()
    raise SystemExit(run(args.output.resolve(), args.scene, args.prior_f2))
