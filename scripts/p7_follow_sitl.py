#!/usr/bin/env python3
"""One bounded P7 FOLLOW flight, using existing owned-session and evidence tools."""

import argparse
import hashlib
import json
import os
import signal
import socket
from pathlib import Path
import subprocess
import sys
import time

import yaml

import p5_minco_sitl as harness
from p4_sitl_session import WORKSPACE, identity, preflight
from p51_execution_audit import audit
from p53_minco_sitl import FollowHealthGuard, GroundStartupHealth
from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
from uav_control.usv_motion_profiles import MOTION_MODES, normalize_motion_mode

R11 = WORKSPACE/'data/experiments/20261013_p53_closed_loop_11'
CONFIG = WORKSPACE/'src/uav_usv_bringup/config/p7_guidance.yaml'


def runtime_hashes():
    """Current online code and configs; read-only observer revisions do not change flight code."""
    paths = list((WORKSPACE/'src/uav_control/uav_control').rglob('*.py'))
    for directory, pattern in (('launch', '*.py'), ('config', '*.yaml')):
        paths.extend((WORKSPACE/'src/uav_usv_bringup'/directory).glob(pattern))
    paths.extend((WORKSPACE/'src/uav_usv_bringup/models/x500_mono_cam/model.sdf',
                  WORKSPACE/'scripts/prepare_gz_camera_model.py'))
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
        for key in ('src/uav_usv_bringup/models/x500_mono_cam/model.sdf',
                    'scripts/prepare_gz_camera_model.py'):
            if key in previous:
                previous_runtime[key] = previous[key]
        same_source = previous_runtime == runtime_hashes()
    if not (summary.get('healthy') and summary.get('completed') and summary.get('f3_eligible')
            and same_source):
        raise ValueError('CURRENT_HEALTHY_F2_REQUIRED')


class P7FollowHealthGuard(FollowHealthGuard):
    """Observe the controller's existing input-wait window separately from visual loss."""

    def __init__(self, initial_acquisition=False, flight_altitude=-5.):
        super().__init__(initial_acquisition)
        self.flight_altitude = float(flight_altitude)
        self.input_wait_since = None
        self.prediction_response_since = None

    def __call__(self, health, now):
        health['follow_reference_altitude'] = self.flight_altitude
        control = health.get('control', {})
        prediction = health.get('prediction', {})
        if (control.get('status') == 'MINCO_FOLLOW' and not prediction.get('valid')
                and prediction.get('receipt', 0.) > control.get('receipt', 0.)):
            reason = super().__call__(health, now)
            if reason == 'PREDICTION_INPUT_STALE':
                # DDS callbacks are asynchronous: a new invalid prediction
                # cannot prove that the preceding control sample ignored it.
                # Observe at most one existing 20 Hz cycle for the response.
                if self.prediction_response_since is None:
                    self.prediction_response_since = now
                elapsed = now-self.prediction_response_since
                if 0 <= elapsed <= .05:
                    reason = harness.safety_reason(
                        health, now, planning_required=False,
                        maximum_observer_input_age=self.maximum_observer_input_age)
                    health['p53_decision'].update(
                        fatal=reason, inhibit='AWAIT_NEXT_CONTROL_SAMPLE',
                        prediction_response_seconds=elapsed,
                        prediction_response_limit=.05)
            return reason
        if control.get('status') != 'MINCO_FOLLOW' or prediction.get('valid'):
            self.prediction_response_since = None
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


def validate_p7_geometry(flight):
    """Reject inconsistent P7 source geometry before creating an experiment directory."""
    tracker = flight['trajectory_tracker_node']['ros__parameters']
    if not tracker.get('p7_spatial_enabled', False):
        return
    evaluator = flight['intercept_evaluator_node']['ros__parameters']
    radius = tracker['p7_capture_radius']
    if not radius == evaluator['evaluation_capture_radius'] == evaluator['planned_capture_radius']:
        raise ValueError('P7_CAPTURE_RADIUS_CONFIG_MISMATCH')
    camera = flight['rgbd_target_localizer']['ros__parameters']
    if (not camera.get('geometry_diagnostics_enabled', False)
            or any(flight[node]['ros__parameters']['camera_pitch_down']
                   != tracker['p7_camera_pitch_down'] for node in (
                       'rgbd_target_localizer', 'target_bearing_node', 'front_tof_monitor'))
            or tuple(camera['camera_translation_'+axis] for axis in 'xyz') != (.18, 0., .39)
            or camera['target_radius'] != .25 or camera['target_reference_z_offset'] != .42):
        raise ValueError('P7_CAMERA_CONFIG_MISMATCH')
    planner = flight['p4_follow_planner_node']['ros__parameters']
    if (not tracker.get('p7_follow_geometry_enabled')
            or not planner.get('p7_follow_geometry_enabled')
            or planner.get('p7_camera_pitch_down') != tracker['p7_camera_pitch_down']):
        raise ValueError('P7_FOLLOW_CAMERA_CONFIG_MISMATCH')
    if tracker['flight_altitude'] != flight['p4_follow_planner_node']['ros__parameters'][
            'flight_altitude']:
        raise ValueError('P7_FOLLOW_HEIGHT_CONFIG_MISMATCH')


def prepare(output, scene, motion=None, capture=False):
    """Copy scene geometry and current P7 constant speed into an exclusive run."""
    if scene not in ('f2', 'f3'):
        raise ValueError('P7_F2_OR_F3_REQUIRED')
    flight = yaml.safe_load(CONFIG.read_text())
    constant_speed = flight['moving_target']['ros__parameters']['horizontal_speed']
    sources = [CONFIG]
    if scene == 'f3':
        source = R11/f'{scene}_h16_01/config/flight.yaml'
        original = yaml.safe_load(source.read_text())
        flight['moving_target'] = original['moving_target']
        sources.append(source)
        # F3 retains its explicit historical geometry, never invented defaults.
        flight['moving_target']['ros__parameters']['horizontal_speed'] = constant_speed
    # F2 uses the current P7 source. Its target block was verified identical to
    # the recorded manual F2; startup must not depend on a deleted old experiment.
    if motion is not None:
        mode = normalize_motion_mode(motion)
        flight['moving_target']['ros__parameters']['trajectory_type'] = mode
    if capture:
        flight['trajectory_tracker_node']['ros__parameters']['p7_capture_enabled'] = True
        flight['intercept_evaluator_node']['ros__parameters']['maximum_duration'] = 60.
    validate_p7_geometry(flight)
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
        motion_mode=flight['moving_target']['ros__parameters']['trajectory_type'],
        capture_enabled=capture,
        source_fingerprint=source_fingerprint(),
        config_sources={str(p): harness.sha(p) for p in sources},
        observation_seconds=25. if scene == 'f2' else 60.), indent=2)+'\n')


def guidance_health(health, now, capture=False):
    """Keep physical stops; P7 guidance has no MINCO prediction/owner requirement."""
    observed = dict(health, control=dict(health.get('control', {}), status='FOLLOW', locked=True))
    return harness.safety_reason(observed, now, planning_required=False,
                                 maximum_observer_input_age=.3, capture_height=capture)


def capture_health(health, now, result, seen):
    """A terminal evaluator event supersedes live-flight freshness only after capture."""
    if result:
        if not result['success']:
            return seen, 'CAPTURE_'+result['reason'], True
        seen = now if seen is None else seen
        return seen, 'P7_CAPTURE_COMPLETE' if now-seen >= 5. else None, True
    return seen, guidance_health(health, now, capture=True), False


def clean_shutdown(cleanup):
    """Both layers must prove no errors or task-owned survivors."""
    late = cleanup['late_components']
    return not (cleanup['errors'] or cleanup['survivors'] or late['errors'] or late['survivors'])


def manual(output, motion=None, capture=False):
    """A bounded, isolated native X/Y console using the same P7 configs and sole Tracker."""
    if not output.is_relative_to(WORKSPACE/'data/experiments'):
        raise ValueError('OUTPUT_MUST_BE_EXPERIMENT_DIRECTORY')
    check = preflight()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port:
        port.bind(('127.0.0.1', 14550))
    prepare(output, 'f2', motion, capture)
    started = time.time()
    env = dict(os.environ, UAV_USV_WS=str(WORKSPACE),
               UAV_USV_RESEARCH_DIRECTORY=str(output),
               UAV_USV_EXPERIMENT_LAUNCH='p4_follow_research.launch.py',
               UAV_USV_EXPERIMENT_CONFIG_FILE=str(output/'config/flight.yaml'),
               UAV_USV_RESEARCH_CONFIG_FILE=str(output/'config/research.yaml'),
               UAV_USV_MINCO_EXECUTE='true', UAV_USV_P4_SHADOW='true',
               UAV_USV_P6_PREDICTOR='bctra', UAV_USV_P7_MODE='true',
               UAV_USV_P7_CAPTURE='true' if capture else 'false',
               UAV_USV_HEADLESS_GCS='true', UAV_USV_ENABLE_SHADOW_PERCEPTION='false',
               ROS_DOMAIN_ID=str(check['domain']), GZ_PARTITION='p7_'+str(int(started)),
               ROS_LOCALHOST_ONLY='0',
               ROS_LOG_DIR=str(output/'ros_logs'),
               UAV_USV_TERMINAL_LOG_ROOT=str(output),
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    grant = output/'sitl_execution_grant.json'
    grant.write_text(json.dumps(dict(
        pid=os.getpid(), process_start=identity(os.getpid())['start'], issued=started,
        domain=check['domain'], partition=env['GZ_PARTITION'], preflight_clear=True,
        authorization='User explicitly authorized P7 isolated SITL X/Y control'), indent=2)+'\n')
    grant.chmod(0o600)
    env['UAV_USV_SITL_GRANT'] = str(grant)
    (output/'environment.json').write_text(json.dumps(
        {k: v for k, v in env.items() if k.startswith(('UAV_USV_', 'ROS_', 'GZ_'))},
        indent=2))
    from experiment_source_snapshot import freeze_sources
    freeze_sources(output, WORKSPACE)
    processes, launcher = [], None
    try:
        for arguments, name in (([sys.executable, str(WORKSPACE/'scripts/p45_gcs_heartbeat.py')],
                                 'gcs_heartbeat.txt'),
                                ([sys.executable, str(WORKSPACE/'scripts/p4_shadow_monitor.py'),
                                  str(output/'offline_evidence.jsonl')], 'monitor.txt')):
            processes.append(harness.shell_process(arguments, env, output/name))
        action = 'radius capture and pause' if capture else 'safe guidance'
        print(f'P7 SITL: 1-5 select motion before X; X FOLLOW; Y {action}; Q abort. '
              'Session bound: 280 seconds.', flush=True)
        launcher = subprocess.Popen([str(WORKSPACE/'scripts/uav_lab.sh'), '--no-build'],
                                    cwd=WORKSPACE, env=env)
        launcher.wait(timeout=280.)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        print('P7 manual session ended; stopping this session only.', flush=True)
    finally:
        if launcher is not None and launcher.poll() is None:
            launcher.send_signal(signal.SIGINT)
            try:
                launcher.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                launcher.terminate()
                launcher.wait(timeout=5.)
        cleanup = harness.stop_owned(processes)
        cleanup['late_components'] = harness.stop_late_components(output)
        (output/'manual_cleanup.json').write_text(json.dumps(cleanup, indent=2)+'\n')
    return 0 if clean_shutdown(cleanup) else 1


def run(output, scene, prior_f2=None, guidance=None, motion=None, capture=False):
    """Retain original admission warmup and strict health/lease checks, without P6 gates."""
    if not output.is_relative_to(WORKSPACE/'data/experiments'):
        raise ValueError('OUTPUT_MUST_BE_EXPERIMENT_DIRECTORY')
    if scene == 'f3':
        if prior_f2 is None:
            raise ValueError('CURRENT_HEALTHY_F2_REQUIRED')
        require_f2(prior_f2)
    preflight()
    prepare(output, scene, motion, capture)
    condition = json.loads((output/'condition.json').read_text())
    if guidance:
        condition.update(guidance_experiment=guidance,
                         guidance_observation_seconds=(60. if capture else
                                                       25. if guidance == 'c' else 20.))
        (output/'condition.json').write_text(json.dumps(condition, indent=2)+'\n')
    env = dict(os.environ, UAV_USV_HEADLESS_GCS='true', UAV_USV_MINCO_EXECUTE='true',
               UAV_USV_P7_MODE='true', UAV_USV_P7_CAPTURE='true' if capture else 'false',
               UAV_USV_REQUIRE_PRE_X_HEALTH='true', UAV_USV_P6_PREDICTOR='bctra')
    processes, health = [], {}
    started = time.time()
    first_minco = follow_start = stable_start = planner_started = None
    health_process = planner = None
    health_started = None
    y_sent = guidance_started = target_lost_since = None
    capture_seen = None
    tracker_config = yaml.safe_load((output/'config/flight.yaml').read_text())[
        'trajectory_tracker_node']['ros__parameters']
    check = P7FollowHealthGuard(initial_acquisition=True,
                                flight_altitude=tracker_config['flight_altitude'])
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
            if capture and y_sent is not None:
                result_path = output/'terminal_result.json'
                result = json.loads(result_path.read_text()) if result_path.exists() else None
                capture_seen, problem, terminal = capture_health(health, now, result, capture_seen)
                if terminal:
                    if problem:
                        reason = problem
                        break
                    time.sleep(.05)
                    continue
            if health_process is not None:
                problem = monitor_check(health_process.poll(), health, now, health_started)
                if problem:
                    reason = problem
                    break
            if not health:
                time.sleep(.05)
                continue
            status = health.get('control', {}).get('status')
            if y_sent is not None:
                problem = guidance_health(health, now, capture)
                if problem:
                    reason = problem
                    break
                if status == 'P7_GUIDANCE' and guidance_started is None:
                    guidance_started = now
                if guidance_started is None and now-y_sent > 5.:
                    reason = 'Y_NOT_ACCEPTED_WITHIN_5S'
                    break
                visible = health.get('control', {}).get('visible', False)
                target_lost_since = None if visible else (target_lost_since or now)
                if target_lost_since is not None and now-target_lost_since > 2.:
                    reason = 'P7_TARGET_LOST_FOR_2S'
                    break
                if guidance_started is not None and now-guidance_started >= condition[
                        'guidance_observation_seconds']:
                    reason = 'P7_GUIDANCE_OBSERVATION_COMPLETE'
                    break
                time.sleep(.05)
                continue
            if follow_start is None and status in ('FOLLOW', 'MINCO_FOLLOW'):
                follow_start = now
            if follow_start is not None:
                problem = check(health, now)
                if problem:
                    reason = problem
                    break
                navigation, tracking = health.get('navigation', {}), health.get('tracking', {})
                p, q = navigation.get('p', ()), tracking.get('p', ())
                distance = (sum((p[i]-q[i])**2 for i in (0, 1))**.5
                            if len(p) == len(q) == 3 else 0.)
                ready_for_y = (guidance == 'c' and health['control'].get('locked')
                               and 8. <= distance <= 20.)
                ready_for_y |= (guidance == 'd' and first_minco is not None
                                and now-first_minco >= 5. and health['control'].get('locked'))
                if ready_for_y:
                    y, owned = harness.shell_process([
                        'timeout', '8s', 'ros2', 'topic', 'pub', '--times', '3', '--rate', '10',
                        '--wait-matching-subscriptions', '2', '--keep-alive', '.5',
                        '/simulation/impact/command', 'std_msgs/msg/String', '{data: Y}'],
                        env, output/'Y_command.txt')
                    processes.append((y, owned))
                    y_sent = now
                    (output/'Y_event.json').write_text(json.dumps(dict(
                        sent=now, horizontal_distance=distance,
                        first_minco=first_minco, experiment=guidance), indent=2)+'\n')
                    continue
                if planner is None:
                    stable_start = harness.settled_since(
                        stable_start, health.get('follow_settling', {}), now)
                    if now-follow_start >= 60.:
                        reason = 'ORIGINAL_FOLLOW_NOT_SETTLED_WITHIN_60S'
                        break
                    if guidance != 'c' and stable_start is not None and now-stable_start >= 5.:
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
                if (not guidance and first_minco is not None
                        and now-first_minco >= condition['observation_seconds']):
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
    if guidance and (output/'offline_evidence.jsonl').exists():
        if capture:
            from p7_capture_analysis import capture_metrics
            summary = capture_metrics(output, reason)
        else:
            from p7_follow_analysis import guidance_metrics
            summary = guidance_metrics(output, reason)
    elif (output/'offline_evidence.jsonl').exists():
        summary = harness.assess(output, scene, reason, first_minco, ended,
                                 task_root=output, extra_audit=audit)
    else:
        summary = dict(completed=False, healthy=False, reason=reason, f3_eligible=False)
    summary.update(source_fingerprint=condition['source_fingerprint'], cleanup=cleanup)
    if not clean_shutdown(cleanup):
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
    parser.add_argument('--scene', choices=('f2', 'f3'), default='f2')
    parser.add_argument('--prior-f2', type=Path)
    parser.add_argument('--guidance', choices=('c', 'd'))
    parser.add_argument('--manual', action='store_true')
    parser.add_argument('--capture', action='store_true',
                        help='Use the configured 3-D capture radius and evaluator world pause')
    parser.add_argument('--motion', choices=tuple(MOTION_MODES.values()))
    args = parser.parse_args()
    if args.manual and (args.guidance or args.prior_f2):
        parser.error('--manual cannot be combined with automated flight options')
    if args.capture and not (args.manual or args.guidance):
        parser.error('--capture requires --guidance or --manual')
    if args.manual:
        raise SystemExit(manual(args.output.resolve(), args.motion, args.capture))
    raise SystemExit(run(args.output.resolve(), args.scene, args.prior_f2,
                         args.guidance, args.motion, args.capture))
