#!/usr/bin/env python3
"""One scoped F2, then at most one qualified F3; use real nominal ACK/lease execution."""
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

import yaml

from p43_sitl_matrix import prepare as legacy_prepare
from p4_sitl_session import WORKSPACE, identity, matches, preflight
from p5_release import CANDIDATE, qualify, sha, verify_manifest
from uav_control.guidance.follow_sitl_gate import require_isolated_sitl

# The human continued after the first tooling abort; keep that consumed attempt intact.
ROOT = WORKSPACE/'data/experiments/20261012_p5_revalidation_02'
AUTHORIZATION = 'User continued and authorized F3 after healthy repaired F2; one each'


def prepare(root, condition, overlay_config=None):
    """Keep historical preparation, or copy a complete independent flight config."""
    if overlay_config is None:
        return legacy_prepare(root, condition)
    source = Path(overlay_config).resolve()
    flight = yaml.safe_load(source.read_text())
    name, scene, shadow, engine, acceleration, yaw, horizon, refinement = condition
    tracker = flight['trajectory_tracker_node']['ros__parameters']
    tracker.update(follow_minco_enabled=False, follow_minco_shadow_check=shadow,
                   follow_research_horizontal_acceleration=acceleration,
                   follow_research_yaw_rate=yaw)
    research = yaml.safe_load((WORKSPACE/'src/uav_usv_bringup/config/p41_follow_research.yaml')
                              .read_text())
    research['p4_follow_planner_node']['ros__parameters'].update(
        minco_engine=engine, maximum_horizontal_acceleration=acceleration,
        maximum_yaw_rate=yaw, short_horizon=horizon, p43_refinement=refinement)
    root.mkdir(exist_ok=False)
    (root/'.gitignore').write_text('*\n')
    (root/'COLCON_IGNORE').touch()
    (root/'config').mkdir()
    (root/'config/flight.yaml').write_text(yaml.safe_dump(flight, sort_keys=False))
    (root/'config/research.yaml').write_text(yaml.safe_dump(research, sort_keys=False))
    (root/'condition.json').write_text(json.dumps(dict(
        name=name, scene=scene, shadow=shadow, engine=engine,
        acceleration=acceleration, yaw=yaw, horizon=horizon, refinement=refinement,
        actual_controller='ORIGINAL_FOLLOW', minco_authorized=False,
        config_source=str(source), config_source_sha256=sha(source)), indent=2)+'\n')


def observation_window(scene, observation_seconds=None):
    """Default P5 windows stay 25/60s; explicit P6 observation is bounded to 60s."""
    duration = (25. if scene == 'f2' else 60.) if observation_seconds is None else float(
        observation_seconds)
    if not math.isfinite(duration) or not 0. < duration <= 60.:
        raise ValueError('BOUNDED_OBSERVATION_WINDOW_REQUIRED')
    return duration


def constraint_metadata(baseline_fingerprint, research, p6_enabled=False):
    """Bind actual P6 limits; preserve the historical field only for legacy runs."""
    params = research.get('p4_follow_planner_node', {}).get('ros__parameters', {})
    if not p6_enabled or 'p6_visibility_mode' not in params:
        return dict(constraint_fingerprint=baseline_fingerprint)
    from uav_control.guidance.follow_limits import constraint_snapshot
    from uav_control.guidance.p6_follow import model_from_parameters
    model = model_from_parameters(params)
    snapshot = constraint_snapshot(model)
    if not model.p6_policy.enabled and snapshot.fingerprint != baseline_fingerprint:
        raise ValueError('P6_NOMINAL_BASELINE_CONSTRAINT_MISMATCH')
    return dict(baseline_reference_constraint_fingerprint=baseline_fingerprint,
                constraint_fingerprint=snapshot.fingerprint,
                actual_constraint_snapshot=snapshot.values)


def safety_reason(health, now, planning_required=True, maximum_observer_input_age=.125):
    """Experiment bounds only; controller TTL, lease and whole-sphere admission stay intact."""
    if not 0 <= now-health.get('written', 0.) <= .25:
        return 'HEALTH_MONITOR_STALE'
    c, n, v, p, a = (health.get(k, {}) for k in (
        'control', 'navigation', 'vehicle', 'prediction', 'attitude'))
    if not n.get('valid') or c.get('status') in ('STATE_STALE', 'WAITING_FOR_STATE'):
        return 'NAVIGATION_INVALID'
    if v.get('nav_state') != 14 or v.get('armed') != 2:
        return 'OFFBOARD_OR_ARMING_LOST'
    if not c.get('locked') or c.get('status') not in ('FOLLOW', 'MINCO_FOLLOW'):
        return 'NO_LAWFUL_FOLLOW_PATH'
    inputs = (c, n, p, a) if planning_required else (c, n, a)
    # Separate <=250ms acquisition age and <=250ms snapshot delivery budgets.
    # Read-time receipt age can reach 500ms (vehicle_status: 1s); controller TTL is separate.
    receipt_epoch = health['written']
    if any(not 0 <= receipt_epoch-x.get('receipt', 0.) <= .25 for x in inputs):
        return 'HEALTH_INPUT_LOST'
    # PX4 Commander publishes vehicle_status at a 500ms steady-state cadence.
    if not 0 <= receipt_epoch-v.get('receipt', 0.) <= .75:
        return 'VEHICLE_STATUS_LOST'
    stamp = health.get('ros_now', float('inf'))
    if not 0 <= stamp-n.get('stamp', 0.) <= maximum_observer_input_age:
        return 'NAVIGATION_INPUT_STALE'
    position, velocity, q = n.get('p', ()), n.get('v', ()), a.get('q', ())
    if len(position) != 3 or len(velocity) != 3 or len(q) != 4 or not all(
            math.isfinite(x) for x in (*position, *velocity, *q)):
        return 'INVALID_PHYSICAL_STATE'
    if abs(position[2]+5.) > 1.:
        return 'HEIGHT_ERROR_OVER_1M'
    if sum(x*x for x in velocity) > 8.5**2:
        return 'VELOCITY_DIVERGENCE'
    w, x, y, z = q
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    if max(abs(roll), abs(pitch)) > .7:
        return 'ATTITUDE_DIVERGENCE'
    prediction_ages = [stamp-p.get(k, 0.) for k in ('stamp', 'observation')]
    if planning_required and (not p.get('valid')
                              or not all(0 <= age <= maximum_observer_input_age
                                         for age in prediction_ages)):
        return 'PREDICTION_INPUT_STALE'
    return None


def health_monitor_reason(returncode, written, now, started):
    """A dead/stalled observer must stop the harness before X or FOLLOW too."""
    if returncode is not None:
        return 'HEALTH_MONITOR_EXITED'
    if written is None:
        return 'HEALTH_MONITOR_NO_FIRST_SAMPLE' if now-started > 2. else None
    if not 0 <= now-written <= .25:
        return 'HEALTH_MONITOR_STALLED'
    return None


def read_health_snapshot(path):
    """Observe time after the atomic file read; a concurrent writer can run during it."""
    health = json.loads(path.read_text()) if path.exists() else {}
    return health, time.time()


def health_start_allowed(output, marker):
    """Start a scoped recorder after ROS launch begins, when explicitly requested."""
    if marker is None:
        return True
    log = output/'launcher.txt'
    return log.exists() and marker in log.read_text()


def settled_since(previous, settling, now):
    """Count five seconds only while the original FOLLOW is kinematically settled."""
    return (previous if previous is not None else now) if settling.get('settled') else None


def publication_stop_reason(events):
    """Three consecutive cycles beyond raw input TTL cannot support lawful replacement."""
    if len(events) >= 3 and all(r.get('whole_cycle_time', 0.) > .125 for r in events[-3:]):
        return 'SUSTAINED_PLANNER_OVERRUN'
    return None


def read_publications(root, offsets):
    """Consume complete flushed log lines only; preserve partial tails for the next poll."""
    events = []
    for path in (root/'shadow').glob('mpc_seed_shadow_*.jsonl'):
        with path.open() as stream:
            stream.seek(offsets.get(str(path), 0))
            while True:
                line = stream.readline()
                if not line.endswith('\n'):
                    break
                offsets[str(path)] = stream.tell()
                row = json.loads(line)
                if row.get('event') == 'follow_publication_final':
                    events.append(row)
    return events


def allow_f3(summary):
    """Predeclared sustained-control threshold, assessed only on completed F2 observation."""
    return bool(summary.get('completed') and summary.get('healthy')
                and summary.get('minco_fraction', 0) >= .8
                and summary.get('longest_minco_seconds', 0) >= 5.
                and summary.get('observation_seconds', 0) >= 24.
                and summary.get('diagnostic_coverage', 0) >= .95)


def shell_process(arguments, env, path):
    overlay = shlex.quote(str(WORKSPACE/'install/setup.bash'))
    command = ('set -eo pipefail; source /opt/ros/humble/setup.bash; source '+overlay
               + '; exec '+shlex.join(arguments))
    with path.open('x') as log:
        process = subprocess.Popen(['bash', '-c', command], cwd=WORKSPACE, env=env,
                                   stdout=log, stderr=log, start_new_session=True)
    return process, identity(process.pid)


def validate_candidate_config(config, limits):
    """Bind actual loaded parameters to the scoped candidate and original hard limits."""
    params = config['p4_follow_planner_node']['ros__parameters']
    required = dict(minco_engine='p47_follow_guided', short_horizon=1.6,
                    p47_terminal_mode='qva', p47_refinement=False, p47_scheduling='adaptive',
                    p44_ablation='D', p44_refinement='none', maximum_input_age=.125,
                    follow_guided_response_time=.5)
    required.update({k: v for k, v in limits.items() if k != 'maximum_input_age'})
    if any(params.get(k) != v for k, v in required.items()):
        raise ValueError('P5_CONFIG_SCOPE_MISMATCH')


def validate_release_sources(release):
    """An algorithm edit after the offline audit requires new qualification evidence."""
    checks = release.get('source_checks', [])
    if not checks or any(sha(WORKSPACE/v['path']) != v['current_sha256'] for v in checks):
        raise ValueError('AUDITED_SOURCE_CHANGED')


def claim(output, scene, qualification, task_root=None, authorization=None,
          f3_gate=None, release_gate=None):
    """Verify backing artifacts and consume one persistent attempt before creating a flight."""
    verify_manifest(qualification.parent)
    release = json.loads(qualification.read_text())
    qualified = (release_gate(release) if release_gate else qualify(release['evidence'])[
        'h16_scoped_gate_pass'])
    if release['candidate'] != CANDIDATE or not qualified:
        raise ValueError('H16_SCOPED_EVIDENCE_REQUIRED')
    validate_release_sources(release)
    verify_manifest(Path(release['formal_directory']))
    task_root = ROOT if task_root is None else task_root
    authorization = AUTHORIZATION if authorization is None else authorization
    output = output.resolve()
    if task_root.resolve() not in output.parents or output.exists():
        raise ValueError('NEW_P5_OUTPUT_REQUIRED')
    task_root.mkdir(exist_ok=True)
    if scene == 'f3':
        previous = json.loads((task_root/'f2_summary.json').read_text())
        if not (f3_gate or allow_f3)(previous):
            raise ValueError('F2_SUSTAINED_HEALTHY_EXECUTION_REQUIRED')
    check = preflight()
    with (task_root/f'attempt_{scene}.json').open('x') as stream:
        json.dump(dict(scene=scene, output=str(output), qualification=str(qualification),
                       qualification_sha256=sha(qualification), preflight=check,
                       maximum_attempts=1, authorization=authorization,
                       stamp=time.time()), stream, indent=2)
    return release


def ownership_metrics(diag):
    """Read the real analysis schema; max span is not the solver distribution schema."""
    from p45_execution_analysis import ownership
    owner = ownership(diag)
    duration = sum(owner['seconds'].values())
    unknown = owner['seconds'].get('UNKNOWN_GAP', 0.)
    continuous = owner['continuous_seconds'] or {}
    return dict(observation_seconds=duration, minco_fraction=owner['minco_fraction'],
                longest_minco_seconds=continuous.get('max', 0.),
                diagnostic_coverage=1.-unknown/duration if duration else 0., ownership=owner)


def execution_completed(reason, ended, authority, safety_stop):
    """Never relabel an inner safety abort as the outer planned completion."""
    return bool(reason == 'MINCO_OBSERVATION_COMPLETE' and ended.get('reason') == 'WATCHDOG'
                and ended.get('monitor_returncode') == 0 and not ended.get('still_alive', True)
                and authority.get('discovered_samples', 0) > 0
                and authority.get('all_discovered_single_tracker')
                and not authority.get('conflicts') and not safety_stop)


def stop_owned(processes, timeouts=(8., 5., 3.)):
    """Escalate exact task-owned identities, continue through failures, retain survivors."""
    errors = []
    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
        for process, entry in reversed(processes):
            if process.poll() is None and matches(entry):
                try:
                    os.killpg(entry['pgid'], sig)
                    process.wait(timeout=timeout)
                except (OSError, subprocess.TimeoutExpired) as error:
                    errors.append(dict(identity=entry, signal=int(sig), error=str(error)))
    survivors = [entry for process, entry in processes
                 if process.poll() is None and matches(entry)]
    return dict(errors=errors, survivors=survivors)


def read_environment_snapshot(output):
    """The historical writer is non-atomic; do not use a partially published scope."""
    environment = output/'environment.json'
    if not environment.exists():
        return None
    try:
        expected = json.loads(environment.read_text())
    except json.JSONDecodeError:
        return None
    keys = ('UAV_USV_RESEARCH_DIRECTORY', 'ROS_DOMAIN_ID', 'GZ_PARTITION')
    if not isinstance(expected, dict) or any(not expected.get(k) for k in keys):
        return None
    if expected[keys[0]] != str(output.resolve()) or not all(
            isinstance(expected[k], str) for k in keys):
        raise ValueError('EXPERIMENT_SCOPE_MISMATCH')
    return expected


def scoped_processes(output):
    """Find late children by the exact experiment scope, then capture PID/start identity."""
    expected = read_environment_snapshot(output)
    if expected is None:
        raise ValueError('LATE_CLEANUP_SCOPE_UNAVAILABLE')
    keys = ('UAV_USV_RESEARCH_DIRECTORY', 'ROS_DOMAIN_ID', 'GZ_PARTITION')
    entries = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit() or int(path.name) == os.getpid():
            continue
        try:
            before = identity(int(path.name))
            env = dict(v.decode(errors='replace').split('=', 1)
                       for v in (path/'environ').read_bytes().split(b'\0') if b'=' in v)
            if all(env.get(k) == expected[k] for k in keys) and identity(before['pid']) == before:
                entries.append(before)
        except (OSError, ValueError):
            pass
    return entries


def stop_late_components(output, timeouts=(2., 2., 2.)):
    """A component may start while the unchanged inner harness is already draining."""
    events, errors = [], []
    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
        entries = scoped_processes(output)
        for entry in entries:
            try:
                if identity(entry['pid']) == entry:
                    os.kill(entry['pid'], sig)
                    events.append(dict(identity=entry, signal=int(sig)))
            except ProcessLookupError:
                pass
            except OSError as error:
                errors.append(dict(identity=entry, signal=int(sig), error=str(error)))
        deadline = time.monotonic()+timeout
        while scoped_processes(output) and time.monotonic() < deadline:
            time.sleep(.02)
    return dict(events=events, errors=errors, survivors=scoped_processes(output))


def execution_chain_proven(report):
    """Require recorded real acceptance, activation and native PX4 command evidence."""
    lifecycle = report.get('lifecycle', {})
    commands = report.get('native_command', {}).get('period_seconds') or {}
    return bool(lifecycle.get('accepted_unique', 0) > 0
                and lifecycle.get('active_unique', 0) > 0 and commands.get('count', 0) > 0)


def assess(output, scene, reason, first_minco, ended, task_root=None,
           extra_audit=None, f3_gate=None, observation_seconds=None):
    """Actual diagnostics determine control ownership; planned curves cannot imply ACTIVE."""
    from p4_follow_analysis import read
    from p45_execution_analysis import analyze
    report = analyze(output)
    rows, malformed = read(output/'offline_evidence.jsonl')
    duration = observation_window(scene, observation_seconds)
    diag = [r for r in rows if r['kind'] == 'diagnostic' and first_minco is not None
            and first_minco <= r['receipt'] <= first_minco+duration]
    metrics = ownership_metrics(diag)
    complete = execution_completed(reason, ended, report['authority'],
                                   (output/'safety_stop.json').exists())
    summary = dict(completed=complete, healthy=complete and not malformed
                   and report['authority']['all_discovered_single_tracker']
                   and execution_chain_proven(report),
                   reason=reason, first_minco_receipt=first_minco,
                   **metrics, harness_end=ended,
                   analysis='execution_analysis.json; FOLLOW windows include original warmup',
                   f3_eligible=False)
    if extra_audit is not None:
        summary['p51_audit'] = extra_audit(output, rows, first_minco, scene)
        summary['healthy'] = summary['healthy'] and summary['p51_audit']['passed']
    summary['f3_eligible'] = scene == 'f2' and (f3_gate or allow_f3)(summary)
    (output/'p5_run_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    task_root = ROOT if task_root is None else task_root
    (task_root/f'{scene}_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary


def execution_condition(output, scene, research):
    """Preserve audited planner limits in the independent receiver's runtime config."""
    params = research['p4_follow_planner_node']['ros__parameters']
    return (output.name, scene, False, 'p47_follow_guided',
            params['maximum_horizontal_acceleration'], params['maximum_yaw_rate'],
            params['short_horizon'], 'none')


def run(output, qualification, scene, task_root=None, authorization=None,
        health_check=None, research_config=None, flight_parameters=None,
        extra_audit=None, f3_gate=None, release_gate=None, health_monitor_check=None,
        health_start_marker=None, overlay_config=None, observation_seconds=None):
    task_root = ROOT if task_root is None else task_root
    authorization = AUTHORIZATION if authorization is None else authorization
    health_check = safety_reason if health_check is None else health_check
    duration = observation_window(scene, observation_seconds)
    release = json.loads(qualification.read_text())
    formal = Path(release['formal_directory'])
    raw = json.loads(next((formal/f'{scene}_h1.6_main_C_single.jsonl').open()))
    research_config = (WORKSPACE/'src/uav_usv_bringup/config/p5_follow_research.yaml'
                       if research_config is None else research_config)
    research = yaml.safe_load(research_config.read_text())
    validate_candidate_config(research, raw['model_config']['mpc'])
    constraints = constraint_metadata(raw['constraint_fingerprint'], research,
                                      p6_enabled=overlay_config is not None)
    claim(output, scene, qualification, task_root, authorization, f3_gate, release_gate)
    condition_spec = execution_condition(output, scene, research)
    if overlay_config is None:
        prepare(output, condition_spec)
    else:
        prepare(output, condition_spec, overlay_config=overlay_config)
    if flight_parameters:
        flight_path = output/'config/flight.yaml'
        flight = yaml.safe_load(flight_path.read_text())
        flight['trajectory_tracker_node']['ros__parameters'].update(flight_parameters)
        flight_path.write_text(yaml.safe_dump(flight, sort_keys=False))
    research['p4_follow_planner_node']['ros__parameters']['follow_execution_enabled'] = True
    (output/'config/research.yaml').write_text(yaml.safe_dump(research, sort_keys=False))
    condition_path = output/'condition.json'
    condition = json.loads(condition_path.read_text())
    condition.update(CANDIDATE, actual_controller='NOMINAL_MINCO', fallback='P4.4/D',
                     commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=WORKSPACE,
                                                    text=True).strip(),
                     qualification_sha256=sha(qualification), execution_lease_seconds=.45,
                     maximum_input_age=.125, minco_authorized=False,
                     experiment_observer_input_age_seconds=getattr(
                         health_check, 'maximum_observer_input_age', .125),
                     authorization=authorization,
                     original_follow_warmup_seconds=5.,
                     original_follow_settling=dict(horizontal_error_max=1.,
                                                   relative_speed_max=.5,
                                                   height_error_max=.1,
                                                   timeout_seconds=60.),
                     observation_seconds=duration,
                     research_config_sha256=sha(output/'config/research.yaml'),
                     qualified_holding=False)
    # Legacy reference is explicit for P6; current limits/calibration bind the
    # actual local model, independently of the research YAML byte hash.
    condition.update(constraints)
    from dataclasses import asdict
    from uav_control.guidance.follow_time_contract import input_time_contract
    condition['input_time_contract'] = asdict(input_time_contract(
        research['p4_follow_planner_node']['ros__parameters'].get(
            'target_time_profile', 'baseline_125')))
    if flight_parameters:
        condition['original_follow_settling']['relative_speed_max'] = flight_parameters.get(
            'follow_admission_relative_enter', .5)
    if overlay_config is not None:
        loaded = yaml.safe_load((output/'config/flight.yaml').read_text())
        tracker = loaded['trajectory_tracker_node']['ros__parameters']
        condition['original_follow_settling'].update(
            horizontal_error_max=tracker.get('follow_admission_horizontal_enter', 1.),
            relative_speed_max=tracker.get('follow_admission_relative_enter', .5),
            height_error_max=tracker.get('follow_admission_height_enter', .1))
    condition_path.write_text(json.dumps(condition, indent=2)+'\n')
    env = dict(os.environ, UAV_USV_HEADLESS_GCS='true', UAV_USV_MINCO_EXECUTE='true')
    if health_start_marker is not None:
        env['UAV_USV_REQUIRE_PRE_X_HEALTH'] = 'true'
    command = [sys.executable, str(WORKSPACE/'scripts/p4_sitl_session.py'), str(output),
               '--follow-seconds', '180', '--watchdog', '280']
    processes = []
    started, follow_start, stable_start, first_minco = time.time(), None, None, None
    planner_started = None
    overrun_since = None
    publication_offsets, publication_events = {}, []
    reason = 'SESSION_ENDED_BEFORE_OBSERVATION'
    health, health_observed = {}, None
    try:
        with (output/'session_driver.txt').open('x') as log:
            session = subprocess.Popen(command, cwd=WORKSPACE, env=env, stdout=log,
                                       stderr=log, start_new_session=True)
        processes.append((session, identity(session.pid)))
        health_process = None
        health_started = None
        planner = None
        while session.poll() is None and time.time()-started < 295.:
            now = time.time()
            publication_events.extend(read_publications(output, publication_offsets))
            if publication_stop_reason(publication_events):
                reason = 'SUSTAINED_PLANNER_OVERRUN'
                break
            environment = read_environment_snapshot(output) if health_process is None else None
            if (health_process is None and environment is not None
                    and health_start_allowed(output, health_start_marker)):
                env.update(environment)
                health_process, owned = shell_process(
                    [sys.executable, str(WORKSPACE/'scripts/p5_live_health.py'), str(output)],
                    env, output/'p5_health_monitor.txt')
                processes.append((health_process, owned))
                health_started = now
            health_path = output/'p5_health.json'
            health, now = read_health_snapshot(health_path)
            health_observed = now
            if health_process is not None:
                problem = (health_monitor_check(health_process.poll(), health, now, health_started)
                           if health_monitor_check else health_monitor_reason(
                               health_process.poll(), health.get('written'), now, health_started))
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
                problem = health_check(health, now)
                if problem:
                    reason = problem
                    break
                overrun_since = ((overrun_since or now) if
                                 health['control']['compute'] > .05 else None)
                if overrun_since is not None and now-overrun_since > .5:
                    reason = 'SUSTAINED_CONTROL_OVERRUN'
                    break
                if planner is None:
                    stable_start = settled_since(stable_start, health.get('follow_settling', {}),
                                                 now)
                    if now-follow_start >= 60.:
                        reason = 'ORIGINAL_FOLLOW_NOT_SETTLED_WITHIN_60S'
                        break
                if planner is None and stable_start is not None and now-stable_start >= 5.:
                    grant = require_isolated_sitl(env)
                    condition.update(minco_authorized=True, actual_execution_grant=grant,
                                     planner_started_wall=now)
                    condition_path.write_text(json.dumps(condition, indent=2)+'\n')
                    planner, owned = shell_process(
                        ['ros2', 'run', 'uav_control', 'p4_follow_planner_node', '--ros-args',
                         '--params-file', str(output/'config/research.yaml'),
                         '-p', 'log_directory:='+str(output/'shadow')],
                        env, output/'p5_planner.txt')
                    processes.append((planner, owned))
                    planner_started = now
                    (output/'p5_owned_processes.json').write_text(json.dumps(
                        [entry for _, entry in processes], indent=2)+'\n')
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
            first_minco=first_minco, health_observed_wall=health_observed,
            health_snapshot=health), indent=2)+'\n')
        (output/'STOP').touch()
        # Let the existing harness drain evidence and stop its own exact groups first.
        if processes and processes[0][0].poll() is None:
            try:
                processes[0][0].wait(timeout=25.)
            except subprocess.TimeoutExpired:
                pass
        cleanup = stop_owned(processes)
        try:
            late_cleanup = stop_late_components(output)
            late_cleanup['scope_verified'] = True
        except (OSError, ValueError) as error:
            late_cleanup = dict(errors=[str(error)], survivors=[], scope_verified=False)
            reason = 'LATE_CLEANUP_UNVERIFIED'
        (output/'p5_late_cleanup.json').write_text(json.dumps(late_cleanup, indent=2)+'\n')
        cleanup['late_components'] = late_cleanup
        cleanup['survivors'].extend(late_cleanup['survivors'])
        (output/'p5_cleanup.json').write_text(json.dumps(cleanup, indent=2)+'\n')
        if cleanup['survivors']:
            reason = 'OWNED_PROCESSES_SURVIVED'
    ended_path = output/'watchdog_end.json'
    ended = json.loads(ended_path.read_text()) if ended_path.exists() else {}
    if not (output/'offline_evidence.jsonl').exists():
        summary = dict(completed=False, healthy=False, reason=reason, harness_end=ended,
                       first_minco_receipt=first_minco, f3_eligible=False)
        (output/'p5_run_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        (task_root/f'{scene}_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    else:
        summary = assess(output, scene, reason, first_minco, ended, task_root,
                         extra_audit, f3_gate, observation_seconds)
    print(json.dumps(summary, indent=2))
    return 0 if summary['completed'] and (extra_audit is None or summary['healthy']) else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--qualification', type=Path, required=True)
    parser.add_argument('--scene', choices=('f2', 'f3'), default='f2')
    args = parser.parse_args()
    raise SystemExit(run(args.output.resolve(), args.qualification.resolve(), args.scene))
