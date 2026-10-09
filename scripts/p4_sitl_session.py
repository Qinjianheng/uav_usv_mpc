#!/usr/bin/env python3
"""Bound an explicitly authorized isolated SITL X-only run; only terminate owned identities."""
import argparse
import json
import os
from pathlib import Path
import signal
import shlex
import socket
import subprocess
import time

from experiment_source_snapshot import freeze_sources


WORKSPACE = Path(__file__).resolve().parents[1]


def identity(pid):
    """Linux boot identity and process group protect against recycled PID signals."""
    raw = Path(f'/proc/{pid}/stat').read_text()
    fields = raw[raw.rfind(')')+2:].split()
    return dict(pid=pid, pgid=int(fields[2]), start=fields[19])


def matches(entry):
    """Signal only the exact task-owned group leader, never arbitrary name matches."""
    try:
        return identity(entry['pid']) == entry and entry['pid'] == entry['pgid']
    except (OSError, ValueError):
        return False


def is_simulation_process(comm, executable, argv):
    """Recognize native gz's Ruby launcher by tokens, without substring-matching analysis."""
    if comm in ('px4', 'MicroXRCEAgent') or 'gz-sim' in executable:
        return True
    return any(Path(token).name in ('gz', 'ign') and argv[i+1:i+2] in (['sim'], ['gazebo'])
               for i, token in enumerate(argv))


def preflight():
    """Reject external simulations/agents/serial devices before launching."""
    processes = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            comm = (path/'comm').read_text().strip()
            executable = os.readlink(path/'exe')
            argv = (path/'cmdline').read_bytes().decode(errors='replace').split('\0')
            if is_simulation_process(comm, executable, argv):
                processes.append(dict(pid=int(path.name), comm=comm, executable=executable))
        except OSError:
            pass
    serial = [str(p) for glob in ('ttyACM*', 'ttyUSB*') for p in Path('/dev').glob(glob)]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('0.0.0.0', 8888))
    except OSError as error:
        raise RuntimeError('DDS_PORT_IN_USE') from error
    finally:
        sock.close()
    if processes or serial:
        raise RuntimeError('EXTERNAL_SIMULATION_OR_HARDWARE: '+json.dumps((processes, serial)))
    occupied_domains = set()
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            for value in (path/'environ').read_bytes().split(b'\0'):
                if value.startswith(b'ROS_DOMAIN_ID='):
                    occupied_domains.add(int(value.split(b'=', 1)[1]))
        except (OSError, ValueError):
            pass
    domain = next((n for n in range(43, 50) if n not in occupied_domains), None)
    if domain is None:
        raise RuntimeError('NO_UNUSED_EXPERIMENT_DDS_DOMAIN')
    return dict(processes=processes, hardware_serial=serial, udp8888_free=True, domain=domain)


def run(root, shadow, follow_seconds, watchdog, kill_planner_after=None):
    """Reuse repository launcher only after exclusion gates; preserve every failure/run log."""
    root = root.resolve()
    if not root.is_relative_to(WORKSPACE/'data/experiments'):
        raise ValueError('OUTPUT_MUST_BE_EXPERIMENT_DIRECTORY')
    start = time.time()
    shim = root/'terminal_shim'
    shim.mkdir(exist_ok=False)
    wrapper = shim/'gnome-terminal'
    wrapper.write_bytes((WORKSPACE/'scripts/p4_terminal_wrapper.py').read_bytes())
    wrapper.chmod(0o755)
    check = preflight()
    (root/'preflight.json').write_text(json.dumps(check, indent=2))
    env = dict(os.environ, UAV_USV_WS=str(WORKSPACE), UAV_USV_RESEARCH_DIRECTORY=str(root),
               UAV_USV_EXPERIMENT_LAUNCH='p4_follow_research.launch.py',
               UAV_USV_EXPERIMENT_CONFIG_FILE=str(root/'config/flight.yaml'),
               UAV_USV_RESEARCH_CONFIG_FILE=str(root/'config/research.yaml'),
               UAV_USV_P4_SHADOW='true' if shadow else 'false',
               UAV_USV_ENABLE_SHADOW_PERCEPTION='false', ROS_LOG_DIR=str(root/'ros_logs'),
               CAMERA_STARTUP_TIMEOUT='90', FLIGHT_READY_TIMEOUT='60',
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', ROS_LOCALHOST_ONLY='0',
               ROS_DOMAIN_ID=str(check['domain']), GZ_PARTITION='p4_'+str(int(start)),
               UAV_USV_TERMINAL_LOG_ROOT=str(root), PATH=str(shim)+':'+os.environ['PATH'])
    (root/'environment.json').write_text(json.dumps({k: v for k, v in env.items() if k.startswith(
        ('UAV_USV_', 'ROS_', 'GZ_', 'OPENBLAS_', 'OMP_'))}, indent=2))
    initial_ulogs = set(Path('/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log')
                        .rglob('*.ulg'))
    freeze_sources(root, WORKSPACE)
    console = (root/'launcher.txt').open('x')
    command = [str(WORKSPACE/'scripts/uav_lab.sh'), '--no-build']
    launcher = subprocess.Popen(command, cwd=WORKSPACE, env=env, stdin=subprocess.PIPE,
                                stdout=console, stderr=console, start_new_session=True)
    identities = {'launcher': identity(launcher.pid)}
    owned_session, x_time, reason = None, None, 'WATCHDOG'
    planner_killed = False
    previous_load, load_time, loads = {}, time.monotonic(), []
    monitor_log = (root/'monitor.txt').open('x')
    overlay = shlex.quote(str(WORKSPACE/'install/setup.bash'))
    monitor_script = shlex.quote(str(WORKSPACE/'scripts/p4_shadow_monitor.py'))
    monitor_output = shlex.quote(str(root/'offline_evidence.jsonl'))
    shell = ('set -eo pipefail; source /opt/ros/humble/setup.bash; source '
             + overlay+'; exec python3 '+monitor_script+' '+monitor_output)
    monitor = subprocess.Popen(
        ['bash', '-c', shell], env=env, stdout=monitor_log,
        stderr=monitor_log, start_new_session=True)
    identities['monitor'] = identity(monitor.pid)
    try:
        while time.time()-start < watchdog and not (root/'STOP').exists():
            if owned_session is None:
                found = [p for p in Path('/tmp').glob('uav_usv_lab.*')
                         if p.stat().st_mtime >= start]
                if len(found) > 1:
                    reason = 'AMBIGUOUS_SESSION'
                    break
                if found:
                    owned_session = found[0]
            if owned_session:
                for file in owned_session.glob('*.pid'):
                    try:
                        entry = identity(int(file.read_text()))
                        if entry['pid'] != entry['pgid']:
                            raise RuntimeError('SHARED_PROCESS_GROUP')
                        identities.setdefault(file.stem, entry)
                    except (OSError, ValueError):
                        pass
            if time.monotonic()-load_time >= 1.:
                now_mono = time.monotonic()
                load = {}
                groups = {v['pgid'] for v in identities.values()}
                for process in Path('/proc').iterdir():
                    if not process.name.isdigit():
                        continue
                    try:
                        stat_raw = (process/'stat').read_text()
                        fields = stat_raw[stat_raw.rfind(')')+2:].split()
                        if int(fields[2]) not in groups:
                            continue
                        ticks = int(fields[11])+int(fields[12])
                        load[(process.name, fields[19])] = (ticks, int(fields[21]))
                    except (OSError, ValueError):
                        pass
                ticks = sum(v[0]-previous_load[k][0] for k, v in load.items()
                            if k in previous_load)
                loads.append(dict(
                    stamp=time.time(),
                    cpu_percent=100*ticks/os.sysconf('SC_CLK_TCK')/(now_mono-load_time),
                    rss_mib=sum(v[1] for v in load.values())*os.sysconf('SC_PAGE_SIZE')/2**20))
                previous_load, load_time = load, now_mono
            record = dict(start=start, command=command, world='default',
                          session=str(owned_session), components=identities,
                          domain=env.get('ROS_DOMAIN_ID', '0'), x_time=x_time)
            (root/'owned_session.json').write_text(json.dumps(record, indent=2))
            graph_path = root/'live_graph.json'
            graph = json.loads(graph_path.read_text()) if graph_path.exists() else {}
            topics = ('/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode',
                      '/fmu/in/vehicle_command')
            safe_graph = all(len(graph.get(t, [])) == 1 and graph[t][0]['node'] ==
                             'trajectory_tracker_node' for t in topics)
            if any(len(graph.get(t, [])) > 1 for t in topics):
                reason = 'CONTROL_AUTHORITY_CONFLICT'
                break
            ready = 'Two-stage control is ready' in (root/'launcher.txt').read_text()
            if ready and x_time is None and safe_graph:
                with (root/'X_command.txt').open('x') as log:
                    x_shell = ('set -eo pipefail; source /opt/ros/humble/setup.bash; source '
                               + overlay+'; timeout 8s ros2 topic pub '
                               '--times 3 --rate 10 --wait-matching-subscriptions 2 '
                               '--keep-alive 0.5 /simulation/impact/command '
                               'std_msgs/msg/String "{data: X}"')
                    x = subprocess.run(['bash', '-c', x_shell], env=env,
                                       stdout=log, stderr=log, timeout=12)
                if x.returncode:
                    reason = 'X_DELIVERY_FAILED'
                    break
                x_time = time.time()
            if (shadow and x_time and kill_planner_after is not None and not planner_killed
                    and time.time()-x_time >= kill_planner_after and 'experiment' in identities):
                for process in Path('/proc').iterdir():
                    if not process.name.isdigit():
                        continue
                    try:
                        entry = identity(int(process.name))
                        argv = (process/'cmdline').read_bytes().split(b'\0')
                        if (entry['pgid'] == identities['experiment']['pgid']
                                and any(Path(v.decode()).name == 'p4_follow_planner_node'
                                        for v in argv if v)
                                and identity(entry['pid']) == entry):
                            os.kill(entry['pid'], signal.SIGINT)
                            planner_killed = True
                            (root/'planner_crash_injection.json').write_text(json.dumps(
                                dict(stamp=time.time(), identity=entry, signal='SIGINT',
                                     task_owned_child=True), indent=2))
                    except (OSError, ValueError):
                        pass
            if x_time and time.time()-x_time >= follow_seconds:
                reason = 'FOLLOW_WINDOW_COMPLETE'
                break
            if launcher.poll() is not None and not ready:
                reason = 'LAUNCHER_FAILED'
                break
            if monitor.poll() is not None:
                reason = 'MONITOR_FAILED'
                break
            time.sleep(.5)
    finally:
        for sig, delay in ((signal.SIGINT, 6), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
            for entry in identities.values():
                if matches(entry):
                    os.killpg(entry['pgid'], sig)
            time.sleep(delay)
        launcher.wait(timeout=5)
        monitor.wait(timeout=5)
        console.close()
        monitor_log.close()
        ended = dict(start=start, ended=time.time(), reason=reason, x_time=x_time,
                     identities=identities,
                     monitor_returncode=monitor.returncode,
                     launcher_returncode=launcher.returncode,
                     still_alive=[e for e in identities.values() if matches(e)])
        (root/'watchdog_end.json').write_text(json.dumps(ended, indent=2))
        (root/'owned_load.json').write_text(json.dumps(loads, indent=2))
        logs = root/'ulogs'
        logs.mkdir(exist_ok=False)
        for file in set(Path('/home/qin/Projects/PX4-Autopilot/build/px4_sitl_default/rootfs/log')
                        .rglob('*.ulg'))-initial_ulogs:
            if file.stat().st_mtime >= start:
                (logs/file.name).write_bytes(file.read_bytes())
        print(json.dumps(ended), flush=True)
    return 0 if reason == 'FOLLOW_WINDOW_COMPLETE' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--shadow', action='store_true')
    parser.add_argument('--follow-seconds', type=float, default=90.)
    parser.add_argument('--watchdog', type=float, default=200.)
    parser.add_argument('--kill-planner-after', type=float)
    args = parser.parse_args()
    raise SystemExit(run(args.root, args.shadow, args.follow_seconds, args.watchdog,
                         args.kill_planner_after))
