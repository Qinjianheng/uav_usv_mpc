#!/usr/bin/env python3
"""Own one disarmed PX4/Gazebo interface check with a 120-second wall watchdog."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

from p4_sitl_session import WORKSPACE, identity, preflight
from p5_minco_sitl import shell_process, stop_late_components, stop_owned


def run(output):
    """A diagnostic graph has no mission manager, Tracker, arming or takeoff command."""
    output = output.resolve()
    if not output.is_relative_to(WORKSPACE/'data/experiments'):
        raise ValueError('EXPERIMENT_OUTPUT_REQUIRED')
    output.mkdir(parents=True, exist_ok=False)
    check = preflight()
    (output/'preflight.json').write_text(json.dumps(check, indent=2)+'\n')
    shim = output/'terminal_shim'
    shim.mkdir()
    wrapper = shim/'gnome-terminal'
    wrapper.write_text('''#!/usr/bin/env python3
import json, os, pathlib, subprocess, sys
root = pathlib.Path(os.environ['UAV_USV_RESEARCH_DIRECTORY'])
title = next(a.split('=', 1)[1] for a in sys.argv if a.startswith('--title='))
with (root/(title.replace(' ', '_')+'.txt')).open('x') as log:
    p = subprocess.Popen(sys.argv[sys.argv.index('--')+1:], stdout=log,
                         stderr=log, start_new_session=True)
raw = pathlib.Path('/proc/'+str(p.pid)+'/stat').read_text()
fields = raw[raw.rfind(')')+2:].split()
entry = dict(pid=p.pid, pgid=int(fields[2]), start=fields[19])
with (root/'component_identities.jsonl').open('a') as f:
    f.write(json.dumps(entry)+'\\n')
''')
    wrapper.chmod(0o755)
    started = time.time()
    env = dict(os.environ, UAV_USV_WS=str(WORKSPACE), UAV_USV_RESEARCH_DIRECTORY=str(output),
               UAV_USV_EXPERIMENT_LAUNCH='p76_px4_probe.launch.py',
               UAV_USV_EXPERIMENT_CONFIG_FILE=str(
                   WORKSPACE/'src/uav_usv_bringup/config/p76_yan_paper.yaml'),
               UAV_USV_START_QGC='false', UAV_USV_HEADLESS_GCS='true',
               UAV_USV_GZ_SERVER_ONLY='1', ROS_DOMAIN_ID=str(check['domain']),
               GZ_PARTITION='p76_probe_'+str(os.getpid()), ROS_LOCALHOST_ONLY='0',
               ROS_LOG_DIR=str(output/'ros_logs'), OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               CAMERA_STARTUP_TIMEOUT='60', FLIGHT_READY_TIMEOUT='40',
               PATH=str(shim)+':'+os.environ['PATH'])
    grant = output/'sitl_execution_grant.json'
    grant.write_text(json.dumps(dict(
        pid=os.getpid(), process_start=identity(os.getpid())['start'], issued=started,
        domain=check['domain'], partition=env['GZ_PARTITION'], preflight_clear=True,
        authorization='User requested PX4 completion; disarmed interface check only')))
    grant.chmod(0o600)
    env['UAV_USV_SITL_GRANT'] = str(grant)
    (output/'environment.json').write_text(json.dumps(
        {k: v for k, v in env.items() if k.startswith(('UAV_USV_', 'ROS_', 'GZ_'))}, indent=2))
    processes = []
    try:
        gcs_command = [sys.executable, str(WORKSPACE/'scripts/p45_gcs_heartbeat.py')]
        processes.append(shell_process(gcs_command, env, output/'gcs.txt'))
        processes.append(shell_process([str(WORKSPACE/'scripts/start_px4_ros2.sh'), '--no-build'],
                                       env, output/'launcher.txt'))
        while time.time()-started < 120:
            if (output/'result.json').exists():
                return json.loads((output/'result.json').read_text())['success']
            if processes[-1][0].poll() is not None:
                break
            time.sleep(.2)
        (output/'harness_failure.json').write_text(json.dumps(dict(
            reason='PROBE_LAUNCH_OR_WATCHDOG', elapsed=time.time()-started), indent=2)+'\n')
        return False
    finally:
        cleanup = stop_owned(processes)
        cleanup['late_components'] = stop_late_components(output)
        # The terminal replacements are task-owned process group leaders too.
        path = output/'component_identities.jsonl'
        if path.exists():
            import signal
            from p4_sitl_session import matches
            for row in path.read_text().splitlines():
                entry = json.loads(row)
                if matches(entry):
                    os.killpg(entry['pgid'], signal.SIGTERM)
        (output/'cleanup.json').write_text(json.dumps(cleanup, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    return 0 if run(args.output) else 1


if __name__ == '__main__':
    raise SystemExit(main())
