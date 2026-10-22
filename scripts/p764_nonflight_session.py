#!/usr/bin/env python3
"""Own exactly one bounded, isolated and unarmed synchronous P764 diagnostic session."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

from p4_sitl_session import WORKSPACE, identity, matches, preflight
from p5_minco_sitl import shell_process, stop_late_components, stop_owned
from p761_session import launcher_process


def write_terminal_wrapper(wrapper):
    """Inherit the launcher-held stdin; closing a temporary child pipe makes PX4 spin at EOF."""
    wrapper.write_text('''#!/usr/bin/env python3
import json, os, pathlib, subprocess, sys
root = pathlib.Path(os.environ['UAV_USV_RESEARCH_DIRECTORY'])
title = next(a.split('=', 1)[1] for a in sys.argv if a.startswith('--title='))
with (root/(title.replace(' ', '_')+'.txt')).open('x') as log:
    child = subprocess.Popen(sys.argv[sys.argv.index('--')+1:], stdout=log,
                             stderr=log, start_new_session=True)
raw = pathlib.Path('/proc/'+str(child.pid)+'/stat').read_text()
fields = raw[raw.rfind(')')+2:].split()
with (root/'component_identities.jsonl').open('a') as stream:
    stream.write(json.dumps(dict(pid=child.pid, pgid=int(fields[2]), start=fields[19]))+'\\n')
''')
    wrapper.chmod(0o755)


def run(output, *, launch_name='p764_perception_probe.launch.py',
        output_scope='p764_offline_validation',
        authorization='P764 one unarmed perception diagnostic only', snapshot=False,
        completion_files=()):
    output = output.resolve()
    if not output.is_relative_to(WORKSPACE/'data/experiments'/output_scope):
        raise ValueError('P764_NONFLIGHT_OUTPUT_SCOPE')
    output.mkdir(parents=True, exist_ok=False)  # An existing attempt cannot be silently retried.
    check = preflight()
    (output/'preflight.json').write_text(json.dumps(check, indent=2)+'\n')
    if snapshot:
        from experiment_source_snapshot import freeze_sources
        freeze_sources(output, WORKSPACE)
    shim = output/'terminal_shim'
    shim.mkdir()
    write_terminal_wrapper(shim/'gnome-terminal')
    started = time.time()
    env = dict(os.environ, UAV_USV_WS=str(WORKSPACE), UAV_USV_RESEARCH_DIRECTORY=str(output),
               UAV_USV_EXPERIMENT_LAUNCH=launch_name,
               UAV_USV_EXPERIMENT_CONFIG_FILE=str(
                   WORKSPACE/'src/uav_usv_bringup/config/p76_yan_paper.yaml'),
               UAV_USV_P76_CONTRACT=str(WORKSPACE/'src/uav_usv_bringup/config/p76_contract.yaml'),
               UAV_USV_START_QGC='false', UAV_USV_HEADLESS_GCS='true', UAV_USV_GZ_SERVER_ONLY='1',
               ROS_DOMAIN_ID=str(check['domain']), GZ_PARTITION='p764_unarmed_'+str(os.getpid()),
               ROS_LOCALHOST_ONLY='0', ROS_LOG_DIR=str(output/'ros_logs'),
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               CAMERA_STARTUP_TIMEOUT='60', FLIGHT_READY_TIMEOUT='60',
               UAV_USV_TERMINAL_LOG_ROOT=str(output), PATH=str(shim)+':'+os.environ['PATH'])
    grant = output/'sitl_execution_grant.json'
    grant.write_text(json.dumps(dict(pid=os.getpid(), process_start=identity(os.getpid())['start'],
                                     issued=started, domain=check['domain'],
                                     partition=env['GZ_PARTITION'], preflight_clear=True,
                                     authorization=authorization)))
    grant.chmod(0o600)
    env['UAV_USV_SITL_GRANT'] = str(grant)
    (output/'environment.json').write_text(json.dumps({k: v for k, v in env.items()
                                                      if k.startswith(
                                                          ('UAV_USV_', 'ROS_', 'GZ_'))},
                                                      indent=2)+'\n')
    processes = []
    try:
        heartbeat = [sys.executable, str(WORKSPACE/'scripts/p45_gcs_heartbeat.py')]
        processes.append(shell_process(heartbeat,
                                       env, output/'gcs.txt'))
        launch = [str(WORKSPACE/'scripts/start_px4_ros2.sh'), '--no-build']
        processes.append(launcher_process(launch,
                                          env, output/'launcher.txt'))
        while time.time()-started < 150.:
            if ((output/'result.json').exists()
                    and all((output/name).is_file() for name in completion_files)):
                return json.loads((output/'result.json').read_text())
            if processes[-1][0].poll() is not None:
                break
            time.sleep(.2)
        result = dict(success=False, flight_qualified=False, reason='NONFLIGHT_LAUNCH_OR_WATCHDOG')
        (output/'harness_failure.json').write_text(json.dumps(result, indent=2)+'\n')
        return result
    finally:
        cleanup = stop_owned(processes)
        cleanup['late_components'] = stop_late_components(output)
        path = output/'component_identities.jsonl'
        if path.exists():
            for row in path.read_text().splitlines():
                entry = json.loads(row)
                if matches(entry):
                    os.killpg(entry['pgid'], signal.SIGTERM)
        (output/'cleanup.json').write_text(json.dumps(cleanup, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.output)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['success'] else 2)
