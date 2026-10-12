#!/usr/bin/env python3
"""At most one qualified, isolated P47 F2 run; stop after 25 observed FOLLOW seconds."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from p43_sitl_matrix import prepare
from p4_sitl_session import WORKSPACE, identity, matches

ROOT = WORKSPACE/'data/experiments/20261010_p47_follow_guided'


def claim(output, qualification):
    """Persistent exclusive claim prevents this task's wrapper from automatically retrying."""
    output = Path(output).resolve()
    if ROOT.resolve() not in output.parents or output.exists():
        raise ValueError('NEW_P47_OUTPUT_REQUIRED')
    payload = json.loads(Path(qualification).read_text())
    if payload.get('gates') != {str(i): True for i in range(1, 6)}:
        raise ValueError('FIVE_GATES_REQUIRED')
    with (ROOT/'sitl_attempt.json').open('x') as stream:
        json.dump(dict(output=str(output), qualification=str(qualification),
                       stamp=time.time(), maximum_attempts=1), stream, indent=2)


def stop_reason(health, follow_start, now, seconds):
    """Additional experiment stop bounds; no change to controller admission or limits."""
    if not 20 <= seconds <= 30:
        raise ValueError('FOLLOW_WINDOW_20_TO_30_REQUIRED')
    if follow_start is None:
        return None
    control, nav, vehicle = (health.get(key, {})
                             for key in ('control', 'navigation', 'vehicle'))
    if now-control.get('receipt', 0.) > 2.:
        return 'CONTROL_OUTPUT_LOST'
    if not nav.get('valid') or control.get('status') in ('STATE_STALE', 'WAITING_FOR_STATE'):
        return 'NAVIGATION_INVALID'
    if vehicle.get('nav_state') != 14 or vehicle.get('armed') != 2:
        return 'OFFBOARD_OR_ARMING_LOST'
    if abs(nav.get('p', [0., 0., 0.])[2]+5.) > 1.:
        return 'HEIGHT_ERROR_OVER_1M'
    if now-follow_start >= seconds:
        return 'FOLLOW_25S_COMPLETE'
    return None


def run(output, qualification):
    """Reuse the original harness grant, monitor, publisher and exact owned-group cleanup."""
    output = Path(output).resolve()
    claim(output, qualification)
    prepare(output, (output.name, 'f2', True, 'p47_follow_guided', 3., 1., 1.2, 'none'))
    research_path = output/'config/research.yaml'
    config = yaml.safe_load(research_path.read_text())
    config['p4_follow_planner_node']['ros__parameters'].update(
        follow_execution_enabled=True, p47_terminal_mode='qva', p47_refinement=False,
        p47_scheduling='adaptive', follow_guided_response_time=.5)
    research_path.write_text(yaml.safe_dump(config, sort_keys=False))
    condition_path = output/'condition.json'
    condition = json.loads(condition_path.read_text())
    condition.update(actual_controller='NOMINAL_MINCO', minco_authorized=True,
                     authorization='User P4.7: at most one F2 after all five gates',
                     p47_terminal_mode='qva', p47_refinement=False, p47_scheduling='adaptive',
                     execution_lease_seconds=.45, qualified_holding=False,
                     follow_window_seconds=25., extra_height_stop_error_m=1.,
                     qualification=str(qualification), original_harness_unchanged=True)
    condition_path.write_text(json.dumps(condition, indent=2)+'\n')
    command = [sys.executable, str(WORKSPACE/'scripts/p4_sitl_session.py'), str(output),
               '--shadow', '--follow-seconds', '90', '--watchdog', '300']
    env = dict(os.environ, UAV_USV_HEADLESS_GCS='true', UAV_USV_MINCO_EXECUTE='true')
    started, follow_start, reason = time.time(), None, None
    with (output/'session_driver.txt').open('x') as log:
        child = subprocess.Popen(command, cwd=WORKSPACE, env=env, stdout=log,
                                 stderr=log, start_new_session=True)
        owned = identity(child.pid)
        try:
            while child.poll() is None and time.time()-started < 325.:
                path = output/'live_health.json'
                if path.exists():
                    health = json.loads(path.read_text())
                    status = health.get('control', {}).get('status')
                    if follow_start is None and status in ('FOLLOW', 'MINCO_FOLLOW'):
                        follow_start = time.time()
                    reason = stop_reason(health, follow_start, time.time(), 25.)
                    if reason:
                        (output/'bounded_stop.json').write_text(json.dumps(dict(
                            reason=reason, stamp=time.time(), follow_start=follow_start,
                            elapsed=time.time()-follow_start, health=health), indent=2)+'\n')
                        (output/'STOP').touch()
                        break
                time.sleep(.1)
            if child.poll() is None and reason is None:
                reason = 'OUTER_WATCHDOG'
                (output/'STOP').touch()
            child.wait(timeout=30.)
        finally:
            if child.poll() is None and matches(owned):
                os.kill(child.pid, signal.SIGINT)
                child.wait(timeout=30.)
    ended_path = output/'watchdog_end.json'
    ended = json.loads(ended_path.read_text()) if ended_path.exists() else {}
    complete = (reason == 'FOLLOW_25S_COMPLETE' and ended.get('monitor_returncode') == 0
                and not ended.get('still_alive', True))
    (output/'p47_run_summary.json').write_text(json.dumps(dict(
        completed=complete, reason=reason, harness_returncode=child.returncode,
        harness_end=ended, observed_follow_start=follow_start,
        note='Original harness reports WATCHDOG for external STOP; bounded_stop is authoritative',
        maximum_attempts=1), indent=2)+'\n')
    return 0 if complete else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--qualification', type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(run(args.output, args.qualification))


if __name__ == '__main__':
    main()
