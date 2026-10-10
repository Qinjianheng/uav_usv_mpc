#!/usr/bin/env python3
"""P44 bounded matched shadow matrix; original FOLLOW controls every flight."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from p43_sitl_matrix import prepare
from p4_sitl_session import WORKSPACE, identity, matches

CONDITIONS = (
    ('a_original', False, 'p43_fast', 1.2, 'D', False),
    ('b_p43', True, 'p43_fast', 1.2, 'D', False),
    ('c_p44_h12', True, 'p44_adaptive', 1.2, 'D', False),
    ('d_p44_h16', True, 'p44_adaptive', 1.6, 'D', False),
    ('e_p44_fallback', True, 'p44_adaptive', 1.2, 'F', False),
    ('f_p44_repeat', True, 'p44_adaptive', 1.2, 'D', False),
    ('f_p44_load', True, 'p44_adaptive', 1.2, 'D', True),
)


def main():
    """Keep at most one bounded retry; every failure and exact group identity stays recorded."""
    p = argparse.ArgumentParser()
    p.add_argument('output', type=Path)
    p.add_argument('--start', type=int, default=0)
    p.add_argument('--stop', type=int, default=len(CONDITIONS))
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    records = []
    for name, shadow, engine, horizon, ablation, load in CONDITIONS[args.start:args.stop]:
        for attempt in range(2):
            run = args.output/(name+('_retry' if attempt else ''))
            prepare(run, (run.name, 'f3', shadow, engine, 3., 1., horizon, 'none'))
            config_path = run/'config/research.yaml'
            config = yaml.safe_load(config_path.read_text())
            config['p4_follow_planner_node']['ros__parameters'].update(
                p44_ablation=ablation, p44_refinement='none')
            config_path.write_text(yaml.safe_dump(config, sort_keys=False))
            condition = json.loads((run/'condition.json').read_text())
            condition.update(ablation=ablation, controlled_load=load)
            (run/'condition.json').write_text(json.dumps(condition, indent=2))
            task_load, owned = None, None
            try:
                if load:
                    # One half-duty CPU worker, independent bounded lifetime, owned process group.
                    code = ('import time\nend=time.monotonic()+300\n'
                            'while time.monotonic()<end:\n'
                            ' t=time.monotonic()+.01\n'
                            ' while time.monotonic()<t: pass\n'
                            ' time.sleep(.01)\n')
                    task_load = subprocess.Popen(
                        [sys.executable, '-c', code], start_new_session=True)
                    owned = identity(task_load.pid)
                    (run/'load_identity.json').write_text(json.dumps(owned))
                command = [sys.executable, str(WORKSPACE/'scripts/p4_sitl_session.py'), str(run),
                           '--follow-seconds', '80', '--watchdog', '300']
                if shadow:
                    command.append('--shadow')
                print('START', run.name, flush=True)
                with (run/'session_driver.txt').open('x') as log:
                    result = subprocess.run(command, cwd=WORKSPACE, stdout=log, stderr=log,
                                            timeout=350)
            finally:
                if task_load is not None and owned is not None:
                    if matches(owned):
                        import os
                        os.killpg(owned['pgid'], signal.SIGTERM)
                    task_load.wait(timeout=3)
                    cleanup = dict(identity=owned, returncode=task_load.returncode,
                                   still_matches=matches(owned))
                    (run/'load_cleanup.json').write_text(json.dumps(cleanup))
            ended = json.loads((run/'watchdog_end.json').read_text()) if (
                run/'watchdog_end.json').exists() else {}
            records.append(dict(name=run.name, returncode=result.returncode, ended=ended,
                                finished=time.time()))
            (args.output/'matrix_results.json').write_text(json.dumps(records, indent=2))
            print('FINISH', run.name, result.returncode, ended, flush=True)
            if ended.get('still_alive'):
                raise RuntimeError('OWNED_GROUPS_STILL_ALIVE')
            if result.returncode == 0:
                break
            if 'EXTERNAL_SIMULATION' in (run/'session_driver.txt').read_text():
                raise RuntimeError('EXTERNAL_SESSION_PREVENTS_LAUNCH')


if __name__ == '__main__':
    main()
