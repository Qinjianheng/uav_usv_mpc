#!/usr/bin/env python3
"""Sequential isolated A/B/C repeats; retain failures and stop on flight safety anomalies."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from p4_sitl_session import WORKSPACE
from p45_execution_analysis import analyze


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    p.add_argument('--scene', choices=('f2', 'f3'), required=True)
    args = p.parse_args()
    records = []
    for repeat in range(1, 4):
        for group, horizon, original in (('a', 1.2, True), ('b', 1.2, False), ('c', 1.6, False)):
            for attempt in range(2):
                name = f'{group}_{args.scene}_{repeat}'+('_retry' if attempt else '')
                root = args.root/name
                cmd = [sys.executable, str(WORKSPACE/'scripts/p45_minco_sitl.py'), str(root),
                       '--scene', args.scene, '--seconds', '80', '--horizon', str(horizon)]
                if original:
                    cmd.append('--original')
                print('START', name, flush=True)
                result = subprocess.run(cmd, cwd=WORKSPACE, timeout=350)
                end = json.loads((root/'watchdog_end.json').read_text()) if (
                    root/'watchdog_end.json').exists() else {}
                report = analyze(root) if end.get('x_time') else {}
                healthy = report.get('healthy', False)
                records.append(dict(name=name, returncode=result.returncode, healthy=healthy,
                                    reason=end.get('reason'),
                                    ownership=report.get('ownership'),
                                    windows=report.get('windows')))
                (args.root/f'{args.scene}_matrix.json').write_text(json.dumps(records, indent=2))
                print('FINISH', name, 'healthy', healthy, end.get('reason'), flush=True)
                if end.get('still_alive') or end.get('reason') not in (
                        'FOLLOW_WINDOW_COMPLETE', 'LAUNCHER_FAILED'):
                    raise RuntimeError('SAFETY_OR_HARNESS_FAILURE: inspect before further flights')
                if healthy:
                    break
                if end.get('x_time'):
                    raise RuntimeError('UNHEALTHY_FLIGHT: inspect before expanding')
            else:
                raise RuntimeError('TWO_STARTUP_FAILURES: inspect GCS/PX4/launcher')


if __name__ == '__main__':
    main()
