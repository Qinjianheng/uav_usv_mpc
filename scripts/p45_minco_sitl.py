#!/usr/bin/env python3
"""Bounded X-only nominal MINCO experiments; reuse all original process exclusion/ownership."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml

from p43_sitl_matrix import prepare
from p4_sitl_session import WORKSPACE


def main():
    p = argparse.ArgumentParser()
    p.add_argument('output', type=Path)
    p.add_argument('--scene', choices=('f2', 'f3'), default='f2')
    p.add_argument('--seconds', type=float, default=80.)
    p.add_argument('--kill-planner-after', type=float)
    p.add_argument('--original', action='store_true')
    p.add_argument('--fault-suite', action='store_true')
    p.add_argument('--horizon', type=float, choices=(1.2, 1.6), default=1.2)
    p.add_argument('--engine', choices=('follow_guided_minco', 'p44_adaptive'),
                   default='follow_guided_minco')
    p.add_argument('--refinement', choices=('none', 'q', 'qt'), default='none')
    args = p.parse_args()
    prepare(args.output, (args.output.name, args.scene, True, args.engine,
                          3., 1., args.horizon, 'none'))
    path = args.output/'config/research.yaml'
    config = yaml.safe_load(path.read_text())
    config['p4_follow_planner_node']['ros__parameters'].update(
        p44_ablation='D', p44_refinement=(
            args.refinement if args.engine == 'p44_adaptive' else 'none'),
        follow_execution_enabled=not args.original,
        follow_guided_refinement=args.refinement, follow_guided_response_time=.5,
        follow_guided_optimization_budget=.025)
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    condition = json.loads((args.output/'condition.json').read_text())
    condition.update(horizon=args.horizon, engine=args.engine, refinement=args.refinement,
                     actual_controller='ORIGINAL_FOLLOW' if args.original else 'NOMINAL_MINCO',
                     qualified_holding=False, execution_lease_seconds=.45,
                     authorization='explicit SITL CLI; isolated-session grant required')
    (args.output/'condition.json').write_text(json.dumps(condition, indent=2))
    command = [sys.executable, str(WORKSPACE/'scripts/p4_sitl_session.py'), str(args.output),
               '--shadow', '--follow-seconds', str(args.seconds), '--watchdog', '300']
    if args.fault_suite:
        command.append('--fault-suite')
    if args.kill_planner_after is not None:
        command += ['--kill-planner-after', str(args.kill_planner_after)]
    env = dict(os.environ, UAV_USV_HEADLESS_GCS='true',
               UAV_USV_MINCO_EXECUTE='false' if args.original else 'true')
    with (args.output/'session_driver.txt').open('x') as log:
        result = subprocess.run(command, cwd=WORKSPACE, env=env, stdout=log, stderr=log,
                                timeout=350)
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
