#!/usr/bin/env python3
"""Bounded P43 original/shadow matrix; reused session owns every signal and runtime group."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

import yaml

from p4_sitl_session import WORKSPACE

# Ordered A-G; H-J deliberately absent without independent control qualification.
CONDITIONS = (
    ('a_original_f3', 'f3', False, 'p41_tracking', 3., 1., 1.2, 'none'),
    ('b_p41_f3', 'f3', True, 'p41_tracking', 3., 1., 1.2, 'none'),
    ('c_acceleration_f3', 'f3', True, 'p43_fast', 3.5, 1., 1.2, 'none'),
    ('d_yaw_f3', 'f3', True, 'p43_fast', 3., 1.25, 1.2, 'none'),
    ('e_joint_f3', 'f3', True, 'p43_fast', 3.5, 1.25, 1.2, 'none'),
    ('f_qt_f3', 'f3', True, 'p43_fast', 3., 1., 1.2, 'qt'),
    ('g_h12_f3', 'f3', True, 'p43_fast', 3., 1., 1.2, 'none'),
    ('g_h16_f3', 'f3', True, 'p43_fast', 3., 1., 1.6, 'none'),
    ('g_q_h16_f3', 'f3', True, 'p43_fast', 3., 1., 1.6, 'q'),
    ('f2_original', 'f2', False, 'p41_tracking', 3., 1., 1.2, 'none'),
    ('f2_p43', 'f2', True, 'p43_fast', 3., 1., 1.6, 'none'),
    ('a_original_f3_repeat', 'f3', False, 'p41_tracking', 3., 1., 1.2, 'none'),
    ('g_h16_f3_repeat', 'f3', True, 'p43_fast', 3., 1., 1.6, 'none'),
)


def prepare(root, condition):
    """Copy exact historical scene configs into a NEW run; never edit source evidence."""
    name, scene, shadow, engine, acceleration, yaw, horizon, refinement = condition
    root.mkdir(exist_ok=False)
    (root/'.gitignore').write_text('*\n')
    (root/'COLCON_IGNORE').touch()
    (root/'config').mkdir()
    prior = WORKSPACE/'data/experiments/20261009_p41_rolling_follow'
    source = prior/('f2_original_retry_1' if scene == 'f2' else 'f3_original_2')
    flight = yaml.safe_load((source/'config/flight.yaml').read_text())
    tracker = flight['trajectory_tracker_node']['ros__parameters']
    tracker.update(follow_minco_enabled=False, follow_minco_shadow_check=shadow,
                   follow_research_horizontal_acceleration=acceleration,
                   follow_research_yaw_rate=yaw)
    research = yaml.safe_load((WORKSPACE/'src/uav_usv_bringup/config/p41_follow_research.yaml')
                              .read_text())
    research['p4_follow_planner_node']['ros__parameters'].update(
        minco_engine=engine, maximum_horizontal_acceleration=acceleration,
        maximum_yaw_rate=yaw, short_horizon=horizon, p43_refinement=refinement)
    (root/'config/flight.yaml').write_text(yaml.safe_dump(flight, sort_keys=False))
    (root/'config/research.yaml').write_text(yaml.safe_dump(research, sort_keys=False))
    condition_data = dict(name=name, scene=scene, shadow=shadow, engine=engine,
                          acceleration=acceleration, yaw=yaw, horizon=horizon,
                          refinement=refinement, actual_controller='ORIGINAL_FOLLOW',
                          minco_authorized=False, config_source=str(source/'config/flight.yaml'))
    (root/'condition.json').write_text(json.dumps(condition_data, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--stop', type=int, default=len(CONDITIONS))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    records = []
    for condition in CONDITIONS[args.start:args.stop]:
        for attempt in range(2):
            name = condition[0]+('_retry' if attempt else '')
            root = args.output/name
            prepare(root, condition)
            command = [sys.executable, str(WORKSPACE/'scripts/p4_sitl_session.py'), str(root),
                       '--follow-seconds', '80', '--watchdog', '300']
            if condition[2]:
                command.append('--shadow')
            print('START', name, flush=True)
            with (root/'session_driver.txt').open('x') as log:
                result = subprocess.run(command, cwd=WORKSPACE, stdout=log, stderr=log,
                                        timeout=350)
            ended = json.loads((root/'watchdog_end.json').read_text()) if (
                root/'watchdog_end.json').exists() else {}
            records.append(dict(name=name, returncode=result.returncode, ended=ended,
                                finished=time.time()))
            (args.output/'matrix_results.json').write_text(json.dumps(records, indent=2))
            print('FINISH', name, result.returncode, ended, flush=True)
            if ended.get('still_alive'):
                raise RuntimeError('OWNED_GROUPS_STILL_ALIVE')
            if result.returncode == 0:
                break
            if 'EXTERNAL_SIMULATION' in (root/'session_driver.txt').read_text():
                raise RuntimeError('EXTERNAL_SESSION_PREVENTS_LAUNCH')


if __name__ == '__main__':
    main()
