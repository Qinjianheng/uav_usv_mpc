"""Startup profiles keep required control nodes and own their optional GCS."""

import importlib.util
import ast
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[3]
LAUNCHES = ROOT / 'src/uav_usv_bringup/launch'
RUNNER = ROOT / 'scripts/run_with_sitl_gcs.py'


@pytest.mark.parametrize('capture', [False, True])
def test_automated_p7_driver_propagates_its_profile_to_launcher(capture):
    tree = ast.parse((ROOT / 'scripts/p7_follow_sitl.py').read_text())
    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run')
    assignment = next(n for n in run.body if isinstance(n, ast.Assign)
                      and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'env')
    from types import SimpleNamespace
    env = eval(compile(ast.Expression(assignment.value), 'p7_driver_env', 'eval'),
               {'os': SimpleNamespace(environ={}), 'capture': capture})
    assert env.get('UAV_USV_P7_MODE') == 'true'
    assert env.get('UAV_USV_P7_CAPTURE') == str(capture).lower()


def graph(filename, tmp_path, monkeypatch, overrides=None):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    spec = importlib.util.spec_from_file_location('startup_launch', LAUNCHES / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(LAUNCHES.parent))
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return description, context


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_legacy_intercept_planner_can_be_disabled_without_removing_follow(
        enabled, tmp_path, monkeypatch):
    from launch_ros.actions import Node
    description, context = graph('modular_intercept.launch.py', tmp_path, monkeypatch,
                                 {'enable_intercept_planner': enabled})
    nodes = {n.node_executable: n for n in description.entities if isinstance(n, Node)}
    condition = nodes['intercept_planner_node'].condition
    assert condition is not None, 'legacy planner still starts unconditionally'
    assert condition.evaluate(context) == (enabled == 'true')
    assert nodes['p4_follow_planner_node'].condition.evaluate(context)
    for name in ('trajectory_tracker_node', 'mission_manager_node', 'target_kalman_filter',
                 'target_bearing_node', 'rgbd_target_localizer'):
        assert nodes[name].condition is None
    assert nodes['intercept_evaluator_node'].condition.evaluate(context)


@pytest.mark.parametrize('filename,p7,expected', [
    ('modular_intercept.launch.py', False, 'true'),
    ('p7_guidance.launch.py', False, 'false'),
    ('p4_follow_research.launch.py', True, 'false'),
    ('p4_follow_research.launch.py', False, 'true'),
])
def test_entry_point_resolves_and_forwards_planner_switch(
        filename, p7, expected, tmp_path, monkeypatch):
    from launch.actions import IncludeLaunchDescription
    monkeypatch.setenv('UAV_USV_P7_MODE', str(p7).lower())
    for override in (None, {'enable_intercept_planner': 'true'}):
        description, context = graph(filename, tmp_path, monkeypatch, override)
        value = context.launch_configurations.get('enable_intercept_planner')
        assert value == ('true' if override else expected)
        for action in description.entities:
            if isinstance(action, IncludeLaunchDescription):
                args = dict(action.launch_arguments)
                forwarded = args['enable_intercept_planner']
                assert context.perform_substitution(forwarded) == value


def wait_file(path, process):
    deadline = time.monotonic() + 5.
    while time.monotonic() < deadline:
        if path.exists():
            return int(path.read_text())
        assert process.poll() is None, process.communicate()
        time.sleep(.02)
    pytest.fail(f'child did not start: {path}')


def is_running(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


@pytest.mark.parametrize('external,existing,auto,local,started', [
    (False, False, False, True, False),
    (False, True, False, False, False),
    (True, False, False, False, False),
    (True, False, True, False, False),
    (False, False, True, False, True),
])
def test_launcher_selects_one_gcs_owner(tmp_path, external, existing, auto, local, started):
    launcher = (ROOT / 'scripts/start_px4_ros2.sh').read_text()
    begin = launcher.index('if [[ "${UAV_USV_HEADLESS_GCS:-false}" == true ]]')
    end = launcher.index('\nif timeout 3s gz topic', begin)
    marker = tmp_path / 'qgc_started'
    script = '''set -eo pipefail
pgrep() { [[ "$EXISTING" == true || -f "$MARKER" ]]; }
nohup() { touch "$MARKER"; }
sleep() { command sleep .01; }
EXPERIMENT_GCS_PREFIX=""
''' + launcher[begin:end] + '\nprintf "PREFIX=%s\\n" "$EXPERIMENT_GCS_PREFIX"\n'
    env = dict(PATH=os.environ['PATH'], TMPDIR=str(tmp_path),
               UAV_USV_HEADLESS_GCS=str(external).lower(),
               EXISTING=str(existing).lower(), START_QGC=str(auto).lower(),
               WS_ROOT=str(ROOT), QGC_APPIMAGE='/bin/true', MARKER=str(marker))
    result = subprocess.run(['bash', '-c', script], env=env, capture_output=True,
                            text=True, timeout=5, check=True)
    prefix = result.stdout.split('PREFIX=', 1)[1].strip()
    assert ('run_with_sitl_gcs.py' in prefix) == local
    assert marker.exists() == started


@pytest.mark.parametrize('stop', ['command_exit', 'signal', 'hangup', 'heartbeat_exit'])
def test_local_gcs_lives_with_experiment_and_children_are_reaped(tmp_path, stop):
    assert RUNNER.is_file(), 'experiment-owned GCS runner is missing'
    worker = tmp_path / 'worker.py'
    worker.write_text('''import os, pathlib, sys, time
pathlib.Path(sys.argv[1]).write_text(str(os.getpid()))
while not pathlib.Path(sys.argv[2]).exists():
    time.sleep(.02)
''')
    command_pid, gcs_pid = tmp_path / 'command.pid', tmp_path / 'gcs.pid'
    command_stop, gcs_stop = tmp_path / 'command.stop', tmp_path / 'gcs.stop'
    code = (
        'import importlib.util, sys; '
        f'spec=importlib.util.spec_from_file_location("runner", {str(RUNNER)!r}); '
        'm=importlib.util.module_from_spec(spec); spec.loader.exec_module(m); '
        f'sys.exit(m.run({[sys.executable, str(worker), str(command_pid), str(command_stop)]!r}, '
        f'heartbeat_command={[sys.executable, str(worker), str(gcs_pid), str(gcs_stop)]!r}))'
    )
    process = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True)
    try:
        command = wait_file(command_pid, process)
        heartbeat = wait_file(gcs_pid, process)
        assert is_running(heartbeat) and is_running(command)
        if stop == 'command_exit':
            command_stop.touch()
        elif stop == 'heartbeat_exit':
            gcs_stop.touch()
        else:
            process.send_signal(signal.SIGHUP if stop == 'hangup' else signal.SIGTERM)
        _, error = process.communicate(timeout=10)
        assert process.returncode == {'command_exit': 0, 'signal': 143, 'hangup': 129,
                                      'heartbeat_exit': 1}[stop], error
        assert not is_running(command)
        assert not is_running(heartbeat)
    finally:
        if process.poll() is None:
            process.terminate()
            process.communicate(timeout=10)
        for child_file in (command_pid, gcs_pid):
            if child_file.exists():
                pid = int(child_file.read_text())
                if is_running(pid):
                    os.killpg(pid, signal.SIGTERM)
