"""Exercise the real one-click shell without launching Gazebo or PX4."""

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def launcher(tmp_path):
    workspace = tmp_path / 'workspace with spaces'
    scripts = workspace / 'scripts'
    scripts.mkdir(parents=True)
    (workspace / 'src').mkdir()
    (workspace / 'install').mkdir()
    (workspace / 'install/setup.bash').write_text(
        'export TEST_WORKSPACE_OVERLAY=' + shlex.quote(str(workspace)) + '\n')
    wrapper = scripts / 'uav_lab.sh'
    wrapper.write_bytes((ROOT / 'scripts/uav_lab.sh').read_bytes())
    wrapper.chmod(0o755)
    recorder = '''#!PYTHON
import json, os, pathlib, subprocess, sys
record = pathlib.Path(os.environ['TEST_RECORD'])
name = pathlib.Path(__file__).name
with record.open('a') as stream:
    stream.write(json.dumps(dict(name=name, args=sys.argv[1:],
                                workspace=os.environ.get('UAV_USV_WS'),
                                overlay=os.environ.get('TEST_WORKSPACE_OVERLAY'))) + '\\n')
if name == 'p7_follow_sitl.py' and os.environ.get('TEST_REENTER') == 'true':
    env = dict(os.environ, UAV_USV_EXPERIMENT_LAUNCH='p4_follow_research.launch.py')
    subprocess.run(['bash', str(pathlib.Path(__file__).with_name('uav_lab.sh')),
                    '--no-build'], env=env, check=True, timeout=5)
'''.replace('PYTHON', sys.executable)
    for name in ('start_px4_ros2.sh', 'build_workspace.sh', 'p7_follow_sitl.py'):
        path = scripts / name
        path.write_text(recorder)
        path.chmod(0o755)
    record = tmp_path / 'calls.jsonl'
    # Deliberately exclude inherited profile variables: this is a fresh user's
    # shell, and no credentials or unrelated environment enter failure output.
    env = dict(PATH=os.environ['PATH'], TEST_RECORD=str(record))

    def run(*args, reenter=False, custom_launch=None, check=True):
        selected = dict(env, TEST_REENTER=str(reenter).lower())
        if custom_launch:
            selected['UAV_USV_EXPERIMENT_LAUNCH'] = custom_launch
        result = subprocess.run(['bash', str(wrapper), *args], cwd=tmp_path,
                                env=selected, capture_output=True, text=True, timeout=10)
        calls = ([json.loads(line) for line in record.read_text().splitlines()]
                 if record.exists() else [])
        record.unlink(missing_ok=True)
        if check:
            assert result.returncode == 0, result.stderr
        return result, calls

    return workspace, run


@pytest.mark.parametrize('skip_build', [False, True])
def test_one_click_defaults_to_bounded_manual_p7_capture(launcher, skip_build):
    workspace, run = launcher
    _, calls = run(*(['--no-build'] if skip_build else []))
    assert [c['name'] for c in calls] == (
        [] if skip_build else ['build_workspace.sh']) + ['p7_follow_sitl.py']
    call = calls[-1]
    assert call['args'][1:] == ['--manual', '--capture']
    output = Path(call['args'][0])
    assert output.parent == workspace / 'data/experiments'
    assert not output.exists(), 'P7 prepare owns creation of the new run directory'
    assert call['workspace'] == call['overlay'] == str(workspace)


def test_two_launches_never_reuse_the_same_experiment_directory(launcher):
    _, run = launcher
    _, a = run('--no-build')
    _, b = run('--no-build')
    assert a[-1]['name'] == b[-1]['name'] == 'p7_follow_sitl.py'
    assert a[-1]['args'][0] != b[-1]['args'][0]


def test_p7_driver_reentry_reaches_backend_without_recursing(launcher):
    _, run = launcher
    _, calls = run('--no-build', reenter=True)
    assert [c['name'] for c in calls] == ['p7_follow_sitl.py', 'start_px4_ros2.sh']
    assert calls[-1]['args'] == ['--no-build']


def test_baseline_is_an_explicit_preserved_entry(launcher):
    _, run = launcher
    _, calls = run('--baseline', '--no-build')
    assert [c['name'] for c in calls] == ['start_px4_ros2.sh']
    assert calls[0]['args'] == ['--no-build']


def test_explicit_research_launch_still_uses_its_existing_backend(launcher):
    _, run = launcher
    _, calls = run('--no-build', custom_launch='p4_follow_research.launch.py')
    assert [c['name'] for c in calls] == ['start_px4_ros2.sh']


@pytest.mark.parametrize('option', ['--help', '--unknown'])
def test_help_and_invalid_options_do_not_build_or_start(launcher, option):
    _, run = launcher
    result, calls = run(option, check=False)
    assert not calls
    assert result.returncode == (0 if option == '--help' else 2)


def test_console_keeps_all_five_modes_before_x_and_y_capture():
    source = (ROOT / 'scripts/start_px4_ros2.sh').read_text()
    begin = source.index('motion_reset_pending=0')
    script = '''set -eo pipefail
timeout() { shift; "$@"; }
python3() { printf 'MODE %s\\n' "${*:3}"; }
publish_command() { printf 'COMMAND %s\\n' "$1"; }
restart_lab() { exit 99; }
''' + source[begin:]
    result = subprocess.run(['bash', '-c', script], input='1\n2\n3\n4\n5\nX\nY\nQ\n',
                            env={'PATH': os.environ['PATH'], 'UAV_USV_P7_MODE': 'true'},
                            capture_output=True, text=True, timeout=5, check=True)
    records = [line for line in result.stdout.splitlines()
               if line.startswith(('MODE ', 'COMMAND '))]
    assert records == ['MODE 1', 'MODE 2', 'MODE 3', 'MODE 4', 'MODE 5', 'MODE ',
                       'COMMAND X', 'COMMAND Y', 'COMMAND Q']
