"""A missing clock in one partition never proves ownership of another PX4."""

import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize('present', [True, False])
def test_ordinary_launcher_never_signals_an_unowned_px4(present):
    root = Path(__file__).resolve().parents[3]
    source = (root / 'scripts/start_px4_ros2.sh').read_text()
    begin = source.index('PX4_EXECUTABLE="$(readlink')
    end = source.index('\nsource /opt/ros/humble/setup.bash', begin)
    script = '''set -eo pipefail
readlink() { printf '/fake/px4\\n'; }
pgrep() { if [[ "$PRESENT" == true ]]; then printf '424242\\n'; else return 1; fi; }
kill() {
    if [[ "$1" == -0 ]]; then return 1; fi
    printf 'SIGNALLED %s\\n' "$*"
}
PX4_ROOT=/fake
GZ_PARTITION=our_task
''' + source[begin:end]
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True,
                            env={'PATH': os.environ['PATH'], 'PRESENT': str(present).lower()})
    assert 'SIGNALLED' not in result.stdout
    assert (result.returncode != 0) == present
    if present:
        assert '424242' in result.stderr
