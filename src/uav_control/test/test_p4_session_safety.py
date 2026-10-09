"""Task-owned signaling and native Gazebo launcher exclusion, independent of a real flight."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
import p4_sitl_session as session  # noqa: E402


def test_recycled_pid_and_nonleader_never_authorize_group_signal(monkeypatch):
    own = dict(pid=42, pgid=42, start='100')
    monkeypatch.setattr(session, 'identity', lambda pid: dict(pid=42, pgid=42, start='101'))
    assert not session.matches(own)
    monkeypatch.setattr(session, 'identity', lambda pid: dict(pid=43, pgid=42, start='100'))
    assert not session.matches(dict(pid=43, pgid=42, start='100'))
    monkeypatch.setattr(session, 'identity', lambda pid: own)
    assert session.matches(own)


@pytest.mark.parametrize('comm,exe,argv,expected', [
    ('px4', '/build/px4', ['px4'], True),
    ('MicroXRCEAgent', '/bin/MicroXRCEAgent', ['MicroXRCEAgent', 'udp4'], True),
    ('ruby', '/usr/bin/ruby', ['/usr/bin/ruby', '/usr/bin/gz', 'sim', '-s'], True),
    ('gz', '/usr/bin/gz', ['gz', 'sim', '-g'], True),
    ('gz', '/usr/bin/gz', ['gz', 'topic', '-l'], False),
    ('python3', '/usr/bin/python3', ['python3', 'offline.py', 'gz sim'], False),
])
def test_preflight_detects_native_ruby_gazebo_without_matching_analysis(comm, exe, argv, expected):
    assert session.is_simulation_process(comm, exe, argv) == expected
