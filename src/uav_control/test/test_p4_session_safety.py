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


def test_prelaunch_snapshot_contains_untracked_source_and_is_immutable(tmp_path, monkeypatch):
    from experiment_source_snapshot import freeze_sources
    workspace = tmp_path/'workspace'
    (workspace/'src').mkdir(parents=True)
    (workspace/'src/new.py').write_text('value = 1\n')
    (workspace/'src/setup.cfg').write_text('[settings]\n')
    (workspace/'README.md').write_text('snapshot\n')

    def git(command, **kwargs):
        if 'ls-files' in command:
            return b'src/new.py\x00src/setup.cfg\x00README.md\x00'
        return b'test-head\n'
    monkeypatch.setattr(session.subprocess, 'check_output', git)
    output = tmp_path/'run'
    output.mkdir()
    freeze_sources(output, workspace)
    (workspace/'src/new.py').write_text('value = 2\n')
    assert (output/'source_snapshot/src/new.py').read_text() == 'value = 1\n'
    assert (output/'source_snapshot/src/setup.cfg').read_text() == '[settings]\n'
    with pytest.raises(FileExistsError):
        freeze_sources(output, workspace)


def test_rendered_sphere_pose_is_already_its_visual_center():
    import math
    from types import SimpleNamespace
    import numpy as np
    from p4_follow_analysis import rendered_marker_visibility
    from uav_control.guidance.camera_visibility import (
        CameraIntrinsics, CameraExtrinsics, TargetBoundingSphere, VisibilityConstraints,
    )
    model = SimpleNamespace(
        intrinsics=CameraIntrinsics.from_horizontal_fov(640, 480, math.pi/2),
        extrinsics=CameraExtrinsics.from_sdf_pose((0., 0., 0.), 0., 0., 0.),
        target=TargetBoundingSphere(.25, (0., 0., -.42)),
        visibility=VisibilityConstraints(.05, 25.))
    # The rendered entity centre lies on the optical axis; applying -.42 again is wrong.
    view = rendered_marker_visibility((0., 0., 0.), np.eye(3), (10., 0., 0.), model)
    assert view.image_center_uv == pytest.approx((320., 240.))
    assert model.target.center_offset_ned == (0., 0., -.42)


def test_monitor_shutdown_abort_cannot_make_session_successful():
    from p4_sitl_session import session_exit_code
    good = dict(reason='FOLLOW_WINDOW_COMPLETE', monitor_returncode=0, still_alive=[])
    assert session_exit_code(good) == 0
    assert session_exit_code(dict(good, monitor_returncode=-6)) == 1
    assert session_exit_code(dict(good, still_alive=[{'pid': 7}])) == 1
    assert session_exit_code(dict(good, reason='WATCHDOG')) == 1
