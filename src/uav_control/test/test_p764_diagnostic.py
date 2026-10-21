"""Real input coverage cannot be qualified by image/detector summaries alone."""
import numpy as np

from test_p761_calibration import script


def test_numpy_timing_samples_preserve_real_percentiles():
    m = script('p764_diagnostic_collect')
    assert m.stats(np.array([.004, .005, .006]))['p50'] == .005


def test_missing_native_acquisition_identity_cannot_qualify_image_only_coverage():
    m = script('p764_diagnostic_collect')
    rows = []
    for i in range(60):
        stamp = dict(sec=10+i//10, nanosec=(i % 10)*100000000)
        for kind, message in (
            ('detection', dict(valid=True, stamp=stamp, raw_stamp=stamp, mask_counts=[0, 1, 99])),
            ('observation', dict(valid=True, stamp=stamp)),
            ('rgb_frame', dict(header=dict(stamp=stamp))),
            ('depth_frame', dict(header=dict(stamp=stamp))),
            ('image', dict(valid=True)), ('spatial', dict(valid=True)),
        ):
            rows.append(dict(kind=kind, receipt=10+i*.1+.02, message=message))
    graph = {k: [] for k in m.COMMAND_TOPICS}
    graph['/perception/target_detection'] = ['yolo_target_detector']
    result = m.analyze_coverage(rows, [dict(phase='Y_HANDOVER', stamp=13.)], [graph])
    assert not result['qualified']


def test_terminal_wrapper_keeps_launcher_stdin_open_after_wrapper_exits(tmp_path):
    import os
    import subprocess
    import sys
    import time
    m = script('p764_nonflight_session')
    wrapper = tmp_path/'gnome-terminal'
    m.write_terminal_wrapper(wrapper)
    child = tmp_path/'read_input.py'
    child.write_text('import pathlib,sys\n'
                     'value=sys.stdin.read(1)\n'
                     f'pathlib.Path({str(tmp_path/"received")!r}).write_text(value or "EOF")\n')
    process = subprocess.Popen([str(wrapper), '--title=PX4', '--', sys.executable, str(child)],
                               stdin=subprocess.PIPE,
                               env=dict(os.environ, UAV_USV_RESEARCH_DIRECTORY=str(tmp_path)))
    try:
        assert process.wait(timeout=5) == 0
        assert not (tmp_path/'received').exists()
        process.stdin.write(b'Q')
        process.stdin.flush()
        deadline = time.monotonic()+5
        while not (tmp_path/'received').exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert (tmp_path/'received').read_text() == 'Q'
    finally:
        process.stdin.close()


def test_synthetic_fixture_cannot_satisfy_real_executor_e1_gate():
    import pytest
    session = script('p761_session')
    with pytest.raises(ValueError, match='P764_REAL_E1'):
        session.validate_e1_evidence(dict(
            e1_matched_real_replay_passed=True,
            coverage={'matched_real_replay_qualified': True},
            fixture={'tracker_scheduling_qualified': True}))
