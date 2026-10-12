"""One qualified P47 F2 attempt, bounded FOLLOW time and observed-height stop."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest


def module():
    path = Path(__file__).resolve().parents[3]/'scripts/p47_minco_sitl.py'
    assert path.exists(), 'P47 bounded experiment wrapper missing'
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location('p47_minco_sitl', path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def test_all_five_gates_required_and_no_second_attempt(tmp_path, monkeypatch):
    m = module()
    monkeypatch.setattr(m, 'ROOT', tmp_path)
    qualifier = tmp_path/'qualification.json'
    qualifier.write_text(json.dumps({'gates': {str(i): True for i in range(1, 6)}}))
    m.claim(tmp_path/'f2', qualifier)
    with pytest.raises(FileExistsError):
        m.claim(tmp_path/'another', qualifier)
    (tmp_path/'sitl_attempt.json').unlink()
    qualifier.write_text(json.dumps({'gates': {'1': True}}))
    with pytest.raises(ValueError, match='FIVE_GATES_REQUIRED'):
        m.claim(tmp_path/'f2', qualifier)
    assert not (tmp_path/'sitl_attempt.json').exists()


def test_height_and_state_failure_stop_and_window_uses_follow_start():
    m = module()
    health = dict(control=dict(status='FOLLOW', receipt=100.),
                  navigation=dict(valid=True, p=[0., 0., -5.]),
                  vehicle=dict(nav_state=14, armed=2))
    assert m.stop_reason(health, 99., 100., 25.) is None
    assert m.stop_reason(health, 75., 100., 25.) == 'FOLLOW_25S_COMPLETE'
    health['navigation']['p'][2] = -6.1
    assert m.stop_reason(health, 99., 100., 25.) == 'HEIGHT_ERROR_OVER_1M'
    health['navigation']['p'][2] = -5.
    health['navigation']['valid'] = False
    assert m.stop_reason(health, 99., 100., 25.) == 'NAVIGATION_INVALID'


def test_no_stop_before_follow_and_unsafe_window_rejected():
    m = module()
    assert m.stop_reason({}, None, 100., 25.) is None
    with pytest.raises(ValueError, match='FOLLOW_WINDOW_20_TO_30_REQUIRED'):
        m.stop_reason({}, None, 100., 80.)
