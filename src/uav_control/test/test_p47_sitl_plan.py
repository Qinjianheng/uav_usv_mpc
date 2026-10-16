"""Retain bounded-attempt and health checks on the maintained P5/P6 harness."""
import importlib
import json
from pathlib import Path
import sys

import pytest


def module():
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
    return importlib.import_module('p5_minco_sitl')


def test_current_protocol_required_before_one_shot_attempt(tmp_path, monkeypatch):
    m = module()
    from p6_minco_sitl import KIND, validate_release
    formal, audit, task = (tmp_path/name for name in ('formal', 'audit', 'task'))
    formal.mkdir()
    audit.mkdir()
    (formal/'raw.json').write_text('{}\n')
    (formal/'SHA256SUMS').write_text(m.sha(formal/'raw.json')+'  raw.json\n')
    qualifier = audit/'qualification.json'
    source = m.WORKSPACE/'scripts/p4_sitl_session.py'
    payload = dict(kind=KIND, candidate=dict(m.CANDIDATE), offline_gate_pass=True,
                   formal_directory=str(formal), source_checks=[dict(
                       path='scripts/p4_sitl_session.py', current_sha256=m.sha(source))])

    def save(record):
        qualifier.write_text(json.dumps(record))
        (audit/'SHA256SUMS').write_text(m.sha(qualifier)+'  qualification.json\n')

    monkeypatch.setattr(m, 'preflight', lambda: {'isolated': True})
    # Old five-booleans cannot replace the current protocol/candidate gate.
    save(dict(payload, kind='P47', gates={str(i): True for i in range(1, 6)}))
    with pytest.raises(ValueError, match='H16_SCOPED_EVIDENCE_REQUIRED'):
        m.claim(task/'f2', 'f2', qualifier, task_root=task, release_gate=validate_release)
    assert not (task/'attempt_f2.json').exists()
    save(dict(payload, offline_gate_pass=False))
    with pytest.raises(ValueError, match='H16_SCOPED_EVIDENCE_REQUIRED'):
        m.claim(task/'f2', 'f2', qualifier, task_root=task, release_gate=validate_release)
    save(payload)
    m.claim(task/'f2', 'f2', qualifier, task_root=task, release_gate=validate_release)
    with pytest.raises(FileExistsError):
        m.claim(task/'another', 'f2', qualifier, task_root=task, release_gate=validate_release)
    assert not (task/'another').exists()


def test_height_and_state_failure_stop_with_current_freshness_checks():
    m = module()
    health = dict(ros_now=100., written=100.,
                  control=dict(status='FOLLOW', receipt=100., locked=True, compute=.001),
                  navigation=dict(valid=True, stamp=99.98, receipt=100.,
                                  p=[0., 0., -5.], v=[2., 0., 0.]),
                  vehicle=dict(nav_state=14, armed=2, receipt=100.),
                  prediction=dict(stamp=99.98, observation=99.98, valid=True, receipt=100.),
                  attitude=dict(receipt=100., q=[1., 0., 0., 0.]))
    assert m.safety_reason(health, 100.) is None
    health['navigation']['p'][2] = -6.1
    assert m.safety_reason(health, 100.) == 'HEIGHT_ERROR_OVER_1M'
    health['navigation']['p'][2] = -5.
    health['navigation']['valid'] = False
    assert m.safety_reason(health, 100.) == 'NAVIGATION_INVALID'


def test_current_observation_duration_is_bounded_without_extending_legacy_defaults():
    m = module()
    assert m.observation_window('f2') == 25.
    assert m.observation_window('f3') == 60.
    assert m.observation_window('f2', 60.) == 60.
    for duration in (80., 0., -1., float('nan'), float('inf')):
        with pytest.raises(ValueError, match='BOUNDED_OBSERVATION_WINDOW_REQUIRED'):
            m.observation_window('f2', duration)
