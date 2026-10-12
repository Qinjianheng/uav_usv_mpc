"""P5.1 flight bounds and explicit one-shot authorization; these tests never fly."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
from test_p5_sitl import health  # noqa: E402


def test_stale_planning_input_inhibits_entry_while_lawful_original_follow_continues():
    from p51_minco_sitl import health_decision
    h = health()
    h['prediction']['observation'] = 99.8654
    d = health_decision(h, 100.)
    assert d['fatal'] is None
    assert d['inhibit'] == 'PREDICTION_INPUT_STALE'
    h['control']['status'] = 'MINCO_FOLLOW'
    assert health_decision(h, 100.)['fatal'] == 'PREDICTION_INPUT_STALE'


@pytest.mark.parametrize('section,patch', [
    ('navigation', {'stamp': 99.8}), ('control', {'locked': False}),
    ('vehicle', {'nav_state': 0}), ('navigation', {'v': [9., 0., 0.]}),
])
def test_inhibition_never_masks_original_tracker_or_physical_safety_failure(section, patch):
    from p51_minco_sitl import health_decision
    h = health()
    h['prediction']['observation'] = 99.8
    h[section].update(patch)
    assert health_decision(h, 100.)['fatal']


def test_flight_requires_new_explicit_authorization_and_completed_readiness(tmp_path):
    from p51_minco_sitl import require_readiness
    with pytest.raises(ValueError, match='EXPLICIT_P51_SITL_AUTHORIZATION_REQUIRED'):
        require_readiness(tmp_path/'missing.json', False)
    with pytest.raises(FileNotFoundError):
        require_readiness(tmp_path/'missing.json', True)


def test_new_attempt_root_is_distinct_and_f3_needs_real_f2_metrics():
    import p51_minco_sitl as task
    import p5_minco_sitl as old
    assert task.ROOT != old.ROOT
    assert task.ROOT.name == '20261012_p51_closed_loop_01'
    failed = dict(completed=True, healthy=True, minco_fraction=.9,
                  longest_minco_seconds=.4, observation_seconds=25., diagnostic_coverage=1.)
    assert not old.allow_f3(failed)


def test_readiness_rejects_failure_or_post_verification_source_edit(tmp_path):
    import hashlib
    import json
    from p51_minco_sitl import require_readiness
    source = tmp_path/'source.py'
    source.write_text('checked')
    readiness = dict(a_e_pass=True, source_checks=[dict(
        path=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest())])
    path = tmp_path/'ready.json'
    path.write_text(json.dumps(readiness))
    assert require_readiness(path, True) == readiness
    source.write_text('changed')
    with pytest.raises(ValueError, match='READINESS_SOURCE_CHANGED'):
        require_readiness(path, True)
    readiness['a_e_pass'] = False
    path.write_text(json.dumps(readiness))
    with pytest.raises(ValueError, match='P51_A_E_NOT_PASSED'):
        require_readiness(path, True)


def test_p51_f3_requires_explicit_complete_execution_audit():
    from p51_minco_sitl import allow_f3
    good = dict(completed=True, healthy=True, minco_fraction=.9,
                longest_minco_seconds=10., observation_seconds=25., diagnostic_coverage=.99)
    assert not allow_f3(good)
    checks = dict(reference_height=True, entry_recovery=True, execution_gates=True,
                  evidence_coverage=True, switching=True, handovers=True)
    good['p51_audit'] = dict(passed=True, checks=checks)
    assert allow_f3(good)
    for name in checks:
        bad = dict(good, p51_audit=dict(passed=True, checks=dict(checks, **{name: False})))
        assert not allow_f3(bad)
    assert not allow_f3(dict(good, p51_audit=dict(passed=True, checks={})))


def test_new_protocol_release_distinguishes_historical_math_from_current_source_checks():
    import json
    import p51_minco_sitl as task
    historical = task.WORKSPACE/'docs/current/audit/p5_20261012/release.json'
    old = json.loads(historical.read_text())
    release = dict(candidate=old['candidate'], formal_directory=old['formal_directory'],
                   historical_h16_release=str(historical),
                   historical_h16_sha256=task.harness.sha(historical),
                   a_e_offline_pass=True, kind='P51_TESTED_PROTOCOL_CANDIDATE')
    assert task.validate_release(release)
    # Neither the old matrix nor a label can certify edited wrappers by themselves.
    release['a_e_offline_pass'] = False
    assert not task.validate_release(release)
    release['a_e_offline_pass'] = True
    release['historical_h16_sha256'] = 'bad'
    assert not task.validate_release(release)
