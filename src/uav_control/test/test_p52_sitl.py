"""P5.2 authorization and current-source binding; never launches simulation."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def test_p52_has_its_own_attempt_root_and_explicit_permission():
    import p52_minco_sitl as task
    import p51_minco_sitl as old
    assert task.ROOT != old.ROOT
    assert task.ROOT.name == '20261013_p52_closed_loop_01'
    with pytest.raises(ValueError, match='EXPLICIT_P52_SITL_AUTHORIZATION_REQUIRED'):
        task.require_readiness(Path('/missing'), False)


def test_authorized_diagnostic_rounds_get_new_bounded_roots_without_resetting_old_attempts():
    import p52_minco_sitl as task
    assert task.round_root(1) == task.ROOT
    assert task.round_root(2).name == '20261013_p52_closed_loop_02'
    assert task.round_root(3).name == '20261013_p52_closed_loop_03'
    for invalid in [0, 4, -1, True]:
        with pytest.raises(ValueError, match='BOUNDED_P52_ROUND_REQUIRED'):
            task.round_root(invalid)


def ready(tmp_path):
    import p52_minco_sitl as task
    source = tmp_path/'source.py'
    source.write_text('checked')
    verification = tmp_path/'verification.json'
    verification.write_text(json.dumps(dict(passed=True, independent_review_pass=True)))
    qualification = tmp_path/'qualification.json'
    qualification.write_text('{}')
    row = dict(path=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    value = dict(kind=task.KIND, a_e_pass=True, source_checks=[row],
                 qualification_sha256=task.harness.sha(qualification),
                 verification_sha256=task.harness.sha(verification))
    path = tmp_path/'readiness.json'
    path.write_text(json.dumps(value))
    return path, source, verification


def test_p52_ready_binds_verification_independent_review_and_sources(tmp_path):
    import p52_minco_sitl as task
    path, source, verification = ready(tmp_path)
    assert task.require_readiness(path, True)['a_e_pass']
    source.write_text('changed')
    with pytest.raises(ValueError, match='READINESS_SOURCE_CHANGED'):
        task.require_readiness(path, True)
    source.write_text('checked')
    verification.write_text(json.dumps(dict(passed=True, independent_review_pass=False)))
    value = json.loads(path.read_text())
    value['verification_sha256'] = task.harness.sha(verification)
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='P52_A_E_NOT_PASSED'):
        task.require_readiness(path, True)


def test_old_p51_readiness_cannot_authorize_p52(tmp_path):
    import p52_minco_sitl as task
    path, _, _ = ready(tmp_path)
    value = json.loads(path.read_text())
    value['kind'] = 'P51_TESTED_PROTOCOL_CANDIDATE'
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='P52_A_E_NOT_PASSED'):
        task.require_readiness(path, True)


def test_p52_release_reuses_only_named_math_and_f3_still_needs_actual_f2():
    import p52_minco_sitl as task
    old = json.loads((task.WORKSPACE/'docs/current/audit/p51_follow_minco/qualification.json')
                     .read_text())
    assert not task.validate_release(old)
    old['kind'] = task.KIND
    assert task.validate_release(old)
    assert not task.allow_f3(dict(
        completed=True, healthy=True, minco_fraction=.99, longest_minco_seconds=25.,
        observation_seconds=25., diagnostic_coverage=1.))
