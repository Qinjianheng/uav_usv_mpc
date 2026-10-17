"""Evidence scope, freshness and regressions must fail closed before SITL."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def evidence():
    return dict(integrity=True, source_equivalent=True, horizon=1.6,
                engine='p47_follow_guided', terminal_mode='qva', refinement=False,
                scheduling='adaptive', scenes={scene: dict(
                    same_inputs=True, baseline_total=9, candidate_total=9,
                    independent=7, fallback=2, attempted=10,
                    steady_count=8, baseline_xy=.5, candidate_xy=.4,
                    full_coverage=True, revalidated=9, validation_failures=0,
                    height_rmse=.01, p95=.012, p99=.015,
                    minimum_margin_after_reserve=.001)
                    for scene in ('f2', 'f3')}, h12_counterexample_retained=True)


def test_scoped_release_keeps_old_gate_and_does_not_claim_receiver_execution():
    from p5_release import qualify
    result = qualify(evidence())
    assert result['offline_candidate_pass']
    assert result['h16_scoped_gate_pass']
    assert result['old_all_horizon_gate_changed'] is False
    assert result['sitl_validated'] is False
    assert result['receiver_margin_measured'] is False


@pytest.mark.parametrize('field,value', [
    ('candidate_total', 8), ('candidate_xy', .51), ('full_coverage', False),
    ('validation_failures', 1), ('minimum_margin_after_reserve', -.0001),
    ('p99', .026), ('steady_count', 0), ('fallback', 3), ('height_rmse', 1.1),
])
def test_regression_in_either_scene_blocks_scoped_gate(field, value):
    from p5_release import qualify
    data = evidence()
    data['scenes']['f3'][field] = value
    assert not qualify(data)['h16_scoped_gate_pass']


def test_new_horizon_or_missing_provenance_cannot_inherit_h16_gate():
    from p5_release import qualify
    for change in ({'horizon': 1.2}, {'integrity': False},
                   {'source_equivalent': False}, {'h12_counterexample_retained': False}):
        data = deepcopy(evidence())
        data.update(change)
        assert not qualify(data)['offline_candidate_pass']
    data = evidence()
    del data['scenes']['f2']
    assert not qualify(data)['h16_scoped_gate_pass']


def test_corrupted_or_escaping_evidence_is_rejected(tmp_path):
    from p5_release import verify_manifest
    (tmp_path/'raw.json').write_text('{}\n')
    (tmp_path/'SHA256SUMS').write_text('0'*64+'  raw.json\n')
    with pytest.raises(ValueError, match='EVIDENCE_HASH_MISMATCH'):
        verify_manifest(tmp_path)
    (tmp_path/'SHA256SUMS').write_text('0'*64+'  ../raw.json\n')
    with pytest.raises(ValueError, match='EVIDENCE_HASH_MISMATCH'):
        verify_manifest(tmp_path)


def test_executable_source_change_cannot_be_excused_as_a_docstring(tmp_path):
    from p5_release import semantic_tree
    a, b = tmp_path/'a.py', tmp_path/'b.py'
    a.write_text('"""old"""\ndef value():\n    """old"""\n    return 1\n')
    b.write_text('"""new"""\ndef value():\n    """new"""\n    return 1\n')
    assert semantic_tree(a) == semantic_tree(b)
    b.write_text('def value():\n    return 2\n')
    assert semantic_tree(a) != semantic_tree(b)
