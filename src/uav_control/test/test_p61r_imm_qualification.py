"""Current IMM bootstrap uses real raw-observation math and sealed offline evidence."""
import importlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))


def module():
    return importlib.import_module('p61r_imm_qualification')


def raw(stamp=1792048950., **changes):
    row = dict(kind='p6_observation', stamp=stamp, receipt=stamp+.03,
               p=[8., 0., -.15], covariance=[.02, 0., 0., 0., .02, 0., 0., 0., .02],
               frame='local_ned', source='front_rgbd_red_sphere', valid=True,
               reason='', image_clock_reset_count=0, px4_clock_reset_count=0,
               received=stamp+.02, image_measurement_stamp=stamp)
    row.update(changes)
    return row


def test_real_replay_preserves_acquisition_and_rejects_duplicate_and_stale_rows():
    base = 1792048950.
    rows = [raw(base+i*.05, p=[8.+i*.1, 0., -.15]) for i in range(8)]
    rows += [raw(base+.35, receipt=base+.39), raw(base+.4, receipt=base+.526)]
    report = module().replay_rows(rows)
    assert report['attempts'] == 10
    assert report['accepted'] == report['strict_identity_matches'] == 8
    assert report['rejections'] == {'DUPLICATE_OBSERVATION': 1, 'OBSERVATION_STALE': 1}
    assert report['unsafe_output_count'] == 0
    assert report['sample_counts'] == {'41': 8}
    assert report['expired_rejections'] == 8


def test_shadow_boolean_alone_cannot_authorize_current_imm_pilot(tmp_path):
    metadata = dict(role='pilot', motion_mode='linear', group='B', prediction_model='imm',
                    visibility_mode='nominal')
    with pytest.raises(ValueError, match='IMM_PRIMARY_EVIDENCE_REQUIRED'):
        module().validate_primary_evidence(dict(shadow_gate_pass=True), metadata)


@pytest.fixture
def primary_bundle(tmp_path):
    entry = module()
    source, replay, review, verification = [tmp_path / name for name in (
        'source.py', 'replay.json', 'review.json', 'verification.json')]
    source.write_text('current')
    replay.write_text(json.dumps(dict(attempts=1200, accepted=1200, strict_identity_matches=1200,
                                      expired_rejections=1200, unsafe_output_count=0,
                                      sample_counts={'41': 1200}, compute_seconds={'p95': .01})))
    review.write_text(json.dumps(dict(review_pass=True)))
    verification.write_text(json.dumps(dict(passed=True, independent_review_pass=True)))
    payload = dict(kind=entry.KIND, scope='linear_nominal_pilot', passed=True,
                   evidence=[dict(path=str(p), sha256=entry.sha(p), role=role)
                             for p, role in ((replay, 'replay'), (review, 'review'),
                                             (verification, 'verification'))],
                   source_checks=[dict(path=str(source), sha256=entry.sha(source))])
    path = tmp_path / 'primary.json'
    path.write_text(json.dumps(payload))
    release = dict(imm_primary_evidence=dict(path=str(path), sha256=entry.sha(path)))
    metadata = dict(role='pilot', motion_mode='linear', group='B', prediction_model='imm',
                    visibility_mode='nominal', adaptive_viewpoint=False)
    return entry, release, metadata, source, path, review


@pytest.mark.parametrize('role,mode', [
    ('pilot', 'linear'), ('baseline', 'figure_eight'), ('experiment', 'variable_speed_s'),
])
def test_primary_evidence_is_scoped_hashed_and_binds_current_source(primary_bundle, role, mode):
    entry, release, metadata, source, _, review = primary_bundle
    metadata.update(role=role, motion_mode=mode)
    assert entry.validate_primary_evidence(release, metadata, required_sources=[source])
    invalid_pilot = dict(metadata, role='pilot', motion_mode='figure_eight')
    with pytest.raises(ValueError, match='IMM_PRIMARY_SCOPE_MISMATCH'):
        entry.validate_primary_evidence(release, invalid_pilot, required_sources=[source])
    source.write_text('changed')
    with pytest.raises(ValueError, match='IMM_PRIMARY_SOURCE_CHANGED'):
        entry.validate_primary_evidence(release, metadata, required_sources=[source])
    source.write_text('current')
    review.write_text('{"review_pass":false}')
    with pytest.raises(ValueError, match='IMM_PRIMARY_EVIDENCE_CHANGED'):
        entry.validate_primary_evidence(release, metadata, required_sources=[source])


@pytest.mark.parametrize('role,mode,group,visibility,adaptive', [
    ('pilot', 'linear', 'B', 'nominal', False),
    ('baseline', 'figure_eight', 'B', 'nominal', False),
    *[('experiment', mode, group, visibility, adaptive)
      for mode in ('linear', 'figure_eight', 'variable_speed_eight',
                   'variable_speed_s', 'variable_speed_z')
      for group, visibility, adaptive in (('B', 'nominal', False), ('D', 'robust', True))],
])
def test_pilot_bootstrap_supports_only_valid_imm_follow_on_cases(
        primary_bundle, role, mode, group, visibility, adaptive):
    entry, release, metadata, source, _, _ = primary_bundle
    target = dict(metadata, role=role, motion_mode=mode, group=group,
                  visibility_mode=visibility, adaptive_viewpoint=adaptive)
    report = entry.validate_primary_evidence(release, target, required_sources=[source])
    assert report['scope'] == 'linear_nominal_pilot'


@pytest.mark.parametrize('changes', [
    dict(role='unknown'), dict(role='experiment', motion_mode='unknown'),
    dict(prediction_model='bctra'), dict(role='experiment', group='A'),
    dict(role='experiment', group='C'), dict(role='experiment', group='D'),
    dict(role='experiment', visibility_mode='robust'),
    dict(role='experiment', adaptive_viewpoint=True),
    dict(role='pilot', motion_mode='figure_eight'),
    dict(role='pilot', group='D', visibility_mode='robust', adaptive_viewpoint=True),
    dict(role='baseline', motion_mode='linear'),
    dict(role='baseline', motion_mode='figure_eight', group='D', visibility_mode='robust',
         adaptive_viewpoint=True),
])
def test_shared_bootstrap_rejects_unsupported_case_scope(primary_bundle, changes):
    entry, release, metadata, source, _, _ = primary_bundle
    with pytest.raises(ValueError, match='IMM_PRIMARY_SCOPE_MISMATCH'):
        entry.validate_primary_evidence(release, dict(metadata, **changes),
                                        required_sources=[source])


@pytest.mark.parametrize('scope', [
    'all_experiments', 'offline raw-observation compatibility only',
])
def test_follow_on_case_cannot_relabel_bootstrap_evidence_scope(primary_bundle, scope):
    entry, release, metadata, source, path, _ = primary_bundle
    report = json.loads(path.read_text())
    report['scope'] = scope
    path.write_text(json.dumps(report))
    release['imm_primary_evidence']['sha256'] = entry.sha(path)
    target = dict(metadata, role='baseline', motion_mode='figure_eight')
    with pytest.raises(ValueError, match='IMM_PRIMARY_SCOPE_MISMATCH'):
        entry.validate_primary_evidence(release, target, required_sources=[source])


@pytest.mark.parametrize('evidence_role,changes', [
    ('replay', dict(unsafe_output_count=1)), ('review', dict(review_pass=False)),
    ('verification', dict(passed=False)), ('verification', dict(independent_review_pass=False)),
])
def test_follow_on_requires_passing_evidence_even_with_updated_hashes(
        primary_bundle, evidence_role, changes):
    entry, release, metadata, source, path, _ = primary_bundle
    report = json.loads(path.read_text())
    row = next(row for row in report['evidence'] if row['role'] == evidence_role)
    evidence_path = Path(row['path'])
    evidence = json.loads(evidence_path.read_text())
    evidence.update(changes)
    evidence_path.write_text(json.dumps(evidence))
    row['sha256'] = entry.sha(evidence_path)
    path.write_text(json.dumps(report))
    release['imm_primary_evidence']['sha256'] = entry.sha(path)
    with pytest.raises(ValueError, match='IMM_PRIMARY_EVIDENCE_REQUIRED'):
        entry.validate_primary_evidence(release, dict(metadata, role='experiment'),
                                        required_sources=[source])
