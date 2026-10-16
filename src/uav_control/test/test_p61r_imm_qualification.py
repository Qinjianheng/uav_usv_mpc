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


def test_primary_evidence_is_scoped_hashed_and_binds_current_source(tmp_path):
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
                    visibility_mode='nominal')
    assert entry.validate_primary_evidence(release, metadata, required_sources=[source])
    with pytest.raises(ValueError, match='IMM_PRIMARY_SCOPE_MISMATCH'):
        entry.validate_primary_evidence(release, dict(metadata, role='baseline'),
                                        required_sources=[source])
    source.write_text('changed')
    with pytest.raises(ValueError, match='IMM_PRIMARY_SOURCE_CHANGED'):
        entry.validate_primary_evidence(release, metadata, required_sources=[source])
    source.write_text('current')
    review.write_text('{"review_pass":false}')
    with pytest.raises(ValueError, match='IMM_PRIMARY_EVIDENCE_CHANGED'):
        entry.validate_primary_evidence(release, metadata, required_sources=[source])
