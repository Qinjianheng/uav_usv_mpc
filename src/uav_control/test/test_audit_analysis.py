"""Capture statistics use one physical window and distinguish end from median."""

import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from p7_follow_analysis import guidance_metrics  # noqa: E402
from p7_capture_analysis import capture_metrics  # noqa: E402
from p7_capture_analysis import native_reference_distances  # noqa: E402


def write_run(tmp_path, rows):
    for row in rows:
        row.setdefault('receipt', row.get('stamp', 10.))
    (tmp_path / 'offline_evidence.jsonl').write_text(
        ''.join(json.dumps(row) + '\n' for row in rows))
    (tmp_path / 'bounded_stop.json').write_text('{"stamp": 11.0}')
    (tmp_path / 'condition.json').write_text('{"motion_mode": "variable_s"}')
    return tmp_path


def evidence():
    rows = []
    for stamp, error in ((10., .1), (11., .2), (12., 99.)):
        rows.append(dict(kind='p7_guidance', stamp=stamp, valid=True,
                         error_u=error, error_v=error, yaw_rate=error,
                         lateral_acceleration=error, mode='PNG'))
    for stamp, x in ((10., 0.), (11., 4.)):
        rows += [dict(kind='uav', stamp=stamp, valid=True, p=[x, 0., -5.],
                      v=[1., 0., 0.], a=[0., 0., 0.]),
                 dict(kind='ideal', stamp=stamp, valid=True, p=[5., 0., 0.], v=[1., 0., 0.]),
                 dict(kind='diagnostic', stamp=stamp, status='P7_GUIDANCE', visible=True)]
    return rows


def test_post_capture_guidance_cannot_change_count_or_extrema(tmp_path):
    run = write_run(tmp_path, evidence())
    metrics = guidance_metrics(run, 'P7_CAPTURE_COMPLETE', stop=11.)
    assert metrics['guidance_samples'] == 2
    for key in ('image_error_u_max', 'image_error_v_max', 'yaw_rate_max_radps',
                'lateral_acceleration_max_mps2'):
        assert metrics[key] == pytest.approx(.2)
    assert metrics['png_samples'] == 2
    assert metrics['final_distance_m'] == pytest.approx(1.)
    assert metrics['final_2s_median_distance_m'] == pytest.approx(3.)


def test_native_distance_uses_same_pose_epoch_and_never_pause_receipt_or_extrapolation():
    rows = [dict(kind='clock', sim=1., system=10.),
            dict(kind='clock', sim=1.04, system=10.04),
            dict(kind='clock', sim=1.04, system=14.04),
            dict(kind='rendered_uav', sim=1.02, p=[.3, 0., -.35]),
            dict(kind='rendered_target', sim=1.02, p=[0., 0., -.42]),
            dict(kind='rendered_uav', sim=2., p=[0., 0., 0.]),
            dict(kind='rendered_target', sim=2., p=[0., 0., -.42])]
    values = native_reference_distances(rows, 10., 10.03, .42)
    assert len(values) == 1
    assert values[0] == pytest.approx((10.02, 1.02, (.3**2+.35**2)**.5, .35))


@pytest.mark.parametrize('missing', ['all', 'ideal', 'uav', 'before_stop'])
def test_empty_physical_window_returns_unhealthy_without_crashing(tmp_path, missing):
    rows = evidence()
    if missing == 'all':
        rows = []
    elif missing == 'before_stop':
        rows = [r for r in rows if r.get('stamp', 0.) > 11.]
    else:
        rows = [r for r in rows if r['kind'] != missing]
    run = write_run(tmp_path, rows)
    assert not guidance_metrics(run, 'P7_CAPTURE_COMPLETE', stop=11.)['healthy']
    assert not capture_metrics(run, 'P7_CAPTURE_COMPLETE')['healthy']
