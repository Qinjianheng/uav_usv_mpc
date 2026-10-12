"""Frozen diagnostic counterfactuals must preserve the real start and terminal position."""
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

from uav_control.guidance.greedy_follow_initializer import FollowSeed


def audit():
    path = Path(__file__).resolve().parents[3]/'scripts/p47_derivative_diagnosis.py'
    assert path.exists(), 'P4.7 derivative diagnosis is not implemented'
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location('p47_derivative_diagnosis', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def seed():
    return FollowSeed(None, True, start=(0.,)*10, end=(1., 0., 0., 0., 0., 0., 0., 0., 0.),
                      q=((17/81, 0., 0.), (64/81, 0., 0.)),
                      durations=(1/3,)*3, yaw=(0.,)*4)


def test_gauss_quadrature_matches_known_exact_quintic_jerk_energy():
    report = audit().derivatives(seed())
    assert report['jerk_integral'] == pytest.approx(720., rel=1e-8)
    assert report['peaks']['horizontal_jerk']['value'] == pytest.approx(60., rel=1e-8)
    peak_time = report['peaks']['horizontal_jerk']['time']
    assert min(abs(peak_time), abs(peak_time-1.)) < 1e-8
    # Diagnostic reproducer: the legacy cross terms do not equal the independent integral.
    assert report['legacy_reported_control_effort'] == pytest.approx(2231.1111111111)
    weighted = audit().weighted_jerk(seed())
    assert weighted@weighted == pytest.approx(720., rel=1e-8)


@pytest.mark.parametrize('mode', ('A', 'VA', 'QVA', 'Q'))
def test_linear_jerk_optimization_respects_domains_and_fixed_boundary(mode):
    original = seed()
    changed, detail = audit().quadratic_counterfactual(original, mode)
    assert changed.start == original.start
    assert changed.end[:3] == original.end[:3]
    assert changed.durations == original.durations
    assert np.max(np.abs(np.asarray(changed.end[6:9])-original.end[6:9])) <= 1.5+1e-8
    assert np.max(np.abs(np.asarray(changed.end[3:6])-original.end[3:6])) <= .5+1e-8
    assert np.max(np.abs(np.asarray(changed.q)-original.q)) <= .2+1e-8
    assert detail['objective_after'] <= detail['objective_before']+1e-7
    if mode in ('A', 'VA'):
        assert changed.q == original.q
    if mode == 'Q':
        assert changed.end == original.end


def test_fixed_total_segment_time_scan_cannot_change_horizon_or_pva():
    changed, detail = audit().time_counterfactual(seed())
    assert sum(changed.durations) == pytest.approx(1.)
    assert changed.start == seed().start
    assert changed.end == seed().end
    assert changed.q == seed().q
    assert all(.8/3-1e-9 <= t <= 1.2/3+1e-9 for t in changed.durations)
    assert detail['candidate_count'] > 1


def test_yaw_only_diagnostic_keeps_xyz_map_identical():
    changed, detail = audit().yaw_counterfactual(seed(), 0.)
    assert changed.start == seed().start
    assert changed.end == seed().end
    assert changed.q == seed().q
    assert changed.durations == seed().durations
    assert changed.yaw[0] == seed().yaw[0]
    assert detail['maximum_rate_after'] <= detail['maximum_rate_before']+1e-8
