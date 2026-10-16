"""Current actual-record analysis compares physical epochs, never receipt times."""
import importlib.util
from pathlib import Path

import pytest


def analysis():
    path = Path(__file__).resolve().parents[3] / 'scripts/p6_actual_analysis.py'
    spec = importlib.util.spec_from_file_location('current_actual_epoch_analysis', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_reference_interpolation_rejects_extrapolation_and_missing_bracket():
    truth = analysis().BoundedTruth([
        dict(stamp=t, receipt=t, p=[4*(t-10), 0, 0], v=[4, 0, 0])
        for t in (10., 10.1, 10.5)])
    assert truth.at(10.05)[0]['p'][0] == pytest.approx(.2)
    for stamp in (9.99, 10.3, 10.51):
        assert truth.at(stamp)[0] is None


def test_kf_comparison_uses_sample_epoch_despite_delivery_delay():
    module = analysis()
    truth = module.BoundedTruth([
        dict(stamp=t, receipt=t, p=[4*(t-10), 0, 0], v=[4, 0, 0])
        for t in (10., 10.1, 10.2, 10.3, 10.4)])
    rows = [dict(valid=True, stamp=10.2, receipt=10.35, p=[.8, 0., 0.])]
    stats = module._position_metrics(rows, truth)
    assert stats['horizontal']['count'] == 1
    assert stats['horizontal']['rmse'] == pytest.approx(0., abs=1e-12)
    # Receipt relabelling would create 0.6m false error, showing sensitivity.
    stats = module._position_metrics([dict(rows[0], stamp=10.35)], truth)
    assert stats['horizontal']['rmse'] == pytest.approx(.6)
