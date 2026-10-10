"""Cached invariant math never caches target-dependent admission or extends freshness."""
from dataclasses import replace
import numpy as np

from test_follow_revalidation import inputs
from test_progress_follow import model
from uav_control.guidance import follow_fast_validation as fast
from uav_control.guidance.follow_profile import CycleProfile
from uav_control.guidance.follow_revalidation import revalidate_prediction


def test_vectorized_polynomial_fields_have_independent_known_values():
    xyz = np.zeros((1, 6, 3))
    xyz[0, 0] = (1, 2, -5)
    xyz[0, 1] = (2, 0, 0)
    xyz[0, 2] = (3, 0, 0)
    yaw = np.array(((.2, .4, 0, 0),))
    values = fast.fields(xyz, (1.,), yaw, np.array((0., .5, 1.)))
    assert np.allclose(values[0][:, 0], (1., 2.75, 6.))
    assert np.allclose(values[1][:, 0], (2., 5., 8.))
    assert np.allclose(values[2][:, 0], (6., 6., 6.))
    assert np.allclose(values[4], (.2, .4, .6))


def test_same_curve_reuses_dynamics_but_rechecks_new_prediction_full_fov():
    req, output, pred = inputs()
    cache = fast.InvariantCache()
    with CycleProfile(1) as first:
        _, report = revalidate_prediction(req, output, pred, req.now_stamp, model(), cache=cache)
    assert report['valid'] and not report['invariants_reused']
    pred.target_positions = tuple((x-100, y, z) for x, y, z in pred.target_positions)
    with CycleProfile(2) as second:
        _, report = revalidate_prediction(req, output, pred, req.now_stamp, model(), cache=cache)
    assert not report['valid'] and report['invariants_reused']
    assert second.report()['counts']['projection_samples'] > 0
    assert second.report()['counts'].get('root_searches', 0) == 0
    assert first.report()['stages']['derivative_bounds']['calls'] == 6


def test_changed_coefficients_camera_envelope_or_epoch_cannot_hit_invariant_cache():
    req, output, pred = inputs()
    cache = fast.InvariantCache()
    _, report = revalidate_prediction(req, output, pred, req.now_stamp, model(), cache=cache)
    xyz = np.asarray(output.metrics['xyz_coefficients']).copy()
    xyz[0, 3, 0] = 50.
    bad = replace(output, metrics=dict(output.metrics, xyz_coefficients=xyz.tolist()))
    _, report = revalidate_prediction(req, bad, pred, req.now_stamp, model(), cache=cache)
    assert not report['valid'] and not report.get('invariants_reused', False)
    shifted = replace(req, context=replace(req.context, clock_generation=99))
    assert cache.key(req, output.metrics, model()) != cache.key(shifted, output.metrics, model())
    optical = model()
    optical.target = replace(optical.target, radius=.3)
    assert cache.key(req, output.metrics, model()) != cache.key(req, output.metrics, optical)


def test_cache_does_not_accept_same_bytes_with_invalid_shape():
    req, output, pred = inputs()
    cache = fast.InvariantCache()
    revalidate_prediction(req, output, pred, req.now_stamp, model(), cache=cache)
    bad = replace(output, metrics=dict(output.metrics, xyz_coefficients=np.asarray(
        output.metrics['xyz_coefficients']).reshape(-1, 3).tolist()))
    _, report = revalidate_prediction(req, bad, pred, req.now_stamp, model(), cache=cache)
    assert not report['valid'] and report['reason'] == 'INVALID_COEFFICIENTS'
