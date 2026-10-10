"""Conservative continuous attitude proof falls back at limits and on cancellation."""
import numpy as np
import pytest
from test_progress_follow import model
from test_follow_revalidation import inputs
from uav_control.guidance.follow_attitude_bounds import attitude_extrema


def test_bernstein_attitude_random_and_degenerate_conservatism():
    from uav_control.guidance.p44_invariant_cache import bounded_attitude
    config = model().attitude_config
    rng = np.random.default_rng(144)
    for scale in (0., .01, .1, 1., 4.):
        for _ in range(30):
            xyz = rng.normal(size=(3, 6, 3))*scale
            durations = rng.uniform(.05, .8, 3)
            got = bounded_attitude(xyz, durations, config)
            exact = attitude_extrema(xyz, durations, config)
            assert not got['valid'] or exact['valid']
            assert got['minimum_thrust'] <= exact['minimum_thrust']+1e-7
            assert got['maximum_thrust'] >= exact['maximum_thrust']-1e-7
            assert got['maximum_tilt'] >= exact['maximum_tilt']-1e-7


def test_near_tilt_limit_uses_exact_roots_and_nonfinite_rejected():
    from uav_control.guidance.p44_invariant_cache import bounded_attitude
    c = model().attitude_config
    xyz = np.zeros((1, 6, 3))
    xyz[0, 2, 0] = c.gravity*np.tan(c.maximum_tilt_rad)/2
    got = bounded_attitude(xyz, (1., ), c)
    assert got.get('method') != 'BERNSTEIN_SUFFICIENT'
    xyz[0, 2, 0] = float('nan')
    with pytest.raises(ValueError):
        bounded_attitude(xyz, (1., ), c)


def test_new_forecast_full_fov_even_when_math_is_reused():
    from uav_control.guidance.p44_invariant_cache import P44InvariantCache
    from uav_control.guidance.follow_revalidation import revalidate_prediction
    from uav_control.guidance.follow_profile import CycleProfile
    req, out, pred = inputs()
    cache = P44InvariantCache()
    _, r = revalidate_prediction(req, out, pred, req.now_stamp, model(), cache=cache)
    assert r['valid']
    pred.target_positions = tuple((x-100, y, z) for x, y, z in pred.target_positions)
    with CycleProfile(2) as cycle:
        _, r = revalidate_prediction(req, out, pred, req.now_stamp, model(), cache=cache)
    assert not r['valid'] and r['invariants_reused']
    assert cycle.report()['counts']['projection_samples'] > 0
    assert cycle.report()['counts'].get('root_searches', 0) == 0


def test_invalid_polynomial_shapes_explicitly_rejected():
    from uav_control.guidance.p44_invariant_cache import bounded_attitude
    for durations in (1., (), (float('inf'),)):
        with pytest.raises(ValueError):
            bounded_attitude(np.zeros((1, 6, 3)), durations, model().attitude_config)
