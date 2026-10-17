"""Direct references preserve the measured/ACKed start and enforce independent validation."""
from dataclasses import replace

import numpy as np
import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.guidance.follow_problem import future_request, FollowProblem


def test_known_constant_speed_rear_reference_has_nonzero_terminal_velocity():
    from uav_control.controllers.direct_reference_minco import direct_seed
    raw = request(2.)
    raw = replace(raw, state=(0., 0., -5., 2., 0., 0., 0., 0., 0., 0.),
                  target_positions=tuple((5.+2*t, 0., 0.) for t in raw.prediction_times))
    req = future_request(raw, .15)
    seed = direct_seed(req, FollowProblem(req, model(), 1.2), 1.2)
    assert seed.start == req.state
    assert seed.end[:3] == pytest.approx((2.7, 0., -5.))
    assert seed.end[3:6] == pytest.approx((2., 0., 0.))
    assert seed.end[6:9] == pytest.approx((0., 0., 0.), abs=1e-10)
    assert seed.q[0] == pytest.approx((1.1, 0., -5.))


def test_virtual_frontend_is_causal_and_uses_propagated_acceleration():
    from uav_control.controllers.direct_reference_minco import direct_seed
    raw = request(2.)
    raw = replace(raw, state=(*raw.state[:3], 0., 0., 0., *raw.state[6:]))
    req = future_request(raw, .15)
    seed = direct_seed(req, FollowProblem(req, model(), 1.2), 1.2, frontend='virtual_follow')
    assert seed.start == req.state
    assert np.all(np.isfinite(seed.end))
    assert seed.metrics['frontend'] == 'virtual_follow'
    assert np.linalg.norm(seed.end[6:8]) > 0


@pytest.mark.parametrize('refinement', ['none', 'q', 'qt'])
def test_direct_solver_never_grants_invalid_or_expired_execution(refinement):
    from uav_control.controllers.direct_reference_minco import DirectReferenceMinco
    req = future_request(request(2.), .15)
    solver = DirectReferenceMinco(model(), refinement=refinement,
                                  wall_clock=lambda: req.now_stamp)
    result = solver.solve(req)
    assert result.metrics['direct']['fallback_used'] in (True, False)
    if result.valid:
        assert result.metrics['full_projection_validated']
        assert result.context == req.context
    expired = DirectReferenceMinco(model(), wall_clock=lambda: req.now_stamp+.126).solve(req)
    assert not expired.valid
