"""Causal ideal-model rollouts preserve actual recorded inputs without masquerading as flown."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from follow_minco_experiment import scenario  # noqa: E402
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig  # noqa: E402
from uav_control.guidance.follow_problem import (  # noqa: E402
    FollowProblem, hypothetical_request,
)
from uav_control.guidance.follow_contract import FollowCurve  # noqa: E402


def test_hypothetical_boundary_comes_from_own_previous_curve_not_logged_uav():
    request = scenario('constant_velocity')
    own = FollowCurve(1, 1, 0, 1, 0, 99.9, 99.9, 99.9, 99.9, 100., 104.,
                      99.9, 101.1, 0., (1.2,),
                      ((-6., 0., -5.), (2., 0., 0.), (0., 0., 0.),
                       (0., 0., 0.), (0., 0., 0.), (0., 0., 0.)), ((0., 0., 0., 0.),))
    result = hypothetical_request(request, own, 100.)
    assert np.allclose(result.state[:3], (-5.8, 0., -5.))
    assert tuple(result.state) != tuple(request.state)
    assert np.array_equal(result.measurement_state, request.state)
    assert result.context.navigation_stamp == request.context.navigation_stamp
    assert result.boundary_policy == 'hypothetical_previous_curve'
    model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    FollowProblem(result, model, 1.2)
    with pytest.raises(ValueError, match='INVALID_HYPOTHETICAL_BOUNDARY'):
        FollowProblem(replace(result, state=request.state), model, 1.2)
