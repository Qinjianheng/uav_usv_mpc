"""Integrated paper path uses physical force and bounded rates, without P7 velocity helpers."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from uav_control.common.p76_contract import load_contract
from uav_control.control.p76_bodyrate_safety import BodyrateSeaBarrier
from uav_control.control.p76_paper_controller import PaperController
from uav_control.control.p76_paper_thrust import CalibratedThrustMap


def test_integrated_candidate_respects_dt_force_and_rate_slew():
    root = Path(__file__).resolve().parents[3]
    contract = deepcopy(load_contract(root/'src/uav_usv_bringup/config/p76_contract.yaml'))
    # Analytic fixture; these samples do not qualify the real PX4 thrust curve.
    mapping = CalibratedThrustMap([0., 20., 60.], [0., .5, 1.], qualified=True)
    barrier = BodyrateSeaBarrier([[0., 0., .1]], .33, .1, 4.)
    control = PaperController(contract, 2., 60., mapping, barrier)
    image, rotation, velocity = np.zeros(4), np.eye(3), np.array([3., 0., 0.])
    control.initialize(image, rotation, velocity, 1., [0., 0., 0.])
    result = control.step(image, rotation, [0., 0., -4.], velocity, 1.02, .005)
    assert max(abs(result['rates'])) <= .015+1e-9
    assert -1 <= result['thrust'] < 0
    assert 0 < result['force_newtons'] <= 60.
    assert np.linalg.norm(result['acceleration']) <= 2.+1e-6
    with pytest.raises(ValueError, match='DT'):
        control.step(image, rotation, [0., 0., -4.], velocity, 1.02, .2)
