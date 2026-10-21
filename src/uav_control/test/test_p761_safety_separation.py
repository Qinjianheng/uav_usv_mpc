"""The sea adapter withdraws authority; it must not rewrite the paper acceleration."""
from pathlib import Path

import numpy as np
import pytest

from uav_control.common.p76_contract import load_contract
from uav_control.control.p76_paper_controller import PaperController
from uav_control.control.p76_paper_thrust import CalibratedThrustMap


def test_paper_step_does_not_query_or_modify_sea_safety():
    class SeparateBarrier:
        def check(self, *args):
            raise AssertionError('Sea safety must be outside the paper guidance law')
    contract = load_contract(Path(__file__).resolve().parents[3] /
                             'src/uav_usv_bringup/config/p76_contract.yaml')
    control = PaperController(contract, 2., 60.,
                              CalibratedThrustMap([0., 20., 60.], [0., .5, 1.], qualified=True),
                              SeparateBarrier())
    image, rotation, velocity = np.zeros(4), np.eye(3), np.array([3., 0., 0.])
    control.initialize(image, rotation, velocity, 1., [0., 0., 0.])
    assert np.isfinite(control.step(image, rotation, [0., 0., -4.], velocity, 1.02, .005)[
        'acceleration']).all()


def test_recovery_velocity_decelerates_from_latched_entry_state_not_each_new_sample():
    from uav_control.control.p76_recovery import VelocityRecovery
    recovery = VelocityRecovery()
    assert recovery.command([0., 0., 2.], 10., 4.) == pytest.approx([0., 0., 2.])
    assert recovery.command([0., 0., 1.99], 10.25, 4.) == pytest.approx([0., 0., 1.])
    assert recovery.command([0., 0., 1.7], 10.5, 4.) == pytest.approx([0., 0., 0.])


def test_recovery_feedforward_is_bounded_and_uses_command_derivative():
    from uav_control.control.p76_recovery import VelocityRecovery
    recovery = VelocityRecovery()
    recovery.setpoint([0., 0., -8.], [0., 0., 2.], 10., 4.)
    command = recovery.setpoint([0., 0., -7.5], [0., 0., 1.99], 10.25, 4.)
    assert command.velocity == pytest.approx([0., 0., 1.])
    assert command.acceleration == pytest.approx([0., 0., -4.])
    stopped = recovery.setpoint([0., 0., -7.], [0., 0., 1.7], 10.6, 4.)
    assert stopped.acceleration == pytest.approx([0., 0., 0.])
