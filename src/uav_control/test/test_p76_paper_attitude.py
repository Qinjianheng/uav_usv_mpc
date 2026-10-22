"""SO(3), physical thrust, and no implicit newton-to-normalized-thrust conversion."""
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_control.control.p76_paper_attitude import rotation_between, attitude_rates
from uav_control.control.p76_paper_thrust import physical_thrust, CalibratedThrustMap


@pytest.mark.parametrize('angle', (0., .3, math.pi))
def test_normalized_rodrigues_is_so3_and_maps_lift_direction(angle):
    a = np.array((0., 0., -1.))
    b = Rotation.from_euler('y', angle).apply(a)
    result = rotation_between(a, b)
    assert result.T@result == pytest.approx(np.eye(3))
    assert np.linalg.det(result) == pytest.approx(1.)
    assert result@a == pytest.approx(b)


def test_hover_and_forward_acceleration_signs_and_body_yaw_axis():
    result = attitude_rates(np.eye(3), (1., 0., 0.), .2, 9.81)
    assert result[1] < 0.  # FRD pitch nose down accelerates forward.
    assert result[2] == pytest.approx(.2)
    assert physical_thrust(2., np.eye(3), (0., 0., 0.), 9.81, 60.) == pytest.approx(19.62)


def test_thrust_mapping_requires_a_monotone_calibration_and_never_writes_newtons():
    mapping = CalibratedThrustMap((0., 19.62, 60.), (0., .5, 1.), qualified=True)
    assert mapping.to_body_z(19.62) == pytest.approx(-.5)
    assert mapping.to_body_z(100.) == -1.
    with pytest.raises(ValueError, match='THRUST_CALIBRATION'):
        CalibratedThrustMap((0., 20., 10.), (0., .5, 1.), qualified=True)
