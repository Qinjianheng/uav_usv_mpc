"""Offline attribution is bounded and cannot be mistaken for new measurements."""


import numpy as np
import pytest
from scipy.spatial.transform import Rotation


@pytest.fixture
def audit():
    from uav_control.analysis import observation_pairing
    return observation_pairing


def test_surrogate_holds_position_and_uses_independent_attitude(audit):
    position = np.array([2., 3., -5.])
    # Known body point [10,0,0], yaw estimate +90 degrees, truth yaw zero.
    vision, reference = position+[0., 10., 0.], position+[10., 0., 0.]
    error = audit.attitude_surrogate(position, vision, reference,
                                     Rotation.from_euler('z', 90., degrees=True),
                                     Rotation.identity())
    np.testing.assert_allclose(error, [0., 0., 0.], atol=1e-12)
    unchanged = audit.attitude_surrogate(position, vision, reference,
                                         Rotation.identity(), Rotation.identity())
    np.testing.assert_allclose(unchanged, vision-reference)


def test_position_query_requires_native_time_bracket(audit):
    times = np.array([1., 1.02, 1.2])
    positions = np.array([[0., 0., 0.], [2., 0., 0.], [20., 0., 0.]])
    np.testing.assert_allclose(audit.vector_at(1.01, times, positions), [1., 0., 0.])
    for stamp in (.99, 1.21):
        with pytest.raises(ValueError, match='position_not_bracketed'):
            audit.vector_at(stamp, times, positions)
    with pytest.raises(ValueError, match='position_bracket_too_wide'):
        audit.vector_at(1.1, times, positions)
