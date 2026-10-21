"""Independent camera optical/FRD/NED axis and origin witnesses."""
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_control.control.p76_paper_png import camera_los, paper_angles, direction_from_angles


def test_optical_right_down_forward_maps_to_frd_right_down_forward():
    assert camera_los((320., 240.), (320., 240., 300., 300.), np.eye(3), 0.) == pytest.approx(
        (1., 0., 0.))
    assert camera_los((620., 240.), (320., 240., 300., 300.), np.eye(3), 0.) == pytest.approx(
        np.array((1., 1., 0.))/math.sqrt(2))
    assert camera_los((320., 540.), (320., 240., 300., 300.), np.eye(3), 0.) == pytest.approx(
        np.array((1., 0., 1.))/math.sqrt(2))


@pytest.mark.parametrize('degrees', (32., 65.))
def test_mount_and_full_pose_rotate_the_optical_los(degrees):
    rotation = Rotation.from_euler('xyz', (.2, -.3, 1.1)).as_matrix()
    pitch = math.radians(degrees)
    expected = rotation@np.array((math.cos(pitch), 0., math.sin(pitch)))
    assert camera_los((320., 240.), (320., 240., 300., 300.), rotation, pitch) == pytest.approx(
        expected)


@pytest.mark.parametrize('direction', ((1., 0., 0.), (0., -1., 0.), (.3, .4, .5)))
def test_equations7_8_10_roundtrip_all_horizontal_quadrants(direction):
    unit = np.asarray(direction)/np.linalg.norm(direction)
    assert direction_from_angles(*paper_angles(unit)) == pytest.approx(unit)
