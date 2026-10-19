"""Contact preparation cannot climb and reopen pursuit on each heave crest."""

import pytest

from uav_control.control.p7_visual_guidance import capture_reference


@pytest.mark.parametrize('target_z', (-.15, 0., .15))
@pytest.mark.parametrize('target_vz', (-.2356, .2356))
def test_settled_sea_contact_layer_stays_settled_through_all_heave_phases(target_z, target_vz):
    standoff, vz = capture_reference((0., 0., -.33), (1., 0., target_z),
                                     (4., 0., target_vz))
    assert vz == pytest.approx(0.)
    assert standoff == pytest.approx(.3)


@pytest.mark.parametrize('target_z,target_vz', (
    (-.15, -.2356), (-.15, .2356), (.15, -.2356), (.15, .2356)))
def test_descent_corridor_depends_on_remaining_uav_descent_not_wave_direction(target_z, target_vz):
    # UAV is 1.33 m above the sea-safe layer: 1 m of descent remains.
    standoff, vz = capture_reference((0., 0., -1.33), (2., 0., target_z),
                                     (4., 0., target_vz))
    assert standoff == pytest.approx(2.3)
    assert vz == pytest.approx(.6)


def test_contact_profile_recovers_above_floor_after_vertical_tracking_undershoot():
    standoff, vz = capture_reference((0., 0., -.30), (1., 0., -.15), (4., 0., -.2))
    assert standoff == pytest.approx(.3)
    assert vz == pytest.approx(-.03)
