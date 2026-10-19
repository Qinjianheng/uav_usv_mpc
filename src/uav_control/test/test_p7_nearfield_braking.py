"""Independent delayed-sensing, heave and actuator counterexample for P7 capture."""

import math

import numpy as np
import pytest

from uav_control.control.p7_visual_guidance import P7VisualGuidance, capture_reference
from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState


def test_capture_brakes_measured_overspeed_with_identical_previous_command():
    commands = []
    for measured in (3., 4.2):
        controller = P7VisualGuidance()
        controller.reset((3., 0.))
        output = controller.step(10., .05, (0., 0.), (measured, 0.), (.5, 0.),
                                 (3., 0.), 9.98, 0., 1., approach_distance=.4)
        commands.append(output.velocity[0])
        assert output.closing_speed == pytest.approx(measured-3.)
    assert commands[1] < commands[0]-.05


def test_capture_anticipates_lower_edge_and_holds_forecast_for_repeated_image():
    controller = P7VisualGuidance()
    controller.reset((3., 0.))
    first = controller.step(10., .05, (0., 0.), (3., 0.), (.8, 0.), (3., 0.),
                            9.98, 0., .5, approach_distance=.4)
    second = controller.step(10.05, .05, (0., 0.), (3., 0.), (.8, 0.), (3., 0.),
                             10.03, 0., .6, approach_distance=.4)
    assert first.acceleration[0] > 0.
    assert second.acceleration[0] < 0.  # Current image error is still below .7.
    repeated = controller.step(10.10, .05, (0., 0.), (3., 0.), (.8, 0.), (3., 0.),
                               10.03, 0., .6, approach_distance=.4)
    assert repeated.acceleration[0] < 0.
    assert not controller.step(10.3, .05, (0., 0.), (3., 0.), (.8, 0.), (3., 0.),
                               10.03, 0., .6, approach_distance=.4).valid
    controller.reset((3., 0.))
    reset = controller.step(10.35, .05, (0., 0.), (3., 0.), (.8, 0.), (3., 0.),
                            10.33, 0., .6, approach_distance=.4)
    assert reset.acceleration[0] > 0.


def test_contact_height_tracks_configured_sea_layer_instead_of_target_heave():
    # At a configured sea z=.50, the .33 m contact layer is z=.17.
    # UAV z=.07 has .10 m descent left, independent of target wave velocity.
    for vz in (-.2, .2):
        distance, actual = capture_reference((0., 0., .07), (1., 0., .4),
                                             (3., 0., vz), sea_surface_z=.5)
        assert distance == pytest.approx(.5)
        assert actual == pytest.approx(.1)


@pytest.mark.parametrize('heave_phase', (0., math.pi/2., math.pi, 3.*math.pi/2.))
def test_speed_heave_lag_counterexample_captures_without_leaving_camera(heave_phase):
    p, v = np.array([-5., 0., -5.]), np.array([4., 0., 0.])
    controller = P7VisualGuidance()
    controller.reset(v)
    flight = FlightGuidanceCore()
    flight.previous_velocity = tuple(v)
    dt, frequency, pitch = .05, 2.*math.pi/24., math.radians(28.)
    maximum_error = -math.inf
    for i in range(1200):
        t = i*dt

        def target(s):
            return (np.array([3.*s+math.sin(frequency*s)/frequency, 0.,
                              .15*math.sin(math.pi*s/2.+heave_phase)]),
                    np.array([3.+math.cos(frequency*s), 0.,
                              .15*math.pi/2.*math.cos(math.pi*s/2.+heave_phase)]))

        q, w = target(t-.04)
        q += w*.04
        actual, _ = target(t)
        distance = float(np.linalg.norm(actual-p))
        # Independent level-camera transformation, not production geometry.
        forward, up = actual[0]-p[0]-.18, p[2]-actual[2]+.42-.39
        optical_forward = math.cos(pitch)*forward-math.sin(pitch)*up
        optical_up = math.sin(pitch)*forward+math.cos(pitch)*up
        assert optical_forward > 0.
        image_v = -optical_up/optical_forward
        maximum_error = max(maximum_error, image_v)
        assert image_v < math.tan(1.74/2.)*480./640.
        # .33 is the reference height; actuator lag may undershoot it. The
        # unchanged production sea barrier protects the .07 m reserve.
        assert -p[2] >= flight.reserve_clearance
        if distance <= .5:
            break
        reference, vz = capture_reference(p, q, w)
        out = controller.step(10.+t, dt, p[:2], v[:2], q[:2], w[:2], 10.+t-.04,
                              0., image_v, approach_distance=reference)
        assert out.valid
        assert np.linalg.norm(out.acceleration) <= 2.+1e-10
        assert np.linalg.norm(out.velocity) <= 6.2+1e-10
        command = flight._velocity_command(
            FlightKinematicState(tuple(p), tuple(v)), (*out.velocity, vz), dt, True, True)
        v += (np.array(command.velocity)-v)*dt/.25
        p += v*dt
    assert distance <= .5
    assert t < 60.
    if heave_phase == 0.:
        assert maximum_error < .7
