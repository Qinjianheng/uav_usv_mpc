"""Review regressions use original native arming epochs and full physical speed bounds."""
import json
from types import SimpleNamespace as Row

import numpy as np
import pytest

from test_p764_initialization import runtime_fixture, advance


def test_genuine_post_arm_seed_can_arrive_before_the_control_tick():
    runtime = runtime_fixture()
    advance(runtime)
    runtime.now += .005
    runtime.node.vehicle_armed = True
    runtime.land_observation.update(status_receipt=runtime.now-.002, status_native_wire=101,
                                    armed_time_native=10000000, disarmed=False)
    runtime.receive_thrust(Row(data=json.dumps(dict(
        stamp=runtime.now-.001, native_timestamp=10009000, px4_timestamp=10009000,
        xyz=[0., 0., -.49]))))
    runtime.px4_mode = runtime.now-.001, dict(
        offboard=True, rates=True, position=False, velocity=False, attitude=False,
        altitude=False, climb_rate=False)
    runtime.tick()
    assert runtime.control_state == 'BODYRATE_READY'
    assert runtime.initialization_count == 1
    assert runtime.outputs[-1]['thrust'] == -.49 and not runtime.lands


def test_hover_admission_checks_combined_horizontal_and_descent_speed():
    runtime = runtime_fixture()
    with pytest.raises(ValueError, match='HOVER_RAMP_SPEED_INFEASIBLE'):
        runtime.outer.hover_initialization_margin(
            [0., 0., -8.], [6.99, 0., 0.], np.eye(3), -.45, .02, .005,
            runtime.controller.barrier)


def test_freshly_delivered_pre_arm_native_seed_cannot_initialize_flight():
    runtime = runtime_fixture()
    advance(runtime)
    runtime.now += .005
    runtime.node.vehicle_armed = True
    runtime.land_observation.update(status_receipt=runtime.now-.002, status_native_wire=101,
                                    armed_time_native=10010000, disarmed=False)
    runtime.receive_thrust(Row(data=json.dumps(dict(
        stamp=runtime.now-.001, native_timestamp=10009000, px4_timestamp=10009000,
        xyz=[0., 0., -.49]))))
    runtime.px4_mode = runtime.now-.001, dict(
        offboard=True, rates=True, position=False, velocity=False, attitude=False,
        altitude=False, climb_rate=False)
    runtime.tick()
    assert runtime.control_state == 'LAND_REQUESTED'
    assert runtime.initialization_count == 0
