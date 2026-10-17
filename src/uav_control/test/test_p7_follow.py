"""P7 restoration: real BCTRA series, acquisition TTL and exclusive launch selection."""

import ast
import math
from pathlib import Path

import pytest


def test_r11_bctra_restores_position_velocity_acceleration_and_acquisition_ttl():
    from uav_control.tracking.target_prediction import PredictionEngine, TargetKinematicState
    engine = PredictionEngine()
    assert engine.update(TargetKinematicState(10., (8., 0., 0.), (2., 0., 0.), 9.96), now=10.)
    result = engine.generate(10.01, 3, 7)
    assert result.valid and result.model == 'BCTRA_BOUNDED_Z'
    assert (result.mission_id, result.sequence_id) == (3, 7)
    assert result.source_stamp == 10. and result.observation_stamp == 9.96
    assert result.valid_until == pytest.approx(10.085)
    assert len(result.samples) == 41
    assert result.samples[-1].relative_time == 4.
    assert result.samples[-1].position == pytest.approx((16., 0., 0.))
    assert result.samples[-1].velocity == pytest.approx((2., 0., 0.))
    assert result.samples[-1].acceleration == pytest.approx((0., 0., 0.))
    assert not engine.generate(10.086, 3, 8).valid


def test_kf_projection_does_not_create_another_velocity_observation_or_refresh_ttl():
    from uav_control.tracking.target_prediction import PredictionEngine, TargetKinematicState
    engine = PredictionEngine()
    engine.update(TargetKinematicState(10., (8., 0., 0.), (2., 0., 0.), 9.96), now=10.)
    engine.update(TargetKinematicState(10.05, (8.1, 0., 0.), (2., 0., 0.), 9.96), now=10.05)
    assert engine.last_velocity_observation_stamp == 9.96
    assert engine.generate(10.05, 1, 1).valid_until == pytest.approx(10.085)
    assert not engine.generate(10.086, 1, 2).valid


@pytest.mark.parametrize('stamp,observed,now', [
    (10., 10.01, 10.02), (10.1, 10., 10.), (10., 9.8, 10.),
])
def test_bctra_rejects_invalid_epoch_or_stale_image(stamp, observed, now):
    from uav_control.tracking.target_prediction import PredictionEngine, TargetKinematicState
    engine = PredictionEngine()
    state = TargetKinematicState(stamp, (8., 0., 0.), (2., 0., 0.), observed)
    assert not engine.update(state, now=now)
    assert not engine.generate(now, 1, 1).valid


def test_bctra_turning_series_has_finite_bounded_kinematics():
    from uav_control.tracking.target_prediction import PredictionEngine, TargetKinematicState
    engine = PredictionEngine()
    for i in range(8):
        t = 10. + .1 * i
        engine.update(TargetKinematicState(t, (8., 0., 0.),
                                           (2. * math.cos(.02*i), 2. * math.sin(.02*i), 0.)), t)
    result = engine.generate(t+.01, 1, 1)
    assert result.valid and result.turn_rate > 0.
    assert result.samples[10].position[1] > 0.
    assert all(math.isfinite(x) for sample in result.samples
               for x in (*sample.position, *sample.velocity, *sample.acceleration))


@pytest.mark.parametrize('model,expected', [
    ('bctra', 'target_predictor_node'), ('imm', 'imm_target_predictor_node'),
])
@pytest.mark.parametrize('shadow', ('true', 'false'))
def test_launch_selects_exactly_one_main_predictor(model, expected, shadow):
    path = Path(__file__).parents[2]/'uav_usv_bringup/launch/modular_intercept.launch.py'
    choices = dict(prediction_model=model, enable_shadow_perception=shadow)
    primary = []
    for call in ast.walk(ast.parse(path.read_text())):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            continue
        if call.func.id != 'Node':
            continue
        fields = {item.arg: item.value for item in call.keywords}
        name = fields['name'].value
        if name not in ('target_predictor_node', 'imm_target_predictor_node'):
            continue
        active = 'condition' not in fields or eval(compile(
            ast.Expression(fields['condition']), str(path), 'eval'), dict(
            IfCondition=lambda value: value, PythonExpression=lambda parts: eval(''.join(parts)),
            LaunchConfiguration=lambda key: choices[key]))
        if active:
            primary.append(name)
    assert primary == [expected]


def test_bctra_wire_preserves_original_large_epoch_timestamps():
    from builtin_interfaces.msg import Time
    from uav_usv_interfaces.msg import TargetState
    from uav_control.tracking.target_prediction import PredictionEngine
    from uav_control.tracking.target_predictor_node import (
        prediction_to_message, state_from_message,
    )
    state = TargetState()
    state.stamp = Time(sec=1792000000, nanosec=987654321)
    state.source_stamp = Time(sec=1792000000, nanosec=923456789)
    state.position.x, state.velocity.x = 8., 2.
    engine = PredictionEngine()
    assert engine.update(state_from_message(state), 1792000001.)
    result = engine.generate(1792000001., 1, 1)
    wire = prediction_to_message(result, 0., source_message=state)
    assert wire.source_stamp == state.stamp
    assert wire.observation_stamp == state.source_stamp
    assert wire.valid_until == Time(sec=1792000001, nanosec=48456789)
