"""Offline comparison contracts using the same pure optimizer and P2 scenario generator."""

from importlib import import_module
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[3] / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
experiment = import_module('follow_minco_experiment')


def test_all_initializers_share_input_boundary_and_fixed_total_horizon():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    request = experiment.scenario('constant_velocity')
    records = experiment.compare(request, FollowMpcSeed(MpcConfig(
        allow_synthetic_predictions=True)), variants=('fixed',))
    assert {r['method'] for r in records} == {'simple', 'greedy', 'mpc'}
    assert len({r['request_sha256'] for r in records}) == 1
    assert len({tuple(r['common_endpoint']) for r in records}) == 1
    assert all(sum(r['seed']['durations']) == pytest.approx(2.4) for r in records)
    assert all(not r['accepted_by_tracker'] for r in records)


def test_epoch_mismatch_survives_reused_scenario_generation():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    from uav_control.guidance.follow_problem import FollowProblem
    with pytest.raises(ValueError):
        FollowProblem(experiment.scenario('epoch_mismatch'),
                      FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True)))


def test_deceleration_fixture_preserves_derivative_on_both_sides():
    request = experiment.scenario('deceleration')
    assert request.target_velocities[0][0] == 3.
    assert request.target_positions[5][0] == pytest.approx(3.)
    assert request.target_positions[10][0] == pytest.approx(4.)
    assert request.target_velocities[-1][0] == 1.
