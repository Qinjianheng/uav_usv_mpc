#!/usr/bin/env python3
"""Explicit synthetic probes; never counted as Gazebo or flight proof."""
from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np
from scipy.integrate import cumulative_trapezoid

from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcRequest, PlanningContext
from uav_control.controllers.p43_follow_solver import P43FollowSolver
from uav_control.guidance.follow_limits import ResearchConfig
from uav_control.guidance.follow_problem import future_request
from uav_control.guidance.camera_visibility_batch import attitude_batch, visibility_batch


def request(scene):
    """Construct an analytic target, explicit forecast and synchronous measured start."""
    t = np.linspace(0., 4., 81)
    dense = np.linspace(0., 4., 4001)
    if scene in ('constant_turn', 'variable_turn'):
        h = .2*dense if scene == 'constant_turn' else .15*dense+.08*(1-np.cos(dense))
        vx, vy = 4*np.cos(h), 4*np.sin(h)
        x = 5+cumulative_trapezoid(vx, dense, initial=0)
        y = cumulative_trapezoid(vy, dense, initial=0)
        p = np.c_[np.interp(t, dense, x), np.interp(t, dense, y), np.zeros(len(t))]
        v = np.c_[np.interp(t, dense, vx), np.interp(t, dense, vy), np.zeros(len(t))]
    else:
        x = 5+4*t+.3*np.sin(2*t) if scene == 'accel_decel' else 5+4*t
        vx = 4+.6*np.cos(2*t) if scene == 'accel_decel' else np.full(len(t), 4.)
        p = np.c_[x, np.zeros(len(t)), np.zeros(len(t))]
        v = np.c_[vx, np.zeros(len(t)), np.zeros(len(t))]
    truth = p.copy()
    if scene == 'fov_edge':
        p[:, 1] += 6.
        truth = p.copy()
    if scene == 'wrong_prediction':
        p[:, 1] += np.where((t > .4) & (t < 1.2),
                            1.2*np.sin(np.pi*(t-.4)/.8)**2, 0.)
    c = PlanningContext(1, 1, 100., 100., 100., 100., 100., 1, 100.125,
                        prediction_source='synthetic')
    state = (0., 0., -5., float(v[0, 0]), 0., 0., 0., 0., 0., 0.)
    req = MpcRequest(c, state, tuple(t), tuple(map(tuple, p)), tuple(map(tuple, v)),
                     tuple(map(tuple, np.eye(3))), 100.)
    return future_request(req, .15), truth


def run(output):
    """Save synthetic requests and directional-error sensitivity at each horizon."""
    model = FollowMpcSeed(ResearchConfig(allow_synthetic_predictions=True))
    rows = []
    scenes = ('line', 'constant_turn', 'variable_turn', 'accel_decel',
              'fov_edge', 'wrong_prediction')
    for scene in scenes:
        for horizon in (1.2, 1.6):
            req, truth = request(scene)
            result = P43FollowSolver(model, duration=horizon, refinement='none',
                                     wall_clock=lambda: req.now_stamp).solve(req)
            sensitivity = []
            if result.valid:
                q = np.asarray(result.relative_times)
                target = np.column_stack([
                    np.interp(q+.15, req.prediction_times,
                              np.asarray(req.target_positions)[:, i]) for i in range(3)])
                attitudes = attitude_batch(result.accelerations, result.yaw_refs,
                                           model.attitude_config)
                rotations = np.array([a.rotation_frd_to_ned for a in attitudes])
                for error in (0., .2, .4, .8):
                    for direction in ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)):
                        perturbed = target+error*np.asarray(direction)
                        views = visibility_batch(
                            result.positions, rotations, perturbed, model.intrinsics,
                            model.extrinsics, model.target, model.visibility)
                        sensitivity.append(dict(
                            error_m=error, direction=direction,
                            safe_fraction=float(np.mean([v.whole_target_safe for v in views]))))
            rows.append(dict(scene=scene, horizon=horizon, request=asdict(req),
                             result=asdict(result), synthetic_truth=truth.tolist(),
                             sensitivity=sensitivity,
                             empirical_error_is_guaranteed_bound=False, actual_flight=False))
    output.write_text(json.dumps(rows, indent=2, allow_nan=False))


if __name__ == '__main__':
    run(Path(sys.argv[1]))
