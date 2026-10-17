"""Independent centered differences check complete adjoint Q/T/yaw gradients."""
from dataclasses import replace
import numpy as np
import pytest

from test_follow_revalidation import inputs
from test_progress_follow import model
from uav_control.controllers.short_follow_solver import local_seed
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance import p43_minco_objective as objective


@pytest.mark.parametrize('mode', ('q', 'qt'))
def test_full_tracking_and_mount_proxy_gradients_match_independent_differences(mode):
    req, _, _ = inputs()
    seed = local_seed(req, 1.2, np.array((.1, -.15, 0.)), .1, 2.4)
    f = objective.TrackingMincoObjective(FollowProblem(req, model(), 1.2), seed,
                                         MincoConfig(mode=mode), beta=0.)
    x = f.initial()
    x[:6] += np.array((.01, -.01, .02, -.02, .01, -.01))
    cost, analytic = f(x)
    numerical = []
    for i in range(len(x)):
        dx = np.zeros_like(x)
        dx[i] = 2e-6
        numerical.append((f(x+dx)[0]-f(x-dx)[0])/(4e-6))
    assert np.isfinite(cost)
    assert np.allclose(analytic, numerical, rtol=3e-3, atol=3e-4)
    assert f.counts['adjoint_solves'] > 0


def test_proxy_uses_mount_translation_and_28_degree_tilt():
    req, _, _ = inputs()
    m = model()
    seed = local_seed(req, 1.2, np.zeros(3), 0., 2.4)
    f = objective.TrackingMincoObjective(FollowProblem(req, m, 1.2), seed, MincoConfig(mode='q'))
    # Independent optical-axis construction: FRD camera origin + known downward ray.
    angle = np.deg2rad(28.)
    origin = np.array((.18, 0., -.39))
    target = origin+5*np.array((np.cos(angle), 0., np.sin(angle)))
    target -= np.asarray(m.target.center_offset_ned)
    optical, *_ = f.geometry(np.zeros((1, 3)), np.zeros((1, 3)), np.zeros(1), target[None, :])
    assert np.allclose(optical, ((0., 0., 5.),), atol=1e-10)
    shifted = replace(m.extrinsics, translation_flu=(0., 0., 0.))
    m.extrinsics = shifted
    g = objective.TrackingMincoObjective(FollowProblem(req, m, 1.2), seed, MincoConfig(mode='q'))
    displaced = g.geometry(np.zeros((1, 3)), np.zeros((1, 3)), np.zeros(1), target[None, :])[0]
    assert not np.allclose(displaced, optical)


def test_local_partials_use_one_batched_proxy_evaluation():
    req, _, _ = inputs()
    seed = local_seed(req, 1.2, np.zeros(3), 0., 2.4)
    f = objective.TrackingMincoObjective(FollowProblem(req, model(), 1.2), seed,
                                         MincoConfig(mode='q'))
    f(f.initial())
    assert f.counts['batch_geometry'] == 2  # primal + batched local partials
