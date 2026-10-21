"""Use model-origin collision geometry instead of the inherited origin-altitude floor."""
import numpy as np
import pytest

from scipy.spatial.transform import Rotation

from uav_control.common.p76_body_geometry import (
    collision_envelope, lowest_world_z, navigation_vertices,
)
from uav_control.control.p76_bodyrate_safety import BodyrateSeaBarrier
from uav_control.evaluation.intercept_evaluator import InterceptEvaluatorCore, KinematicState


def test_nested_model_pose_is_included_in_physical_foot_height(tmp_path):
    for name in ('x500_base', 'x500', 'x500_mono_cam'):
        (tmp_path/name).mkdir()
    (tmp_path/'x500_base/model.sdf').write_text(
        '<sdf><model name="x500_base"><pose>0 0 .24 0 0 0</pose>'
        '<link name="base_link"><collision name="foot"><pose>0 0 -.22 0 0 0</pose>'
        '<geometry><box><size>.2 .1 .02</size></box></geometry></collision></link></model></sdf>')
    for name in ('x500', 'x500_mono_cam'):
        (tmp_path/name/'model.sdf').write_text(f'<sdf><model name="{name}"/></sdf>')
    geometry = collision_envelope(tmp_path)
    vertices = np.asarray(geometry['vertices_frd'])
    assert np.max(vertices[:, 2]) == pytest.approx(-.01)
    barrier = BodyrateSeaBarrier(vertices, .02, .1, 4.)
    assert barrier.check([0., 0., -.05], [0., 0., 0.], np.eye(3)) == 'CLEAR'
    # Independent frame witness: shifting both navigation origin and body
    # reference must preserve actual clearance even with roll, pitch and yaw.
    rotation = Rotation.from_euler('xyz', [.3, -.2, .7]).as_matrix()
    model_position = np.array([5., 6., -1.])
    navigation_position = model_position+rotation@np.array([0., 0., -.24])+[0., 0., 4.]
    model_lowest = lowest_world_z(vertices, model_position, rotation)
    native_lowest = lowest_world_z(navigation_vertices(geometry, tmp_path),
                                   navigation_position, rotation)
    assert 4.-native_lowest == pytest.approx(-model_lowest)


def test_clearance_is_from_real_lowest_point_not_uav_origin():
    barrier = BodyrateSeaBarrier([[0., 0., .1]], .02, .1, 4.)
    assert barrier.check([0., 0., -.2], [0., 0., 0.], np.eye(3)) == 'CLEAR'
    assert barrier.check([0., 0., -.11], [0., 0., 0.], np.eye(3)) == 'UNRECOVERABLE'


def test_native_evaluation_uses_measured_lowest_point_and_still_fails_water_contact():
    evaluator = InterceptEvaluatorCore(capture_radius=.25, body_lower_extent=.23)
    evaluator.begin(1, 1.)
    uav = KinematicState((0., 0., -.05), (0., 0., 0.))
    target = KinematicState((0., 0., .15), (0., 0., 0.))
    result = evaluator.update(1.01, uav, target, capture_segment=False,
                              sea_segment=False, body_lowest_z=-.06)
    assert result.outcome == 'SUCCESS'
    evaluator.begin(2, 2.)
    result = evaluator.update(2.01, uav, target, capture_segment=False,
                              sea_segment=False, body_lowest_z=.001)
    assert result.reason == 'SEA_CONTACT'
