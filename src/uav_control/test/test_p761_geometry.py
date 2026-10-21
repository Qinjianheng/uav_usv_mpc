"""Independent ray/box and native-radius witnesses for finite mount screening."""
import numpy as np
import pytest


def test_obstacle_entry_is_in_front_of_sphere_and_parallel_rays_miss():
    from uav_control.common import p761_geometry
    box = dict(center_flu=[2., 0., 0.], rotation_flu=np.eye(3).tolist(), size=[1., 1., 1.])
    entries = p761_geometry.ray_box_entry([0., 0., 0.], [[1., 0., 0.], [0., 1., 0.]], box)
    assert entries[0] == pytest.approx(1.5)
    assert np.isinf(entries[1])


def test_terminal_path_has_native_capture_and_positive_body_clearance_at_all_heaves():
    from uav_control.common import p761_geometry
    from uav_control.common.p76_contract import load_contract
    from pathlib import Path
    contract = load_contract(Path(__file__).resolve().parents[3] /
                             'src/uav_usv_bringup/config/p76_contract.yaml')
    cases = list(p761_geometry.geometry_cases(contract))
    assert len(cases) == 33
    for heave in (-.15, 0., .15):
        terminal = [c for c in cases if c['case']['phase'] == 'TERMINAL'
                    and c['case']['heave'] == heave]
        end = terminal[-1]
        assert end['case']['origin_distance'] < .25
        assert end['origin'][2]+.013 > .02
