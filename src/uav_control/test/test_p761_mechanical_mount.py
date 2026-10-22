"""Front camera candidates need real mass, support and collision geometry."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from uav_control.common.p76_contract import load_contract, parameter_bindings
from uav_control.common.p76_model import prepare_model, verify_model

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('x', (.35, .45, .55))
def test_front_candidate_extrinsics_keep_physical_model_and_px4_reference_separate(x):
    contract = deepcopy(load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'))
    contract['camera']['translation_flu'] = [x, 0., .55]
    bindings = parameter_bindings(contract)
    assert bindings['trajectory_tracker_node']['p7_camera_translation_flu'] == pytest.approx(
        [x, 0., .31])


def test_session_model_has_connected_massive_camera_support_and_housing_collision(tmp_path):
    contract = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    metadata = prepare_model(contract['dynamics']['source_camera_model'],
                             contract['dynamics']['px4_model_root'], tmp_path/'models', contract)
    tree = ET.parse(tmp_path/'models/x500_mono_cam/model.sdf')
    camera = tree.find(".//link[@name='front_camera_link']")
    support = tree.find(".//link[@name='p761_camera_support']")
    assert camera.find('collision/geometry/box/size') is not None
    assert support is not None
    assert float(support.findtext('inertial/mass')) > 0
    assert support.find('collision/geometry/box/size') is not None
    assert tree.find(".//joint[@name='p761_camera_support_joint']/parent").text == 'base_link'
    assert metadata['total_mass'] > 2.1643076923
    assert np.isclose(metadata['static_model_twr'], 3.)
    assert any(c['link'] == 'p761_camera_support'
               for c in metadata['collision_envelope']['collisions'])
    assert verify_model(tmp_path/'models', contract)['consistent']
