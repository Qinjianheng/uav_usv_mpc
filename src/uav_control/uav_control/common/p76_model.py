"""Prepare and verify isolated P7.6 camera/propulsion models; preserve all source SDFs."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation

from .p76_body_geometry import collision_envelope


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _pose(element):
    values = [float(v) for v in element.text.split()] if element is not None else [0.]*6
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('P76_MODEL_POSE')
    return np.asarray(values)


def _base_pose(tree):
    # x500_base carries its +.24m in model/pose, not link/pose. Merged
    # includes retain that transform when resolving the base_link frame.
    parent = _pose(tree.find('./model/pose'))
    child = _pose(tree.find(".//link[@name='base_link']/pose"))
    rotation = Rotation.from_euler('xyz', parent[3:])
    return np.r_[parent[:3]+rotation.apply(child[:3]),
                 (rotation*Rotation.from_euler('xyz', child[3:])).as_euler('xyz')]


def _install_mount(tree, camera, contract, body, body_rotation, camera_rotation):
    """Add a connected carbon tube and a rear housing, with actual inertia and collisions."""
    mechanical = contract['mechanics']
    size = np.asarray(mechanical['housing_size'])
    center = np.asarray(mechanical['housing_center_camera_flu'])
    visual = camera.find('visual')
    visual.find('geometry/box/size').text = ' '.join(map(str, size))
    pose = visual.find('pose')
    if pose is None:
        pose = ET.SubElement(visual, 'pose')
    pose.text = ' '.join(map(str, (*center, 0., 0., 0.)))
    housing = ET.SubElement(camera, 'collision', name='p761_camera_housing')
    ET.SubElement(housing, 'pose').text = pose.text
    housing.append(deepcopy(visual.find('geometry')))
    camera_inertial = camera.find('inertial')
    inertial_pose = camera_inertial.find('pose')
    if inertial_pose is None:
        inertial_pose = ET.SubElement(camera_inertial, 'pose')
    inertial_pose.text = pose.text
    camera_mass = float(camera_inertial.findtext('mass'))
    for index, axis in enumerate(('ixx', 'iyy', 'izz')):
        camera_inertial.find('inertia/'+axis).text = repr(float(
            camera_mass*(np.dot(size, size)-size[index]**2)/12))
    start = np.asarray(mechanical['support_root_flu'])
    end = np.asarray(contract['camera']['translation_flu'])+camera_rotation@(
        center-np.array([size[0]/2, 0., 0.]))
    direction = end-start
    length = float(np.linalg.norm(direction))
    outer, inner = mechanical['support_outer_width'], mechanical['support_inner_width']
    if not 0 < inner < outer or length <= outer:
        raise ValueError('P761_CAMERA_SUPPORT_DIMENSIONS')
    mass = mechanical['support_density']*(outer**2-inner**2)*length
    rotation = Rotation.from_euler('xyz', (0., -math.atan2(direction[2],
                                                           np.linalg.norm(direction[:2])),
                                           math.atan2(direction[1], direction[0]))).as_matrix()
    angles = Rotation.from_matrix(body_rotation.T@rotation).as_euler('xyz')
    position = body_rotation.T@((start+end)/2-body[:3])
    link = ET.SubElement(tree.find('model'), 'link', name='p761_camera_support')
    ET.SubElement(link, 'pose', relative_to='base_link').text = ' '.join(
        repr(float(v)) for v in (*position, *angles))
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = repr(mass)
    inertia = ET.SubElement(inertial, 'inertia')
    # Hollow square tube, local X along its length.
    axial = mass*(outer**2+inner**2)/6
    transverse = mass*(length**2+outer**2+inner**2)/12
    for name, value in zip(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'),
                           (axial, transverse, transverse, 0., 0., 0.)):
        ET.SubElement(inertia, name).text = repr(value)
    for tag in ('visual', 'collision'):
        item = ET.SubElement(link, tag, name='p761_support_'+tag)
        geometry = ET.SubElement(item, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = f'{length} {outer} {outer}'
    joint = ET.SubElement(tree.find('model'), 'joint',
                          name='p761_camera_support_joint', type='fixed')
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'p761_camera_support'


def prepare_model(source_camera, px_models, output, contract):
    """Copy the full composite, derive thrust and transform model-origin camera calibration."""
    source_camera, px_models, output = map(Path, (source_camera, px_models, output))
    output.mkdir(parents=True, exist_ok=False)
    source_files = []
    for name, source in (('x500_mono_cam', source_camera), ('x500', px_models/'x500'),
                         ('x500_base', px_models/'x500_base')):
        if output.resolve() == source.resolve() or source.resolve() in output.resolve().parents:
            raise ValueError('P76_MODEL_SEPARATE_OUTPUT')
        shutil.copytree(source, output/name)
        source_files.append(source/'model.sdf')
    base = ET.parse(output/'x500_base/model.sdf')
    body = _base_pose(base)
    body_rotation = Rotation.from_euler('xyz', body[3:]).as_matrix()
    camera_tree = ET.parse(output/'x500_mono_cam/model.sdf')
    camera = camera_tree.find(".//link[@name='front_camera_link']")
    desired_rotation = Rotation.from_euler('y', math.radians(
        contract['camera']['pitch_down_degrees'])).as_matrix()
    relative_translation = body_rotation.T@(np.asarray(contract['camera']['translation_flu'])
                                            - body[:3])
    relative_angles = Rotation.from_matrix(body_rotation.T@desired_rotation).as_euler('xyz')
    camera.find('pose').text = ' '.join(repr(float(v)) for v in (
        *relative_translation, *relative_angles))
    sensor = camera.find("sensor[@name='front_tof_camera']")
    sensor.find('camera/horizontal_fov').text = repr(contract['camera']['horizontal_fov'])
    sensor.find('update_rate').text = repr(contract['paper']['camera_rate_hz'])
    for key, value in zip(('width', 'height'), (contract['camera']['width'],
                                                contract['camera']['height'])):
        sensor.find('camera/image/'+key).text = str(value)
    down = camera_tree.find(".//sensor[@name='down_tof_camera']")
    if down is not None:
        camera_tree.find(".//link[@name='down_camera_link']").remove(down)
    _install_mount(camera_tree, camera, contract, body, body_rotation, desired_rotation)
    camera_tree.write(output/'x500_mono_cam/model.sdf', encoding='utf-8', xml_declaration=True)
    mass = sum(float(m.text) for name in ('x500_mono_cam', 'x500', 'x500_base')
               for m in ET.parse(output/name/'model.sdf').findall('.//link/inertial/mass'))
    motor_tree = ET.parse(output/'x500/model.sdf')
    motors = [p for p in motor_tree.findall('.//plugin') if p.find('motorConstant') is not None]
    if len(motors) != 4 or mass <= 0:
        raise ValueError('P76_MODEL_COMPOSITION')
    gravity = contract['dynamics']['gravity']
    twr = contract['paper']['paper_thrust_to_weight_target']
    maximum = twr*mass*gravity
    coefficient = maximum/sum(float(p.findtext('maxRotVelocity'))**2 for p in motors)
    for motor in motors:
        motor.find('motorConstant').text = repr(coefficient)
    motor_tree.write(output/'x500/model.sdf', encoding='utf-8', xml_declaration=True)
    metadata = dict(scope='Model-derived candidate, not actual flight/throttle qualification.',
                    total_mass=mass, gravity=gravity, maximum_thrust=maximum,
                    static_model_twr=twr, derived_motor_constant=coefficient,
                    camera_model_translation=contract['camera']['translation_flu'],
                    camera_base_translation=list(relative_translation),
                    collision_envelope=collision_envelope(output),
                    source_sha256={str(p): _sha(p) for p in source_files},
                    files={str(p.relative_to(output)): _sha(p)
                           for p in sorted(output.rglob('*')) if p.is_file()})
    (output/'p76_model_manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def verify_model(output, contract):
    """Read actual model files and reject altered calibration, propulsion or file provenance."""
    output = Path(output)
    metadata = json.loads((output/'p76_model_manifest.json').read_text())
    if any(not Path(name).is_file() or _sha(name) != sha
           for name, sha in metadata['source_sha256'].items()):
        raise ValueError('P76_MODEL_SOURCE_DRIFT')
    if metadata.get('collision_envelope') != collision_envelope(output):
        raise ValueError('P76_MODEL_COLLISION_ENVELOPE')
    if any(not (output/name).is_file() or _sha(output/name) != sha
           for name, sha in metadata['files'].items()):
        raise ValueError('P76_MODEL_FILE_HASH')
    base = ET.parse(output/'x500_base/model.sdf')
    body = _base_pose(base)
    camera = ET.parse(output/'x500_mono_cam/model.sdf')
    relative = _pose(camera.find(".//link[@name='front_camera_link']/pose"))
    body_rotation = Rotation.from_euler('xyz', body[3:]).as_matrix()
    model_translation = body[:3]+body_rotation@relative[:3]
    expected_rotation = Rotation.from_euler('y', math.radians(
        contract['camera']['pitch_down_degrees'])).as_matrix()
    actual_rotation = body_rotation@Rotation.from_euler('xyz', relative[3:]).as_matrix()
    if (not np.allclose(model_translation, contract['camera']['translation_flu'], atol=1e-10)
            or not np.allclose(expected_rotation, actual_rotation, atol=1e-10)):
        raise ValueError('P76_MODEL_CAMERA_CALIBRATION')
    sensor = camera.find(".//sensor[@name='front_tof_camera']")
    values = {'camera/horizontal_fov': contract['camera']['horizontal_fov'],
              'camera/image/width': contract['camera']['width'],
              'camera/image/height': contract['camera']['height'],
              'update_rate': contract['paper']['camera_rate_hz']}
    for key, value in values.items():
        if float(sensor.findtext(key)) != value:
            raise ValueError('P76_MODEL_INTRINSICS')
    for branch, clips in (('camera/clip', contract['camera']['rgb_clip']),
                          ('camera/depth_camera/clip', contract['camera']['depth_clip'])):
        if any(float(sensor.findtext(branch+'/'+key)) != expected
               for key, expected in zip(('near', 'far'), clips)):
            raise ValueError('P76_MODEL_CLIP')
    motor_tree = ET.parse(output/'x500/model.sdf')
    maximum = sum(float(p.findtext('motorConstant'))*float(p.findtext('maxRotVelocity'))**2
                  for p in motor_tree.findall('.//plugin') if p.find('motorConstant') is not None)
    twr = maximum/(metadata['total_mass']*contract['dynamics']['gravity'])
    if not math.isclose(twr, contract['paper']['paper_thrust_to_weight_target'], abs_tol=1e-10):
        raise ValueError('P76_MODEL_TWR')
    return dict(consistent=True, flight_qualified=False, total_mass=metadata['total_mass'],
                static_model_twr=twr, camera_model_translation=model_translation.tolist())
