"""Prepare and verify isolated P7.6 camera/propulsion models; preserve all source SDFs."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

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
