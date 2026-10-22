"""SDF-derived model-origin envelope; no navigation or target truth correction."""
import hashlib
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def _transform(pose):
    values = np.zeros(6) if pose is None else np.asarray(list(map(float, pose.text.split())))
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('P76_COLLISION_POSE')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def base_reference_flu(root):
    """Resolve the actual navigation body reference in model FLU, never fit an offset."""
    model = ET.parse(Path(root)/'x500_base/model.sdf').find('model')
    link = model.find("link[@name='base_link']")
    if link is None:
        raise ValueError('P76_NAVIGATION_REFERENCE_LINK')
    pose = link.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('P76_NAVIGATION_REFERENCE_FRAME')
    return _transform(model.find('pose'))@_transform(pose)


def navigation_camera_translation(contract):
    """Convert model-origin camera mounting to the actual PX4 base-link pose reference."""
    body = base_reference_flu(contract['dynamics']['px4_model_root'])
    if not np.allclose(body[:3, :3], np.eye(3)):
        raise ValueError('P76_NAVIGATION_REFERENCE_ROTATION')
    return (np.asarray(contract['camera']['translation_flu'])-body[:3, 3]).tolist()


def navigation_vertices(envelope, model_root):
    """Express model-origin vertices about PX4's base_link using the SDF transform."""
    body = base_reference_flu(model_root)
    flip = np.diag([1., -1., -1.])
    points = np.asarray(envelope['vertices_frd'])@flip
    local = (points-body[:3, 3])@body[:3, :3]
    return local@flip


def collision_envelope(root):
    """
    Resolve the pinned merged x500 composite, including model poses and spinning rotors.

    Revolute rotor boxes use a circumscribed 16-sided swept disk, so unknown
    motor yaw cannot shrink the envelope. Unsupported geometry/frames fail closed.
    Only actual collision tags enter the envelope, including P761's physical mount.
    """
    root = Path(root)
    frames, pending, provenance, points, boxes = {}, {}, [], [], []
    models = []
    for name in ('x500_base', 'x500', 'x500_mono_cam'):
        path = root/name/'model.sdf'
        model = ET.parse(path).find('model')
        model_frame = '__'+name
        frames[model_frame] = _transform(model.find('pose'))
        for include in model.findall('include'):
            uri = include.findtext('uri', '').removeprefix('model://')
            if (include.get('merge') != 'true' or uri not in ('x500', 'x500_base')
                    or include.find('pose') is not None):
                raise ValueError('P76_COLLISION_INCLUDE')
        for link in model.findall('link'):
            if link.get('name') in pending:
                raise ValueError('P76_COLLISION_DUPLICATE_FRAME')
            pose = link.find('pose')
            parent = pose.get('relative_to', model_frame) if pose is not None else model_frame
            pending[link.get('name')] = parent, _transform(pose)
        models.append((path, model))
    while pending:
        resolved = [name for name, (parent, _) in pending.items() if parent in frames]
        if not resolved:
            raise ValueError('P76_COLLISION_UNKNOWN_FRAME')
        for name in resolved:
            parent, transform = pending.pop(name)
            frames[name] = frames[parent]@transform
    for path, model in models:
        rotors = {j.findtext('child') for j in model.findall('joint')
                  if j.get('type') == 'revolute'}
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size_node = collision.find('geometry/box/size')
                if size_node is None:
                    raise ValueError('P76_COLLISION_UNSUPPORTED_SHAPE')
                size = np.asarray(list(map(float, size_node.text.split())))
                if size.shape != (3,) or not np.isfinite(size).all() or np.min(size) <= 0:
                    raise ValueError('P76_COLLISION_SIZE')
                pose = collision.find('pose')
                parent = pose.get('relative_to', link.get('name')) if pose is not None \
                    else link.get('name')
                if parent not in frames:
                    raise ValueError('P76_COLLISION_UNKNOWN_FRAME')
                transform = frames[parent]@_transform(pose)
                local = np.array(list(itertools.product(*[(-s/2, s/2) for s in size])))
                swept = link.get('name') in rotors
                if swept:
                    # Pinned rotor joints rotate about local +Z through their link origins.
                    joint = next(j for j in model.findall('joint')
                                 if j.findtext('child') == link.get('name'))
                    axis = np.asarray(list(map(float, joint.findtext('axis/xyz').split())))
                    if (not np.allclose(axis, [0, 0, 1]) or parent != link.get('name')
                            or not np.allclose(_transform(pose), np.eye(4))):
                        raise ValueError('P76_COLLISION_ROTOR_AXIS')
                    radius = np.linalg.norm(size[:2]/2)/np.cos(np.pi/16)
                    local = np.array([(radius*np.cos(a), radius*np.sin(a), z)
                                      for a in np.arange(16)*2*np.pi/16
                                      for z in (-size[2]/2, size[2]/2)])
                flu = local@transform[:3, :3].T+transform[:3, 3]
                points.extend((flu*np.array([1., -1., -1.])).tolist())
                bound = [2*radius, 2*radius, size[2]] if swept else size.tolist()
                boxes.append(dict(link=link.get('name'), size=bound,
                                  center_flu=transform[:3, 3].tolist(),
                                  rotation_flu=transform[:3, :3].tolist(), swept_rotor=swept))
                provenance.append(dict(file=str(path.relative_to(root)), link=link.get('name'),
                                       collision=collision.get('name'), swept_rotor=swept))
    if not points:
        raise ValueError('P76_COLLISION_EMPTY')
    return dict(frame='MODEL_ORIGIN_FRD', vertices_frd=points, collisions=provenance, boxes=boxes,
                source_sha256={str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p, _ in models})


def lowest_world_z(vertices_frd, position_ned, rotation_frd_to_ned):
    """Return NED Z of the lowest physical point using the complete attitude."""
    vertices, position, rotation = map(
        np.asarray, (vertices_frd, position_ned, rotation_frd_to_ned))
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices)
            or position.shape != (3,) or rotation.shape != (3, 3)
            or not all(np.isfinite(v).all() for v in (vertices, position, rotation))
            or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)):
        raise ValueError('P76_COLLISION_WORLD_POSE')
    return float(position[2]+np.max((rotation@vertices.T)[2]))
