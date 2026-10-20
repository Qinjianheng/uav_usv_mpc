#!/usr/bin/env python3
"""Collect one Gazebo-only, static-pose dataset session; never connect to PX4 or control."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.common.p76_contract import load_contract  # noqa: E402
from uav_control.common.p76_model import prepare_model, verify_model  # noqa: E402
from uav_control.gazebo_target_visualizer import red_sphere_sdf  # noqa: E402


def polygon_label(mask):
    """Offline annotation only: retain the visible silhouette in original image coordinates."""
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_TREE,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return ''
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    polygons = [cv2.approxPolyDP(contour, .5, True)[:, 0, :].astype(float)
                for contour in contours if cv2.contourArea(contour) >= 2]
    if not polygons or len(polygons[0]) < 3:
        return ''
    points = polygons[0]
    # A zero-area out-and-back bridge encodes visible holes/disjoint fragments
    # as a single polygon. Never label an occluding airframe as the red target.
    for other in polygons[1:]:
        distances = np.sum((points[:, None, :]-other[None, :, :])**2, axis=2)
        i, j = np.unravel_index(np.argmin(distances), distances.shape)
        tour = np.r_[other[j:], other[:j+1]]
        points = np.r_[points[:i+1], tour, points[i:i+1], points[i+1:]]
    points[:, 0] /= mask.shape[1]
    points[:, 1] /= mask.shape[0]
    return '0 '+' '.join(f'{v:.8f}' for v in points.ravel())+'\n'


def geometry_cases(contract):
    """Finite primary/backup FOLLOW and terminal samples, with actual mount and heave."""
    follow = contract['follow']
    mount = Rotation.from_euler('y', math.radians(contract['camera']['pitch_down_degrees']))
    for heave in (-.15, 0., .15):
        settings = [(follow['distance'], follow['height'], roll, pitch, 'FOLLOW')
                    for roll, pitch in ((0., 0.), (.25, 0.), (-.25, 0.), (0., .25), (0., -.25))]
        settings += [(distance, height, 0., 0., 'TERMINAL') for distance, height in (
            (3., 2.), (1., 1.), (.5, .5), (.25, .33), (.1, .33), (0., .33))]
        for distance, height, roll, pitch, phase in settings:
            rotation = Rotation.from_euler('xyz', (roll, pitch, 0.))
            origin = np.array((-distance, 0., height))
            ball = np.array((0., 0., .42+heave))
            delta = ball-origin-rotation.apply(contract['camera']['translation_flu'])
            center = mount.inv().apply(rotation.inv().apply(delta))
            yield dict(origin=origin, ball=ball, rotation=rotation, center=center,
                       case=dict(phase=phase, heave=heave, roll=roll, pitch=pitch,
                                 origin_distance=float(np.linalg.norm(origin-(0., 0., heave)))))


def collect(output, contract_path, samples, seed, split, geometry=False, live_yolo=False,
            live_graph=False):
    """Own one bounded rendering process and preserve paired background/target images."""
    from p4_sitl_session import preflight
    check = preflight()
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node
    output.mkdir(parents=True, exist_ok=False)
    for name in ('images', 'labels', 'masks'):
        (output/name).mkdir()
    c = load_contract(contract_path)
    (output/'contract.yaml').write_bytes(contract_path.read_bytes())
    (output/'collector.py').write_bytes(Path(__file__).read_bytes())
    models = output/'models'
    prepare_model(c['dynamics']['source_camera_model'], c['dynamics']['px4_model_root'], models, c)
    verify_model(models, c)
    tree = ET.parse(ROOT/'src/uav_usv_bringup/worlds/ocean.sdf')
    world = tree.find('world')
    # Sensor-only collection does not arm, fly or modify a target motion profile.
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system',
                               name='gz::sim::systems::'+name)
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    world.find('light/intensity').text = str(.75+.1*(seed % 5))
    world_path = output/'dataset_world.sdf'
    tree.write(world_path, encoding='utf-8', xml_declaration=True)
    env = dict(os.environ, GZ_PARTITION=f'p76_dataset_{seed}_{os.getpid()}',
               GZ_SIM_RESOURCE_PATH=str(models)+':'+c['dynamics']['px4_model_root']+':'+str(
                   ROOT/'src/uav_usv_bringup/models'), OPENBLAS_NUM_THREADS='1',
               OMP_NUM_THREADS='1')
    os.environ['GZ_PARTITION'] = env['GZ_PARTITION']
    env['ROS_DOMAIN_ID'] = str(check['domain'])
    lock = threading.Condition()
    latest = dict(sequence=0, image=None, raw_stamp=None)

    def image_callback(message):
        try:
            array = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
            rgb = array[:, :message.width*3].reshape(message.height, message.width, 3).copy()
            stamp = message.header.stamp.sec+message.header.stamp.nsec*1e-9
            with lock:
                latest.update(sequence=latest['sequence']+1, image=rgb, raw_stamp=stamp)
                lock.notify_all()
        except (ValueError, AttributeError):
            return

    node = Node()
    node.subscribe(Image, '/uav/camera/front/image', image_callback)

    def request(service, message):
        executed, response = node.request('/world/default/'+service, message,
                                          type(message), Boolean, 2000)
        if not executed or not response.data:
            raise RuntimeError('P76_DATASET_RPC:'+service)

    def set_pose(name, position, rotation=None):
        pose = Pose()
        pose.name = name
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
        q = Rotation.identity().as_quat() if rotation is None else rotation.as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(
            float, q)
        request('set_pose', pose)

    def frame():
        with lock:
            target = latest['sequence']+3
            if not lock.wait_for(lambda: latest['sequence'] >= target, timeout=4.):
                raise RuntimeError('P76_DATASET_IMAGE_TIMEOUT')
            return latest['image'].copy(), latest['raw_stamp']

    process = None
    children = []
    child_names = []
    records, error = [], None
    began = time.monotonic()
    try:
        process = subprocess.Popen(['gz', 'sim', '-s', '-r', str(world_path)], env=env,
                                   stdout=(output/'gazebo.txt').open('x'),
                                   stderr=subprocess.STDOUT,
                                   start_new_session=True)
        time.sleep(3.)
        camera = ET.parse(models/'x500_mono_cam/model.sdf')
        ET.SubElement(camera.find('model'), 'static').text = 'true'
        factory = EntityFactory()
        factory.name = 'p76_dataset_uav'
        factory.sdf = ET.tostring(camera.getroot(), encoding='unicode')
        request('create', factory)
        target = EntityFactory()
        target.name, target.sdf = 'p76_dataset_ball', red_sphere_sdf('p76_dataset_ball', .5)
        request('create', target)
        if live_yolo:
            commands = [
                ('bridge', ['ros2', 'run', 'ros_gz_image', 'image_bridge',
                            '/uav/camera/front/image', '/uav/camera/front/depth_image',
                            '--ros-args', '-r',
                            '/uav/camera/front/image:=/camera/front/image_raw', '-r',
                            '/uav/camera/front/depth_image:=/camera/front/depth/image_raw']),
                ('observer', ['python3', str(ROOT/'scripts/p76_perception_probe.py'),
                              str(output/'live_probe'), '--duration', '90']),
                ('detector', [c['detector']['runtime_python'], '-m',
                              'uav_control.perception.yolo_target_detector', '--ros-args',
                              '-p', 'contract_file:='+str(contract_path.resolve())])]
            if live_graph:
                profile = ROOT/'src/uav_usv_bringup/config/p76_yan_paper.yaml'
                for executable in ('target_bearing_node', 'front_tof_monitor',
                                   'rgbd_target_localizer', 'spatial_dkf_node', 'image_dkf_node'):
                    commands.append((executable, ['ros2', 'run', 'uav_control', executable,
                                                  '--ros-args', '--params-file', str(profile)]))
            for name, command in commands:
                child_env = dict(env)
                if name == 'detector':
                    child_env['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
                children.append(subprocess.Popen(command, env=child_env, start_new_session=True,
                                                 stdout=(output/(name+'.txt')).open('x'),
                                                 stderr=subprocess.STDOUT))
                child_names.append(name)
            time.sleep(3.)
        frame()
        rng = np.random.default_rng(seed)
        mount = Rotation.from_euler('y', math.radians(c['camera']['pitch_down_degrees']))
        cases = list(geometry_cases(c)) if geometry else None
        if cases is not None:
            samples = len(cases)
        for index in range(samples):
            dead = [(name, child.returncode) for name, child in zip(child_names, children)
                    if child.poll() is not None]
            if dead:
                raise RuntimeError('P76_LIVE_NODE_EXIT:'+json.dumps(dead))
            if time.monotonic()-began > 240.:
                raise RuntimeError('P76_DATASET_WATCHDOG')
            rotation = Rotation.from_euler('xyz', (*rng.uniform(-.25, .25, 2),
                                                   rng.uniform(-math.pi, math.pi)))
            distance = float(np.exp(rng.uniform(math.log(.28), math.log(12.))))
            horizontal, vertical = rng.uniform(-1.05, 1.05), rng.uniform(-1.05, 1.05)
            direction = np.array((math.cos(vertical)*math.cos(horizontal),
                                  math.cos(vertical)*math.sin(horizontal), math.sin(vertical)))
            delta = rotation.apply(mount.apply(distance*direction))
            camera_offset = rotation.apply(c['camera']['translation_flu'])
            origin = np.array((0., 0., max(.4, .42+rng.uniform(-.15, .15)
                                           - delta[2]-camera_offset[2])))
            ball = origin+camera_offset+delta
            case = None
            if cases is not None:
                item = cases[index]
                origin, ball, rotation = item['origin'], item['ball'], item['rotation']
                distance = float(np.linalg.norm(item['center']))
                direction = item['center']/distance
                case = item['case']
            set_pose('p76_dataset_uav', origin, rotation)
            set_pose('p76_dataset_ball', (100., 100., -10.))
            background, _ = frame()
            set_pose('p76_dataset_ball', ball)
            rgb, stamp = frame()
            red = ((rgb[:, :, 0] > 40) & (rgb[:, :, 0] > rgb[:, :, 1]*1.4)
                   & (rgb[:, :, 0] > rgb[:, :, 2]*1.4))
            changed = np.max(np.abs(rgb.astype(np.int16)-background.astype(np.int16)), axis=2) > 15
            mask = red & changed
            label = polygon_label(mask)
            name = f'{index:05d}'
            cv2.imwrite(str(output/'images'/f'{name}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            (output/'labels'/f'{name}.txt').write_text(label)
            np.savez_compressed(output/'masks'/f'{name}.npz', mask=mask)
            records.append(dict(name=name, source='Gazebo RGB paired-background annotation',
                                session=output.name, split=split, seed=seed, raw_stamp=stamp,
                                uav_position=origin.tolist(), target_marker_position=ball.tolist(),
                                quaternion_xyzw=rotation.as_quat().tolist(),
                                camera_relative_center=(distance*direction).tolist(),
                                projected_sphere_center_uv=[float(320.-320./math.tan(
                                    c['camera']['horizontal_fov']/2)*direction[1]/direction[0]),
                                    float(240.-320./math.tan(c['camera']['horizontal_fov']/2)
                                          * direction[2]/direction[0])],
                                visible_label=bool(label), visible_pixels=int(mask.sum()),
                                geometry_case=case))
            if index % 5 == 0:
                name += '_negative'
                cv2.imwrite(str(output/'images'/f'{name}.png'),
                            cv2.cvtColor(background, cv2.COLOR_RGB2BGR))
                (output/'labels'/f'{name}.txt').write_text('')
        (output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    except BaseException as exception:
        error = repr(exception)
        raise
    finally:
        node_exit_before_cleanup = {name: child.poll()
                                    for name, child in zip(child_names, children)}
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5.)
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.)
        manifest = dict(session=output.name, split=split, seed=seed, completed=error is None,
                        samples=len(records), labeled=sum(r['visible_label'] for r in records),
                        error=error, seconds=time.monotonic()-began,
                        live_nodes_exit_before_cleanup=node_exit_before_cleanup,
                        images={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted((output/'images').glob('*.png'))},
                        contract_sha256=hashlib.sha256(contract_path.read_bytes()).hexdigest(),
                        qualification='Dataset collection only; not FOLLOW or flight evidence.')
        (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--contract', type=Path,
                        default=ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    parser.add_argument('--samples', type=int, default=180)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--split', choices=('train', 'val', 'test'), required=True)
    parser.add_argument('--geometry', action='store_true')
    parser.add_argument('--live-yolo', action='store_true')
    parser.add_argument('--live-graph', action='store_true')
    args = parser.parse_args()
    collect(args.output.resolve(), args.contract, args.samples, args.seed, args.split,
            args.geometry, args.live_yolo or args.live_graph, args.live_graph)
