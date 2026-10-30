#!/usr/bin/env python3
"""Render paired original/realistic Gazebo images with exact static poses.

Run with system Python after sourcing ROS. No robot controller is started.
Joint frames are read from the expanded URDF; only temporary render models
are made static. Camera frames and optics come from that same URDF.
"""

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import warnings
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf'
WORLD = ROOT / 'mycobot_description/worlds/real_table.sdf'
JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
LINKS = ['base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6']
POSES = [[0, 0, 0, 0, 0, 0], [30, -30, 55, -20, 20, 30],
         [-40, 20, -45, 30, -30, -60], [70, -55, 60, -40, 40, 90]]
CAMERAS = {'front': 'camera_link', 'right': 'camera_link_right',
           'left': 'camera_link_left', 'top': 'camera_link_top'}


def element(parent, tag, text=None, **attributes):
    out = ET.SubElement(parent, tag, attributes)
    if text is not None:
        out.text = str(text)
    return out


def origin_matrix(origin):
    matrix = np.eye(4)
    if origin is not None:
        matrix[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        matrix[:3, :3] = Rotation.from_euler(
            'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return matrix


def pose_text(matrix):
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)
        rpy = Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz')
    return ' '.join(f'{x:.15g}' for x in np.r_[matrix[:3, 3], rpy])


def forward_frames(robot, angles):
    frames = {'world': np.eye(4), 'base': np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            local = origin_matrix(joint.find('origin'))
            if joint.get('type') in ('revolute', 'continuous'):
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                angle = angles.get(joint.get('name'), 0.0)
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
                local = local @ rotation
            frames[joint.find('child').get('link')] = frames[parent] @ local
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('Unresolved URDF joint tree')
    return frames


def render_template(robot, output, appearance):
    clean = copy.deepcopy(robot)
    for entry in list(clean):
        if entry.tag == 'link' and (entry.get('name') == 'world'
                                    or entry.get('name', '').startswith('camera_')):
            clean.remove(entry)
        elif entry.tag == 'joint' and entry.find('parent').get('link') == 'world':
            clean.remove(entry)
        elif entry.tag == 'gazebo' and (entry.get('reference') is None
                                      or entry.get('reference', '').startswith('camera_')):
            clean.remove(entry)
        elif entry.tag == 'ros2_control':
            clean.remove(entry)
    path = output / f'{appearance}_render.urdf'
    ET.ElementTree(clean).write(path)
    sdf = subprocess.check_output(['gz', 'sdf', '-p', str(path)], text=True)
    (output / f'{appearance}_render.sdf').write_text(sdf)
    return ET.fromstring(sdf).find('model')


def build(output):
    sdf = ET.Element('sdf', version='1.11')
    world = element(sdf, 'world', name='appearance_comparison')
    for name, system in [('physics', 'Physics'), ('sensors', 'Sensors')]:
        plugin = element(world, 'plugin', filename=f'gz-sim-{name}-system',
                         name=f'gz::sim::systems::{system}')
        if name == 'sensors':
            element(plugin, 'render_engine', 'ogre2')
    physics = element(world, 'physics', name='default', type='ode')
    element(physics, 'max_step_size', '0.01')
    element(physics, 'real_time_factor', '1')
    original_world = ET.parse(WORLD).getroot().find('world')
    world.append(copy.deepcopy(original_world.find('scene')))
    for light in original_world.findall('light'):
        if light.get('type') == 'directional':
            world.append(copy.deepcopy(light))
    ground = copy.deepcopy(original_world.find("model[@name='ground_plane']"))
    for size in ground.findall('.//plane/size'):
        size.text = '80 80'
    world.append(ground)
    manifest = {'keypoint_names': ['mycobot320_' + link for link in LINKS],
                'samples': [], 'poses_degrees': POSES,
                'urdf_sha256': hashlib.sha256(URDF.read_bytes()).hexdigest(),
                'notes': 'Static paired renders; no visibility/occlusion filtering; '
                         'joint-origin ground truth; no test of physical control.'}
    captures = []
    for appearance_index, appearance in enumerate(('original', 'realistic')):
        robot = ET.fromstring(subprocess.check_output(
            ['xacro', str(URDF), f'robot_appearance:={appearance}'], text=True))
        template = render_template(robot, output, appearance)
        (output / appearance).mkdir(exist_ok=True)
        for pose_index, degrees in enumerate(POSES):
            angles = dict(zip(JOINTS, np.deg2rad(degrees)))
            frames = forward_frames(robot, angles)
            assert min(frames[n][2, 3] for n in LINKS[1:]) > 0.025
            cell = element(world, 'model', name=f'{appearance}_{pose_index}')
            element(cell, 'static', 'true')
            element(cell, 'pose', f'{pose_index * 5} {appearance_index * 5} 0 0 0 0')
            for entry in original_world:
                if ((entry.tag == 'model' and entry.get('name') not in
                     ('ground_plane', 'table_camera')) or entry.tag == 'include'):
                    item = copy.deepcopy(entry)
                    if item.tag == 'model':
                        static = item.find('static')
                        if static is None:
                            element(item, 'static', 'true')
                        else:
                            static.text = 'true'
                        for link in item.findall('link'):
                            for collision in link.findall('collision'):
                                link.remove(collision)
                    cell.append(item)
            model = element(cell, 'model', name='robot')
            element(model, 'static', 'true')
            for source_link in template.findall('link'):
                name = source_link.get('name')
                link = element(model, 'link', name=name)
                element(link, 'pose', pose_text(frames[name]))
                for visual in source_link.findall('visual'):
                    link.append(copy.deepcopy(visual))
            for camera_name, camera_link in CAMERAS.items():
                sample_id = f'pose{pose_index}_{camera_name}'
                camera_pose = frames[camera_link]
                sensor_src = robot.find(f"gazebo[@reference='{camera_link}']/sensor")
                sensor = copy.deepcopy(sensor_src)
                sensor.set('name', f'{appearance}_{sample_id}')
                sensor.find('update_rate').text = '1'
                sensor.find('topic').text = f'/appearance/{appearance}/{sample_id}'
                camera = sensor.find('camera')
                camera.find('clip/far').text = '2.0'
                frame_dir = output / '_frames' / appearance / sample_id
                frame_dir.mkdir(parents=True, exist_ok=True)
                element(element(camera, 'save', enabled='true'), 'path', str(frame_dir))
                camera_model = element(cell, 'model', name=f'camera_{camera_name}')
                element(camera_model, 'static', 'true')
                camera_body = element(camera_model, 'link', name='camera')
                element(camera_body, 'pose', pose_text(camera_pose))
                camera_body.append(sensor)
                captures.append((frame_dir, output / appearance / f'{sample_id}.png'))
                if appearance_index == 0:
                    width = int(camera.findtext('image/width'))
                    height = int(camera.findtext('image/height'))
                    focal = width / (2 * np.tan(float(camera.findtext('horizontal_fov')) / 2))
                    projections = []
                    valid = []
                    for link_name in LINKS:
                        # Sensor axes: +X forward, +Y left, +Z up.
                        p = np.linalg.inv(camera_pose) @ frames[link_name][:, 3]
                        u, v = width / 2 - focal * p[1] / p[0], height / 2 - focal * p[2] / p[0]
                        projections.append([float(u), float(v)])
                        valid.append(bool(p[0] > 0 and 0 <= u < width and 0 <= v < height))
                    manifest['samples'].append({'id': sample_id,
                        'keypoints': projections, 'keypoints_valid': valid,
                        'camera': camera_name, 'pose_index': pose_index,
                        'image_size': [width, height], 'fx': focal})
    ET.indent(sdf)
    world_path = output / 'comparison.sdf'
    ET.ElementTree(sdf).write(world_path, encoding='utf-8', xml_declaration=True)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return world_path, captures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Never accidentally reuse images from an earlier run.
    if (output / 'manifest.json').exists():
        raise SystemExit('Output already contains a run; choose a fresh directory.')
    world_path, captures = build(output)
    if args.prepare_only:
        print(world_path)
        return
    env = os.environ.copy()
    env['GZ_PARTITION'] = f'appearance_check_{os.getpid()}'
    env['GZ_SIM_RESOURCE_PATH'] = str(ROOT) + ':' + env.get('GZ_SIM_RESOURCE_PATH', '')
    env['ROS_DOMAIN_ID'] = '71'
    env['ROS_LOG_DIR'] = str(output / 'ros_log')
    # Gazebo Sim 8 schedules rendering for sensors with transport subscribers.
    # The save flag alone does not count as a connection. An eager image
    # bridge activates every camera; frames are still saved losslessly by GZ.
    topics = [f'/appearance/{target.parent.name}/{target.stem}'
              '@sensor_msgs/msg/Image[gz.msgs.Image' for _, target in captures]
    with (output / 'gazebo.log').open('w') as log, (output / 'bridge.log').open('w') as bridge_log:
        process = subprocess.Popen(
            ['gz', 'sim', '-r', '-s', '--headless-rendering', '-v', '3', str(world_path)],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        bridge = None
        try:
            bridge = subprocess.Popen(
                ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *topics],
                env=env, stdout=bridge_log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                ready = [sorted(directory.glob('*.png')) for directory, _ in captures]
                if all(len(paths) >= 2 for paths in ready):
                    for paths, (_, target) in zip(ready, captures):
                        shutil.copy2(paths[0], target)
                    print(f'Saved {len(captures)} images with identical paired poses and optics.')
                    return
                if process.poll() is not None:
                    raise RuntimeError(f'Gazebo exited early; inspect {output / "gazebo.log"}')
                if bridge.poll() is not None:
                    raise RuntimeError(f'Bridge exited early; inspect {output / "bridge.log"}')
                time.sleep(0.5)
            raise TimeoutError(f'Camera timeout; inspect {output / "gazebo.log"}')
        finally:
            for child in (bridge, process):
                if child is not None and child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()


if __name__ == '__main__':
    main()
