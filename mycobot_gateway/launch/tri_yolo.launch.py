#!/usr/bin/env python3
"""Sorting scene for YOLO validation: four pieces, four bins, the DREAM 50K cameras.

The board, markers and robot are those of real_table; the four cameras are at
the poses that rendered the 50K DREAM dataset (camera_layout:=dream50k); robot
masses are the original ones. Pieces and bins are randomized on every launch
(vision/tri_scene.py); the seed is logged and seed:=N replays a scene.
Protocol: docs/PROTOCOLE_YOLO_GAZEBO.md.

    ros2 launch mycobot_gateway tri_yolo.launch.py
    ros2 launch mycobot_gateway tri_yolo.launch.py seed:=7 headless:=true
    ros2 launch mycobot_gateway tri_yolo.launch.py seed:=7 log_dir:=results/yolo_gazebo/2026-10-05_7

dream:=true (protocol step 10) runs DREAM on the same four images, one
dream_inference per camera (/dream_front, /dream_right, /dream_left, /dream_top),
and, with log_dir, dream_fk_compare writes dream_vs_fk.csv and dream_pose.csv.

log_dir (protocol step 11): run.yaml (seed, scene, cameras, masses, commit,
yolo26 weights) and yolo_vs_gt.csv. Pass the same folder to sim_sorting_grasp
as csv_path:=<log_dir>/tri.csv.
"""

import datetime
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from mycobot_gateway.vision import tri_scene
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras

# DREAM's PYTHONPATH (venv_dream, per node), as dream_multicam.launch.py sets it.
_spec = importlib.util.spec_from_file_location(
    'dream_multicam', os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                   'dream_multicam.launch.py'))
_dream_multicam = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_dream_multicam)

WORLD_NAME = 'tri_yolo'
# Front, right, left, top: the 2x2 order of the YOLO vs ground-truth mosaic.
CAMERAS = ('synth_camera', 'synth_camera_right', 'synth_camera_left', 'synth_camera_top')
# Protocol 2.1: the sorting robot (gripper + 50K cameras) and the 50K render itself.
ROBOTS = {'robot_yolo_pickplace': '', 'robot_dream_baseline': '_nogripper'}
ARM_JOINTS = ('joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6')


def write_run_yaml(path, seed, layout, cameras, urdf, piece_reach, robot):
    """Header of a logged run (protocol step 11); yolo_gazebo_node appends the weights."""
    urdf = urdf.replace('.urdf', ROBOTS[robot] + '.urdf')
    root = ET.fromstring(xacro.process_file(urdf, mappings={'camera_layout': 'dream50k'}).toxml())
    mass = sum(float(m.get('value')) for m in root.iter('mass'))
    repo = Path(__file__).resolve().parents[2]
    git = ['git', '-C', str(repo)]
    commit = subprocess.run(git + ['rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(git + ['status', '--porcelain', '--untracked-files=no'],
                           capture_output=True, text=True).stdout.count('\n')
    lines = [f'date: {datetime.datetime.now().isoformat(timespec="seconds")}', f'seed: {seed}', f'robot: {robot}', f'scene: {"tri" if layout else "dream50k"}',
             f'piece_reach_m: {piece_reach}', f'commit: {commit}', f'modified_files: {dirty}',
             f'robot_total_mass_kg: {mass:.4f}', 'camera_layout: dream50k', 'cameras:']
    for name in CAMERAS:
        c = cameras[name]
        lines.append(f'  {name}: {{fx: {c.K[0, 0]:.4f}, '
                     f'xyz: [{", ".join(f"{v:.4f}" for v in c.world_from_optical[:3, 3])}]}}')
    lines.append('layout_xy_m:')
    lines += [f'  {n}: [{x:.4f}, {y:.4f}]' for n, (x, y) in layout.items()]
    Path(path).write_text('\n'.join(lines) + '\n')


def dream_nodes(seed, log_dir, model, rate):
    from mycobot_gateway.dream_fk_compare import PREFIXES
    env = {'PYTHONPATH': _dream_multicam._pythonpath_venv_dream()}
    nodes = [Node(package='mycobot_gateway', executable='dream_inference',
                  name=f'dream_inference_{PREFIXES[cam].strip("/")}', output='screen',
                  additional_env=env,
                  parameters=[{'use_sim_time': True, 'camera_topic': f'/{cam}/image',
                               'model_name': model, 'output_prefix': PREFIXES[cam],
                               'publish_rate': rate, 'visualize': False}])
             for cam in CAMERAS]
    if log_dir:
        nodes.append(Node(package='mycobot_gateway', executable='dream_fk_compare',
                          output='screen',
                          parameters=[{'use_sim_time': True, 'cameras': list(CAMERAS),
                                       'seed': seed, 'log_dir': str(log_dir)}]))
    return nodes


def launch_scene(context):
    desc_pkg = get_package_share_directory('mycobot_description')
    gateway_pkg = get_package_share_directory('mycobot_gateway')
    models = os.path.join(desc_pkg, 'models')
    source = os.path.join(desc_pkg, 'worlds', 'real_table.sdf')
    urdf = os.path.join(desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf')

    seed = int(LaunchConfiguration('seed').perform(context))
    if seed < 0:
        seed = int(np.random.default_rng().integers(2**31))
    cameras = load_cameras(urdf, {'camera_layout': 'dream50k'})
    top = cameras['synth_camera_top']
    sorting = LaunchConfiguration('scene').perform(context) == 'tri'
    if sorting:
        world_name = WORLD_NAME
        layout = tri_scene.sample_tri_scene(
            np.random.default_rng(seed), tri_scene.load_footprints(models),
            tri_scene.load_board(source, models), top,
            piece_reach=float(LaunchConfiguration('piece_reach').perform(context)) or None)
        fd, world_file = tempfile.mkstemp(prefix='mycobot_tri_yolo_', suffix='.sdf')
        with os.fdopen(fd, 'w') as f:
            f.write(tri_scene.build_world(source, layout, WORLD_NAME))
    else:
        # The world that rendered the 50K DREAM dataset, used as is: no pieces,
        # so no YOLO and no ground truth, DREAM only.
        world_name, layout = 'randomized', {}
        world_file = os.path.join(desc_pkg, 'worlds', 'randomized.sdf')

    def cleanup(_context):
        if sorting and os.path.exists(world_file):
            os.unlink(world_file)
        return []

    placed = ', '.join(f'{n} ({x * 1000:.0f}, {y * 1000:.0f})' for n, (x, y) in layout.items())
    log_dir = LaunchConfiguration('log_dir').perform(context)
    run_yaml, csv_path = '', ''
    if log_dir:
        log_dir = Path(log_dir).expanduser().resolve()
        log_dir.mkdir(parents=True, exist_ok=False)
        run_yaml, csv_path = str(log_dir / 'run.yaml'), str(log_dir / 'yolo_vs_gt.csv')
        write_run_yaml(run_yaml, seed, layout, cameras, urdf,
                       float(LaunchConfiguration('piece_reach').perform(context)),
                       LaunchConfiguration('robot').perform(context))
    q = [math.radians(a) for a in tri_scene.OBSERVATION_Q_DEG]
    trajectory = ('{joint_names: [' + ', '.join(ARM_JOINTS) + '], points: [{positions: ['
                  + ', '.join(f'{v:.6f}' for v in q) + '], time_from_start: {sec: 3}}]}')
    dream = []
    if LaunchConfiguration('dream').perform(context) == 'true':
        # Invariant I8: DREAM only on the training render.
        if LaunchConfiguration('robot_appearance').perform(context) != 'original':
            raise RuntimeError('dream:=true requires robot_appearance:=original (protocol I8)')
        dream = dream_nodes(seed, log_dir,
                            LaunchConfiguration('dream_model').perform(context),
                            float(LaunchConfiguration('dream_rate').perform(context)))

    return dream + [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        LogInfo(msg=f'tri_yolo seed={seed} — {placed} mm'),
        LogInfo(msg=f'log_dir {log_dir}' if log_dir else 'log_dir unset: no CSV'),
        LogInfo(msg='beyond vertical-tool reach (0.28 m): '
                    + (', '.join(tri_scene.beyond_reach(layout)) or 'none')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gateway_pkg, 'launch', 'sim_grasp.launch.py')),
            launch_arguments={
                'world_name': world_name, 'world_file': world_file,
                'camera_layout': 'dream50k', 'bridge_multicam': 'true',
                'headless': LaunchConfiguration('headless'),
                'robot_appearance': LaunchConfiguration('robot_appearance'),
                'robot_model_suffix': ROBOTS[LaunchConfiguration('robot').perform(context)],
                'gui_config': os.path.join(
                    desc_pkg, 'config',
                    'tri_yolo_gui.config' if LaunchConfiguration('panel').perform(context) == 'true'
                    else 'trajectory_gui.config'),
            }.items()),
    ] + (sorting_nodes(seed, run_yaml, csv_path) if sorting else []) + [
        TimerAction(period=12.0, condition=IfCondition(LaunchConfiguration('observe')), actions=[
            LogInfo(msg=f'observation pose {tri_scene.OBSERVATION_Q_DEG} deg'),
            ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '-w', '1',
                                '/mycobot_controller/joint_trajectory',
                                'trajectory_msgs/msg/JointTrajectory', trajectory],
                           output='screen')]),
    ]


def sorting_nodes(seed, run_yaml, csv_path):
    return [
        # Ground truth for VALIDATION only: under /validation/gt, never read by
        # perception or planning (protocol invariant I4).
        Node(package='mycobot_gateway', executable='gazebo_ground_truth', output='screen',
             parameters=[{'use_sim_time': True, 'world_name': WORLD_NAME}]),
        # Perception: yolo26 on the four DREAM cameras, each localized on its
        # own (no fusion, protocol step 7).
        Node(package='mycobot_gateway', executable='yolo_gazebo_node', output='screen',
             condition=IfCondition(LaunchConfiguration('yolo')),
             parameters=[{'use_sim_time': True, 'cameras': list(CAMERAS), 'run_yaml': run_yaml}]),
        Node(package='mycobot_gateway', executable='yolo_localizer', output='screen',
             condition=IfCondition(LaunchConfiguration('yolo')),
             parameters=[{'use_sim_time': True, 'cameras': list(CAMERAS)}]),
        # Validation view in a Gazebo GUI panel: the YOLO boxes, classes and
        # confidences, plus the XY error against the ground truth per piece.
        Node(package='mycobot_gateway', executable='yolo_gt_overlay', output='screen',
             condition=IfCondition(LaunchConfiguration('yolo')),
             parameters=[{'use_sim_time': True, 'cameras': list(CAMERAS), 'seed': seed,
                          'csv_path': csv_path}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             condition=IfCondition(LaunchConfiguration('yolo')),
             arguments=['/validation/yolo_gt/image@sensor_msgs/msg/Image]gz.msgs.Image']),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('seed', default_value='-1',
                              description='Scene seed; -1 draws one and logs it'),
        DeclareLaunchArgument('piece_reach', default_value='0.0',
                              description='Max base distance of the four objects (m); 0 = anywhere'),
        DeclareLaunchArgument('observe', default_value='true',
                              description='Lean the arm back so the top camera sees the board'),
        DeclareLaunchArgument('yolo', default_value='true', choices=['true', 'false'],
                              description='Run yolo26 on the four cameras'),
        DeclareLaunchArgument('panel', default_value='true', choices=['true', 'false'],
                              description='YOLO vs GT image panel in the Gazebo GUI'),
        DeclareLaunchArgument('scene', default_value='tri', choices=['tri', 'dream50k'],
                              description='dream50k = the world of the 50K render (randomized.sdf), '
                                          'DREAM only'),
        DeclareLaunchArgument('robot', default_value='robot_yolo_pickplace', choices=list(ROBOTS),
                              description='robot_dream_baseline = no gripper, for the DREAM '
                                          'non-regression (protocol 2.1, step 10)'),
        DeclareLaunchArgument('dream', default_value='false', choices=['true', 'false'],
                              description='DREAM on the four cameras (protocol step 10)'),
        DeclareLaunchArgument('dream_model', default_value='vgg_ultimate_v4_mix_ft_e30',
                              description='checkpoints_dream/<name>; v4_mix = trained on the 50K renders'),
        DeclareLaunchArgument('dream_rate', default_value='2.0',
                              description='DREAM inferences per second and camera'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description='Folder for run.yaml + yolo_vs_gt.csv; empty = no log'),
        DeclareLaunchArgument('robot_appearance', default_value='original',
                              choices=['original', 'realistic']),
        OpaqueFunction(function=launch_scene),
    ])
