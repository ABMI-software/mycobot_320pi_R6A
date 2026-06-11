#!/usr/bin/env python3
"""Preview launch — 5 poses × 4 cameras = 20 images.

Use this to visually verify the camera intrinsics and placement BEFORE
launching the full 50 k acquisition.

After the node exits (~30 s), a contact sheet is generated at:
    /tmp/synth_preview/preview_grid.png

Usage
-----
  conda deactivate
  source /opt/ros/jazzy/setup.bash
  source ~/ros_jazzy/install/setup.bash

  ros2 launch mycobot_gateway synthetic_data_preview.launch.py

Then open /tmp/synth_preview/preview_grid.png to inspect all 4 views.
If everything looks correct, launch the full acquisition:

  ros2 launch mycobot_gateway synthetic_data.launch.py \\
      num_samples:=12500 output_dir:=/tmp/synth_50k
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PREVIEW_DIR = '/tmp/synth_preview'


def generate_launch_description():
    desc_pkg = get_package_share_directory('mycobot_description')
    gw_pkg   = get_package_share_directory('mycobot_gateway')
    gz_pkg   = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(
        desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf',
    )
    world_path = os.path.join(gw_pkg, 'worlds', 'lab_room.sdf')

    # ---------- robot_state_publisher ----------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]), value_type=str,
            ),
        }],
        output='screen',
    )

    # ---------- Gazebo ----------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': ['-r -s --headless-rendering ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------- Spawn ----------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'mycobot_320', '-z', '0.0'],
    )

    # ---------- gz bridge ----------
    joint_names = [
        'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
        'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
    ]
    bridge_args = [
        '/world/empty/model/mycobot_320/joint_state'
        '@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]
    for jn in joint_names:
        bridge_args.append(
            f'/model/mycobot_320/joint/{jn}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        )
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gz_bridge', output='screen', arguments=bridge_args,
        remappings=[('/world/empty/model/mycobot_320/joint_state', '/joint_states')],
    )

    # ---------- image bridges ----------
    gz_image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        name='gz_image_bridge', output='screen',
        arguments=[
            '/synth_camera/image',
            '/synth_camera_1/image',
            '/synth_camera_2/image',
            '/synth_camera_3/image',
        ],
    )

    # ---------- Collector — 5 poses only ----------
    collector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='mycobot_gateway',
                executable='synthetic_data_collector',
                name='synthetic_data_collector_preview',
                output='screen',
                parameters=[{
                    'num_samples': 5,
                    'output_dir':  PREVIEW_DIR,
                    'settle_time': 1.5,
                }],
            ),
        ],
    )

    # ---------- Contact-sheet generator (runs after ~25 s) ----------
    # Builds a 4×5 PNG grid: rows = cameras, columns = poses
    grid_script = (
        'python3 -c "'
        'import glob, sys, os; '
        'from PIL import Image, ImageDraw, ImageFont; '
        'import numpy as np; '
        'preview_dir = \\"/tmp/synth_preview\\"; '
        'cams = [\\"cam_0\\", \\"cam_1\\", \\"cam_2\\", \\"cam_3\\"]; '
        'rows = []; '
        '[rows.append([Image.open(p) for p in sorted(glob.glob(os.path.join(preview_dir, \\"images\\", c, \\"*.png\\")))]) for c in cams]; '
        'n_poses = len(rows[0]); '
        'W, H = rows[0][0].size; '
        'pad = 4; '
        'label_h = 20; '
        'grid_w = n_poses * (W + pad) + pad; '
        'grid_h = len(cams) * (H + pad + label_h) + pad; '
        'grid = Image.new(\\"RGB\\", (grid_w, grid_h), (30, 30, 30)); '
        'draw = ImageDraw.Draw(grid); '
        '[[grid.paste(rows[r][c], (pad + c*(W+pad), pad + r*(H+pad+label_h)+label_h)) for c in range(n_poses)] for r in range(len(cams))]; '
        '[[draw.text((pad + c*(W+pad)+2, pad + r*(H+pad+label_h)), f\\"pose {c} | {cams[r]}\\", fill=(255,255,100)) for c in range(n_poses)] for r in range(len(cams))]; '
        'out = os.path.join(preview_dir, \\"preview_grid.png\\"); '
        'grid.save(out); '
        'print(f\\"Contact sheet → {out} ({grid.size[0]}×{grid.size[1]} px)\\"); '
        '"'
    )
    make_grid = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', grid_script],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        gz_image_bridge,
        collector,
        make_grid,
    ])
