"""Full Gazebo_to_LeRobot_Pipeline sim bring-up: robot + controllers + camera bridge(s).

Wraps mycobot_gateway/mycobot_teleop.launch.py target:=sim (the one launch
file in the reused mycobot_R6A packages that actually spawns
joint_state_broadcaster + mycobot_controller + gripper_position_controller
and bridges /clock) and adds the camera bridge(s) that file doesn't set up
on its own, using the same ros_gz_image image_bridge pattern already used
by mycobot_gateway/launch/pick_and_place.launch.py.

Only the front camera (/synth_camera/image) is bridged by default. The
other 3 (right/left/top) are set always_on=false in this sandbox's URDF
copy specifically so Gazebo doesn't render them with nothing subscribed --
on the resource-constrained VM this sandbox targets (WSL2, ~5GB RAM,
software or D3D12-translated rendering), rendering all 4 every frame plus
the GUI viewport made the Gazebo GUI barely respond to mouse input. Pass
all_cameras:=true to bridge (and thereby re-activate rendering for) all 4.

Gazebo runs headless (server-only, no 3D viewport window) by default --
the interactive GUI's mouse-driven camera controls (Follow/Move-To/scroll
zoom) were unreliable on this sandbox's VM in practice, and this workflow
(teleop-and-record for a LeRobot dataset) never needs that window: watch
the task via rqt_image_view on /synth_camera/image instead, and drive the
arm via keyboard/joystick teleop, not by dragging the Gazebo viewport.
Dropping the GUI process also frees real CPU/RAM on this constrained VM.
Pass headless:=false to get the GUI window back.

Usage:
  ros2 launch gazebo_to_lerobot_bringup sim_full.launch.py
  ros2 launch gazebo_to_lerobot_bringup sim_full.launch.py headless:=false
  ros2 launch gazebo_to_lerobot_bringup sim_full.launch.py all_cameras:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory("mycobot_description")
    gateway_pkg = get_package_share_directory("mycobot_gateway")

    # Gazebo needs to resolve package://mycobot_description/... mesh URIs.
    # Same pattern already used by mycobot_gateway/launch/sim_grasp.launch.py
    # and pick_and_place.launch.py: prepend the parent of the installed
    # mycobot_description share dir.
    gz_resource_path = os.path.dirname(desc_pkg)
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    full_resource_path = f"{gz_resource_path}:{existing}" if existing else gz_resource_path
    set_gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", full_resource_path)

    all_cameras_arg = DeclareLaunchArgument(
        "all_cameras", default_value="false",
        description="Bridge all 4 cameras instead of just the front one (heavier on this VM).",
    )
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="true",
        description="Run Gazebo server-only, no GUI window (see module docstring for why "
                     "this is the default here). Pass false to get the 3D viewport back.",
    )

    teleop_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gateway_pkg, "launch", "mycobot_teleop.launch.py")
        ),
        launch_arguments={
            "target": "sim",
            "rosbridge": "false",
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    # NOTE: cameras stay as links of the mycobot_320 model (like the source
    # repo). A separate spawned scene_cameras model was tried to fix
    # Follow/Move-To always framing the wide 4-camera bounding box instead
    # of the arm -- but sensors on models spawned at runtime via `create`
    # failed to attach in this gz-sim version ("Parent not found" render
    # errors, then a server crash). Reverted; the camera-view issue is
    # worked around from the CLI instead (gz topic /gui/camera/pose +
    # /gui/track) rather than by restructuring the model.

    front_camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/synth_camera/image"],
        output="screen",
    )

    extra_camera_topics = [
        "/synth_camera_right/image",
        "/synth_camera_left/image",
        "/synth_camera_top/image",
    ]
    extra_camera_bridges = [
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            arguments=[topic],
            output="screen",
            condition=IfCondition(LaunchConfiguration("all_cameras")),
        )
        for topic in extra_camera_topics
    ]

    return LaunchDescription([
        all_cameras_arg,
        headless_arg,
        set_gz_resource,
        teleop_sim,
        front_camera_bridge,
        *extra_camera_bridges,
    ])
