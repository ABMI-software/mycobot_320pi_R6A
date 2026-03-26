import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    res = []

    # 1. Arguments
    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/320_pi/mycobot_pro_320_pi.urdf"
        ),
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_320pi"),
            "config/mycobot_320_pi.rviz"
        ),
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
    )
    res.append(gui_launch_arg)

    num_launch_arg = DeclareLaunchArgument(
        "num",
        default_value="0",
    )
    res.append(num_launch_arg)
    
    # 2. Robot State Publisher (Crucial for TF math)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[LaunchConfiguration("model")]
    )
    res.append(robot_state_publisher_node)

    # 3. Nodes
    follow_display_node = Node(
        name="follow_display",
        package="mycobot_320pi",
        executable="follow_display",
    )
    res.append(follow_display_node)

    opencv_camera_node = Node(
        name="opencv_camera",
        package="mycobot_320pi",
        executable="opencv_camera",
        arguments=[LaunchConfiguration("num")]
    )
    res.append(opencv_camera_node)

    detect_marker_node = Node(
        name="detect_marker",
        package="mycobot_320pi",
        executable="detect_marker"
    )
    res.append(detect_marker_node)

    following_marker_node = Node(
        name="following_marker",
        package="mycobot_320pi",
        executable="following_marker"
    )
    res.append(following_marker_node)

    # 4. RViz2 (For visualization and hardware verification)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    return LaunchDescription(res)