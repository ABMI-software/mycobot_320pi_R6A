# Gazebo integration notes

This file explains how to start a basic Gazebo simulation for the MyCobot 320 Pi.

Prerequisites
- Install ROS2 `gazebo_ros` package for your ROS distribution (Galactic / Jazzy compatible)

Quick start (on the Tour or any machine with ROS2 + Gazebo):

```bash
# Make sure ROS environment is clean (avoid Conda conflicts)
conda deactivate || true
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

# Launch Gazebo and spawn the robot
ros2 launch mycobot_description gazebo_sim.launch.py
```

Notes
- The launch file will look for the URDF at `mycobot_description/urdf/320_pi/mycobot_pro_320_pi.urdf`.
- If you prefer to spawn from the `robot_description` topic, modify `gazebo_sim.launch.py` accordingly.

Troubleshooting
- If `gazebo.launch.py` is not found, ensure `gazebo_ros` is installed and in your ROS2 environment.
- If meshes are missing, verify the `urdf/320_pi` contents.
