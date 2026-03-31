# 🏗️ Gazebo Simulation — MyCobot 320 Pi

## Overview

This launches the MyCobot 320 Pi inside **Gazebo Harmonic** (the simulator
shipped with ROS2 Jazzy) using the `ros_gz_sim` bridge stack.

### What it does

| Component | Role |
|-----------|------|
| `gz sim` (Gazebo Harmonic) | Physics / 3D visualisation |
| `robot_state_publisher` | Publishes `/robot_description` and TF tree |
| `ros_gz_sim create` | Spawns the URDF into the running Gazebo world |
| `ros_gz_bridge` | Forwards `/joint_states` from Gazebo → ROS2 |
| `rviz2` *(optional)* | RViz alongside Gazebo |

### Files

| File | Description |
|------|-------------|
| `launch/gazebo_sim.launch.py` | Main launch file |
| `urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf` | Gazebo-compatible URDF (with inertials, dynamics, world link, Gz plugins) |
| `urdf/320_pi/mycobot_pro_320_pi.urdf` | Original URDF (RViz-only, no inertials) |

---

## Prerequisites

```bash
# These packages must be installed (should already be present on Jazzy)
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

## Quick Start

```bash
# 1. Clean environment (avoid Conda conflicts)
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash

# 2. Source ROS2 + workspace
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

# 3. Launch Gazebo
ros2 launch mycobot_description gazebo_sim.launch.py

# 3b. Launch Gazebo + RViz side-by-side
ros2 launch mycobot_description gazebo_sim.launch.py rviz:=true
```

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `[gz] [Err] Unable to find file …` | Rebuild: `colcon build --packages-select mycobot_description --symlink-install` |
| Gazebo opens but robot is invisible | Check `GZ_SIM_RESOURCE_PATH` includes the install share path |
| Robot falls through the ground | Make sure `mycobot_pro_320_pi_gazebo.urdf` is used (has `world` fixed joint) |
| Meshes are white / untextured | `.dae` files may not include materials — cosmetic only |
