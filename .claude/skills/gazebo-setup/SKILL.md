---
name: gazebo-setup
description: First-time or broken-install Gazebo Harmonic setup for this project. Invoke when a fresh machine is being provisioned, when the robot fails to appear in Gazebo, or when controllers won't activate after a clean rebuild.
---

# Gazebo Harmonic setup

This project uses **Gazebo Harmonic** (not Classic). Every tutorial you find online for "gazebo_ros2_control" is for Classic and does not apply.

## Prerequisites

```bash
# ROS2 packages for the Harmonic stack
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
                 ros-jazzy-ros2-controllers ros-jazzy-joint-state-broadcaster
```

Verify:
```bash
dpkg -l ros-jazzy-gz-ros2-control
dpkg -l ros-jazzy-ros2-controllers
```

## Environment variables

Two must be set for Gazebo to find this project's meshes and worlds:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/ros_jazzy/src/mycobot_R6A/mycobot_description"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib"
```

The launch files export these — if you see the robot invisible or a `plugin not found` error, run them manually in your shell to confirm.

## First-run checklist

1. Clean build:
   ```bash
   conda deactivate
   cd ~/ros_jazzy
   rm -rf build install log   # only if you're sure nothing else depends on the install tree
   colcon build --packages-select mycobot_gateway mycobot_description --symlink-install
   source install/setup.bash
   ```
2. Try RViz first (no Gazebo):
   ```bash
   ros2 launch mycobot_description display.launch.py
   ```
   If the robot doesn't appear in RViz, the URDF is broken — fix before touching Gazebo.
3. Try Gazebo:
   ```bash
   ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim
   ```
4. Verify controllers:
   ```bash
   ros2 control list_controllers
   ```
   Expected output includes `mycobot_controller` → `active`.

## Common failure modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| Gazebo opens empty (no robot) | `GZ_SIM_RESOURCE_PATH` missing | Launch file should set it; if not, add to your shell |
| Robot visible but immobile | Controllers didn't activate | `ros2 control list_controllers` — set state to active manually |
| `plugin 'gz-sim-*' not found` | `GZ_SIM_SYSTEM_PLUGIN_PATH` missing | `export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib` |
| `/joint_states` at 0 Hz | `joint_state_broadcaster` didn't start | Check the controller YAML, restart the launch |
| `/clock` bridge missing | Launch file doesn't bridge `/clock` | Timestamps drift — add to the `ros_gz_bridge` parameter list |
| DART explodes when robot moves | Missing or zero inertials | `urdf-surgeon` territory — check all `<inertial>` blocks |

## World selection

- `worlds/empty.sdf` → minimal, for controller debugging
- `worlds/randomized.sdf` (v1) → 1 light, 1 table, simple bg — for early training
- `worlds/randomized_v2.sdf` (v2) → 6 lights, 12 clutter objects, 3 walls — **canonical for new synthetic data** (CHANGELOG 1.8.0)

Use v2 for any new synthetic dataset collection. v1 produces too-clean data that widens the sim-to-real gap.

## Reference

- [`mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf`](../../../mycobot_description/urdf/320_pi/)
- [`mycobot_description/worlds/`](../../../mycobot_description/worlds/)
- [`mycobot_gateway/launch/mycobot_teleop.launch.py`](../../../mycobot_gateway/launch/)
- [`docs/ARCHITECTURE.md`](../../../docs/ARCHITECTURE.md), [`mycobot_description/README_GAZEBO.md`](../../../mycobot_description/README_GAZEBO.md)
