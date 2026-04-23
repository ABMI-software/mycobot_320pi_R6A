---
description: Launch Gazebo Harmonic simulation with the MyCobot 320 Pi digital twin
---

Start the Gazebo simulation with full controller stack.

**Steps:**

1. Ensure no stale Gazebo / controller processes are running:
   ```bash
   pkill -9 -f "gz sim|gz-sim|controller_manager|ros_gz|robot_state_publisher" || true
   ```
2. Fresh ROS2 shell (no conda):
   ```bash
   conda deactivate
   source /opt/ros/jazzy/setup.bash
   source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
   ```
3. Launch:
   ```bash
   ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim
   ```
4. Wait for these lines in the log before doing anything else:
   - `[controller_manager]: Successful 'activate' of 'mycobot_controller'`
   - `[joint_state_broadcaster]: state 'Active'`
   - `gripper_position_controller Configured and activated`

**Common failure modes:**

- `GZ_SIM_RESOURCE_PATH` not set → mesh loading fails → robot is invisible. Fix: the launch file should export it; if it doesn't, the user needs a clean `colcon build`.
- Controllers stay in `Configured` but never `Active` → `ros2_control` YAML mismatch. Check [`mycobot_description/config/mycobot_controllers.yaml`](../../mycobot_description/config/).
- `/joint_states` not publishing at 150 Hz → `joint_state_broadcaster` didn't start. `ros2 control list_controllers` to diagnose.

**Related:** [`.claude/commands/launch-teleop.md`](launch-teleop.md), [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md)
