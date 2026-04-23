# Rule — Python environments

Three Python environments coexist on this machine. Mixing them breaks everything. **Always check before running code.**

## The three envs

| Env | Activation | Python | Used for |
|-----|-----------|--------|----------|
| System ROS2 | `conda deactivate && source /opt/ros/jazzy/setup.bash` | 3.12 | Any `ros2`, `colcon`, `rclpy` |
| `hand-teleop` (conda) | `conda activate hand-teleop` | 3.10 | Wilor, Orbbec, everything in [`teleop/`](../../teleop/) |
| `venv_dream` | `source ~/ros_jazzy/venv_dream/bin/activate` | 3.12 | DREAM training/eval scripts |

## Hard rules

1. **Before any `ros2`, `colcon`, or `rclpy`-using script: `conda deactivate` first.** Conda's 3.13 shadows ROS2's 3.12 and imports fail with cryptic C-extension errors. Always do this, even if you think you're already out.
2. **Never import `rclpy` from `hand-teleop` or `venv_dream`.** They're Python 3.10 / 3.12 without ROS2 bindings. Use `roslibpy` + rosbridge for ROS ↔ non-ROS2 bridges.
3. **Do not `pip install` into the system Python.** Use the appropriate venv or conda env. If a dependency is missing from the ROS2 side, use `rosdep` or `apt`.

## Quick check

```bash
which python3          # shows active env
python3 -c 'import sys; print(sys.version)'
```

If you see `3.13` when trying to run ROS2 — you forgot `conda deactivate`.

## Why three envs

- ROS2 Jazzy ships with Python 3.12 and its `rclpy` C-extensions are locked to that minor version
- Wilor requires Python 3.10 (PyTorch wheel compatibility at the time of integration)
- DREAM (NVlabs) needs PyTorch 2.6 + CUDA 12.4 and we keep it isolated to avoid polluting the ROS2 interpreter
