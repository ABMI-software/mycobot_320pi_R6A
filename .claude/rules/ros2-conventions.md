# Rule — ROS2 conventions in this repo

## Build

- **Always** from `~/ros_jazzy/`, never from inside `src/mycobot_R6A/`. Colcon writes `build/`, `install/`, `log/` at its CWD; we want those at the workspace root.
- Use `--symlink-install` so edits to Python nodes take effect without rebuild.
- Scope the build: `colcon build --packages-select mycobot_gateway mycobot_description` — full rebuilds pull in unrelated pinned packages.

```bash
conda deactivate
cd ~/ros_jazzy
colcon build --packages-select mycobot_gateway mycobot_description --symlink-install
source install/setup.bash
```

## Sourcing order

Always in this order:

1. `source /opt/ros/jazzy/setup.bash` — base distro
2. `source ~/ros_jazzy/install/setup.bash` — our overlay

Getting it backwards gives `ros2 pkg list` output with missing overlays and mysterious launch errors.

## Node conventions

- **One node per process** for anything long-running (bridges, controllers). Composable nodes only when proven necessary.
- Parameters over CLI args for nodes that might be launched differently (sim vs. real): declare in the constructor, read via `get_parameter()`.
- **Topic names**: `/teleop/*`, `/mycobot_controller/*`, `/from_robot`, `/to_robot`. Don't introduce new top-level namespaces without updating [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md).

## Launch files

- Launch files describe **topology**, not business logic. If a launch file contains `if/else` on robot geometry, that logic belongs in a Python node.
- Every new launch file accepts a `target:={sim,real,both}` argument where applicable.

## Gazebo bridge

- Gazebo Harmonic + `gz_ros2_control` — **not** Gazebo Classic and **not** `gazebo_ros2_control`. The package names differ; don't copy/paste from Classic tutorials.
- The `/clock` bridge must be in the launch file (sim mode), else `/joint_states` timestamps go nuts.

## Never do

- **Do not kill ROS2 graph with `rm -rf build install log`** unless intentionally rebuilding from scratch. It's destructive and doesn't fix import errors — those are almost always env issues (see [`python-environments.md`](python-environments.md)).
- Do not run `ros2 launch` inside a conda env. Always `conda deactivate` first.
- Do not edit generated `install/` files — your changes disappear on next build.
