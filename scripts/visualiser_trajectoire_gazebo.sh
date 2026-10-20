#!/usr/bin/env bash
# Isolated simulation session: no connection to the real robot bridges.
set -e
TASK_REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TASK_BUILD="${TMPDIR:-/tmp}/mycobot_trajectory_build"
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
source /opt/ros/jazzy/setup.bash
cd "$TASK_REPO"
colcon --log-base "$TASK_BUILD/log" build \
    --base-paths mycobot_description mycobot_gateway \
    --build-base "$TASK_BUILD/build" --install-base "$TASK_BUILD/install" \
    --symlink-install --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
source "$TASK_BUILD/install/setup.bash"
export ROS_DOMAIN_ID=$((100 + $$ % 100))
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="mycobot_trajectory_$$"
export ROS_LOG_DIR="$TASK_BUILD/runtime_logs"
printf 'Gazebo uniquement — domaine ROS %s. Demonstration visuelle, pas campagne A/B/G.\n' "$ROS_DOMAIN_ID"
exec ros2 launch mycobot_gateway trajectory_preview.launch.py "$@"
