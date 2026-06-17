#!/usr/bin/env bash
# validate-ros2-build.sh — Claude Code hook (PostToolUse on Edit/Write inside mycobot_gateway or mycobot_description)
#
# Runs a scoped colcon build to catch broken imports / CMake syntax immediately
# after Claude edits a ROS2 package. Does NOT run on every edit — only when a
# file under the two ROS2 packages is touched.
#
# Activation: wire it up in .claude/settings.json under "hooks":
#
#   "hooks": {
#     "PostToolUse": [
#       {
#         "matcher": "Edit|Write",
#         "paths": ["mycobot_gateway/**", "mycobot_description/**"],
#         "hooks": [ { "type": "command", "command": ".claude/hooks/validate-ros2-build.sh" } ]
#       }
#     ]
#   }
#
# Left inactive by default so edits to Python-only changes (teleop/, training/)
# don't pay for an irrelevant colcon run.

set -euo pipefail

WS_ROOT="${HOME}/ros_jazzy"
PKGS=("mycobot_gateway" "mycobot_description")

# Refuse to run inside conda — guaranteed to fail with Python version mismatch.
if [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; then
  echo "[validate-ros2-build] conda env '$CONDA_DEFAULT_ENV' is active — skipping build." >&2
  echo "[validate-ros2-build] run 'conda deactivate' first, then re-edit to re-trigger." >&2
  exit 0
fi

# Source ROS2 if not already in environment.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

cd "$WS_ROOT"

# Scoped build — fast, symlink install so Python edits don't need rebuild.
if ! colcon build --packages-select "${PKGS[@]}" --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -20; then
  echo "[validate-ros2-build] FAILED — see output above" >&2
  exit 1
fi

echo "[validate-ros2-build] OK — ${PKGS[*]} built clean"
