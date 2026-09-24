#!/usr/bin/env bash
# Part 11.6 -- verify this host can run the batch. Read-only. Exit 1 on failure.
fail=0
chk(){ printf '%-46s' "$1"; shift
       if "$@" >/dev/null 2>&1; then echo "OK"; else echo "FAIL"; fail=1; fi; }

# "container image present" only makes sense run from the HOST, before a
# container exists to check anything else from -- checking it from INSIDE
# the container being asked about is a contradiction, found live the first
# time this ran there (a straightforward FAIL that no in-container fix can
# ever clear). Skipped, not failed, when `docker` itself isn't on PATH --
# that absence is itself evidence this is running inside the container,
# which is the thing this specific check exists to confirm from outside.
if command -v docker >/dev/null 2>&1; then
  chk "container image present"        docker images --format '{{.Repository}}' \
                                          --filter reference=gazebo_to_lerobot
else
  printf '%-46s' "container image present (host-only check)"; echo "SKIPPED"
fi
chk "ROS 2 Jazzy present"            test -f /opt/ros/jazzy/setup.bash
chk "Gazebo (gz) on PATH"            which gz
chk "ros_gz_sim create available"    bash -c 'ros2 pkg executables ros_gz_sim | grep -q create'
chk "pyarrow importable"             python3 -c 'import pyarrow'
chk "ffmpeg present"                 which ffmpeg
chk "config/WORLD_PATH readable"     test -s config/WORLD_PATH
# The path in config/WORLD_PATH is relative to the mycobot_320pi_R6A repo
# ROOT (where Part 2's find/grep enumeration was run), not to wherever this
# script happens to be invoked from -- found live: running from
# /workspace/htgspp, a bare `test -f` against that relative path always
# failed, even though the same file genuinely exists in the installed ROS2
# package share directory. Resolve against that share directory instead,
# which is also the actual path every launch file in this project uses.
chk "target world file exists"       bash -c '
  share=$(ros2 pkg prefix mycobot_description 2>/dev/null)/share/mycobot_description
  base=$(basename "$(cat config/WORLD_PATH)")
  test -f "$share/worlds/$base"'
chk "objects.yaml validates"         python3 scripts/validate_objects.py
chk "60 episodes in matrix"          bash -c '[ $(($(wc -l < config/episode_matrix.csv)-1)) -eq 60 ]'
# 360 (60 x 6), confirmed correct with the spec's author -- the "420"
# elsewhere counted HOME as a solved waypoint, but it's a fixed joint-space
# constant needing no IK solve; see doc/WAYPOINT_CONTINUITY.md. Not a
# deviation, just the right count.
chk "waypoints solved, zero discontinuities (360 = 60 eps x 6)" python3 scripts/check_waypoints.py
chk "tasks.jsonl has 4 tasks"        bash -c '[ $(wc -l < config/tasks.jsonl) -eq 4 ]'
chk "verifier self-test passes"      python3 scripts/test_verifier.py
chk "RAM >= 3 GB"                    bash -c '[ $(free -m | awk "/^Mem:/{print \$2}") -ge 3000 ]'
chk "free disk >= 10 GB"             bash -c '[ $(df -BG --output=avail . | tail -1 | tr -dc 0-9) -ge 10 ]'

# moveit_py is NOT required here -- this pipeline uses scripts/mycobot_ik.py
# (diff_ik.py directly) for all IK, precisely because that avoids the
# rclpy+moveit_py same-process hang the previous acquisition hit. Do not
# add a moveit_py check back in without re-verifying that decision.

echo
if [ $fail -eq 0 ]; then echo "PREFLIGHT PASSED"; else
  echo "PREFLIGHT FAILED -- do not start the batch"
fi
exit $fail
