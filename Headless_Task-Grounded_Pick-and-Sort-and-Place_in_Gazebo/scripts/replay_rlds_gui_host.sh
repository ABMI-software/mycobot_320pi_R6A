#!/usr/bin/env bash
# From the WSL2 HOST: read the red-cube episode from the RLDS dataset with tfds
# (rlds_builder container), hand it to gazebo_to_lerobot, replay it with the GUI.
set -e
RLDS=~/tomislav/robotics/mycobot_320pi_R6A_tomislav_07_10_2026_ver_2/ROS2_to_RLDS_Conversion_OpenVLA
docker exec rlds_builder bash -c 'cd /workspace && python extraction/export_rlds_episode.py \
    rlds_dataset_builder/tfds_out/mycobot_tri_sort/1.0.0 0 rlds_extraction_out/rlds_seed1_cube_rouge.npz 2>&1 | tail -1'
docker cp "$RLDS/rlds_extraction_out/rlds_seed1_cube_rouge.npz" gazebo_to_lerobot:/workspace/replay/
docker exec -it gazebo_to_lerobot /workspace/src/scripts/replay_gui.sh rlds
