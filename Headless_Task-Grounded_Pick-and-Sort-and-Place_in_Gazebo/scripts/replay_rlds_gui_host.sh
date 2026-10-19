#!/usr/bin/env bash
# From the HOST: read one episode of the RLDS dataset with tfds (rlds_builder
# container), hand it to the Gazebo container, replay it with the GUI.
#   replay_rlds_gui_host.sh [seed] [piece]        (defaults: 1 cube_rouge)
# CONTAINER (default gazebo_to_lerobot) and RLDS_CONTAINER (default rlds_builder).
set -e
SEED="${1:-1}"
PIECE="${2:-cube_rouge}"
C="${CONTAINER:-gazebo_to_lerobot}"
RC="${RLDS_CONTAINER:-rlds_builder}"
REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
RLDS="$REPO/ROS2_to_RLDS_Conversion_OpenVLA"
NPZ="rlds_seed${SEED}_${PIECE}.npz"
docker exec "$RC" bash -c "cd /workspace && python extraction/export_rlds_episode.py \
    rlds_dataset_builder/tfds_out/mycobot_tri_sort/1.0.0 $SEED $PIECE rlds_extraction_out/$NPZ 2>&1 | tail -1"
docker exec "$C" mkdir -p /workspace/replay
docker cp "$RLDS/rlds_extraction_out/$NPZ" "$C:/workspace/replay/"
docker exec -it "$C" /workspace/src/scripts/replay_gui.sh rlds "$SEED" "$PIECE"
