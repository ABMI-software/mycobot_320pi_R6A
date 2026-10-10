#!/usr/bin/env bash
# From a fresh clone, on the HOST (Linux or WSL2; needs only git and Docker):
# build the images and assemble the ROS workspace that records Osama's
# four-object sort and converts it to LeRobot and RLDS.
#
#   Headless_Task-Grounded_Pick-and-Sort-and-Place_in_Gazebo/docker/setup_workspace.sh [workspace]
#
# workspace defaults to ~/tri_sort_ws. Its src/ is what the container mounts at
# /workspace/src. Re-run after pulling: the copies from the repository are
# refreshed, the cloned rosetta packages are kept. Then run_container.sh.
set -e
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
HT="$(dirname "$HERE")"
REPO="$(dirname "$HT")"
G2L="$REPO/Gazebo_to_LeRobot_Pipeline"
WS="${1:-$HOME/tri_sort_ws}"
S="$WS/src"
say() { echo "== $*"; }

say "1/4 images: gazebo_to_lerobot:jazzy-harmonic (ROS 2 Jazzy, Gazebo Harmonic, MoveIt), then tri_sort:jazzy-harmonic"
docker build -t gazebo_to_lerobot:jazzy-harmonic -f "$G2L/docker/Dockerfile" "$G2L/docker"
docker build -t tri_sort:jazzy-harmonic "$HERE"
docker build -t rlds_builder:py39-tf213 -f "$REPO/ROS2_to_RLDS_Conversion_OpenVLA/docker/Dockerfile" \
    "$REPO/ROS2_to_RLDS_Conversion_OpenVLA"

say "2/4 rosetta packages at their pinned commits ($HERE/rosetta.repos)"
mkdir -p "$S"
if [ ! -d "$S/rosetta" ]; then
    docker run --rm -v "$S:/ws_src" -v "$HERE/rosetta.repos:/rosetta.repos:ro" tri_sort:jazzy-harmonic \
        bash -c 'vcs import /ws_src < /rosetta.repos && chown -R '"$(id -u):$(id -g)"' /ws_src'
fi

say "3/4 packages from this repository"
for p in mycobot_description mycobot_gateway; do
    rm -rf "${S:?}/$p"; cp -a "$REPO/$p" "$S/$p"
done
for p in mycobot_moveit_config gazebo_to_lerobot_bringup; do
    rm -rf "${S:?}/$p"; cp -a "$G2L/src/$p" "$S/$p"
done
# 500 Hz made controller activation time out under WSL2 scheduling jitter; 100 Hz
# is what every recording of 2026-10-10 ran with.
sed -i 's/^    update_rate: 500$/    update_rate: 100/' "$S/mycobot_description/config/controller.yaml"
cp "$HT/worlds/sorting_table.sdf" "$S/mycobot_description/worlds/"
find "$S/mycobot_gateway" "$S/mycobot_description" -name build -o -name '*.egg-info' -o -name __pycache__ \
    | xargs rm -rf

say "4/4 scripts and contracts (/workspace/src/scripts, scripts/rlds, contracts, training/dream)"
rm -rf "$S/scripts" "$S/contracts" "$S/training"
mkdir -p "$S/scripts/rlds" "$S/contracts" "$S/training/dream"
cp "$HT"/scripts/{record_tri_seed.sh,record_tri_batch.sh,record_tri_sort.py,recorder_lifecycle.py} "$S/scripts/"
cp "$HT"/scripts/{commanded_action_relay.py,convert_tri_sort.sh,summarise_tri_batch.py} "$S/scripts/"
cp "$HT"/scripts/{check_lerobot_dataset.py,check_action_lead.py,frame_gaps.py,count_frames.py} "$S/scripts/"
cp "$HT"/scripts/{compare_camera_poses.py,grab_views_on_status.py,view_size_check.py} "$S/scripts/"
cp "$HT"/scripts/{export_lerobot_episode.py,replay_episode.py,replay_gui.sh,tri_cleanup.sh} "$S/scripts/"
cp "$REPO/scripts/run_sort_gui.sh" "$REPO/scripts/diff_ik.py" "$S/scripts/"
cp "$REPO/training/dream/mycobot_fk.py" "$S/training/dream/"
cp "$REPO"/ROS2_to_RLDS_Conversion_OpenVLA/extraction/{extract_episodes.py,extract_tri_sort.py} "$S/scripts/rlds/"
cp "$HT/contracts/mycobot_tri_sort.yaml" "$G2L/src/contracts/mycobot_320_pi.yaml" "$S/contracts/"
chmod +x "$S"/scripts/*.sh

say "done: $S"
echo "next: $HERE/run_container.sh $WS"
