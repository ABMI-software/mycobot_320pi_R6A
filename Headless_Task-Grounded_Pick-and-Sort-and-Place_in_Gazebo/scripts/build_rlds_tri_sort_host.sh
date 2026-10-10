#!/usr/bin/env bash
# From the WSL2 HOST, after convert_tri_sort.sh: bring the extracted episodes
# into the RLDS builder, build the TFDS dataset in rlds_builder, check it.
set -e
RLDS=~/tomislav/robotics/mycobot_320pi_R6A_tomislav_07_10_2026_ver_2/ROS2_to_RLDS_Conversion_OpenVLA
B=$RLDS/rlds_dataset_builder/mycobot_tri_sort
[ -d "$RLDS/rlds_dataset_builder" ] || git clone --depth 1 https://github.com/moojink/rlds_dataset_builder.git "$RLDS/rlds_dataset_builder"
mkdir -p "$B/data/train"
cp "$RLDS"/overrides/rlds_dataset_builder/mycobot_tri_sort/*.py "$B/"
rm -f "$B"/data/train/*.npy
docker cp gazebo_to_lerobot:/workspace/rlds_tri_sort/data/train/. "$B/data/train/"
echo "$(ls "$B"/data/train/*.npy | wc -l) episodes in $B/data/train"
docker exec rlds_builder bash -c 'cd /workspace/rlds_dataset_builder/mycobot_tri_sort && \
    tfds build --overwrite --data_dir /workspace/rlds_dataset_builder/tfds_out 2>&1 | grep -E "num_examples|rror" | tail -3'
docker exec rlds_builder bash -c 'cd /workspace && python extraction/check_rlds_tri_sort.py \
    rlds_dataset_builder/tfds_out/mycobot_tri_sort/1.0.0 rlds_extraction_out/check_sheet_tri_sort.png 2>&1 \
    | grep -vE "cuda|TensorRT|oneDNN|rebuild TensorFlow|cpu_feature_guard|^\s*$"'
