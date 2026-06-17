#!/bin/bash
# ============================================================================
# Full pipeline: merge datasets → convert to NDDS → retrain DREAM
# Run this after data collection is complete.
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
TRAINING_DIR="$REPO_DIR/training/dream"
VENV="/home/genji/ros_jazzy/venv_dream"

# Dataset paths
EXISTING_DATASET="$REPO_DIR/datasets/synthetic_dataset"
NEW_DATASET="/tmp/mycobot_synth_extra"
MERGED_DATASET="/tmp/mycobot_synth_merged"
NDDS_OUTPUT="/tmp/dream_data/synthetic_50k"

# Training config
EPOCHS=50
BATCH_SIZE=32
LR=0.0001
ARCH="vgg"
CHECKPOINT_DIR="$REPO_DIR/training/checkpoints_dream/vgg_weighted_50k_e50"

echo "============================================"
echo "  MyCobot 320 Pi – Full Training Pipeline"
echo "============================================"

# ---- Step 0: Pre-flight checks ----
echo ""
echo "=== Step 0: Pre-flight checks ==="

if [ ! -f "$EXISTING_DATASET/labels.csv" ]; then
    echo "❌ Existing dataset not found at $EXISTING_DATASET"
    exit 1
fi

if [ ! -f "$NEW_DATASET/labels.csv" ]; then
    echo "❌ New dataset not found at $NEW_DATASET"
    echo "   (data collection may still be running)"
    exit 1
fi

EXIST_COUNT=$(( $(wc -l < "$EXISTING_DATASET/labels.csv") - 1 ))
NEW_COUNT=$(( $(wc -l < "$NEW_DATASET/labels.csv") - 1 ))
echo "  Existing dataset: $EXIST_COUNT rows"
echo "  New dataset:      $NEW_COUNT rows"
echo "  Expected merged:  $(( EXIST_COUNT + NEW_COUNT )) rows"

# ---- Step 1: Merge datasets ----
echo ""
echo "=== Step 1: Merging datasets ==="
rm -rf "$MERGED_DATASET"

source "$VENV/bin/activate"
cd "$TRAINING_DIR"

python merge_and_convert.py \
    --datasets "$EXISTING_DATASET" "$NEW_DATASET" \
    --merged "$MERGED_DATASET" \
    --cameras front right left top

MERGED_COUNT=$(( $(wc -l < "$MERGED_DATASET/labels.csv") - 1 ))
MERGED_IMAGES=$(find "$MERGED_DATASET/images" -name "*.png" | wc -l)
echo "  ✅ Merged: $MERGED_COUNT rows, $MERGED_IMAGES images"

# ---- Step 2: Convert to NDDS ----
echo ""
echo "=== Step 2: Converting to NDDS format ==="
rm -rf "$NDDS_OUTPUT"

python convert_to_ndds.py \
    --input "$MERGED_DATASET" \
    --output "$NDDS_OUTPUT" \
    --source synth \
    --cameras front right left top

NDDS_FRAMES=$(ls "$NDDS_OUTPUT"/*.json 2>/dev/null | grep -v "_" | wc -l)
echo "  ✅ NDDS: $NDDS_FRAMES frames"

# ---- Step 3: Train DREAM ----
echo ""
echo "=== Step 3: Training DREAM (weighted VGG) ==="
echo "  Epochs:     $EPOCHS"
echo "  Batch size: $BATCH_SIZE"
echo "  LR:         $LR"
echo "  Output:     $CHECKPOINT_DIR"

mkdir -p "$CHECKPOINT_DIR"

python train_dream_weighted.py \
    --data "$NDDS_OUTPUT" \
    --arch "$ARCH" \
    --epochs "$EPOCHS" \
    --batch-size "$BATCH_SIZE" \
    --lr "$LR" \
    --output "$CHECKPOINT_DIR" \
    2>&1 | tee "$CHECKPOINT_DIR/train.log"

echo ""
echo "============================================"
echo "  Pipeline complete!"
echo "  Checkpoint: $CHECKPOINT_DIR"
echo "============================================"
