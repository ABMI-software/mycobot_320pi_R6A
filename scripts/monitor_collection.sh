#!/bin/bash
# Quick monitor for the synthetic data collection process
DATASET_DIR="/tmp/mycobot_synth_extra"
TARGET=7500

while true; do
    if [ ! -d "$DATASET_DIR/images/front" ]; then
        echo "⏳ Waiting for collection to start..."
        sleep 5
        continue
    fi
    
    CURRENT=$(find "$DATASET_DIR/images/front" -name "*.png" 2>/dev/null | wc -l)
    TOTAL_IMAGES=$(find "$DATASET_DIR/images" -name "*.png" 2>/dev/null | wc -l)
    DISK=$(du -sh "$DATASET_DIR" 2>/dev/null | cut -f1)
    PCT=$((CURRENT * 100 / TARGET))
    REMAINING=$((TARGET - CURRENT))
    
    # Check if process is running
    if pgrep -f "synthetic_data_collector_v2" > /dev/null; then
        STATUS="🟢 Running"
    else
        STATUS="🔴 Stopped"
    fi
    
    echo "[$(date +%H:%M:%S)] $STATUS | $CURRENT/$TARGET poses ($PCT%) | $TOTAL_IMAGES images | $DISK | ~$REMAINING remaining"
    
    if [ "$CURRENT" -ge "$TARGET" ]; then
        echo "✅ Collection complete!"
        break
    fi
    
    if [ "$STATUS" = "🔴 Stopped" ] && [ "$CURRENT" -lt "$TARGET" ]; then
        echo "⚠️  Collection stopped before reaching target!"
        break
    fi
    
    sleep 30
done
