#!/bin/bash
# Accumulating real-data capture in dream_data (NOT /tmp).
# Each run appends 500 NEW poses, continuing the index from the previous
# session — true cross-session continuity, zero repeated movement.
# After the 500 poses, a cooldown countdown is shown so the servos cool.
DATASET="$HOME/Osama_ws/src/mycobot_R6A/training/dream_data/real_3cam"

echo "📁 Dataset accumulé : ${DATASET}"
if [ -f "${DATASET}/sessions.json" ]; then
    DONE=$(grep -c '"session"' "${DATASET}/sessions.json")
    echo "♻️  ${DONE} session(s) déjà faite(s) — celle-ci reprend après la dernière."
else
    echo "🆕 Première session."
fi

# Stable by-id capture nodes (index0 = video stream; index1 = metadata, never use).
# Raw /dev/videoN numbers shuffle when cameras are replugged or another webcam is present.
ARDUCAM="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_8mp_SN0001-video-index0"
SVPRO="/dev/v4l/by-id/usb-5MP_USB_Camera_5MP_USB_Camera_01.00.00-video-index0"

python3 ~/Osama_ws/src/mycobot_R6A/training/capture_real_3cam.py \
    --output        "${DATASET}" \
    --num-samples   500 \
    --cooldown-min  60 \
    --pi-host       10.10.0.224 \
    --arducam-index "${ARDUCAM}" \
    --svpro-index   "${SVPRO}" \
    --preview
