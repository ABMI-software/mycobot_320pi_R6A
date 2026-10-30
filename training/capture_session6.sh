#!/bin/bash
# Session 6 — NEW real-data capture, in its OWN directory (does NOT merge
# with real_3cam's 2500 poses / 7500 images), but continues the global
# pose index at 2500 (real_3cam_session6/labels.csv + sessions.json were
# pre-seeded with real_3cam's last row so capture_real_3cam.py auto-detects
# start_idx=2500 / session_num=6 with zero code changes).
# 100 NEW poses x 3 cameras (arducam, svpro, astra) = 300 NEW images,
# never seen by any model.
export PYTHONNOUSERSITE=1  # évite que ~/.local ne repollue venv_dream avec un opencv incompatible

DATASET="$HOME/Osama_ws/src/mycobot_R6A/training/dream_data/real_3cam_session6"

echo "📁 Dataset (nouveau, séparé de real_3cam) : ${DATASET}"
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
    --num-samples   100 \
    --cooldown-min  20 \
    --pi-host       10.10.0.224 \
    --arducam-index "${ARDUCAM}" \
    --svpro-index   "${SVPRO}" \
    --preview
