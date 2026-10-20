#!/usr/bin/env python3
"""Create mixed dataset : 2K real cam0 only + 10K synth CycleGAN."""

import os
import shutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REAL_DIR   = os.path.join(SCRIPT_DIR, "dream_data", "real_cam0_only")
SYNTH_DIR  = os.path.join(SCRIPT_DIR, "dream_data", "synth_real_style")
MIXED_DIR  = os.path.join(SCRIPT_DIR, "dream_data", "mixed_cam0_cyclegan")

N_REAL_TRAIN  = 2000
N_SYNTH_TRAIN = 10000

def get_frames(directory):
    return sorted([
        f.replace(".json", "")
        for f in os.listdir(directory)
        if f.endswith(".json") and not f.startswith("_")
    ])

def copy_frames(frames, src_dir, dst_dir, offset=0):
    os.makedirs(dst_dir, exist_ok=True)
    for i, frame in enumerate(frames):
        new_idx = f"{offset + i:06d}"
        for ext in [".json", ".rgb.png"]:
            src = os.path.join(src_dir, frame + ext)
            dst = os.path.join(dst_dir, new_idx + ext)
            if os.path.exists(src):
                shutil.copy2(src, dst)

def main():
    if os.path.exists(MIXED_DIR):
        shutil.rmtree(MIXED_DIR)
    os.makedirs(MIXED_DIR, exist_ok=True)

    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(SYNTH_DIR, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(MIXED_DIR, f))

    real_frames  = get_frames(REAL_DIR)[:N_REAL_TRAIN]
    synth_frames = get_frames(SYNTH_DIR)[:N_SYNTH_TRAIN]

    print(f"Real cam0 : {len(real_frames)} frames")
    print(f"Synth CycleGAN : {len(synth_frames)} frames")

    copy_frames(real_frames, REAL_DIR, MIXED_DIR, offset=0)
    copy_frames(synth_frames, SYNTH_DIR, MIXED_DIR, offset=N_REAL_TRAIN)

    print(f"✅ mixed_cam0_cyclegan : {N_REAL_TRAIN + N_SYNTH_TRAIN} frames")

if __name__ == "__main__":
    main()
