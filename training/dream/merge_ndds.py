#!/usr/bin/env python3
"""Merge real + synthetic NDDS datasets with re-indexation."""

import os
import shutil

real_dir = "dream/dream_data/real"
synth_dir = "dream/dream_data/synthetic"
out_dir = "dream/dream_data/mixed_real_synth"
os.makedirs(out_dir, exist_ok=True)

for f in ["_camera_settings.json", "_object_settings.json"]:
    src = os.path.join(synth_dir, f)
    if os.path.exists(src):
        shutil.copy2(src, os.path.join(out_dir, f))

real_frames = sorted([f.replace(".json","") for f in os.listdir(real_dir)
                      if f.endswith(".json") and not f.startswith("_")])
synth_frames = sorted([f.replace(".json","") for f in os.listdir(synth_dir)
                       if f.endswith(".json") and not f.startswith("_")])

print(f"Real frames: {len(real_frames)}, Synth frames: {len(synth_frames)}")

for i, frame in enumerate(real_frames):
    new_idx = f"{i:06d}"
    for ext in [".json", ".rgb.png"]:
        src = os.path.join(real_dir, frame + ext)
        dst = os.path.join(out_dir, new_idx + ext)
        if os.path.exists(src):
            shutil.copy2(src, dst)
    if i % 500 == 0:
        print(f"  Real: {i}/{len(real_frames)}")

offset = len(real_frames)
for i, frame in enumerate(synth_frames):
    new_idx = f"{offset + i:06d}"
    for ext in [".json", ".rgb.png"]:
        src = os.path.join(synth_dir, frame + ext)
        dst = os.path.join(out_dir, new_idx + ext)
        if os.path.exists(src):
            shutil.copy2(src, dst)
    if i % 2000 == 0:
        print(f"  Synth: {i}/{len(synth_frames)}")

print(f"Done! Total: {len(real_frames) + len(synth_frames)} frames")