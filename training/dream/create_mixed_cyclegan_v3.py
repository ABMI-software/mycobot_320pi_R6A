#!/usr/bin/env python3
"""Create mixed dataset CycleGAN : 3K real + 10K synth traduit (style réel).

Structure :
  real (4K total)
    → 3000 frames → mixed_cyclegan (train)
    → 1000 frames → real_test déjà créé → NE PAS TOUCHER

  synth_real_style (10K traduit par CycleGAN)
    → 10000 frames → mixed_cyclegan (train)

Output :
  dream_data/mixed_cyclegan/  → 13 000 frames (3K réel + 10K synth traduit)

Usage:
  cd training/
  python dream/create_mixed_cyclegan.py
"""

import os
import shutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REAL_DIR   = os.path.join(SCRIPT_DIR, "dream_data", "real")
SYNTH_DIR  = os.path.join(SCRIPT_DIR, "dream_data", "synthetic")
MIXED_DIR  = os.path.join(SCRIPT_DIR, "dream_data", "mixed_3k_5k")

N_REAL_TRAIN  = 3000
N_SYNTH_TRAIN = 5000


def get_frames(directory):
    return sorted([
        f.replace(".json", "")
        for f in os.listdir(directory)
        if f.endswith(".json") and not f.startswith("_")
    ])


def copy_frames(frames, src_dir, dst_dir, offset=0):
    os.makedirs(dst_dir, exist_ok=True)
    skipped = 0
    for i, frame in enumerate(frames):
        new_idx = f"{offset + i:06d}"
        for ext in [".json", ".rgb.png"]:
            src = os.path.join(src_dir, frame + ext)
            dst = os.path.join(dst_dir, new_idx + ext)
            if os.path.exists(src):
                shutil.copy2(src, dst)
            else:
                skipped += 1
        if i % 1000 == 0 and i > 0:
            print(f"    {i}/{len(frames)} frames...")
    return skipped


def copy_settings(src_dir, dst_dir):
    os.makedirs(dst_dir, exist_ok=True)
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(src_dir, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(dst_dir, f))
            print(f"  ✅ Copié : {f}")
        else:
            print(f"  ⚠️  Manquant : {f}")


def main():
    print(f"\n{'='*60}")
    print(f"CREATE MIXED CYCLEGAN DATASET")
    print(f"  Real  : {REAL_DIR}")
    print(f"  Synth : {SYNTH_DIR} (traduit CycleGAN)")
    print(f"  Mixed : {MIXED_DIR}")
    print(f"{'='*60}\n")

    # Vérifications
    for d in [REAL_DIR, SYNTH_DIR]:
        if not os.path.exists(d):
            print(f"❌ Dossier manquant : {d}")
            return

    # Vérifier que mixed_cyclegan n'existe pas déjà
    if os.path.exists(MIXED_DIR):
        print(f"⚠️  {MIXED_DIR} existe déjà !")
        answer = input("Écraser ? (o/n) : ")
        if answer.lower() != "o":
            print("Annulé.")
            return
        shutil.rmtree(MIXED_DIR)

    real_frames  = get_frames(REAL_DIR)
    synth_frames = get_frames(SYNTH_DIR)

    print(f"Frames disponibles :")
    print(f"  Real            : {len(real_frames)} frames")
    print(f"  Synth (CycleGAN): {len(synth_frames)} frames")

    real_train  = real_frames[:N_REAL_TRAIN]
    synth_train = synth_frames[:N_SYNTH_TRAIN]

    print(f"\nSplit :")
    print(f"  Real  train : {len(real_train)} frames (000000 → 002999)")
    print(f"  Synth train : {len(synth_train)} frames (003000 → 012999)")

    total_mixed = N_REAL_TRAIN + N_SYNTH_TRAIN
    print(f"\n📁 Création mixed_cyclegan ({total_mixed} frames)...")
    copy_settings(SYNTH_DIR, MIXED_DIR)

    print(f"  Copie réel ({N_REAL_TRAIN} frames)...")
    skipped = copy_frames(real_train, REAL_DIR, MIXED_DIR, offset=0)
    print(f"    → {N_REAL_TRAIN - skipped} frames copiées")

    print(f"  Copie synth traduit ({N_SYNTH_TRAIN} frames)...")
    skipped = copy_frames(synth_train, SYNTH_DIR, MIXED_DIR, offset=N_REAL_TRAIN)
    print(f"    → {N_SYNTH_TRAIN - skipped} frames copiées")

    print(f"\n{'='*60}")
    print(f"✅ TERMINÉ !")
    print(f"  mixed_cyclegan : {total_mixed} frames")
    print(f"    - Réel            : {N_REAL_TRAIN} ({100*N_REAL_TRAIN/total_mixed:.0f}%)")
    print(f"    - Synth CycleGAN  : {N_SYNTH_TRAIN} ({100*N_SYNTH_TRAIN/total_mixed:.0f}%)")
    print(f"\n  ⚠️  real_test et synth_test NON TOUCHÉS ✅")
    print(f"{'='*60}")
    print(f"\nProchaine étape :")
    print(f"  python dream/train_dream_mixed_aug_optimized_v2.py \\")
    print(f"    --data dream/dream_data/mixed_cyclegan \\")
    print(f"    --epochs 50 --batch-size 8 --workers 8 \\")
    print(f"    --output checkpoints_dream/vgg_cyclegan_e50")


if __name__ == "__main__":
    main()