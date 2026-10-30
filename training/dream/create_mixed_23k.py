#!/usr/bin/env python3
"""Create mixed dataset : 3K real + 20K synth = 23K train + 1K real test."""

import os
import shutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.dirname(SCRIPT_DIR)

REAL_DIR   = os.path.join(BASE_DIR, "dream_data", "real")
SYNTH_DIR = os.path.join(BASE_DIR, "dream_data", "synth_real_style_20k")
MIXED_DIR = os.path.join(BASE_DIR, "dream_data", "mixed_23k_cyclegan")
TEST_DIR   = os.path.join(BASE_DIR, "dream_data", "real_test")

N_REAL_TRAIN = 3000
N_REAL_TEST  = 1000
N_SYNTH      = 20000


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
    return skipped


def copy_settings(src_dir, dst_dir):
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(src_dir, f)
        dst = os.path.join(dst_dir, f)
        if os.path.exists(src):
            shutil.copy2(src, dst)
            print(f"  ✅ Copié : {f}")
        else:
            print(f"  ⚠️  Manquant : {f}")


def main():
    print(f"\n{'='*60}")
    print(f"CREATE MIXED DATASET 23K")
    print(f"  Real dir  : {REAL_DIR}")
    print(f"  Synth dir : {SYNTH_DIR}")
    print(f"  Mixed out : {MIXED_DIR}")
    print(f"  Test out  : {TEST_DIR}")
    print(f"{'='*60}\n")

    for d in [REAL_DIR, SYNTH_DIR]:
        if not os.path.exists(d):
            print(f"❌ Dossier manquant : {d}")
            return

    real_frames  = get_frames(REAL_DIR)
    synth_frames = get_frames(SYNTH_DIR)

    print(f"Frames disponibles :")
    print(f"  Real  : {len(real_frames)} frames")
    print(f"  Synth : {len(synth_frames)} frames")

    if len(real_frames) < N_REAL_TRAIN + N_REAL_TEST:
        print(f"❌ Pas assez de frames réelles ({len(real_frames)} < {N_REAL_TRAIN + N_REAL_TEST})")
        return
    if len(synth_frames) < N_SYNTH:
        print(f"❌ Pas assez de frames synthétiques ({len(synth_frames)} < {N_SYNTH})")
        return

    real_train  = real_frames[:N_REAL_TRAIN]
    real_test   = real_frames[N_REAL_TRAIN:N_REAL_TRAIN + N_REAL_TEST]
    synth_train = synth_frames[:N_SYNTH]

    print(f"\nSplit :")
    print(f"  Real train  : {len(real_train)} frames")
    print(f"  Real test   : {len(real_test)} frames")
    print(f"  Synth train : {len(synth_train)} frames")

    # real_test — skip si déjà existant
    if os.path.exists(TEST_DIR):
        print(f"\n⏭️  real_test existe déjà — skip !")
    else:
        print(f"\n📁 Création real_test...")
        os.makedirs(TEST_DIR, exist_ok=True)
        copy_settings(SYNTH_DIR, TEST_DIR)
        copy_frames(real_test, REAL_DIR, TEST_DIR, offset=0)
        print(f"  ✅ {len(real_test)} frames copiées")

    # mixed_23k
    print(f"\n📁 Création mixed_23k ({N_REAL_TRAIN + N_SYNTH} frames)...")
    os.makedirs(MIXED_DIR, exist_ok=True)
    copy_settings(SYNTH_DIR, MIXED_DIR)

    print(f"  Copie réel ({N_REAL_TRAIN} frames)...")
    copy_frames(real_train, REAL_DIR, MIXED_DIR, offset=0)

    print(f"  Copie synth ({N_SYNTH} frames)...")
    copy_frames(synth_train, SYNTH_DIR, MIXED_DIR, offset=N_REAL_TRAIN)

    print(f"\n{'='*60}")
    print(f"✅ TERMINÉ !")
    print(f"  mixed_23k : {N_REAL_TRAIN + N_SYNTH} frames")
    print(f"    - Réel  : {N_REAL_TRAIN} (13%)")
    print(f"    - Synth : {N_SYNTH} (87%)")
    print(f"  real_test : {N_REAL_TEST} frames indépendantes")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()