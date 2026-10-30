#!/usr/bin/env python3
"""Create mixed dataset : 3K real + 10K synth for training, 1K real for test.

Structure :
  real (4K total)
    → 3000 frames → mixed_3k_10k (train)
    → 1000 frames → real_test     (test set, jamais touché)

  synthetic (20K total)
    → 10000 frames → mixed_3k_10k (train)

Output :
  dream_data/mixed_3k_10k/   → 13 000 frames (3K réel + 10K synth)
  dream_data/real_test/      → 1 000 frames réelles (test set indépendant)

Usage:
  cd training/
  python dream/create_mixed_dataset.py
"""

import os
import shutil

# ============================================================
# Chemins
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.dirname(SCRIPT_DIR)   # training/

REAL_DIR   = os.path.join(BASE_DIR, "dream_data", "real")
SYNTH_DIR  = os.path.join(BASE_DIR, "dream_data", "synthetic")
MIXED_DIR  = os.path.join(BASE_DIR, "dream_data", "mixed_3k_10k")
TEST_DIR   = os.path.join(BASE_DIR, "dream_data", "real_test")

N_REAL_TRAIN = 3000   # frames réelles pour l'entraînement
N_REAL_TEST  = 1000   # frames réelles pour le test
N_SYNTH      = 10000  # frames synthétiques pour l'entraînement


def get_frames(directory, exclude_settings=True):
    """Retourne les frames triées (sans les fichiers _settings)."""
    frames = sorted([
        f.replace(".json", "")
        for f in os.listdir(directory)
        if f.endswith(".json") and not f.startswith("_")
    ])
    return frames


def copy_frames(frames, src_dir, dst_dir, offset=0):
    """Copie les frames avec re-indexation."""
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
    """Copie les fichiers _camera_settings.json et _object_settings.json."""
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
    print(f"CREATE MIXED DATASET")
    print(f"  Real dir  : {REAL_DIR}")
    print(f"  Synth dir : {SYNTH_DIR}")
    print(f"  Mixed out : {MIXED_DIR}")
    print(f"  Test out  : {TEST_DIR}")
    print(f"{'='*60}\n")

    # Vérifier que les dossiers source existent
    for d in [REAL_DIR, SYNTH_DIR]:
        if not os.path.exists(d):
            print(f"❌ Dossier manquant : {d}")
            return

    # Récupérer les frames
    real_frames  = get_frames(REAL_DIR)
    synth_frames = get_frames(SYNTH_DIR)

    print(f"Frames disponibles :")
    print(f"  Real  : {len(real_frames)} frames")
    print(f"  Synth : {len(synth_frames)} frames")

    # Vérifier qu'on a assez de frames
    if len(real_frames) < N_REAL_TRAIN + N_REAL_TEST:
        print(f"❌ Pas assez de frames réelles ({len(real_frames)} < {N_REAL_TRAIN + N_REAL_TEST})")
        return
    if len(synth_frames) < N_SYNTH:
        print(f"❌ Pas assez de frames synthétiques ({len(synth_frames)} < {N_SYNTH})")
        return

    # Split réel : 3K train + 1K test
    real_train = real_frames[:N_REAL_TRAIN]
    real_test  = real_frames[N_REAL_TRAIN:N_REAL_TRAIN + N_REAL_TEST]
    synth_train = synth_frames[:N_SYNTH]

    print(f"\nSplit :")
    print(f"  Real train : {len(real_train)} frames (000000 → {N_REAL_TRAIN-1:06d})")
    print(f"  Real test  : {len(real_test)} frames ({N_REAL_TRAIN:06d} → {N_REAL_TRAIN+N_REAL_TEST-1:06d})")
    print(f"  Synth train: {len(synth_train)} frames")

    # ---- Créer real_test ----
    print(f"\n📁 Création real_test ({N_REAL_TEST} frames)...")
    os.makedirs(TEST_DIR, exist_ok=True)
    copy_settings(SYNTH_DIR, TEST_DIR)
    skipped = copy_frames(real_test, REAL_DIR, TEST_DIR, offset=0)
    print(f"  ✅ {len(real_test) - skipped} frames copiées → {TEST_DIR}")

    # ---- Créer mixed_3k_10k ----
    print(f"\n📁 Création mixed_3k_10k ({N_REAL_TRAIN + N_SYNTH} frames)...")
    os.makedirs(MIXED_DIR, exist_ok=True)
    copy_settings(SYNTH_DIR, MIXED_DIR)

    # Copier réel en premier (000000 → 002999)
    print(f"  Copie réel ({N_REAL_TRAIN} frames)...")
    skipped = copy_frames(real_train, REAL_DIR, MIXED_DIR, offset=0)
    print(f"    → {N_REAL_TRAIN - skipped} frames copiées")

    # Copier synth (003000 → 012999)
    print(f"  Copie synth ({N_SYNTH} frames)...")
    skipped = copy_frames(synth_train, SYNTH_DIR, MIXED_DIR, offset=N_REAL_TRAIN)
    print(f"    → {N_SYNTH - skipped} frames copiées")

    # Résumé
    total_mixed = N_REAL_TRAIN + N_SYNTH
    print(f"\n{'='*60}")
    print(f"✅ TERMINÉ !")
    print(f"  mixed_3k_10k : {total_mixed} frames")
    print(f"    - Réel  : {N_REAL_TRAIN} frames ({100*N_REAL_TRAIN/total_mixed:.0f}%)")
    print(f"    - Synth : {N_SYNTH} frames ({100*N_SYNTH/total_mixed:.0f}%)")
    print(f"  real_test    : {N_REAL_TEST} frames (test set indépendant)")
    print(f"{'='*60}")
    print(f"\nProchaine étape :")
    print(f"  python dream/preprocess_synth.py")


if __name__ == "__main__":
    main()