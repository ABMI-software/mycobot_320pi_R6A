#!/usr/bin/env python3
"""Évaluation de tous les checkpoints aug_grid_* sur données RÉELLES.

Basé sur evaluate_grid.py (synthétique) mais adapté pour :
  - Chercher checkpoints_dream/aug_grid_* au lieu de vgg_grid_*
  - Évaluer sur dream_data/real au lieu de synthetic
  - Sauvegarder dans aug_grid_results.txt

Usage:
  python dream/evaluate_grid_real.py \
      --data dream/dream_data/real

  # Ou avec chemin custom
  python dream/evaluate_grid_real.py \
      --data /path/to/real_ndds
"""

import os
import glob
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Évaluation grid search augmentations sur données réelles"
    )
    parser.add_argument(
        "--data", "-d",
        default=os.path.join(SCRIPT_DIR, "dream_data", "real"),
        help="Dataset réel NDDS (default: dream_data/real)"
    )
    parser.add_argument(
        "--output", "-o",
        default=os.path.join(SCRIPT_DIR, "real_augmentation_grid_results.txt"),
        help="Fichier de résultats (default: real_augmentation_grid_results.txt)"
    )
    args = parser.parse_args()

    # Chercher tous les checkpoints aug_grid_*
    grid_dirs = sorted(glob.glob(
        os.path.join(SCRIPT_DIR, "checkpoints_dream", "aug_grid_*")
    ))

    if not grid_dirs:
        print("❌ Aucun checkpoint aug_grid_* trouvé dans checkpoints_dream/")
        print("   Lance d'abord : python dream/train_dream_grid_search_real_augmentation.py")
        sys.exit(1)

    print(f"\n{'='*70}")
    print(f"ÉVALUATION GRID SEARCH — Données réelles")
    print(f"  Data réel  : {args.data}")
    print(f"  Checkpoints: {len(grid_dirs)} trouvés")
    print(f"  Résultats  : {args.output}")
    print(f"{'='*70}\n")

    results = []
    eval_script = os.path.join(SCRIPT_DIR, "evaluate_dream.py")

    for d in grid_dirs:
        weights_path = os.path.join(d, "best_network.pth")
        model_name = os.path.basename(d)

        if not os.path.exists(weights_path):
            print(f"  ⚠️  Pas de best_network.pth dans {model_name} — skip")
            continue

        print(f"\n▶ Évaluation : {model_name}")
        print(f"  Weights : {weights_path}")

        result = subprocess.run(
            [sys.executable, eval_script,
             "--weights", weights_path,
             "--data", args.data],
            capture_output=True, text=True
        )

        output = result.stdout
        print(output)

        results.append({
            "model": model_name,
            "output": output,
        })

    # Sauvegarder les résultats
    with open(args.output, "w") as f:
        f.write("ÉVALUATION GRID SEARCH AUGMENTATIONS — Données réelles\n")
        f.write(f"Date : {time.strftime('%Y-%m-%d %H:%M')}\n")
        f.write(f"Data : {args.data}\n")
        f.write("=" * 60 + "\n\n")
        for r in results:
            f.write(f"\n{'='*50}\n")
            f.write(f"{r['model']}\n")
            f.write(r['output'])

    print(f"\n{'='*70}")
    print(f"✅ {len(results)} modèles évalués")
    print(f"✅ Résultats sauvegardés → {args.output}")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()