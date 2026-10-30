#!/usr/bin/env python3
"""Grid search sur scale_limit de ShiftScaleRotate.

Teste plusieurs valeurs de scale_limit et évalue sur real_test.
Affiche un tableau comparatif à la fin.

Usage:
  python dream/grid_search_scale_limit.py \
      --data      dream/dream_data/mixed_cyclegan \
      --eval-data dream/dream_data/real_test \
      --output    checkpoints_dream/grid_scale \
      --epochs    5
"""

import argparse
import os
import sys
import subprocess
import json
import glob

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def run_training(scale, data, output_dir, epochs):
    cmd = [
        sys.executable,
        os.path.join(SCRIPT_DIR, "train_dream_mixed_aug_optimized_v12.py"),
        "--data",          data,
        "--epochs",        str(epochs),
        "--batch-size",    "8",
        "--workers",       "8",
        "--lr",            "0.0001",
        "--patience",      "5",
        "--seed",          "42",
        "--aug-brightness","0.4",
        "--aug-contrast",  "0.4",
        "--aug-hue",       "5",
        "--aug-sat",       "10",
        "--aug-noise",     "30",
        "--aug-scale-limit", str(scale),
        "--output",        output_dir,
    ]
    print(f"\n  Lancement training scale_limit={scale}...")
    result = subprocess.run(cmd, capture_output=False, text=True)
    return result.returncode == 0


def run_evaluation(weights, eval_data):
    cmd = [
        sys.executable,
        os.path.join(SCRIPT_DIR, "evaluate_dream_v2.py"),
        "--weights",     weights,
        "--data",        eval_data,
        "--split",       "all",
        "--max-samples", "1000",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    output = result.stdout + result.stderr

    # Parser le score
    score = None
    for line in output.split("\n"):
        if "Detection rate:" in line:
            try:
                score = float(line.split("(")[1].split("%")[0])
            except Exception:
                pass

    # Parser médiane
    median = None
    for line in output.split("\n"):
        if "OVERALL" in line and "N/A" not in line:
            parts = line.split()
            try:
                median = float(parts[2])
            except Exception:
                pass

    return score, median


def main():
    parser = argparse.ArgumentParser(description="Grid search scale_limit")
    parser.add_argument("--data",      "-d", required=True, help="Dataset mixte CycleGAN")
    parser.add_argument("--eval-data", "-e", required=True, help="Dataset real_test")
    parser.add_argument("--output",    "-o", required=True, help="Dossier checkpoints")
    parser.add_argument("--epochs",          type=int, default=5)
    parser.add_argument("--scales",          type=float, nargs="+",
                        default=[0.05, 0.08, 0.12, 0.15, 0.2],
                        help="Valeurs scale_limit à tester")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    scales = args.scales
    baseline_score = 52.9  # v10 seed=42 sans scale

    print(f"\n{'='*60}")
    print(f"GRID SEARCH SCALE_LIMIT — {len(scales)} configs × {args.epochs} epochs")
    print(f"  Dataset  : {args.data}")
    print(f"  Eval     : {args.eval_data}")
    print(f"  Scales   : {scales}")
    print(f"  Baseline : scale_limit=0.1 → {baseline_score}%")
    print(f"{'='*60}")

    results = {}

    for scale in scales:
        scale_str = str(scale).replace(".", "_")
        output_dir = os.path.join(args.output, f"scale_{scale_str}")

        print(f"\n{'='*60}")
        print(f"Scale: {scale}")
        print(f"{'='*60}")

        # Training
        success = run_training(scale, args.data, output_dir, args.epochs)
        if not success:
            print(f"  ❌ Training échoué pour scale_limit={scale}")
            continue

        # Évaluation
        weights = os.path.join(output_dir, "best_network.pth")
        if not os.path.exists(weights):
            print(f"  ❌ Checkpoint introuvable : {weights}")
            continue

        print(f"\n  📊 Évaluation sur real_test...")
        score, median = run_evaluation(weights, args.eval_data)

        if score is not None:
            results[f"scale_{scale}"] = {"score": score, "median": median, "scale": scale}
            print(f"  ✅ scale_{scale} : {score:.1f}% médiane={median:.2f}px")
        else:
            print(f"  ❌ Évaluation échouée pour scale_limit={scale}")

        # Sauvegarder résultats intermédiaires
        results_path = os.path.join(args.output, "grid_results.json")
        with open(results_path, "w") as f:
            json.dump(results, f, indent=2)

    # ---- Tableau final ----
    print(f"\n{'='*60}")
    print(f"RÉSULTATS GRID SEARCH SCALE_LIMIT")
    print(f"{'='*60}")
    print(f"  {'Scale':<12s} {'Score':>8s} {'Médiane':>10s}")
    print(f"  {'-'*12} {'-'*8} {'-'*10}")
    print(f"  {'0.1 (natif)':<12s} {baseline_score:>7.1f}%  {'2.82px':>10s}")

    # Trier par score
    sorted_results = sorted(results.values(), key=lambda x: x["score"], reverse=True)
    for r in sorted_results:
        marker = " ✅" if r["score"] > baseline_score else " ❌"
        median_str = f"{r['median']:.2f}px" if r['median'] else "N/A"
        print(f"  {str(r['scale']):<12s} {r['score']:>7.1f}%  {median_str:>10s}{marker}")

    if results:
        best = max(results.values(), key=lambda x: x["score"])
        print(f"\n  🏆 Meilleur : scale_limit={best['scale']} → {best['score']:.1f}%")
        if best["score"] > baseline_score:
            print(f"  🎉 Amélioration vs baseline : +{best['score']-baseline_score:.1f}%")
        else:
            print(f"  ⚠️  Aucune amélioration vs baseline (52.9%)")

    print(f"\n  Résultats sauvegardés : {os.path.join(args.output, 'grid_results.json')}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()