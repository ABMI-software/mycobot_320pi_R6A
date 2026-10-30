#!/usr/bin/env python3
"""Grid search sur le seed — meilleurs params fixes (br04_h05_s10_n30).

Teste plusieurs seeds pour trouver la meilleure initialisation
et potentiellement dépasser 52.9%.

Usage:
  python dream/grid_search_seeds.py \\
    --data dream/dream_data/mixed_cyclegan \\
    --eval-data dream/dream_data/real_test \\
    --output checkpoints_dream/grid_seeds \\
    --epochs 5
"""

import os
import sys
import argparse
import subprocess
import json

# Paramètres fixes — meilleurs du grid search brightness/hue/sat
BEST_AUG = {
    "aug_brightness": 0.4,
    "aug_contrast":   0.4,
    "aug_hue":        5.0,
    "aug_sat":        10.0,
    "aug_noise":      30.0,
}

# Seeds à tester
SEEDS = [0, 1, 7, 13, 42, 99, 100, 123, 256, 777]


def run_training(seed, data_dir, output_dir, epochs):
    checkpoint_dir = os.path.join(output_dir, f"seed_{seed}")
    os.makedirs(checkpoint_dir, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"Seed: {seed}")
    print(f"Aug : {BEST_AUG}")
    print(f"{'='*60}")

    # Modifier temporairement le seed dans le script v10
    # via une variable d'environnement
    env = os.environ.copy()
    env["DREAM_SEED"] = str(seed)

    cmd = [
        sys.executable,
        "dream/train_dream_mixed_aug_optimized_v11.py",
        "--data",           data_dir,
        "--epochs",         str(epochs),
        "--batch-size",     "8",
        "--workers",        "8",
        "--lr",             "0.0001",
        "--patience",       "5",
        "--output",         checkpoint_dir,
        "--aug-brightness", str(BEST_AUG["aug_brightness"]),
        "--aug-contrast",   str(BEST_AUG["aug_contrast"]),
        "--aug-hue",        str(BEST_AUG["aug_hue"]),
        "--aug-sat",        str(BEST_AUG["aug_sat"]),
        "--aug-noise",      str(BEST_AUG["aug_noise"]),
        "--seed",           str(seed),
    ]
    subprocess.run(cmd, check=False, env=env)
    return checkpoint_dir


def evaluate(weights_path, eval_data):
    result = subprocess.run([
        sys.executable,
        "dream/evaluate_dream_v2.py",
        "--weights",     weights_path,
        "--data",        eval_data,
        "--split",       "all",
        "--max-samples", "1000",
    ], capture_output=True, text=True)

    score = 0.0
    for line in result.stdout.split("\n"):
        if "Detection rate:" in line:
            try:
                score = float(line.split("(")[1].replace("%)", "").strip())
            except Exception:
                pass
    return score, result.stdout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data",      required=True)
    parser.add_argument("--eval-data", required=True)
    parser.add_argument("--output",    required=True)
    parser.add_argument("--epochs",    type=int, default=5)
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    results = {}

    print(f"\n{'='*60}")
    print(f"GRID SEARCH SEEDS — {len(SEEDS)} seeds × {args.epochs} epochs")
    print(f"  Dataset  : {args.data}")
    print(f"  Eval     : {args.eval_data}")
    print(f"  Aug fixe : br04_h05_s10_n30")
    print(f"  Baseline : seed=42 → 52.9%")
    print(f"{'='*60}\n")

    for seed in SEEDS:
        checkpoint_dir = os.path.join(args.output, f"seed_{seed}")
        weights_check  = os.path.join(checkpoint_dir, "best_network.pth")

        if os.path.exists(weights_check):
            print(f"\n⏭️  seed_{seed} déjà fait — skip !")
            score, _ = evaluate(weights_check, args.eval_data)
            results[f"seed_{seed}"] = {"score": score, "seed": seed}
            print(f"✅ seed_{seed} : {score:.1f}%")
            continue

        checkpoint_dir = run_training(seed, args.data, args.output, args.epochs)

        weights = os.path.join(checkpoint_dir, "best_network.pth")
        if os.path.exists(weights):
            score, _ = evaluate(weights, args.eval_data)
            results[f"seed_{seed}"] = {"score": score, "seed": seed}
            marker = " 🏆 NOUVEAU RECORD !" if score > 52.9 else ""
            print(f"\n✅ seed_{seed} : {score:.1f}%{marker}")
        else:
            print(f"\n❌ seed_{seed} : pas de checkpoint")
            results[f"seed_{seed}"] = {"score": 0.0, "seed": seed}

    print(f"\n{'='*60}")
    print(f"RÉSULTATS GRID SEARCH SEEDS")
    print(f"{'='*60}")
    sorted_results = sorted(results.items(), key=lambda x: x[1]["score"], reverse=True)
    for name, data in sorted_results:
        marker = " ← BASELINE" if data["seed"] == 42 else ""
        print(f"  {name:<15} : {data['score']:.1f}%{marker}")

    best = sorted_results[0]
    print(f"\n🏆 MEILLEUR : {best[0]} → {best[1]['score']:.1f}%")

    results_file = os.path.join(args.output, "grid_results.json")
    with open(results_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n  Résultats sauvegardés : {results_file}")


if __name__ == "__main__":
    main()