#!/usr/bin/env python3
"""Grid search sur les augmentations — 5 epochs par config via train_dream_mixed_aug_optimized_v6.py.

Usage:
  python dream/grid_search_aug.py \\
    --data dream/dream_data/mixed_cyclegan \\
    --eval-data dream/dream_data/real_test \\
    --output checkpoints_dream/grid_aug \\
    --epochs 5
"""

import os
import sys
import argparse
import subprocess
import json

CONFIGS = {
    "no_aug": {
        "aug_brightness": 0.0, "aug_contrast": 0.0,
        "aug_hue": 0.0, "aug_sat": 0.0,
        "aug_noise": 0.0,
    },
    "brightness_only": {
        "aug_brightness": 0.3, "aug_contrast": 0.3,
        "aug_hue": 0.0, "aug_sat": 0.0,
        "aug_noise": 0.0,
    },
    "hue_only": {
        "aug_brightness": 0.0, "aug_contrast": 0.0,
        "aug_hue": 10.0, "aug_sat": 15.0,
        "aug_noise": 0.0,
    },
    "brightness_hue": {
        "aug_brightness": 0.3, "aug_contrast": 0.3,
        "aug_hue": 5.0, "aug_sat": 10.0,
        "aug_noise": 0.0,
    },
    "brightness_hue_noise_light": {
        "aug_brightness": 0.3, "aug_contrast": 0.3,
        "aug_hue": 5.0, "aug_sat": 10.0,
        "aug_noise": 10.0,
    },
    "brightness_hue_noise_strong": {
        "aug_brightness": 0.4, "aug_contrast": 0.4,
        "aug_hue": 10.0, "aug_sat": 20.0,
        "aug_noise": 20.0,
    },
    "strong_brightness": {
        "aug_brightness": 0.5, "aug_contrast": 0.5,
        "aug_hue": 5.0, "aug_sat": 10.0,
        "aug_noise": 0.0,
    },
}


def run_training(config_name, config, data_dir, output_dir, epochs):
    checkpoint_dir = os.path.join(output_dir, config_name)
    os.makedirs(checkpoint_dir, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"Config: {config_name}")
    print(f"Params: {config}")
    print(f"{'='*60}")

    cmd = [
        sys.executable,
        "dream/train_dream_mixed_aug_optimized_v6.py",
        "--data",           data_dir,
        "--epochs",         str(epochs),
        "--batch-size",     "8",
        "--workers",        "8",
        "--lr",             "0.0001",
        "--patience",       "5",
        "--output",         checkpoint_dir,
        "--aug-brightness", str(config["aug_brightness"]),
        "--aug-contrast",   str(config["aug_contrast"]),
        "--aug-hue",        str(config["aug_hue"]),
        "--aug-sat",        str(config["aug_sat"]),
        "--aug-noise",      str(config["aug_noise"]),
    ]
    subprocess.run(cmd, check=False)
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
    print(f"GRID SEARCH AUGMENTATIONS — {len(CONFIGS)} configs × {args.epochs} epochs")
    print(f"Dataset : {args.data}")
    print(f"Eval    : {args.eval_data}")
    print(f"Script  : train_dream_mixed_aug_optimized_v6.py")
    print(f"{'='*60}\n")

    for config_name, config in CONFIGS.items():
        checkpoint_dir = run_training(
            config_name, config, args.data, args.output, args.epochs
        )

        weights = os.path.join(checkpoint_dir, "best_network.pth")
        if os.path.exists(weights):
            score, output = evaluate(weights, args.eval_data)
            results[config_name] = {"score": score, "config": config}
            print(f"\n✅ {config_name} : {score:.1f}%")
        else:
            print(f"\n❌ {config_name} : pas de checkpoint")
            results[config_name] = {"score": 0.0, "config": config}

    print(f"\n{'='*60}")
    print(f"RÉSULTATS GRID SEARCH")
    print(f"{'='*60}")
    sorted_results = sorted(results.items(), key=lambda x: x[1]["score"], reverse=True)
    for name, data in sorted_results:
        print(f"  {name:<35} : {data['score']:.1f}%")

    best = sorted_results[0]
    print(f"\n🏆 MEILLEUR : {best[0]} → {best[1]['score']:.1f}%")
    print(f"   Params   : {best[1]['config']}")

    results_file = os.path.join(args.output, "grid_results.json")
    with open(results_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n  Résultats sauvegardés : {results_file}")


if __name__ == "__main__":
    main()
