#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grid search automatique CROISÉ des poids link4 x link5 x link6 sur synth_50k_ndds,
en utilisant train_dream_ultimate_v3.py (avec --seed natif).

Reproduit la méthodologie du rapport "Grid Search sur les Weights des Keypoints
Distaux" (20K images) : link4, link5 et link6 varient chacun INDÉPENDAMMENT dans
WEIGHTS_RANGE, base/link1/link2/link3 restent fixés à 1.0.

Pour chaque combinaison (w4, w5, w6) dans WEIGHTS_RANGE^3 :
  1. Entraîne avec kp-weights = "1.0,1.0,1.0,1.0,w4,w5,w6" et --seed fixe
  2. Évalue immédiatement sur le split val de synth_50k_ndds
  3. Sauvegarde les résultats dans grid_search_link456_results.txt (incrémental)

Usage:
    python3 dream/grid_search_50k_weights.py \
        --data dream/dream_data/synth_50k_ndds \
        --epochs 5 --patience 3 \
        --batch-size 8 --workers 8 \
        --seed 42 \
        --values 1.5 3.0 5.0 7.0
"""

import argparse
import itertools
import os
import re
import subprocess
import time


def parse_args():
    p = argparse.ArgumentParser(
        description="Grid search croisé link4 x link5 x link6 (utilise train_dream_ultimate_v3.py)"
    )
    p.add_argument("--data", default="dream/dream_data/synth_50k_ndds")
    p.add_argument("--epochs", type=int, default=5)
    p.add_argument("--patience", type=int, default=3)
    p.add_argument("--batch-size", type=int, default=8)
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--lr", type=float, default=0.0001)
    p.add_argument("--values", type=float, nargs="+", default=[1.5, 3.0, 5.0, 7.0],
                    help="Valeurs à tester pour link4, link5 et link6 (grille croisée)")
    p.add_argument("--output-root", default="checkpoints_dream")
    p.add_argument("--train-script", default="dream/train_dream_ultimate_v3.py")
    p.add_argument("--eval-script", default="dream/evaluate_dream_v2.py")
    p.add_argument("--eval-split", default="val")
    p.add_argument("--eval-max-samples", type=int, default=1000)
    p.add_argument("--seed", type=int, default=42, help="Seed fixe pour reproductibilité")
    p.add_argument("--results-file", default="grid_search_link456_results.txt")
    p.add_argument("--resume", action="store_true",
                    help="Reprend en sautant les combinaisons déjà présentes dans results-file")
    return p.parse_args()


def extract_overall_detection(eval_stdout: str):
    """Extrait le taux de détection global, l'erreur moyenne, et les métriques
    par keypoint distal depuis la sortie de evaluate_dream_v2.py."""
    det_rate = None
    overall_mean = None
    per_kp = {}

    m = re.search(r"Detection rate:\s*(\d+)/(\d+)\s*\(([\d.]+)%\)", eval_stdout)
    if m:
        det_rate = float(m.group(3))

    m2 = re.search(r"OVERALL\s+([\d.]+)\s+([\d.]+)", eval_stdout)
    if m2:
        overall_mean = float(m2.group(1))

    # Per-keypoint detection % + median px, ex: "link4 : 96.4% det, mediane 7.09 px"
    for kp in ["link4", "link5", "link6"]:
        m3 = re.search(
            rf"{kp}\s*:\s*([\d.]+)%\s*det,?\s*m[eé]diane\s*([\d.]+)\s*px",
            eval_stdout, re.IGNORECASE,
        )
        if m3:
            per_kp[kp] = {"det_pct": float(m3.group(1)), "median_px": float(m3.group(2))}

    return det_rate, overall_mean, per_kp


def load_done_combos(results_file):
    """Pour --resume : relit le fichier de résultats existant et retourne le set
    des (w4, w5, w6) déjà traités avec succès."""
    done = set()
    if not os.path.exists(results_file):
        return done
    with open(results_file) as f:
        for line in f:
            m = re.match(r"w4=([\d.]+)\s+w5=([\d.]+)\s+w6=([\d.]+)\s+status=OK", line)
            if m:
                done.add((m.group(1), m.group(2), m.group(3)))
    return done


def main():
    args = parse_args()

    combos = list(itertools.product(args.values, args.values, args.values))
    print("=" * 70)
    print(f"🔬 GRID SEARCH CROISÉ — link4 × link5 × link6 ∈ {args.values}")
    print(f"   Combinaisons : {len(combos)} (= {len(args.values)}³)")
    print(f"   Dataset      : {args.data}")
    print(f"   Epochs       : {args.epochs}  Patience : {args.patience}")
    print(f"   Batch/Workers: {args.batch_size}/{args.workers}")
    print(f"   Seed fixe    : {args.seed}")
    print(f"   Script train : {args.train_script}")
    print("=" * 70)

    done_combos = load_done_combos(args.results_file) if args.resume else set()
    if done_combos:
        print(f"♻️  Reprise : {len(done_combos)} combinaisons déjà terminées, ignorées")

    results = []
    t_start = time.time()

    env = os.environ.copy()
    env["NO_ALBUMENTATIONS_UPDATE"] = "1"

    for w4, w5, w6 in combos:
        w4_str, w5_str, w6_str = f"{w4:.1f}", f"{w5:.1f}", f"{w6:.1f}"

        if (w4_str, w5_str, w6_str) in done_combos:
            print(f"\n⏭️  Skip (déjà fait) — w4={w4_str} w5={w5_str} w6={w6_str}")
            continue

        tag = f"vgg_grid_50k_w4_{w4_str.replace('.', '_')}_w5_{w5_str.replace('.', '_')}_w6_{w6_str.replace('.', '_')}"
        out_dir = os.path.join(args.output_root, tag)
        weights_str = f"1.0,1.0,1.0,1.0,{w4_str},{w5_str},{w6_str}"

        print(f"\n{'='*70}")
        print(f"▶️  ENTRAÎNEMENT — w4={w4_str} w5={w5_str} w6={w6_str}  (seed={args.seed})")
        print(f"   Output: {out_dir}")
        print(f"{'='*70}\n")

        t0 = time.time()
        train_cmd = [
            "python", args.train_script,
            "--data", args.data,
            "--epochs", str(args.epochs),
            "--patience", str(args.patience),
            "--batch-size", str(args.batch_size),
            "--workers", str(args.workers),
            "--lr", str(args.lr),
            "--kp-weights", weights_str,
            "--seed", str(args.seed),
            "--output", out_dir,
        ]
        train_result = subprocess.run(train_cmd, text=True, env=env)
        train_time_min = (time.time() - t0) / 60.0

        if train_result.returncode != 0:
            print(f"  ❌ Entraînement échoué pour w4={w4_str} w5={w5_str} w6={w6_str} — passage au suivant")
            results.append({
                "w4": w4_str, "w5": w5_str, "w6": w6_str, "status": "TRAIN_FAILED",
                "det_rate": None, "overall_mean": None, "per_kp": {},
                "train_time_min": train_time_min,
            })
            _write_results(args.results_file, results, t_start, args)
            continue

        weights_path = os.path.join(out_dir, "best_network.pth")
        if not os.path.exists(weights_path):
            print(f"  ❌ Pas de checkpoint trouvé : {weights_path}")
            results.append({
                "w4": w4_str, "w5": w5_str, "w6": w6_str, "status": "NO_CHECKPOINT",
                "det_rate": None, "overall_mean": None, "per_kp": {},
                "train_time_min": train_time_min,
            })
            _write_results(args.results_file, results, t_start, args)
            continue

        print(f"\n{'='*70}")
        print(f"📊 ÉVALUATION — w4={w4_str} w5={w5_str} w6={w6_str}")
        print(f"{'='*70}\n")

        eval_cmd = [
            "python", args.eval_script,
            "--weights", weights_path,
            "--data", args.data,
            "--split", args.eval_split,
            "--max-samples", str(args.eval_max_samples),
        ]
        eval_result = subprocess.run(eval_cmd, capture_output=True, text=True)
        print(eval_result.stdout)
        if eval_result.returncode != 0:
            print(eval_result.stderr)

        det_rate, overall_mean, per_kp = extract_overall_detection(eval_result.stdout)

        results.append({
            "w4": w4_str, "w5": w5_str, "w6": w6_str,
            "status": "OK",
            "det_rate": det_rate,
            "overall_mean": overall_mean,
            "per_kp": per_kp,
            "train_time_min": train_time_min,
            "eval_output": eval_result.stdout,
        })

        # Sauvegarde incrémentale après chaque run (au cas où ça plante plus tard)
        _write_results(args.results_file, results, t_start, args)

    print(f"\n{'='*70}")
    print("🎉 GRID SEARCH TERMINÉ")
    print(f"{'='*70}")
    _print_summary(results)
    _write_results(args.results_file, results, t_start, args)
    print(f"\n✅ Résultats sauvegardés → {args.results_file}")


def _print_summary(results):
    print(f"\n{'w4':<6}{'w5':<6}{'w6':<6}{'Status':<14}{'Overall%':<10}{'L4%':<8}{'L5%':<8}{'L6%':<8}{'Train(min)':<12}")
    print("-" * 80)
    # Trié par L6 det% décroissant (le keypoint le plus critique), puis overall
    def sort_key(r):
        l6 = r["per_kp"].get("link6", {}).get("det_pct")
        ov = r["det_rate"]
        return (-(l6 if l6 is not None else -1), -(ov if ov is not None else -1))

    for r in sorted(results, key=sort_key):
        ov = f"{r['det_rate']:.1f}" if r["det_rate"] is not None else "-"
        l4 = f"{r['per_kp'].get('link4', {}).get('det_pct', '-')}"
        l5 = f"{r['per_kp'].get('link5', {}).get('det_pct', '-')}"
        l6 = f"{r['per_kp'].get('link6', {}).get('det_pct', '-')}"
        print(f"{r['w4']:<6}{r['w5']:<6}{r['w6']:<6}{r['status']:<14}{ov:<10}{l4:<8}{l5:<8}{l6:<8}{r['train_time_min']:.1f}")


def _write_results(path, results, t_start, args):
    with open(path, "w") as f:
        f.write("GRID SEARCH RESULTS — link4 x link5 x link6 (grille croisée, train_dream_ultimate_v3.py)\n")
        f.write(f"Valeurs testées : {args.values}\n")
        f.write(f"Epochs={args.epochs} Patience={args.patience} Seed={args.seed}\n")
        f.write(f"Total elapsed: {(time.time() - t_start)/60:.1f} min\n")
        f.write("=" * 70 + "\n\n")
        for r in results:
            f.write(f"w4={r['w4']}  w5={r['w5']}  w6={r['w6']}  status={r['status']}  "
                     f"det_rate={r['det_rate']}  overall_mean={r['overall_mean']}  "
                     f"per_kp={r['per_kp']}  "
                     f"train_time_min={r['train_time_min']:.1f}\n")
            if "eval_output" in r:
                f.write("-" * 70 + "\n")
                f.write(r["eval_output"])
                f.write("\n")
            f.write("=" * 70 + "\n\n")


if __name__ == "__main__":
    main()