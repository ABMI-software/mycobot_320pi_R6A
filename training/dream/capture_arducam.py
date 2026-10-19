#!/usr/bin/env python3
"""Capture arducam (vue haut) RGB + encodeurs pour la courbe d'écart eye-to-hand.

Pour chaque pose : commande des angles aléatoires sûrs → lit les encodeurs réels
(vérité terrain) → lit une frame arducam (V4L2, réglage image du 3-cam) → sauve.
Pas de depth (arducam mono) : l'estimation d'angles sera en 2D (reprojection).

Réutilise LocalCamera + RobotBridge + le tirage de pose sûr de capture_real_3cam,
et applique set_arducam_exposure (rendu « session4 », comme les données DREAM).

Sorties (dans --output) :
    labels.csv                 index,j1_deg..j6_deg  (vérité encodeurs)
    images/NNNNNN.color.png    couleur RGB arducam

Mouvement identique aux autres captures : go_home d'abord, speed 25, settle 3 s.

Usage (venv_dream) :
    python capture_arducam.py --output dream_data/real_arducam \\
        --num-samples 10 --pi-host 10.10.0.224 --index 0 --exposure 75
"""
import argparse
import csv
import sys
import time
from pathlib import Path

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # training/
from capture_real_3cam import LocalCamera, RobotBridge, random_joint_angles  # noqa: E402


def next_index(csv_path):
    if not csv_path.exists():
        return 0
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    return (max(int(r["index"]) for r in rows) + 1) if rows else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True, help="dossier dataset (accumule les poses)")
    ap.add_argument("--num-samples", type=int, default=10, help="NOUVELLES poses cette session")
    ap.add_argument("--pi-host", default="10.10.0.224", help="IP Pi (bridge, port 5005)")
    ap.add_argument("--index", type=int, default=0, help="index V4L2 arducam")
    ap.add_argument("--exposure", type=int, default=75, help="réglage image 3-cam")
    ap.add_argument("--speed", type=int, default=25)
    ap.add_argument("--settle", type=float, default=3.0, help="attente après commande (s)")
    ap.add_argument("--limit-fraction", type=float, default=0.5,
                    help="fraction des limites articulaires pour le tirage aléatoire")
    args = ap.parse_args()

    out = Path(args.output)
    img_dir = out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out / "labels.csv"

    cam = LocalCamera(args.index, "arducam", exposure=args.exposure)
    bridge = RobotBridge(args.pi_host)
    bridge.connect()

    print("  🏠 Envoi au home…")
    bridge.go_home()
    time.sleep(3)

    start = next_index(csv_path)
    new_file = not csv_path.exists()
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["index"] + [f"j{j}_deg" for j in range(1, 7)])

        collected = 0
        while collected < args.num_samples:
            idx = start + collected
            target = random_joint_angles(limit_fraction=args.limit_fraction)
            bridge.send_angles(target, speed=args.speed)
            time.sleep(args.settle)

            true_angles = bridge.get_angles()
            if not true_angles or len(true_angles) != 6:
                print("  ⚠️ get_angles invalide, pose sautée"); continue
            rgb = cam.capture(flush=5)
            if rgb is None:
                print("  ⚠️ pas de frame arducam, pose sautée"); continue

            Image.fromarray(rgb).save(img_dir / f"{idx:06d}.color.png")
            w.writerow([idx] + [round(float(a), 2) for a in true_angles])
            f.flush()
            collected += 1
            print(f"\r  pose {collected}/{args.num_samples} (index {idx}) "
                  f"angles={[round(a,1) for a in true_angles]}", end="", flush=True)
    print(f"\n✅ {collected} poses → {out}  (total {start + collected})")
    cam.close()
    bridge.close()


if __name__ == "__main__":
    main()
