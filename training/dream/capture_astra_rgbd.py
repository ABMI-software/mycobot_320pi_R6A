#!/usr/bin/env python3
"""Capture d'un petit dataset astra RGB-D + encodeurs pour la courbe d'écart.

Pour chaque pose : commande des angles aléatoires sûrs → lit les encodeurs réels
(vérité terrain) → lit une frame couleur+depth du grabber (/dev/shm) → sauvegarde.
La courbe d'écart par joint (maillon 3) rejouera DREAM sur ces images couleur,
estimera les angles via la profondeur (estimate_angles_3d) et les comparera aux
encodeurs enregistrés ici.

Réutilise la sécurité de pose et le bridge TCP validés de capture_real_3cam.py.
Le depth est ALIGNÉ couleur (grabber oni_grabber_rgbd) → le pixel keypoint couleur
donne directement sa profondeur.

Sorties (dans --output) :
    labels.csv                 index,j1_deg..j6_deg  (vérité encodeurs)
    images/NNNNNN.color.png    couleur RGB astra
    images/NNNNNN.depth.npy    profondeur (float32, mètres), même résolution

Prérequis : robot + bridge Pi actifs, ./oni_grabber_rgbd en cours.

Usage (env hand-teleop ou venv avec cv2/PIL) :
    python capture_astra_rgbd.py --output dream_data/real_astra_rgbd \\
        --num-samples 80 --pi-host 10.10.0.223
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

# Réutilise le bridge TCP + le tirage de pose sûr déjà validés sur le robot réel.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_real_3cam import RobotBridge, random_joint_angles  # noqa: E402

SHM = Path("/dev/shm")


def read_info():
    kv = dict(l.split("=") for l in (SHM / "oni_info.txt").read_text().strip().splitlines())
    return (int(kv["CW"]), int(kv["CH"])), (int(kv["DW"]), int(kv["DH"]))


def read_frame(color_dims, depth_dims, retries=15):
    CW, CH = color_dims
    DW, DH = depth_dims
    csz, dsz = CW * CH * 3, DW * DH * 2
    for _ in range(retries):
        cf, df = SHM / "oni_color.rgb", SHM / "oni_depth.raw"
        if cf.stat().st_size == csz and df.stat().st_size == dsz:
            color = np.frombuffer(cf.read_bytes(), np.uint8)
            depth = np.frombuffer(df.read_bytes(), np.uint16)
            if color.size == csz and depth.size == DW * DH:
                return (color.reshape(CH, CW, 3).copy(),
                        (depth.reshape(DH, DW).astype(np.float32) * 0.001))
        time.sleep(0.03)
    return None, None


def next_index(csv_path):
    if not csv_path.exists():
        return 0
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    return (max(int(r["index"]) for r in rows) + 1) if rows else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True, help="dossier dataset (accumule les poses)")
    ap.add_argument("--num-samples", type=int, default=80, help="NOUVELLES poses cette session")
    ap.add_argument("--pi-host", default="10.10.0.223", help="IP Pi (bridge, port 5005)")
    ap.add_argument("--speed", type=int, default=25)          # comme capture_real_3cam
    ap.add_argument("--settle", type=float, default=3.0, help="attente après commande (s)")
    ap.add_argument("--limit-fraction", type=float, default=0.5,
                    help="fraction des limites articulaires pour le tirage aléatoire")
    args = ap.parse_args()

    out = Path(args.output)
    img_dir = out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out / "labels.csv"

    color_dims, depth_dims = read_info()   # échoue tôt si le grabber n'est pas lancé
    print(f"Grabber OK — couleur {color_dims}, depth {depth_dims}")

    bridge = RobotBridge(args.pi_host)
    bridge.connect()

    # Départ prévisible : home d'abord, comme capture_real_3cam (évite le saut brusque).
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

            true_angles = bridge.get_angles()      # encodeurs réels = vérité terrain
            if not true_angles or len(true_angles) != 6:
                print(f"  ⚠️ get_angles invalide, pose sautée"); continue
            color, depth = read_frame(color_dims, depth_dims)
            if color is None:
                print(f"  ⚠️ pas de frame RGB-D, pose sautée"); continue

            Image.fromarray(color).save(img_dir / f"{idx:06d}.color.png")
            np.save(img_dir / f"{idx:06d}.depth.npy", depth)
            w.writerow([idx] + [round(float(a), 2) for a in true_angles])
            f.flush()
            collected += 1
            print(f"\r  pose {collected}/{args.num_samples} (index {idx}) "
                  f"angles={[round(a,1) for a in true_angles]}", end="", flush=True)
    print(f"\n✅ {collected} poses → {out}  (total {start + collected})")


if __name__ == "__main__":
    main()
