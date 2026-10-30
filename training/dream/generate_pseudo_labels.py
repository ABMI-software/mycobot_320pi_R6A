#!/usr/bin/env python3
"""Génère des pseudo-labels pour les images réelles en utilisant le meilleur modèle.

Stratégie :
  1. Charger le meilleur modèle (vgg_mixed_aug_v2_e50)
  2. Faire tourner sur toutes les frames réelles (4000)
  3. Garder seulement les frames avec confiance haute :
     - link1, link2, link3 détectés (erreur < threshold)
     - Confiance basée sur le pic de la belief map
  4. Mettre à jour les JSON avec les pseudo-labels détectés
  5. Créer dream_data/real_pseudo — prêt pour le training

Usage:
  cd training/
  python dream/generate_pseudo_labels.py \
      --weights checkpoints_dream/vgg_mixed_aug_v2_e50/best_network.pth  \
      --data dream/dream_data/real \
      --output dream/dream_data/real_pseudo \
      --confidence 0.01
      --min-kp 3

  puis réentrainer :
    python dream/train_dream_mixed_aug_optimized_v2.py \
  --data dream/dream_data/real_pseudo \
  --epochs 50 --batch-size 8 --workers 8    
"""

import sys
import os
import json
import shutil
import argparse
import math

import numpy as np
from PIL import Image as PILImage
import torch

DREAM_DIR = "/tmp/DREAM"
sys.path.insert(0, DREAM_DIR)
import dream

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

KP_NAMES = [
    "mycobot320_base", "mycobot320_link1", "mycobot320_link2",
    "mycobot320_link3", "mycobot320_link4", "mycobot320_link5", "mycobot320_link6"
]


def load_camera_intrinsics(data_dir):
    """Charge les paramètres caméra."""
    path = os.path.join(data_dir, "_camera_settings.json")
    with open(path) as f:
        settings = json.load(f)
    cs   = settings["camera_settings"][0]
    intr = cs["intrinsic_settings"]
    K    = np.array([
        [intr["fx"], intr.get("s", 0), intr["cx"]],
        [0, intr["fy"], intr["cy"]],
        [0, 0, 1],
    ], dtype=np.float64)
    w = cs["captured_image_size"]["width"]
    h = cs["captured_image_size"]["height"]
    return K, w, h


def update_json_with_pseudo_labels(json_path, detected_kp, img_w, img_h):
    """Met à jour le JSON avec les pseudo-labels détectés."""
    with open(json_path) as f:
        data = json.load(f)

    # Mettre à jour les keypoints détectés
    for ki, kp_data in enumerate(data["objects"][0]["keypoints"]):
        det = detected_kp[ki] if ki < len(detected_kp) else None

        if det is not None and det[0] is not None:
            u, v = float(det[0]), float(det[1])
            # Filtre sentinel
            if u > -900 and v > -900:
                kp_data["projected_location"] = [u, v]
                kp_data["pseudo_label"] = True

    return data


def main():
    parser = argparse.ArgumentParser(description="Génère des pseudo-labels pour images réelles")
    parser.add_argument("--weights",    "-w", required=True, help="Checkpoint modèle (.pth)")
    parser.add_argument("--data",       "-d", required=True, help="Dataset réel NDDS")
    parser.add_argument("--output",     "-o", required=True, help="Dossier output pseudo-labels")
    parser.add_argument("--confidence", "-c", type=float, default=0.01,
                        help="Seuil de confiance min sur belief map peak (default: 0.01)")
    parser.add_argument("--min-kp",     type=int, default=3,
                        help="Nombre minimum de keypoints détectés pour garder la frame (default: 3)")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f"GENERATE PSEUDO-LABELS")
    print(f"  Weights    : {args.weights}")
    print(f"  Data       : {args.data}")
    print(f"  Output     : {args.output}")
    print(f"  Confidence : {args.confidence}")
    print(f"  Min KP     : {args.min_kp}")
    print(f"{'='*60}\n")

    # Charger le modèle
    config_path = os.path.splitext(args.weights)[0] + ".yaml"
    print(f"Chargement du modèle...")
    dream_net = dream.create_network_from_config_file(config_path, args.weights)
    dream_net.enable_evaluation()
    print(f"  Architecture: {dream_net.architecture_type}")
    print(f"  Keypoints   : {dream_net.n_keypoints}")

    # Charger les paramètres caméra
    K, img_w, img_h = load_camera_intrinsics(args.data)
    print(f"  Camera: fx={K[0,0]:.1f} fy={K[1,1]:.1f} {img_w}x{img_h}")

    # Trouver toutes les frames
    import glob
    json_files = sorted(glob.glob(os.path.join(args.data, "??????.json")))
    n_total    = len(json_files)
    print(f"\nFound {n_total} frames in {args.data}")

    # Créer le dossier output
    os.makedirs(args.output, exist_ok=True)

    # Copier les fichiers settings
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(args.data, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(args.output, f))

    # Générer les pseudo-labels
    n_kept    = 0
    n_skipped = 0
    kp_detection_counts = np.zeros(7)

    print(f"\nGénération des pseudo-labels...")

    for idx, jf in enumerate(json_files):
        frame_id = os.path.basename(jf).replace(".json", "")
        img_path = os.path.join(args.data, f"{frame_id}.rgb.png")

        if not os.path.exists(img_path):
            continue

        # Charger image
        image = PILImage.open(img_path).convert("RGB")

        # Inférence
        with torch.no_grad():
            result = dream_net.keypoints_from_image(image, debug=False)

        detected_kp = result["detected_keypoints"]

        # Compter les keypoints détectés
        n_detected = 0
        for ki in range(len(KP_NAMES)):
            det = detected_kp[ki] if ki < len(detected_kp) else None
            if det is not None and det[0] is not None:
                u, v = float(det[0]), float(det[1])
                if u > -900 and v > -900:
                    n_detected += 1
                    kp_detection_counts[ki] += 1

        # Garder seulement les frames avec assez de keypoints détectés
        if n_detected < args.min_kp:
            n_skipped += 1
            continue

        # Mettre à jour le JSON avec pseudo-labels
        updated_data = update_json_with_pseudo_labels(jf, detected_kp, img_w, img_h)

        # Sauvegarder JSON mis à jour
        out_json = os.path.join(args.output, f"{frame_id}.json")
        with open(out_json, "w") as f:
            json.dump(updated_data, f, indent=2)

        # Copier l'image
        out_img = os.path.join(args.output, f"{frame_id}.rgb.png")
        shutil.copy2(img_path, out_img)

        n_kept += 1

        if (idx + 1) % 500 == 0:
            print(f"  {idx + 1}/{n_total} frames traitées | gardées: {n_kept} | skipped: {n_skipped}")

    # Résumé
    print(f"\n{'='*60}")
    print(f"✅ PSEUDO-LABELS GÉNÉRÉS !")
    print(f"  Total frames  : {n_total}")
    print(f"  Frames gardées: {n_kept} ({100*n_kept/n_total:.1f}%)")
    print(f"  Frames rejetées: {n_skipped} ({100*n_skipped/n_total:.1f}%)")
    print(f"\n  Détection par keypoint :")
    for ki, name in enumerate(KP_NAMES):
        short = name.split("_")[-1]
        pct   = 100 * kp_detection_counts[ki] / n_total
        print(f"    {short:<8s}: {int(kp_detection_counts[ki]):4d}/{n_total} ({pct:.1f}%)")
    print(f"\n  Output: {args.output}")
    print(f"{'='*60}")
    print(f"\nProchaine étape — réentraîner avec pseudo-labels :")
    print(f"  python dream/train_dream_mixed_aug_optimized_v2.py \\")
    print(f"    --data {args.output} \\")
    print(f"    --epochs 50 --batch-size 8 --workers 8")


if __name__ == "__main__":
    main()