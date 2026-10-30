#!/usr/bin/env python3
"""Overlay de démonstration : DREAM markerless a bien retrouvé la caméra.

Pour chaque image (arducam, session non vue), on dessine :
  - ORANGE : keypoints détectés par DREAM (sortie 2D brute du réseau),
  - VERT   : squelette FK(encodeurs) reprojeté via l'extrinsèque estimé par DREAM
             (self_calibrate_arducam, sans marqueur).

Si le vert se pose sur le bras ET colle à l'orange, c'est la preuve visuelle que
DREAM a retrouvé la géométrie caméra-robot à partir des seules images.

Usage (venv_dream) :
    python overlay_dream_extrinsic.py \\
        --dataset dream_data/real_3cam_session6 \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \\
        --extrinsic ../calibration/arducam_extrinsic_dream_v4.yaml \\
        --n 8 --out ../calibration/overlay_dream_v4.png
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, "/tmp/DREAM")
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dream  # noqa: E402
from convert_to_ndds import load_real_intrinsics  # noqa: E402
from estimate_angles_from_keypoints import project_keypoints  # noqa: E402

KP_USED = 5                              # base..link4 (ceux qui servent au PnP)
SKELETON = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--extrinsic", required=True)
    ap.add_argument("--camera", default="arducam")
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--pick-index", default=None,
                    help="index précis à afficher seul (image 'hero')")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    ds = Path(args.dataset)
    K = load_real_intrinsics(args.camera)
    dist = np.load(Path(__file__).resolve().parents[1] / "calibration" / "cam_0.npz")["dist"].astype(np.float64)
    ext = yaml.safe_load(Path(args.extrinsic).read_text())
    T = np.array(ext["T_cam_world"], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(T[:3, :3]); tvec = T[:3, 3].reshape(3, 1)

    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()

    rows = [r for r in csv.DictReader(open(ds / "labels.csv"))
            if r.get("camera", args.camera) == args.camera
            and (ds / r["image_path"]).exists()]
    if not rows:
        raise SystemExit(f"Aucune image '{args.camera}' trouvée dans {ds/'labels.csv'}")
    if args.pick_index is not None:
        pick = [r for r in rows if r["index"] == str(args.pick_index)]
        if not pick:
            raise SystemExit(f"index {args.pick_index} absent (ou image manquante)")
    else:
        pick = rows[:: max(1, len(rows) // args.n)][:args.n]

    # ── image "hero" pleine résolution (un seul cadre) via cv2 ──
    if args.pick_index is not None and len(pick) == 1:
        r = pick[0]
        rgb = np.array(Image.open(ds / r["image_path"]).convert("RGB"))
        det = np.array(net.keypoints_from_image(Image.fromarray(rgb))["detected_keypoints"], float)
        q = np.radians([float(r[f"j{j}_deg"]) for j in range(1, 7)])
        fk = project_keypoints(q, K, dist, rvec, tvec)
        vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        GREEN, ORANGE, WHITE = (75, 180, 60), (49, 130, 245), (255, 255, 255)
        # style NVlabs : un point par keypoint — vert = vérité (FK), orange = prédiction
        for k in range(len(fk)):                              # vérité (les 7)
            cv2.circle(vis, tuple(fk[k].astype(int)), 6, GREEN, -1)
            cv2.circle(vis, tuple(fk[k].astype(int)), 6, WHITE, 1)
        for k in range(len(det)):                             # prédiction DREAM (les détectés)
            if det[k, 0] > -900 and det[k, 1] > -900:
                cv2.circle(vis, tuple(det[k].astype(int)), 4, ORANGE, -1)
                cv2.circle(vis, tuple(det[k].astype(int)), 4, WHITE, 1)
        cv2.putText(vis, "vert = verite (FK projetee)", (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)
        cv2.putText(vis, "orange = prediction DREAM", (12, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, ORANGE, 2)
        cv2.imwrite(args.out, vis)
        print(f"✅ hero → {args.out}")
        return

    cols = min(4, len(pick)); rows_n = (len(pick) + cols - 1) // cols
    fig, axes = plt.subplots(rows_n, cols, figsize=(4 * cols, 3 * rows_n))
    axes = np.array(axes).reshape(-1)

    for ax, r in zip(axes, pick):
        p = ds / r["image_path"]
        rgb = np.array(Image.open(p).convert("RGB"))
        det = np.array(net.keypoints_from_image(Image.fromarray(rgb))["detected_keypoints"], float)
        q = np.radians([float(r[f"j{j}_deg"]) for j in range(1, 7)])
        fk = project_keypoints(q, K, dist, rvec, tvec)

        ax.imshow(rgb)
        ax.scatter(fk[:, 0], fk[:, 1], c="#3cb44b", s=45,
                   edgecolors="white", linewidths=0.6, label="vérité (FK projetée)", zorder=3)
        vis = (det[:, 0] > -900) & (det[:, 1] > -900)
        ax.scatter(det[vis, 0], det[vis, 1], c="#f58231", s=22,
                   edgecolors="white", linewidths=0.5, label="prédiction DREAM", zorder=4)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_title(f"idx {r['index']}", fontsize=8)

    for ax in axes[len(pick):]:
        ax.axis("off")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.suptitle("DREAM markerless — vert = FK reprojeté via l'extrinsèque estimé par DREAM   ·   "
                 "orange = détection réseau (session 6, jamais vue)", y=0.99, fontsize=12)
    fig.legend(handles, labels, loc="lower center", ncol=2, fontsize=11,
               bbox_to_anchor=(0.5, 0.005))
    fig.tight_layout(rect=[0, 0.05, 1, 0.96])
    out = Path(args.out); fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"✅ overlay → {out}")


if __name__ == "__main__":
    main()
