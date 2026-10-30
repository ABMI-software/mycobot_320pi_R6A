#!/usr/bin/env python3
"""Plateforme visual-servoing — 6 fenêtres (une par joint) : angle réel vs estimé.

Évolution du monitoring téléop vers l'eye-to-hand : pour chaque joint j1..j6, deux
courbes en direct — l'angle RÉEL (encodeurs = vérité) et l'angle ESTIMÉ par la
caméra (DREAM → keypoints → estimate_angles) — plus l'écart en degrés.

Deux modes :
  --mode live    : robot (bridge TCP) + caméra (arducam V4L2) en temps réel.
  --mode replay  : rejoue un dataset capturé (images + labels.csv) — démontrable
                   MAINTENANT, sans dépendre d'un pipeline vision temps-réel parfait.

NB honnêteté : la courbe « estimé » ne vaut que ce que vaut la vision. Sur l'arducam
mono elle est bruitée (pas de depth) ; la plateforme, elle, est correcte et montre
l'écart tel qu'il est. Elle s'améliorera dès que la vision (astra+fine-tune) le sera.

Usage (venv_dream) :
    # démo immédiate sur un dataset capturé
    python visual_servoing_platform.py --mode replay \\
        --dataset dream_data/real_arducam \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \\
        --extrinsic ../calibration/arducam_extrinsic_selfcal.yaml \\
        --intrinsics-npz ../calibration/cam_0.npz

    # live robot + arducam
    python visual_servoing_platform.py --mode live \\
        --pi-host 10.10.0.224 --index 0 --exposure 75 \\
        --weights ... --extrinsic ... --intrinsics-npz ...
"""
import argparse
import csv
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

sys.path.insert(0, "/tmp/DREAM")
import dream  # noqa: E402
from estimate_angles_from_keypoints import estimate_angles  # noqa: E402

N_KP = 7
JOINTS = [f"j{i}" for i in range(1, 7)]


def dream_keypoints(net, pil):
    det = net.keypoints_from_image(pil)["detected_keypoints"]
    out = np.full((N_KP, 2), np.nan)
    for k in range(N_KP):
        u, v = float(det[k][0]), float(det[k][1])
        if u > -900 and v > -900:
            out[k] = (u, v)
    return out


class Platform:
    """6 fenêtres joint : réel (vert) vs estimé (orange) + écart, en direct."""

    def __init__(self, maxlen=120):
        self.t = deque(maxlen=maxlen)
        self.real = [deque(maxlen=maxlen) for _ in range(6)]
        self.est = [deque(maxlen=maxlen) for _ in range(6)]
        plt.ion()
        self.fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        self.fig.suptitle("Visual servoing — angle réel (encodeur) vs estimé (caméra IA)", fontsize=14)
        self.axes = list(axes.flat)
        self.l_real, self.l_est, self.txt = [], [], []
        for j, ax in enumerate(self.axes):
            lr, = ax.plot([], [], color="#2ca02c", lw=2, label="réel")
            le, = ax.plot([], [], color="#ff7f0e", lw=2, label="estimé")
            ax.set_title(JOINTS[j]); ax.set_ylabel("deg"); ax.grid(alpha=0.3)
            ax.legend(loc="upper left", fontsize=8)
            self.l_real.append(lr); self.l_est.append(le)
            self.txt.append(ax.text(0.98, 0.05, "", transform=ax.transAxes,
                                    ha="right", fontsize=9, color="red"))
        self.fig.tight_layout(rect=[0, 0, 1, 0.96])

    def push(self, tstamp, real_deg, est_deg):
        self.t.append(tstamp)
        xs = list(self.t)
        for j in range(6):
            self.real[j].append(real_deg[j])
            self.est[j].append(est_deg[j] if est_deg is not None else np.nan)
            self.l_real[j].set_data(xs, list(self.real[j]))
            self.l_est[j].set_data(xs, list(self.est[j]))
            ax = self.axes[j]
            ax.relim(); ax.autoscale_view()
            if est_deg is not None and np.isfinite(est_deg[j]):
                err = abs((est_deg[j] - real_deg[j] + 180) % 360 - 180)
                self.txt[j].set_text(f"écart {err:.1f}°")
            else:
                self.txt[j].set_text("estim. indispo")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def alive(self):
        return plt.fignum_exists(self.fig.number)


def load_cam2d(npz_path, extrinsic_yaml):
    npz = np.load(npz_path)
    K, dist = npz["mtx"].astype(np.float64), npz["dist"].astype(np.float64)
    ext = yaml.safe_load(Path(extrinsic_yaml).read_text())
    T = np.array(ext["T_cam_world"], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    tvec = T[:3, 3].reshape(3, 1)
    return K, dist, rvec, tvec


def estimate(net, pil, K, dist, rvec, tvec, q0, max_rms_px):
    kps = dream_keypoints(net, pil)
    if np.isnan(kps).all(axis=1).sum() > 4:
        return None, q0
    try:
        q, rms, _ = estimate_angles(kps, K, dist, rvec, tvec, q0=q0, n_restarts=6)
    except ValueError:
        return None, q0
    if rms > max_rms_px:
        return None, q0
    return np.degrees(q), q      # (est_deg, nouveau warm-start)


def run_replay(args, net, cam2d, plat):
    K, dist, rvec, tvec = cam2d
    ds = Path(args.dataset)
    rows = list(csv.DictReader(open(ds / "labels.csv")))
    q0 = None
    for i, r in enumerate(rows):
        if not plat.alive():
            break
        idx = int(r["index"])
        real = np.array([float(r[f"j{j}_deg"]) for j in range(1, 7)])
        p = ds / "images" / f"{idx:06d}.color.png"
        est_deg, q0 = estimate(net, Image.open(p).convert("RGB"), K, dist, rvec, tvec, q0, args.max_rms_px)
        plat.push(i, real, est_deg)
        time.sleep(args.replay_dt)
    print("Rejeu terminé — ferme la fenêtre pour quitter.")
    while plat.alive():
        plt.pause(0.2)


def run_live(args, net, cam2d, plat):
    K, dist, rvec, tvec = cam2d
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from capture_real_3cam import LocalCamera, RobotBridge
    cam = LocalCamera(args.index, "arducam", exposure=args.exposure)
    bridge = RobotBridge(args.pi_host); bridge.connect()
    q0 = None; t0 = time.time()
    while plat.alive():
        enc = bridge.get_angles()
        if not enc or len(enc) != 6:
            plt.pause(0.05); continue
        rgb = cam.capture(flush=2)
        est_deg = None
        if rgb is not None:
            est_deg, q0 = estimate(net, Image.fromarray(rgb), K, dist, rvec, tvec, q0, args.max_rms_px)
        plat.push(time.time() - t0, np.array(enc), est_deg)
    cam.close(); bridge.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["live", "replay"], required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--extrinsic", required=True)
    ap.add_argument("--intrinsics-npz", required=True)
    ap.add_argument("--max-rms-px", type=float, default=10.0)
    # replay
    ap.add_argument("--dataset")
    ap.add_argument("--replay-dt", type=float, default=0.4)
    # live
    ap.add_argument("--pi-host", default="10.10.0.224")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--exposure", type=int, default=75)
    args = ap.parse_args()

    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()
    cam2d = load_cam2d(args.intrinsics_npz, args.extrinsic)
    plat = Platform()

    if args.mode == "replay":
        if not args.dataset:
            raise SystemExit("--mode replay nécessite --dataset")
        run_replay(args, net, cam2d, plat)
    else:
        run_live(args, net, cam2d, plat)


if __name__ == "__main__":
    main()
