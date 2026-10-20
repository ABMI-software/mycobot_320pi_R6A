#!/usr/bin/env python3
"""Dashboard visual-servoing avancé — caméra + squelette + 6 courbes joint.

Version avancée de visual_servoing_platform.py. Ajoute la vue caméra à gauche :
  - orange = keypoints détectés par l'IA (DREAM),
  - vert   = squelette FK projeté depuis les encodeurs (LA VÉRITÉ),
si les deux se superposent, la vision est bonne ; l'écart visuel = l'erreur.
À droite : 6 mini-courbes (une par joint) angle réel vs estimé + écart.
Bas : barre d'état (détection X/7, résidu reproj px, écart moyen j1-j4, pose).

Observabilité honnête : j5 marqué « faible », j6 « aveugle » (keypoint sur l'axe).

Modes : --mode replay (dataset) | --mode live (robot + arducam).

Usage (venv_dream) :
    python visual_servoing_dashboard.py --mode replay \\
        --dataset dream_data/real_arducam \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \\
        --extrinsic ../calibration/arducam_extrinsic_selfcal.yaml \\
        --intrinsics-npz ../calibration/cam_0.npz
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
from matplotlib.gridspec import GridSpec
from PIL import Image

sys.path.insert(0, "/tmp/DREAM")
import dream  # noqa: E402
from estimate_angles_from_keypoints import estimate_angles, project_keypoints  # noqa: E402

N_KP = 7
JOINTS = [f"j{i}" for i in range(1, 7)]
JTAG = {4: " (faible)", 5: " (aveugle)"}          # observabilité
SKELETON = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6)]


def dream_keypoints(net, pil):
    det = net.keypoints_from_image(pil)["detected_keypoints"]
    out = np.full((N_KP, 2), np.nan)
    for k in range(N_KP):
        u, v = float(det[k][0]), float(det[k][1])
        if u > -900 and v > -900:
            out[k] = (u, v)
    return out


class Dashboard:
    def __init__(self, maxlen=120):
        self.t = deque(maxlen=maxlen)
        self.real = [deque(maxlen=maxlen) for _ in range(6)]
        self.est = [deque(maxlen=maxlen) for _ in range(6)]
        plt.ion()
        self.fig = plt.figure(figsize=(16, 8))
        self.fig.suptitle("Dashboard visual servoing — caméra + angles réel vs estimé", fontsize=14)
        gs = GridSpec(3, 4, figure=self.fig, width_ratios=[1.4, 1.4, 1, 1])
        self.ax_cam = self.fig.add_subplot(gs[0:3, 0:2])
        self.ax_cam.set_title("Caméra — vert=vérité (FK), orange=IA (DREAM)")
        self.ax_cam.axis("off")
        self.im = self.ax_cam.imshow(np.zeros((480, 640, 3), np.uint8))
        self.j_ax, self.l_real, self.l_est, self.txt = [], [], [], []
        slots = [(0, 2), (0, 3), (1, 2), (1, 3), (2, 2), (2, 3)]
        for j, (r, c) in enumerate(slots):
            ax = self.fig.add_subplot(gs[r, c])
            lr, = ax.plot([], [], color="#2ca02c", lw=2)
            le, = ax.plot([], [], color="#ff7f0e", lw=2)
            ax.set_title(JOINTS[j] + JTAG.get(j, ""), fontsize=10,
                         color="red" if j == 5 else ("darkorange" if j == 4 else "black"))
            ax.tick_params(labelsize=7); ax.grid(alpha=0.3)
            self.j_ax.append(ax); self.l_real.append(lr); self.l_est.append(le)
            self.txt.append(ax.text(0.98, 0.04, "", transform=ax.transAxes,
                                    ha="right", fontsize=8, color="red"))
        self.status = self.fig.text(0.5, 0.005, "", ha="center", fontsize=10, family="monospace")
        self.fig.tight_layout(rect=[0, 0.03, 1, 0.96])

    def push(self, frame_bgr, det_kps, fk_kps, real_deg, est_deg, rms_px, tstamp):
        # --- vue caméra + overlays ---
        vis = frame_bgr.copy()
        if fk_kps is not None:                       # squelette vérité (vert)
            pts = [tuple(np.round(p).astype(int)) for p in fk_kps]
            for a, b in SKELETON:
                cv2.line(vis, pts[a], pts[b], (0, 180, 0), 2)
            for p in pts:
                cv2.circle(vis, p, 5, (0, 180, 0), -1)
        for k in range(N_KP):                         # keypoints IA (orange)
            if not np.isnan(det_kps[k, 0]):
                cv2.circle(vis, tuple(det_kps[k].astype(int)), 6, (0, 140, 255), -1)
                cv2.circle(vis, tuple(det_kps[k].astype(int)), 7, (255, 255, 255), 1)
        self.im.set_data(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))

        # --- courbes joint ---
        self.t.append(tstamp); xs = list(self.t)
        for j in range(6):
            self.real[j].append(real_deg[j])
            self.est[j].append(est_deg[j] if est_deg is not None else np.nan)
            self.l_real[j].set_data(xs, list(self.real[j]))
            self.l_est[j].set_data(xs, list(self.est[j]))
            self.j_ax[j].relim(); self.j_ax[j].autoscale_view()
            if est_deg is not None and np.isfinite(est_deg[j]):
                err = abs((est_deg[j] - real_deg[j] + 180) % 360 - 180)
                self.txt[j].set_text(f"{err:.1f}°")
            else:
                self.txt[j].set_text("—")

        # --- barre d'état ---
        ndet = int((~np.isnan(det_kps[:, 0])).sum())
        if est_deg is not None:
            e14 = np.mean([abs((est_deg[j] - real_deg[j] + 180) % 360 - 180) for j in range(4)])
            emsg = f"écart moy j1-j4 {e14:5.1f}°"
            rmsg = f"résidu {rms_px:4.1f}px" if rms_px is not None else "résidu —"
        else:
            emsg = "écart j1-j4 — (estim. rejetée)"; rmsg = "résidu —"
        self.status.set_text(f"détection {ndet}/7   |   {rmsg}   |   {emsg}   |   échantillon {len(self.t)}")

        self.fig.canvas.draw_idle(); self.fig.canvas.flush_events()

    def alive(self):
        return plt.fignum_exists(self.fig.number)


def load_cam2d(npz_path, extrinsic_yaml):
    npz = np.load(npz_path)
    K, dist = npz["mtx"].astype(np.float64), npz["dist"].astype(np.float64)
    ext = yaml.safe_load(Path(extrinsic_yaml).read_text())
    T = np.array(ext["T_cam_world"], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(T[:3, :3]); tvec = T[:3, 3].reshape(3, 1)
    return K, dist, rvec, tvec


def process(net, pil, cam2d, real_deg, q0, max_rms_px):
    """-> (det_kps, fk_skeleton_2d, est_deg, rms, q0)."""
    K, dist, rvec, tvec = cam2d
    det = dream_keypoints(net, pil)
    fk2d = project_keypoints(np.radians(real_deg), K, dist, rvec, tvec)   # squelette vérité
    if np.isnan(det).all(axis=1).sum() > 4:
        return det, fk2d, None, None, q0
    try:
        q, rms, _ = estimate_angles(det, K, dist, rvec, tvec, q0=q0, n_restarts=6)
    except ValueError:
        return det, fk2d, None, None, q0
    if rms > max_rms_px:
        return det, fk2d, None, rms, q0
    return det, fk2d, np.degrees(q), rms, q


def run_replay(args, net, cam2d, dash):
    ds = Path(args.dataset)
    rows = list(csv.DictReader(open(ds / "labels.csv")))
    q0 = None
    for i, r in enumerate(rows):
        if not dash.alive():
            break
        idx = int(r["index"])
        real = np.array([float(r[f"j{j}_deg"]) for j in range(1, 7)])
        bgr = cv2.imread(str(ds / "images" / f"{idx:06d}.color.png"))
        pil = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        det, fk2d, est, rms, q0 = process(net, pil, cam2d, real, q0, args.max_rms_px)
        dash.push(bgr, det, fk2d, real, est, rms, i)
        time.sleep(args.replay_dt)
    print("Rejeu terminé — ferme la fenêtre pour quitter.")
    while dash.alive():
        plt.pause(0.2)


def run_live(args, net, cam2d, dash):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from capture_real_3cam import LocalCamera, RobotBridge
    cam = LocalCamera(args.index, "arducam", exposure=args.exposure)
    bridge = RobotBridge(args.pi_host); bridge.connect()
    q0 = None; t0 = time.time()
    while dash.alive():
        enc = bridge.get_angles()
        rgb = cam.capture(flush=2)
        if not enc or len(enc) != 6 or rgb is None:
            plt.pause(0.05); continue
        real = np.array(enc); bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        det, fk2d, est, rms, q0 = process(net, Image.fromarray(rgb), cam2d, real, q0, args.max_rms_px)
        dash.push(bgr, det, fk2d, real, est, rms, time.time() - t0)
    cam.close(); bridge.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["live", "replay"], required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--extrinsic", required=True)
    ap.add_argument("--intrinsics-npz", required=True)
    ap.add_argument("--max-rms-px", type=float, default=10.0)
    ap.add_argument("--dataset")
    ap.add_argument("--replay-dt", type=float, default=0.5)
    ap.add_argument("--pi-host", default="10.10.0.224")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--exposure", type=int, default=75)
    args = ap.parse_args()

    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()
    cam2d = load_cam2d(args.intrinsics_npz, args.extrinsic)
    dash = Dashboard()

    if args.mode == "replay":
        if not args.dataset:
            raise SystemExit("--mode replay nécessite --dataset")
        run_replay(args, net, cam2d, dash)
    else:
        run_live(args, net, cam2d, dash)


if __name__ == "__main__":
    main()
