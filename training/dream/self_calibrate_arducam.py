#!/usr/bin/env python3
"""Self-calibration extrinsèque arducam — SANS marqueurs, via le robot.

Le robot sert de mire : pour chaque pose, FK(encodeurs) donne les 7 keypoints 3D
en repère base, DREAM les détecte en 2D. On optimise l'extrinsèque (rvec,tvec,
partagé par toutes les poses, caméra fixe) pour que la projection FK colle aux
détections DREAM. Marche parce que la détection arducam est bonne (~94%).

Amorçage : la transform overhead approximative de convert_to_ndds. Sortie : le
résidu de reprojection par keypoint (px) = qualité objective, + l'extrinsèque
raffiné écrit en YAML (format load_camera : T_cam_world) pour la courbe.

Usage (venv_dream) :
    python self_calibrate_arducam.py \\
        --dataset dream_data/real_arducam \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \\
        --out ../calibration/arducam_extrinsic_selfcal.yaml
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy.optimize import least_squares

sys.path.insert(0, "/tmp/DREAM")
import dream  # noqa: E402
from convert_to_ndds import _real_camera_transform, load_real_intrinsics  # noqa: E402
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES  # noqa: E402

KP = ["base", "link1", "link2", "link3", "link4", "link5", "link6"]


def fk_obj(q):
    pos, _ = forward_kinematics(q)
    return np.array([pos[n] for n in KEYPOINT_NAMES], dtype=np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--out", default="../calibration/arducam_extrinsic_selfcal.yaml")
    ap.add_argument("--keypoints", default="base,link1,link2,link3,link4",
                    help="keypoints à utiliser pour PnP (les faibles link5/link6 "
                         "bruitent l'extrinsèque en réel — exclus par défaut)")
    ap.add_argument("--reject-px", type=float, default=8.0,
                    help="seuil bas de rejet des correspondances aberrantes (px)")
    ap.add_argument("--camera", default="arducam",
                    help="caméra à isoler dans labels.csv (colonne 'camera')")
    ap.add_argument("--compare-extrinsic", default=None,
                    help="yaml d'un extrinsèque (ex. hand-eye) : mesure son résidu de "
                         "reprojection sur CES détections, sans ré-optimiser (diagnostic)")
    ap.add_argument("--plot-out", default=None,
                    help="si donné : PNG du résidu de reprojection médian par keypoint")
    args = ap.parse_args()

    kp_keep = {KP.index(n.strip()) for n in args.keypoints.split(",") if n.strip()}
    print(f"Keypoints PnP : {sorted(KP[k] for k in kp_keep)}")

    ds = Path(args.dataset)
    K = load_real_intrinsics("arducam")
    dist = np.load(Path(__file__).resolve().parents[1] / "calibration" / "cam_0.npz")["dist"].astype(np.float64)

    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()

    rows = list(csv.DictReader(open(ds / "labels.csv")))
    rows = [r for r in rows if r.get("camera", args.camera) == args.camera]
    print(f"{len(rows)} poses pour la caméra '{args.camera}'")
    objs, dets, kidx = [], [], []       # correspondances 3D<->2D + n° keypoint
    for r in rows:
        p = ds / r["image_path"] if r.get("image_path") else \
            ds / "images" / f"{int(r['index']):06d}.color.png"
        if not p.exists():
            continue
        q = np.radians([float(r[f"j{j}_deg"]) for j in range(1, 7)])
        obj = fk_obj(q)
        kps = np.array(net.keypoints_from_image(Image.open(p).convert("RGB"))["detected_keypoints"], float)
        for k in range(7):
            if k in kp_keep and kps[k, 0] > -900 and kps[k, 1] > -900:
                objs.append(obj[k]); dets.append(kps[k]); kidx.append(k)
    objs = np.array(objs); dets = np.array(dets); kidx = np.array(kidx)
    print(f"{len(objs)} correspondances sur {len(rows)} poses")

    if args.compare_extrinsic:
        cw = yaml.safe_load(Path(args.compare_extrinsic).read_text())
        Tc = np.array(cw["T_cam_world"], float)
        rvc, _ = cv2.Rodrigues(Tc[:3, :3]); tvc = Tc[:3, 3].reshape(3, 1)
        proj, _ = cv2.projectPoints(objs, rvc, tvc, K, dist)
        e = np.linalg.norm(proj.reshape(-1, 2) - dets, axis=1)
        print(f"[compare] '{Path(args.compare_extrinsic).name}' reprojette ces détections à "
              f"médian {np.median(e):.1f}px  (part <10px : {(e < 10).mean()*100:.0f}%)")
        print("  -> ~quelques px = cet extrinsèque explique aussi la session (ambiguïté).")
        print("  -> gros (>30px)  = il ne colle pas (caméra déplacée entre les captures).")

    # amorçage : transform overhead approximative
    T0 = np.linalg.inv(_real_camera_transform("arducam"))    # world->cam
    rvec0, _ = cv2.Rodrigues(T0[:3, :3])
    x0 = np.concatenate([rvec0.ravel(), T0[:3, 3]])

    def resid_on(mask):
        def r(x):
            rvec = x[:3].reshape(3, 1); tvec = x[3:].reshape(3, 1)
            proj, _ = cv2.projectPoints(objs[mask], rvec, tvec, K, dist)
            return (proj.reshape(-1, 2) - dets[mask]).ravel()
        return r

    all_mask = np.ones(len(objs), bool)
    r0 = resid_on(all_mask)(x0).reshape(-1, 2)
    print(f"résidu AVANT (overhead brut) : médian {np.median(np.linalg.norm(r0,axis=1)):.0f}px")

    # PnP robuste : optimise puis rejette itérativement les correspondances aberrantes
    keep = all_mask.copy(); x = x0
    for it in range(5):
        res = least_squares(resid_on(keep), x, loss="soft_l1", f_scale=5.0, max_nfev=2000)
        x = res.x
        err = np.linalg.norm(resid_on(all_mask)(x).reshape(-1, 2), axis=1)
        thr = max(args.reject_px, 3.0 * np.median(err[keep]))
        new_keep = err <= thr
        print(f"  itér {it}: {keep.sum()} corresp. -> médian {np.median(err[keep]):.1f}px, "
              f"rejet > {thr:.0f}px -> {new_keep.sum()}")
        if new_keep.sum() == keep.sum() or new_keep.sum() < 10:
            break
        keep = new_keep

    rvec = x[:3].reshape(3, 1); tvec = x[3:].reshape(3, 1)

    print(f"\n{'keypoint':8s} {'n':>4s} {'résidu médian (px)':>20s}")
    print("-" * 36)
    for k in range(7):
        m = (kidx == k) & keep
        if m.sum():
            print(f"{KP[k]:8s} {m.sum():4d} {np.median(err[m]):20.1f}")
    print("-" * 36)
    print(f"{'INLIERS':8s} {keep.sum():4d} {np.median(err[keep]):20.1f}")

    if args.plot_out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        ks = sorted(kp_keep)
        meds = [np.median(err[(kidx == k) & keep]) for k in ks]
        ns = [int(((kidx == k) & keep).sum()) for k in ks]
        fig, ax = plt.subplots(figsize=(6, 4))
        bars = ax.bar([KP[k] for k in ks], meds, color="#3cb44b", edgecolor="black")
        for b, m, nn in zip(bars, meds, ns):
            ax.text(b.get_x() + b.get_width()/2, m + 0.1, f"{m:.1f}px\nn={nn}",
                    ha="center", va="bottom", fontsize=9)
        ax.axhline(np.median(err[keep]), ls="--", color="gray",
                   label=f"médiane globale {np.median(err[keep]):.1f}px")
        ax.set_ylabel("résidu de reprojection médian (px)")
        ax.set_title("DREAM markerless — précision par keypoint (session 6, non vue)")
        ax.set_ylim(0, max(meds) * 1.35); ax.legend()
        fig.tight_layout(); fig.savefig(args.plot_out, dpi=130)
        print(f"📊 graphe → {args.plot_out}")

    R, _ = cv2.Rodrigues(rvec)
    T_cam_world = np.eye(4); T_cam_world[:3, :3] = R; T_cam_world[:3, 3] = tvec.ravel()
    T_world_cam = np.linalg.inv(T_cam_world)
    out = Path(args.out)
    out.write_text(yaml.safe_dump({
        "source": "self_calibrate_arducam (robot-as-target, no markers)",
        "weights": str(args.weights),
        "keypoints_used": sorted(KP[k] for k in kp_keep),
        "n_inliers": int(keep.sum()),
        "median_reproj_px": float(np.median(err[keep])),
        "T_cam_world": T_cam_world.tolist(),
        "T_world_cam": T_world_cam.tolist(),
        "camera_position_base_m": (-R.T @ tvec.ravel()).tolist(),
    }, sort_keys=False))
    print(f"\n✅ extrinsèque raffiné → {out}")
    print("Lecture : résidu global < ~5px = bon (courbe fiable) ; > 15px = keypoints trop bruités.")


if __name__ == "__main__":
    main()
