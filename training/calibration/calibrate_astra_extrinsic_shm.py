#!/usr/bin/env python3
"""Calibration extrinsèque de l'astra (RGB-D) via /dev/shm — repère base.

Lit les frames écrites par oni_grabber_rgbd (couleur + depth aligné couleur +
champ de vision dans oni_info.txt), détecte les marqueurs sol ArUco, place chaque
marqueur en 3D dans le repère caméra grâce à la profondeur, puis aligne
rigidement (Kabsch) sur ses positions connues en repère base.

Pas de ROS, pas de ChArUco : intrinsèques d'usine (depuis le FOV couleur),
profondeur d'usine (déjà alignée sur la couleur par le grabber).

Prérequis : le grabber tourne (`./oni_grabber_rgbd`), astra fixe, 4 marqueurs
sol (workspace_markers.yaml) visibles.

Usage (dans venv_dream, non-ROS) :
    source ~/ros_jazzy/venv_dream/bin/activate
    python calibrate_astra_extrinsic_shm.py --num-frames 30
"""
import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

SHM = Path("/dev/shm")
HERE = Path(__file__).resolve().parent


def read_info():
    """Parse /dev/shm/oni_info.txt -> dims + intrinsèques couleur (K)."""
    txt = (SHM / "oni_info.txt").read_text()
    kv = dict(line.split("=") for line in txt.strip().splitlines())
    CW, CH = int(kv["CW"]), int(kv["CH"])
    DW, DH = int(kv["DW"]), int(kv["DH"])
    if "CHFOV" not in kv:
        raise SystemExit("oni_info.txt sans CHFOV — recompile avec oni_grabber_rgbd.cpp.")
    fx = (CW / 2.0) / math.tan(float(kv["CHFOV"]) / 2.0)
    fy = (CH / 2.0) / math.tan(float(kv["CVFOV"]) / 2.0)
    K = np.array([[fx, 0, CW / 2.0], [0, fy, CH / 2.0], [0, 0, 1]], dtype=np.float64)
    return (CW, CH), (DW, DH), K


def read_frame(color_dims, depth_dims):
    """Lit couleur (RGB888) + depth (16UC1 mm) depuis /dev/shm, gère l'écriture partielle."""
    CW, CH = color_dims
    DW, DH = depth_dims
    csz, dsz = CW * CH * 3, DW * DH * 2
    for _ in range(10):
        cf, df = SHM / "oni_color.rgb", SHM / "oni_depth.raw"
        if cf.stat().st_size == csz and df.stat().st_size == dsz:
            color = np.frombuffer(cf.read_bytes(), np.uint8)
            depth = np.frombuffer(df.read_bytes(), np.uint16)
            if color.size == CW * CH * 3 and depth.size == DW * DH:
                return (color.reshape(CH, CW, 3).copy(),
                        depth.reshape(DH, DW).astype(np.float64) * 0.001)
        time.sleep(0.02)
    return None, None


def kabsch(P_cam, P_base):
    """Rigide : P_base ≈ R @ P_cam + t  (T_base_cam)."""
    c1, c2 = P_cam.mean(0), P_base.mean(0)
    H = (P_cam - c1).T @ (P_base - c2)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, c2 - R @ c1


def depth_at(depth_m, u, v, win=5):
    h, w = depth_m.shape
    patch = depth_m[max(0, v - win):v + win + 1, max(0, u - win):u + win + 1]
    valid = patch[(patch > 0.05) & np.isfinite(patch)]
    return float(np.median(valid)) if valid.size >= 3 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--markers-yaml", default=str(HERE / "workspace_markers.yaml"))
    ap.add_argument("--num-frames", type=int, default=30)
    ap.add_argument("--out-yaml", default=str(HERE / "astra_extrinsic.yaml"))
    ap.add_argument("--out-npz", default=str(HERE / "cam_astra.npz"))
    args = ap.parse_args()

    spec = yaml.safe_load(Path(args.markers_yaml).read_text())
    frame_id = spec.get("frame_id", "base_link")
    markers = {int(m): np.array(xyz, float) for m, xyz in spec["markers"].items()}
    print(f"{len(markers)} marqueurs : {sorted(markers)}")

    color_dims, depth_dims, K = read_info()
    print(f"Intrinsèques (FOV) : fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f} @ {color_dims[0]}x{color_dims[1]}")
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
        cv2.aruco.DetectorParameters())

    acc = {m: [] for m in markers}
    while min((len(acc[m]) for m in acc if acc[m]), default=0) < args.num_frames \
            or sum(1 for m in acc if acc[m]) < 3:
        color, depth = read_frame(color_dims, depth_dims)
        if color is None:
            print("… pas de frame /dev/shm (grabber lancé ?)"); time.sleep(0.3); continue
        gray = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = det.detectMarkers(gray)
        if ids is not None:
            for c, mid in zip(corners, ids.flatten()):
                if int(mid) not in markers:
                    continue
                u, v = c.reshape(4, 2).mean(0)
                Z = depth_at(depth, int(round(u)), int(round(v)))
                if Z:
                    acc[int(mid)].append([(u - cx) / fx * Z, (v - cy) / fy * Z, Z])
        seen = sum(1 for m in acc if acc[m])
        got = min((len(acc[m]) for m in acc if acc[m]), default=0)
        print(f"\r  frames {got}/{args.num_frames} sur {seen} marqueurs vus", end="", flush=True)
    print()

    seen = [m for m in sorted(markers) if acc[m]]
    P_cam = np.array([np.mean(acc[m], 0) for m in seen])
    P_base = np.array([markers[m] for m in seen])
    R, t = kabsch(P_cam, P_base)
    resid = np.linalg.norm((P_cam @ R.T + t) - P_base, axis=1) * 1000
    print("Résidu par marqueur (mm) :", {int(m): round(float(e), 1) for m, e in zip(seen, resid)})
    print(f"Résidu moyen : {resid.mean():.2f} mm | caméra à {np.round(t,3)} m (repère base)")

    redun = None
    if len(seen) >= 4:
        cams = [kabsch(np.delete(P_cam, k, 0), np.delete(P_base, k, 0))[1]
                for k in range(len(seen))]
        redun = float(np.max(np.linalg.norm(np.array(cams) - t, axis=1)) * 1000)
        print(f"Redondance leave-one-out : {redun:.2f} mm")

    T_world_cam = np.eye(4); T_world_cam[:3, :3] = R; T_world_cam[:3, 3] = t
    out = {
        "frame_id": frame_id, "camera_frame": "astra_color", "method": "depth_3d_kabsch_shm",
        "markers_used": [int(m) for m in seen],
        "physically_valid": bool(resid.mean() < 10.0),
        "residual_mean_mm": round(float(resid.mean()), 3),
        "per_marker_resid_mm": {int(m): round(float(e), 3) for m, e in zip(seen, resid)},
        "redundancy_max_mm": None if redun is None else round(redun, 3),
        "camera_position_world_m": [round(float(x), 5) for x in t],
        "T_cam_world": np.linalg.inv(T_world_cam).tolist(),
        "T_world_cam": T_world_cam.tolist(),
    }
    Path(args.out_yaml).write_text(yaml.safe_dump(out, sort_keys=False))
    np.savez(args.out_npz, mtx=K, dist=np.zeros(5), resolution=np.array(color_dims))
    print(f"✅ écrit {Path(args.out_yaml).name} + {Path(args.out_npz).name}")
    if resid.mean() > 10:
        print("⚠️ résidu > 10 mm : marqueurs bien à plat/visibles ? Surélève-en un "
              "(~10 cm) pour casser la coplanarité et resserrer le fit.")


if __name__ == "__main__":
    main()
