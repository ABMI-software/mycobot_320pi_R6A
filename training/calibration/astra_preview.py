#!/usr/bin/env python3
"""Preview live de l'astra (couleur + profondeur) pour juger le placement.

Lit /dev/shm (grabber oni_grabber_rgbd) et affiche en direct, côte à côte :
  - la couleur (avec marqueurs ArUco surlignés + profondeur au centre),
  - la profondeur colorisée (JET, 0–3 m).

Sert à positionner l'astra : voir que le bras / la zone de travail / les
marqueurs sont bien cadrés et que la profondeur est cohérente.

Touches : q ou Échap pour quitter · s pour sauver un snapshot.
Si pas d'affichage (headless/SSH), utilise plutôt check_astra_markers.py
qui écrit un PNG.

Usage (venv_dream) :
    python astra_preview.py
"""
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

SHM = Path("/dev/shm")
HERE = Path(__file__).resolve().parent


def read_info():
    kv = dict(l.split("=") for l in (SHM / "oni_info.txt").read_text().strip().splitlines())
    return (int(kv["CW"]), int(kv["CH"])), (int(kv["DW"]), int(kv["DH"]))


def read_frame(cdims, ddims):
    CW, CH = cdims; DW, DH = ddims
    csz, dsz = CW * CH * 3, DW * DH * 2
    for _ in range(15):
        cf, df = SHM / "oni_color.rgb", SHM / "oni_depth.raw"
        if cf.stat().st_size == csz and df.stat().st_size == dsz:
            c = np.frombuffer(cf.read_bytes(), np.uint8)
            d = np.frombuffer(df.read_bytes(), np.uint16)
            if c.size == csz and d.size == DW * DH:
                return c.reshape(CH, CW, 3).copy(), d.reshape(DH, DW).astype(np.float32) * 0.001
        time.sleep(0.03)
    return None, None


def colorize_depth(depth_m, dmax=3.0):
    norm = np.clip(depth_m / dmax, 0, 1)
    vis = (norm * 255).astype(np.uint8)
    vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    vis[(depth_m <= 0.05) | ~np.isfinite(depth_m)] = 0   # trous en noir
    return vis


def main():
    try:
        expected = set(int(m) for m in
                       yaml.safe_load((HERE / "workspace_markers.yaml").read_text())["markers"])
    except Exception:
        expected = set()
    cdims, ddims = read_info()
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
        cv2.aruco.DetectorParameters())
    CW, CH = cdims
    print("Preview astra — q/Échap pour quitter, s pour snapshot.")
    headless = False

    while True:
        color, depth = read_frame(cdims, ddims)
        if color is None:
            print("… pas de frame (grabber lancé ?)"); time.sleep(0.3); continue
        bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

        corners, ids, _ = det.detectMarkers(cv2.cvtColor(color, cv2.COLOR_RGB2GRAY))
        seen = set()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
            for c, mid in zip(corners, ids.flatten()):
                if int(mid) in expected:
                    seen.add(int(mid))

        # profondeur au centre + réticule
        cu, cv_ = CW // 2, CH // 2
        cz = depth[cv_, cu]
        cv2.drawMarker(bgr, (cu, cv_), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(bgr, f"centre: {cz:.2f} m" if cz > 0.05 else "centre: (pas de depth)",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if expected:
            miss = sorted(expected - seen)
            txt = "marqueurs: LES 4 VUS" if not miss else f"marqueurs vus {sorted(seen)} / manque {miss}"
            cv2.putText(bgr, txt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if not miss else (0, 165, 255), 2)

        panel = np.hstack([bgr, colorize_depth(depth)])

        if headless:
            cv2.imwrite(str(HERE / "astra_preview.png"), panel)
            time.sleep(0.3)
            continue
        try:
            cv2.imshow("astra preview  (couleur | depth)", panel)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                cv2.imwrite(str(HERE / "astra_preview.png"), panel)
                print(f"snapshot -> {HERE/'astra_preview.png'}")
        except cv2.error:
            headless = True
            print("Pas d'affichage GUI -> j'écris astra_preview.png en boucle (ouvre-le).")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
