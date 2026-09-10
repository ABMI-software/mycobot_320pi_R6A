#!/usr/bin/env python3
"""Aide au cadrage : dit combien de marqueurs sol l'astra voit, en direct.

Lit /dev/shm (grabber oni_grabber_rgbd), détecte les marqueurs ArUco attendus
(workspace_markers.yaml), affiche les IDs vus + leur profondeur, et sauve une
image annotée (astra_markers_preview.png) à ouvrir pour ajuster le cadrage.

But : viser l'astra jusqu'à voir les 4 marqueurs (19/23/25/26) avant de lancer
la calibration.

Usage (venv_dream) :
    python check_astra_markers.py            # boucle, Ctrl+C pour arrêter
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


def main():
    spec = yaml.safe_load((HERE / "workspace_markers.yaml").read_text())
    expected = set(int(m) for m in spec["markers"])
    cdims, ddims = read_info()
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
        cv2.aruco.DetectorParameters())
    print(f"Marqueurs attendus : {sorted(expected)}. Vise jusqu'à les voir tous. Ctrl+C pour arrêter.")

    try:
        while True:
            color, depth = read_frame(cdims, ddims)
            if color is None:
                print("… pas de frame (grabber lancé ?)"); time.sleep(0.5); continue
            bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            corners, ids, _ = det.detectMarkers(cv2.cvtColor(color, cv2.COLOR_RGB2GRAY))
            seen = {}
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
                for c, mid in zip(corners, ids.flatten()):
                    if int(mid) in expected:
                        u, v = c.reshape(4, 2).mean(0)
                        patch = depth[max(0, int(v) - 5):int(v) + 6, max(0, int(u) - 5):int(u) + 6]
                        vd = patch[(patch > 0.05) & np.isfinite(patch)]
                        seen[int(mid)] = float(np.median(vd)) if vd.size else None
            missing = sorted(expected - set(seen))
            status = "  ".join(f"{m}@{seen[m]:.2f}m" if seen[m] else f"{m}@?" for m in sorted(seen))
            ok = "✅ LES 4 VUS — lance la calibration" if not missing else f"manque {missing}"
            print(f"\r vus [{len(seen)}/{len(expected)}] {status}  |  {ok}      ", end="", flush=True)
            cv2.imwrite(str(HERE / "astra_markers_preview.png"), bgr)
            time.sleep(0.3)
    except KeyboardInterrupt:
        print(f"\nAperçu sauvé : {HERE / 'astra_markers_preview.png'}")


if __name__ == "__main__":
    main()
