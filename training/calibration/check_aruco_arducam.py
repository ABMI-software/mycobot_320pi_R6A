#!/usr/bin/env python3
"""Ouvre l'arducam et affiche en direct TOUS les marqueurs ArUco détectés.

Sert à vérifier que les 4 tags calibration sont bien vus par la caméra (peu
importe leurs IDs). Réglage image = celui du 3-cam (set_arducam_exposure).

Touches : q/Échap quitter · s snapshot. Headless → écrit aruco_check.png.

Usage (venv_dream) :
    python check_aruco_arducam.py --index 0 --exposure 75
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))            # training/
from capture_real_3cam import LocalCamera        # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--exposure", type=int, default=75)
    args = ap.parse_args()

    cam = LocalCamera(args.index, "arducam", exposure=args.exposure)
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
        cv2.aruco.DetectorParameters())
    print("Détection ArUco arducam — q/Échap quitter, s snapshot.")
    headless = False
    last = None

    while True:
        rgb = cam.capture(flush=2)
        if rgb is None:
            print("… pas de frame"); time.sleep(0.2); continue
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        corners, ids, _ = det.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))

        seen = sorted(int(i) for i in ids.flatten()) if ids is not None else []
        if seen != last:
            print(f"  détectés : {seen}  ({len(seen)}/4)")
            last = seen
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
        col = (0, 255, 0) if len(seen) >= 4 else (0, 165, 255) if seen else (0, 0, 255)
        cv2.putText(bgr, f"IDs vus: {seen}   ({len(seen)}/4)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)

        if headless:
            cv2.imwrite(str(HERE / "aruco_check.png"), bgr); time.sleep(0.3); continue
        try:
            cv2.imshow("arducam - detection ArUco", bgr)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                cv2.imwrite(str(HERE / "aruco_check.png"), bgr)
                print(f"snapshot -> {HERE/'aruco_check.png'}")
        except cv2.error:
            headless = True
            print("Pas d'affichage GUI -> j'écris aruco_check.png en boucle.")

    cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
