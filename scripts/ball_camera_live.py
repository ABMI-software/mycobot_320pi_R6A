#!/usr/bin/env python3
"""Aperçu Arducam live avec détection couleur de la balle.

Fermer avec Q ou Échap.
"""
import argparse
import cv2
import numpy as np
import subprocess
import yaml
from pathlib import Path


parser = argparse.ArgumentParser()
parser.add_argument("--device", type=int, default=2)
parser.add_argument("--name", default="ARDUCAM")
parser.add_argument("--exposure", type=int, default=75)
args = parser.parse_args()

repo = Path(__file__).resolve().parents[1]
cal = repo / "training" / "calibration"
npz = np.load(cal / "cam_0.npz")
K, dist = npz["mtx"].astype(float), npz["dist"].astype(float)
ext = yaml.safe_load((cal / "arducam_extrinsic_markers.yaml").read_text())
T = np.asarray(ext["T_cam_world"], dtype=float)
rvec, _ = cv2.Rodrigues(T[:3, :3]); tvec = T[:3, 3].reshape(3, 1)
markers = yaml.safe_load((cal / "workspace_markers.yaml").read_text())["markers"]
world_xy = np.asarray([v for v in markers.values()], dtype=float)

def project(points):
    uv, _ = cv2.projectPoints(np.asarray(points, dtype=float), rvec, tvec, K, dist)
    return np.round(uv.reshape(-1, 2)).astype(int)

zone_px = cv2.convexHull(project(world_xy).astype(np.int32)).reshape(-1, 2)
target_px = tuple(project([[0.30, 0.0, 0.0335]])[0])

subprocess.run(
    [
        "v4l2-ctl", "-d", f"/dev/video{args.device}",
        "-c", "auto_exposure=1",
        "-c", f"exposure_time_absolute={args.exposure}",
    ],
    check=False,
)
cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise SystemExit(f"{args.name} /dev/video{args.device} indisponible")

window = f"BALLE LIVE {args.name} - Q pour fermer"
cv2.namedWindow(window, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window, 960, 720)

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([20, 170, 70]), np.array([35, 255, 255]))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.polylines(frame, [zone_px], True, (0, 255, 0), 2)
        cv2.circle(frame, target_px, 32, (0, 255, 0), 3)
        cv2.putText(frame, "PLACE LA BALLE ICI", (target_px[0] - 90, target_px[1] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        cv2.putText(
            frame, "Deplace la balle en regardant ce LIVE",
            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
        )

        valid = []
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            circ = 4 * np.pi * area / (perimeter * perimeter) if perimeter else 0
            if 200 <= area <= 2500 and circ >= 0.30:
                valid.append(contour)
        if valid:
            contour = max(valid, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area > 0:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                cv2.circle(frame, center, max(int(radius), 14), (0, 0, 255), 3)
                cv2.drawMarker(
                    frame, center, (0, 255, 255), cv2.MARKER_CROSS, 24, 2
                )
                cv2.putText(
                    frame, f"BALLE ({center[0]}, {center[1]})",
                    (max(5, center[0] - 85), max(55, center[1] - 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2
                )
            else:
                cv2.putText(
                    frame, "BALLE NON DETECTEE", (12, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2
                )
        else:
            cv2.putText(
                frame, "BALLE NON DETECTEE", (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2
            )

        cv2.imshow(window, frame)
        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
