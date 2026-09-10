#!/usr/bin/env python3
"""Détection d'objet YOLOv8 (COCO) → coordonnées dans le repère BASE du robot.

À LANCER AVEC LE .venv d'Osama_ws (torch+cuda+ultralytics) :
    /home/genji/Osama_ws/src/mycobot_R6A/.venv/bin/python \
        scripts/yolo_object_detect.py --once --save

Pipeline :
    Arducam (/dev/video2) → YOLOv8n → bbox centre (u,v)
    → ObjectLocalizer (K cam_3 + extrinsèque caméra→base + plan table)
    → (x,y,z) en mètres dans le repère base.

Pas de cv2.imshow (déclenche les erreurs Qt xcb sous ce venv) : utiliser --save
pour écrire une image annotée sur disque.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'mycobot_gateway'))

from mycobot_gateway.vision.camera_registry import load_intrinsics       # noqa: E402
from mycobot_gateway.vision.object_localizer import ObjectLocalizer      # noqa: E402


def open_camera(index: int, w: int = 640, h: int = 480) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if not cap.isOpened():
        raise RuntimeError(f'impossible d\'ouvrir /dev/video{index}')
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=2, help='index /dev/video (arducam=2)')
    ap.add_argument('--weights', default='yolov8n.pt', help='poids YOLO (COCO par défaut)')
    ap.add_argument('--calib', default='cam_3', help='stem intrinsèque (arducam=cam_3)')
    ap.add_argument('--table-z', type=float, default=0.0, help='hauteur table repère base (m)')
    ap.add_argument('--conf', type=float, default=0.35, help='seuil de confiance YOLO')
    ap.add_argument('--classes', nargs='*', default=None,
                    help='filtrer par noms COCO (ex: cup bottle "sports ball")')
    ap.add_argument('--once', action='store_true', help='une seule frame puis quitte')
    ap.add_argument('--save', action='store_true', help='écrit une image annotée')
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    names = model.names
    want = None
    if args.classes:
        want = {i for i, n in names.items() if n in args.classes}

    K, _ = load_intrinsics(args.calib, 640, 480)
    if K is None:
        raise SystemExit(f'intrinsèque {args.calib} introuvable')
    loc = ObjectLocalizer(K, table_z=args.table_z)

    cap = open_camera(args.device)
    print(f'# YOLO {args.weights} | table_z={args.table_z} m | classes={args.classes or "toutes"}')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('!! lecture caméra échouée'); time.sleep(0.1); continue

            res = model(frame, conf=args.conf, verbose=False)[0]
            annotated = frame.copy()
            for box in res.boxes:
                cls = int(box.cls[0])
                if want is not None and cls not in want:
                    continue
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                try:
                    xyz = loc.locate(u, v)
                    coord = f'base=({xyz[0]*1000:.0f},{xyz[1]*1000:.0f},{xyz[2]*1000:.0f})mm'
                except ValueError as e:
                    coord = f'(déprojection: {e})'
                print(f'{names[cls]:14s} conf={conf:.2f} px=({u:.0f},{v:.0f}) {coord}')
                if args.save:
                    cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(annotated, (int(u), int(v)), 4, (0, 0, 255), -1)
                    cv2.putText(annotated, f'{names[cls]} {conf:.2f}', (int(x1), int(y1) - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if args.save:
                out = _REPO / 'scratch_yolo_detect.jpg'
                cv2.imwrite(str(out), annotated)
                print(f'# image annotée -> {out}')
            if args.once:
                break
    finally:
        cap.release()


if __name__ == '__main__':
    main()
