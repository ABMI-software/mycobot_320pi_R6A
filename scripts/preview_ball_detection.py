#!/usr/bin/env python3
"""Vue arducam en TEMPS RÉEL avec le diagnostic du détecteur de balle.

Montre, image par image, ce que `detect_color` voit réellement : le masque HSV,
chaque blob candidat, son aire, sa circularité, et la RAISON pour laquelle il est
retenu ou rejeté. Les curseurs HSV se règlent en direct — les valeurs affichées
sont à recopier dans `calibrate_pixel_to_base.py --hsv-lo/--hsv-hi`.

⚠️ La caméra ne s'ouvre que par UN process : arrête la calibration et le launch
multicam avant de lancer ceci.

    .venv/bin/python scripts/preview_ball_detection.py
    .venv/bin/python scripts/preview_ball_detection.py --width 640 --height 480

Touches : [q] quitter  ·  [s] sauver la frame annotée  ·  [m] masque / image
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

# Le cv2 du .venv embarque un Qt sans polices et inonde la sortie de
# « QFontDatabase: Cannot find font directory ». À poser AVANT l'import de cv2.
os.environ.setdefault('QT_QPA_FONTDIR', '/usr/share/fonts/truetype/dejavu')

import cv2                                                            # noqa: E402
import numpy as np                                                    # noqa: E402

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'mycobot_gateway'))
from mycobot_gateway.vision.camera_registry import detect_cameras     # noqa: E402

WIN = 'detection balle'


def candidates(frame, hsv_lo, hsv_hi, scale, min_circularity=0.30):
    """Tous les blobs de la plage HSV, avec la raison d'acceptation/rejet.

    Reproduit la logique de detect_color mais SANS jeter l'information : c'est
    justement le « pourquoi ça rate » qu'on veut voir.
    """
    min_area, max_area = 200.0 * scale ** 2, 2500.0 * scale ** 2
    border = int(25 * scale)
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lo), np.array(hsv_hi))
    k_open, k_close = int(3 * scale) | 1, int(9 * scale) | 1
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k_open, k_open), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((k_close, k_close), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    found = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 30:
            continue
        perimeter = cv2.arcLength(c, True)
        circ = 4.0 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0
        M = cv2.moments(c)
        if M['m00'] <= 0:
            continue
        cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
        if area < min_area:
            reason = f'aire {area:.0f} < {min_area:.0f}'
        elif area > max_area:
            reason = f'aire {area:.0f} > {max_area:.0f}'
        elif circ < min_circularity:
            reason = f'circularite {circ:.2f} < {min_circularity}'
        elif cx < border or cx > w - border or cy < border or cy > h - border:
            reason = 'trop pres du bord'
        else:
            reason = None
        found.append({'xy': (cx, cy), 'area': area, 'circ': circ,
                      'reason': reason, 'contour': c})

    accepted = [f for f in found if f['reason'] is None]
    best = max(accepted, key=lambda f: f['area']) if accepted else None
    return found, best, mask


def draw(frame, found, best, hsv_lo, hsv_hi, scale):
    vis = frame.copy()
    for f in found:
        ok = f['reason'] is None
        colour = (0, 255, 0) if ok else (0, 0, 255)
        cv2.drawContours(vis, [f['contour']], -1, colour, 2)
        x, y = int(f['xy'][0]), int(f['xy'][1])
        label = f'{f["area"]:.0f}px c={f["circ"]:.2f}'
        if not ok:
            label += f'  ✗ {f["reason"]}'
        cv2.putText(vis, label, (x + 12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5 * scale, colour, max(1, int(scale)))
    if best is not None:
        x, y = int(best['xy'][0]), int(best['xy'][1])
        cv2.drawMarker(vis, (x, y), (0, 255, 255), cv2.MARKER_CROSS,
                       int(30 * scale), max(2, int(2 * scale)))
        head = f'RETENU ({best["xy"][0]:.1f}, {best["xy"][1]:.1f})'
        colour = (0, 255, 0)
    else:
        head = f'AUCUNE BALLE  ({len(found)} blob(s) rejete(s))'
        colour = (0, 0, 255)
    cv2.putText(vis, head, (10, int(30 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                0.8 * scale, colour, max(2, int(2 * scale)))
    cv2.putText(vis, f'HSV {list(hsv_lo)} -> {list(hsv_hi)}',
                (10, int(60 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                0.6 * scale, (255, 255, 255), max(1, int(scale)))
    return vis


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', default='arducam', choices=['arducam', 'svpro'],
                    help='camera a previsualiser')
    ap.add_argument('--width', type=int, default=1600)
    ap.add_argument('--height', type=int, default=1200)
    ap.add_argument('--exposure', type=int, default=75)
    ap.add_argument('--hsv-lo', nargs=3, type=int, default=[20, 170, 70])
    ap.add_argument('--hsv-hi', nargs=3, type=int, default=[35, 255, 255])
    ap.add_argument('--display-width', type=int, default=1100,
                    help='largeur de la fenetre (l\'image est reduite pour l\'affichage)')
    ap.add_argument('--tune', action='store_true',
                    help='ajoute les curseurs HSV (backend Qt requis)')
    args = ap.parse_args()

    specs = detect_cameras(names=[args.camera])
    if not specs:
        raise SystemExit(
            f'{args.camera} non détectée — branchée ? '
            '(`python3 mycobot_gateway/mycobot_gateway/vision/camera_registry.py`)')
    spec = specs[0]
    index = spec.v4l2_index
    # manual_exposure = -1 => la caméra tourne en réglages d'usine (cas SVPRO,
    # cf. camera_registry : « ni expo, ni focus/contrast v4l2 »). Forcer une
    # exposition dessus la dégraderait.
    if spec.manual_exposure >= 0:
        subprocess.run(['v4l2-ctl', '-d', f'/dev/video{index}', '-c', 'auto_exposure=1',
                        '-c', f'exposure_time_absolute={args.exposure},gain=0,brightness=0'],
                       check=False)
    print(f'  ✅ {spec.name} /dev/video{index}'
          + (f' — expo {args.exposure}' if spec.manual_exposure >= 0 else ' — réglages d\'usine'))
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f'/dev/video{index} occupé — arrête la calibration / le launch')

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setWindowTitle(WIN, f'{spec.name} — detection balle')
    # Le backend Qt d'OpenCV ne « réalise » la fenêtre qu'au premier imshow : sans
    # cette image blanche, createTrackbar échoue en NULL window handler.
    cv2.imshow(WIN, np.zeros((40, args.display_width, 3), np.uint8))
    cv2.waitKey(1)

    names = ['H bas', 'S bas', 'V bas', 'H haut', 'S haut', 'V haut']
    limits = [179, 255, 255, 179, 255, 255]
    tuning = False
    if args.tune:
        try:
            for name, limit, value in zip(names, limits,
                                          list(args.hsv_lo) + list(args.hsv_hi)):
                cv2.createTrackbar(name, WIN, int(value), limit, lambda _v: None)
            tuning = True
        except cv2.error as exc:
            print(f'  ⚠️ curseurs indisponibles ({exc.err.strip()}) — '
                  'vue seule, règle par --hsv-lo/--hsv-hi')

    scale = args.width / 640.0
    lo, hi = list(args.hsv_lo), list(args.hsv_hi)
    show_mask, saved, active = False, 0, 0
    print('  [q] quitter · [s] sauver · [m] masque/image')
    print('  [tab] composante HSV suivante · [+]/[-] ajuster de 5')
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        if tuning:
            lo = [cv2.getTrackbarPos(n, WIN) for n in names[:3]]
            hi = [cv2.getTrackbarPos(n, WIN) for n in names[3:]]
        found, best, mask = candidates(frame, lo, hi, scale)
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if show_mask else \
            draw(frame, found, best, lo, hi, scale)
        if not tuning:
            cv2.putText(vis, f'>>> {names[active]} = {(lo + hi)[active]}',
                        (10, int(90 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6 * scale, (0, 255, 255), max(1, int(scale)))

        display_h = int(vis.shape[0] * args.display_width / vis.shape[1])
        cv2.imshow(WIN, cv2.resize(vis, (args.display_width, display_h)))

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('m'):
            show_mask = not show_mask
        if key == 9:                                   # tab
            active = (active + 1) % 6
        if key in (ord('+'), ord('='), ord('-'), ord('_')) and not tuning:
            step = 5 if key in (ord('+'), ord('=')) else -5
            target = lo if active < 3 else hi
            i = active % 3
            target[i] = int(np.clip(target[i] + step, 0, limits[active]))
        if key == ord('s'):
            path = _REPO / f'preview_ball_{saved}.png'
            cv2.imwrite(str(path), vis)
            print(f'  💾 {path}')
            saved += 1

    print(f'\nHSV retenu :  --hsv-lo {lo[0]} {lo[1]} {lo[2]} '
          f'--hsv-hi {hi[0]} {hi[1]} {hi[2]}')
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
