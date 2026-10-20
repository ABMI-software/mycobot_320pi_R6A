#!/usr/bin/env python3
"""Teach pixel -> commande XY en PILOTANT le robot, à orientation figée.

Pourquoi pas le teach par traction à la main (calibrate_pixel_to_base.py) :
mesuré le 17/08, l'orientation du poignet varie de 6 à 27 deg d'un point à
l'autre quand on tire le bras. Comme la pince est quasi horizontale et longue de
~100 mm, 20 deg d'orientation deplacent les doigts de ~35 mm. Or le pick tourne à
orientation FIXE : un teach à orientation libre n'est donc pas comparable à
l'execution, et le deport bride->doigts n'est pas absorbable par un vecteur
constant.

Ici l'orientation est imposee et identique a chaque point (celle validee du pick),
donc le deport bride->doigts est une constante que l'homographie absorbe
integralement. On enregistre exactement la commande qui saisit la balle.

Commandes (dans la fenetre camera) :
  ESPACE  mesurer le pixel de la balle (bras ecarte)
  i / k   X + / -        (avant / arriere)
  j / l   Y + / -        (gauche / droite)
  u / o   Z + / -        (haut / bas)
  1 2 3   pas 1 / 5 / 10 mm
  ENTREE  enregistrer le point
  p       remonter en pose de degagement
  q       terminer et ajuster

    .venv/bin/python scripts/calibrate_jog.py --samples 9
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_FONTDIR', '/usr/share/fonts/truetype/dejavu')

import cv2                                                            # noqa: E402
import numpy as np                                                    # noqa: E402

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))
sys.path.insert(0, str(_REPO / 'mycobot_gateway'))

from pick_and_place_real import Bridge                                # noqa: E402
from calibrate_pixel_to_base import (open_camera, ball_pixel, fit,    # noqa: E402
                                     save, read_coords)
from preview_ball_detection import candidates, draw                   # noqa: E402

WIN = 'teach jog — arducam'
STEPS = {ord('1'): 1.0, ord('2'): 5.0, ord('3'): 10.0}


def send_pose(bridge, xyz, ori, speed):
    coords = [round(float(v), 2) for v in xyz] + [float(o) for o in ori]
    return bridge.send({'action': 'send_coords', 'coords': coords,
                        'speed': speed, 'mode': 1})


def try_move(bridge, xyz, axis, delta, ori, speed):
    """Applique un pas, et l'ANNULE si le robot refuse la pose.

    Sans ce retour arrière, une consigne hors limite reste dans l'état interne et
    TOUTES les suivantes sont rejetées à leur tour — y compris sur d'autres axes.
    Observé le 17/08 : Z bloqué à 528.4 (limite 523.9), jog entièrement mort.
    """
    xyz[axis] += delta
    reply = send_pose(bridge, xyz, ori, speed)
    if 'ERROR' in reply.upper():
        xyz[axis] -= delta
        return False, reply
    return True, reply


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pi-host', default='10.10.0.224')
    ap.add_argument('--samples', type=int, default=9)
    ap.add_argument('--width', type=int, default=1600)
    ap.add_argument('--height', type=int, default=1200)
    ap.add_argument('--exposure', type=int, default=75)
    ap.add_argument('--frames', type=int, default=8)
    ap.add_argument('--spread-px', type=float, default=12.0)
    ap.add_argument('--hsv-lo', nargs=3, type=int, default=[20, 170, 70])
    ap.add_argument('--hsv-hi', nargs=3, type=int, default=[35, 255, 255])
    ap.add_argument('--display-width', type=int, default=1100)
    ap.add_argument('--speed', type=int, default=20)
    ap.add_argument('--orientation', nargs=3, type=float, default=None,
                    metavar=('RX', 'RY', 'RZ'),
                    help='orientation figee ; defaut = safe_orientation_deg de la '
                         'calibration adaptative (celle du pick valide)')
    ap.add_argument('--park-dz', type=float, default=90.0,
                    help='hauteur de degagement au-dessus du point courant, en mm')
    ap.add_argument('--tip-z', type=float, default=33.5)
    ap.add_argument('--goto-start', default='pick_approach',
                    help="pose de pick_place_positions.json rejointe au demarrage "
                         "(vide = reste ou le bras se trouve)")
    args = ap.parse_args()

    if args.orientation is None:
        cal = json.loads((_REPO / 'scripts' / 'pick_adaptive_calibration.json').read_text())
        args.orientation = [float(v) for v in cal['safe_orientation_deg']]
    print(f'  orientation figée : {args.orientation}')

    cap = open_camera(args.width, args.height, args.exposure)
    bridge = Bridge(args.pi_host)
    bridge.get_angles()
    print(f'  ✅ bridge {args.pi_host}:5005')

    print('\n⚠️  Le bras va bouger sous tension. Zone dégagée ? Main près de la prise.')
    if input('    tape OUI puis ENTER pour commencer : ').strip().lower() != 'oui':
        raise SystemExit('annulé — aucun mouvement')

    if args.goto_start:
        positions = json.loads(
            (_REPO / 'scripts' / 'pick_place_positions.json').read_text())
        if args.goto_start not in positions:
            raise SystemExit(f'pose « {args.goto_start} » absente de pick_place_positions.json')
        print(f'  ↪ rejoint la pose « {args.goto_start} » (évite 40 touches de jog '
              'depuis la position de repos)')
        bridge.send({'action': 'send_angles',
                     'angles': positions[args.goto_start], 'speed': args.speed})
        time.sleep(4.0)

    start = read_coords(bridge)
    xyz = [float(start[0]), float(start[1]), float(start[2])]
    print(f'  pose de départ : {[round(v, 1) for v in xyz]}')
    print('  alignement sur l\'orientation figée  ',
          send_pose(bridge, xyz, args.orientation, args.speed))
    time.sleep(2.0)

    scale = args.width / 640.0
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    samples, step, pixel, last_error = [], 5.0, None, ''

    while len(samples) < args.samples:
        ok, frame = cap.read()
        if not ok:
            continue
        found, best, _ = candidates(frame, args.hsv_lo, args.hsv_hi, scale)
        vis = draw(frame, found, best, args.hsv_lo, args.hsv_hi, scale)
        head = (f'point {len(samples)+1}/{args.samples}   pas {step:.0f}mm   '
                f'cmd=({xyz[0]:.0f},{xyz[1]:.0f},{xyz[2]:.0f})')
        cv2.putText(vis, head, (10, int(120 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7 * scale, (255, 255, 0), max(2, int(2 * scale)))
        cv2.putText(vis, 'pixel balle MESURE' if pixel is not None else 'ESPACE = mesurer la balle',
                    (10, int(150 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6 * scale, (0, 255, 0) if pixel is not None else (0, 165, 255),
                    max(1, int(scale)))
        if last_error:
            cv2.putText(vis, 'REFUSE: ' + last_error.strip()[:70],
                        (10, int(180 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55 * scale, (0, 0, 255), max(1, int(scale)))
        h = int(vis.shape[0] * args.display_width / vis.shape[1])
        cv2.imshow(WIN, cv2.resize(vis, (args.display_width, h)))

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key in STEPS:
            step = STEPS[key]
        moved = {ord('i'): (0, +1), ord('k'): (0, -1), ord('j'): (1, +1),
                 ord('l'): (1, -1), ord('u'): (2, +1), ord('o'): (2, -1)}
        if key in moved:
            axis, sign = moved[key]
            ok_move, reply = try_move(bridge, xyz, axis, sign * step,
                                      args.orientation, args.speed)
            last_error = '' if ok_move else reply
            print(f'  → {[round(v, 1) for v in xyz]}  '
                  + ('OK' if ok_move else f'REFUSÉ (pas annulé) — {reply.strip()}'))
        if key == ord('p'):
            ok_move, reply = try_move(bridge, xyz, 2, args.park_dz,
                                      args.orientation, args.speed)
            last_error = '' if ok_move else reply
            print('  ↑ dégagement  ' + ('OK' if ok_move else f'REFUSÉ — {reply.strip()}'))
        if key == ord(' '):
            print('  📷 mesure du pixel de la balle (bras écarté)…')
            pixel = ball_pixel(cap, args.hsv_lo, args.hsv_hi, args.frames,
                               args.spread_px, scale)
        if key in (13, 10):
            if pixel is None:
                print('  ⚠️ mesure le pixel de la balle (ESPACE) avant d\'enregistrer')
                continue
            samples.append({'pixel': [float(pixel[0]), float(pixel[1])],
                            'coords_mm': [xyz[0], xyz[1], xyz[2]] + list(args.orientation),
                            'angles_deg': list(bridge.get_angles())})
            xy = np.asarray([s['coords_mm'][:2] for s in samples])
            print(f'  ✅ point {len(samples)} : pixel=({pixel[0]:.1f},{pixel[1]:.1f}) '
                  f'-> cmd=({xyz[0]:.1f},{xyz[1]:.1f})')
            if len(samples) >= 3:
                print(f'     📐 étalement : profondeur {np.ptp(xy[:, 0]):.0f} mm × '
                      f'largeur {np.ptp(xy[:, 1]):.0f} mm')
            pixel = None

    cap.release()
    cv2.destroyAllWindows()
    if len(samples) < 6:
        raise SystemExit(f'{len(samples)} point(s) — il en faut au moins 6')

    # Orientation identique partout -> le deport bride->doigts est une constante
    # que l'homographie absorbe. Pas de solve_tool_offset ici : on ajuste
    # directement sur la COMMANDE envoyee, qui est ce que le pick rejouera.
    src = np.asarray([s['pixel'] for s in samples])
    dst = np.asarray([s['coords_mm'][:2] for s in samples])
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 8.0)
    proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    res = np.linalg.norm(proj - dst, axis=1)
    inl = mask.ravel().astype(bool)
    print('\n=== RÉSIDUS ===')
    for i, r in enumerate(res, 1):
        print(f'  point {i:2d}  cmd=({dst[i-1][0]:6.1f},{dst[i-1][1]:6.1f})mm  '
              f'résidu={r:5.1f} mm' + ('' if inl[i-1] else '   ⚠️ écarté'))
    print(f'\n  retenus {inl.sum()}/{len(samples)}  moyen {res[inl].mean():.1f} mm  '
          f'max {res[inl].max():.1f} mm')
    save(H, samples, res, inl, (args.width, args.height))


if __name__ == '__main__':
    main()
