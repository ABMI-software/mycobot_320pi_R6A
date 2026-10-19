#!/usr/bin/env python3
"""Calibration DIRECTE pixel objet -> commande XY, par homographie.

Remplace la chaîne intrinsèque x extrinsèque x plan-table x offset appris par une
seule mesure de bout en bout : on pose la balle à N endroits, on amène la pince
dessus à la main, et on enregistre le couple (pixel de la balle, XY commandé).
L'homographie ajustée sur ces N couples absorbe d'un coup l'erreur d'extrinsèque,
la distorsion résiduelle, la hauteur de table et le déport bride->doigts.

Valable parce que l'objet est TOUJOURS sur le plan de la table : un plan vu par
une caméra sténopé se projette par une homographie exacte (8 ddl, >=4 points).
C'est la méthode du projet Braccio (mapping mesuré sur la planche) faite
proprement : leur affine par morceaux en 3 zones n'était qu'une approximation de
cette homographie.

Protocole par point (le script guide) :
  1. poser la balle sur la planche, bras ECARTE (il ne doit pas la masquer)
  2. le script mesure le pixel de la balle
  3. servos relâchés -> amener les doigts sur la balle -> ENTER
  4. le script relit les angles + la pose cartésienne

À lancer depuis la racine du dépôt, avec le venv (cv2) :
    .venv/bin/python scripts/calibrate_pixel_to_base.py --samples 9
    .venv/bin/python scripts/calibrate_pixel_to_base.py --fit-only   # refit seul
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))
sys.path.insert(0, str(_REPO / 'mycobot_gateway'))

sys.path.insert(0, str(_REPO / 'training' / 'dream'))

from pick_and_place_real import Bridge                                  # noqa: E402
from pick_and_place_vision_live import detect_color                     # noqa: E402
from mycobot_gateway.vision.camera_registry import detect_cameras       # noqa: E402
from mycobot_fk import forward_kinematics                               # noqa: E402

OUT = _REPO / 'scripts' / 'pixel_to_base_homography.json'
PARTIAL = _REPO / 'scripts' / 'pixel_to_base_partial.json'
MIN_POINTS = 6          # 4 = minimum algébrique ; 6 donne des résidus mesurables


def open_camera(width, height, exposure):
    specs = detect_cameras(names=['arducam'])
    if not specs:
        raise SystemExit('arducam non détectée — vérifie le branchement')
    index = specs[0].v4l2_index
    import subprocess
    subprocess.run(['v4l2-ctl', '-d', f'/dev/video{index}', '-c', 'auto_exposure=1',
                    '-c', f'exposure_time_absolute={exposure},gain=0,brightness=0'],
                   check=False)
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(f'/dev/video{index} occupé — arrête le launch multicam')
    for _ in range(10):
        cap.read()
    ok, frame = cap.read()
    print(f'  ✅ arducam /dev/video{index} — {frame.shape[1]}x{frame.shape[0]}')
    return cap


def ball_pixel(cap, hsv_lo, hsv_hi, samples, spread_px, scale):
    """Médiane multi-frame du centre de la balle, ou None si instable.

    ``scale`` = (largeur/640) : les seuils par défaut de detect_color sont calés
    sur du 640x480 (aire 200-2500 px). En 1600x1200 l'aire de la balle est
    multipliée par scale**2 et sortirait du plafond -> on met les bornes à
    l'échelle, sinon la balle n'est jamais détectée.
    """
    hits = []
    for _ in range(samples):
        ok, frame = cap.read()
        if ok:
            px, _ = detect_color(frame, hsv_lo, hsv_hi,
                                 min_area=200.0 * scale ** 2,
                                 max_area=2500.0 * scale ** 2,
                                 border=int(25 * scale))
            if px is not None:
                hits.append(px)
        time.sleep(0.12)
    if not hits:
        print('    ❌ balle non détectée')
        return None
    pts = np.asarray(hits, dtype=np.float64)
    centre = np.median(pts, axis=0)
    spread = float(np.max(np.linalg.norm(pts - centre, axis=1)))
    print(f'    balle : {len(pts)}/{samples} détections, '
          f'pixel=({centre[0]:.1f}, {centre[1]:.1f}), dispersion={spread:.1f}px')
    # Une seule détection donne une dispersion nulle par construction : sans ce
    # seuil, un point où le bras masque la balle (1/8) passerait pour excellent.
    need = max(2, int(round(0.6 * samples)))
    if len(pts) < need:
        print(f'    ❌ trop peu de détections ({len(pts)} < {need}) — '
              'le bras masque probablement la balle')
        return None
    if spread > spread_px:
        print(f'    ❌ instable (> {spread_px}px)')
        return None
    return centre


def read_coords(bridge):
    reply = bridge.send({'action': 'get_coords'})
    if 'COORDS:' not in reply:
        raise RuntimeError(f'get_coords inattendu : {reply}')
    return json.loads(reply.split('COORDS:', 1)[1].strip())


def flange_pose(angles_deg):
    """(position mm, rotation 3x3) de la bride, par cinématique directe.

    On passe par la FK plutôt que par get_coords : les angles ARTICULAIRES sont
    fiables bras relâché, alors que les angles d'EULER de get_coords sautent
    (cf. l'avertissement en tête de pick_and_place_real.py). Vérifié sur les 9
    points du 17/08 : la position FK colle à get_coords à ~10 mm près, biais
    constant, tandis que les Euler donnaient un axe outil quasi horizontal —
    physiquement impossible avec les doigts posés sur la table.
    """
    positions, transforms = forward_kinematics(np.radians(angles_deg))
    return np.asarray(positions['mycobot320_link6']) * 1000.0, transforms[6][:3, :3]


def solve_tool_offset(samples, tip_z_mm):
    """Déport bride->doigts, résolu par moindres carrés.

    Contrainte physique : à chaque point les doigts étaient SUR la balle, donc à
    z = tip_z_mm. Ça donne une équation linéaire par point pour 3 inconnues.
    """
    rows, rhs = [], []
    for s in samples:
        p, R = flange_pose(s['angles_deg'])
        rows.append(R[2, :])
        rhs.append(tip_z_mm - p[2])
    t, _, rank, _ = np.linalg.lstsq(np.asarray(rows), np.asarray(rhs), rcond=None)
    return t, int(rank)


def fingertips(samples, tool_offset):
    out = []
    for s in samples:
        p, R = flange_pose(s['angles_deg'])
        out.append(p + R @ tool_offset)
    return np.asarray(out)


def collect(bridge, cap, args, samples=None):
    """Boucle de teach. ``samples`` est rempli au fur et à mesure : l'appelant en
    garde une référence, donc un Ctrl-C ne perd rien de ce qui est déjà acquis."""
    samples = [] if samples is None else samples
    if samples:
        print(f'\n▶️  reprise : {len(samples)} point(s) déjà acquis')
    print(f'\n=== TEACH — {args.samples} points ===')
    print('Répartis les positions sur TOUTE la planche (coins inclus) : une '
          'homographie ajustée sur une zone étroite extrapole mal ailleurs.\n')

    # Les servos restent RELÂCHÉS pendant tout le teach : ils ne sont remis sous
    # tension qu'un instant, pour figer la pose au moment de la lecture. Sinon le
    # bras reste verrouillé sur la balle et masque la vue du point suivant
    # (symptôme observé : 8/8 détections au point 1, puis 1/8 au point 3).
    print('⚠️  TIENS le bras : les servos vont être relâchés et le resteront '
          'pendant tout le teach.')
    input('    ENTER pour relâcher : ')
    bridge.send({'action': 'power_off'})

    while len(samples) < args.samples:
        n = len(samples) + 1
        print(f'--- point {n}/{args.samples} ---')
        input(f'[{n}] Pose la balle, ÉCARTE le bras à la main (il est mou), ENTER : ')

        pixel = ball_pixel(cap, args.hsv_lo, args.hsv_hi, args.frames,
                           args.spread_px, args.width / 640.0)
        if pixel is None:
            print('    → on recommence ce point\n')
            continue

        answer = input(f'[{n}] Amène les DOIGTS sur la balle, ENTER (s = sauter) : ')
        if answer.strip().lower() == 's':
            print('    → point sauté\n')
            continue

        # On FIGE d'abord, on mesure ensuite. Mesurer pendant que l'opérateur tient
        # un bras désservi n'a aucun sens : la main tremble de plusieurs degrés
        # (3.7° relevé le 17/08). Une fois les servos tenus, la dérive résiduelle
        # est celle du bras seul — c'est elle qui invalide un point.
        bridge.send({'action': 'power_on'})
        time.sleep(args.settle)
        # Médiane de plusieurs lectures : les encodeurs bruitent de quelques
        # dixièmes de degré même servos tenus (0.62° relevé le 17/08). Moyenner
        # écrase ce bruit ; la dispersion résiduelle, elle, signale un vrai
        # mouvement du bras et invalide le point.
        reads = []
        for _ in range(args.angle_reads):
            reads.append(bridge.get_angles())
            time.sleep(0.15)
        reads = np.asarray(reads, dtype=np.float64)
        angles = np.median(reads, axis=0).tolist()
        drift = float(np.max(np.ptp(reads, axis=0)))
        if drift > args.max_drift_deg:
            print(f'    ❌ le bras dérive de {drift:.2f}° servos TENUS '
                  f'(> {args.max_drift_deg}°) — pose instable, on recommence\n')
            continue

        coords = read_coords(bridge)
        tip = flange_pose(angles)[0]
        print(f'    angles = {[round(a, 1) for a in angles]}  (dérive {drift:.2f}°)')
        print(f'    bride FK = ({tip[0]:.1f}, {tip[1]:.1f}, {tip[2]:.1f}) mm')

        samples.append({'pixel': [float(pixel[0]), float(pixel[1])],
                        'coords_mm': [float(c) for c in coords],
                        'angles_deg': [float(a) for a in angles]})
        print(f'    ✅ point {n} enregistré')

        if len(samples) >= 3:
            xy = np.asarray([flange_pose(s['angles_deg'])[0][:2] for s in samples])
            span_x, span_y = np.ptp(xy[:, 0]), np.ptp(xy[:, 1])
            print(f'    📐 étalement courant : profondeur {span_x:.0f} mm × '
                  f'largeur {span_y:.0f} mm')
            if span_x < 0.5 * span_y:
                print('       → prochain point PLUS PRÈS ou PLUS LOIN du robot '
                      '(le nuage s\'aplatit)')

        if len(samples) < args.samples:
            input('    ⚠️  TIENS le bras — ENTER pour re-relâcher : ')
            bridge.send({'action': 'power_off'})
        print()

    return samples


def fit(samples, tip_z_mm=33.5):
    """Homographie pixel -> (X, Y) commandé en mm, + résidus par point."""
    if len(samples) < MIN_POINTS:
        raise SystemExit(f'{len(samples)} points — il en faut au moins {MIN_POINTS}')

    src = np.asarray([s['pixel'] for s in samples], dtype=np.float64)

    tool, rank = solve_tool_offset(samples, tip_z_mm)
    tips = fingertips(samples, tool)
    dst = tips[:, :2]
    print(f'\n  déport bride→doigts résolu : {np.round(tool, 1).tolist()} mm '
          f'(|t| = {np.linalg.norm(tool):.1f} mm, rang {rank}/3)')
    print(f'  Z des doigts : {np.round(tips[:, 2], 1).tolist()} — cible {tip_z_mm:.0f} mm')
    if rank < 3 or abs(np.median(tips[:, 2]) - tip_z_mm) > 15.0:
        print('  ⚠️ déport outil mal déterminé : poignet trop peu varié entre les points')

    # Un nuage de points aplati rend l'homographie (8 ddl) sous-contrainte
    # perpendiculairement : elle interpole bien le long de la bande et diverge
    # ailleurs. Mesuré le 17/08 : aplatissement 8.7x -> résidus de 400-700 mm.
    centred = dst - dst.mean(axis=0)
    sv = np.linalg.svd(centred, compute_uv=False)
    flatness = float(sv[0] / max(sv[1], 1e-9))
    print(f'  étalement des points : X {np.ptp(dst[:, 0]):.0f} mm × '
          f'Y {np.ptp(dst[:, 1]):.0f} mm — aplatissement {flatness:.1f}x')
    if flatness > 3.0:
        print('  ⚠️ points trop alignés — étale-les en PROFONDEUR (près ↔ loin '
              'du robot), pas seulement de gauche à droite.')

    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransacReprojThreshold=8.0)
    if H is None:
        raise SystemExit('ajustement impossible — points probablement colinéaires')

    projected = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    residuals = np.linalg.norm(projected - dst, axis=1)
    inliers = mask.ravel().astype(bool)

    print('\n=== RÉSIDUS (erreur de l\'homographie sur les points de teach) ===')
    for i, (s, r) in enumerate(zip(samples, residuals), start=1):
        flag = '' if inliers[i - 1] else '   ⚠️ écarté (RANSAC)'
        print(f'  point {i:2d}  pixel=({s["pixel"][0]:6.1f},{s["pixel"][1]:6.1f})  '
              f'doigts=({dst[i-1][0]:6.1f},{dst[i-1][1]:6.1f})mm  '
              f'résidu={r:5.1f} mm{flag}')

    keep = residuals[inliers]
    print(f'\n  points retenus : {inliers.sum()}/{len(samples)}')
    print(f'  résidu moyen   : {keep.mean():.1f} mm')
    print(f'  résidu médian  : {np.median(keep):.1f} mm')
    print(f'  résidu max     : {keep.max():.1f} mm')
    if keep.mean() > 10.0:
        print('\n  ⚠️ résidu moyen > 10 mm : teach imprécis, ou la caméra a bougé '
              'pendant la collecte, ou les points sont trop groupés.')
    return H, residuals, inliers


def save(H, samples, residuals, inliers, resolution):
    payload = {
        'description': 'Homographie pixel objet -> commande XY (mm, repere base). '
                       'Mesuree de bout en bout : remplace extrinseque + offset appris.',
        'resolution': list(resolution),
        'H_pixel_to_base_mm': H.tolist(),
        'n_samples': len(samples),
        'n_inliers': int(inliers.sum()),
        'residual_mean_mm': float(residuals[inliers].mean()),
        'residual_max_mm': float(residuals[inliers].max()),
        'samples': samples,
    }
    OUT.write_text(json.dumps(payload, indent=2))
    print(f'\n💾 {OUT}')


def apply_homography(H, u, v, calib_wh=None, frame_wh=None):
    """Pixel -> (X, Y) commandé en mm, à n'importe quelle résolution.

    Une homographie n'est valable qu'à la résolution où elle a été mesurée. Le
    teach tourne en 1600x1200 (plus précis) alors que le pick et le launch ROS
    tournent en 640x480 : sans remise à l'échelle on se trompe d'un facteur 2.5.
    ``calib_wh`` = résolution de calibration, ``frame_wh`` = celle du pixel fourni.
    """
    if calib_wh is not None and frame_wh is not None:
        u = u * (calib_wh[0] / frame_wh[0])
        v = v * (calib_wh[1] / frame_wh[1])
    p = cv2.perspectiveTransform(
        np.array([[[float(u), float(v)]]], dtype=np.float64), np.asarray(H))
    return float(p[0, 0, 0]), float(p[0, 0, 1])


def load_homography(path=OUT):
    """(H, (largeur, hauteur) de calibration) depuis le JSON."""
    data = json.loads(Path(path).read_text())
    return np.asarray(data['H_pixel_to_base_mm']), tuple(data['resolution'])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pi-host', default='10.10.0.224')
    ap.add_argument('--samples', type=int, default=9,
                    help='nombre de points de teach (defaut 9 = grille 3x3)')
    ap.add_argument('--width', type=int, default=1600)
    ap.add_argument('--height', type=int, default=1200)
    ap.add_argument('--exposure', type=int, default=75)
    ap.add_argument('--frames', type=int, default=8, help='frames par mesure de pixel')
    ap.add_argument('--spread-px', type=float, default=12.0)
    ap.add_argument('--hsv-lo', nargs=3, type=int, default=[20, 170, 70])
    ap.add_argument('--hsv-hi', nargs=3, type=int, default=[35, 255, 255])
    ap.add_argument('--fit-only', action='store_true',
                    help='refait l\'ajustement sur les points deja enregistres')
    ap.add_argument('--settle', type=float, default=0.8,
                    help='delai entre les deux lectures d\'angles (detection d\'affaissement)')
    ap.add_argument('--max-drift-deg', type=float, default=1.5,
                    help='derive articulaire toleree servos tenus, en degres')
    ap.add_argument('--angle-reads', type=int, default=5,
                    help='lectures d\'angles moyennees par point (ecrase le bruit encodeur)')
    ap.add_argument('--tip-z', type=float, default=33.5,
                    help='hauteur des doigts sur la balle en mm (rayon de la balle)')
    ap.add_argument('--resume', action='store_true',
                    help='repart des points conserves apres une interruption')
    args = ap.parse_args()

    if args.fit_only:
        data = json.loads(OUT.read_text())
        H, residuals, inliers = fit(data['samples'], args.tip_z)
        save(H, data['samples'], residuals, inliers, data['resolution'])
        return

    if args.samples < MIN_POINTS:
        raise SystemExit(f'--samples doit être >= {MIN_POINTS}')

    cap = open_camera(args.width, args.height, args.exposure)
    bridge = Bridge(args.pi_host)
    # Le bridge du Pi ne sert qu'UN client : un bridge_tour resté en vie monopolise
    # la socket et tout get_* expire au milieu du teach. On le détecte tout de suite.
    try:
        bridge.get_angles()
    except (TimeoutError, RuntimeError) as exc:
        raise SystemExit(
            f'bridge {args.pi_host}:5005 muet ({exc}). Un autre client le monopolise — '
            'arrête le launch multicam / le dashboard, puis vérifie avec '
            '`ss -tnp | grep 5005` qu\'aucune connexion ESTABLISHED ne subsiste.') from exc
    print(f'  ✅ bridge {args.pi_host}:5005')
    partial = []
    if args.resume and PARTIAL.is_file():
        partial = json.loads(PARTIAL.read_text())['samples']
    try:
        samples = collect(bridge, cap, args, partial)
    except (KeyboardInterrupt, EOFError):
        samples = partial
        print(f'\n\n⚠️  interrompu — {len(samples)} point(s) déjà acquis')
    finally:
        bridge.send({'action': 'power_on'})
        cap.release()

    if len(samples) < MIN_POINTS:
        # Le teach coûte cher : on garde les points bruts même s'ils sont trop peu
        # nombreux pour ajuster, et `--resume` les reprendra.
        PARTIAL.write_text(json.dumps({'samples': samples,
                                       'resolution': [args.width, args.height]}, indent=2))
        raise SystemExit(
            f'{len(samples)} point(s) < {MIN_POINTS} requis — rien n\'est ajusté.\n'
            f'Points conservés dans {PARTIAL} ; reprends avec --resume.')

    H, residuals, inliers = fit(samples, args.tip_z)
    save(H, samples, residuals, inliers, (args.width, args.height))


if __name__ == '__main__':
    main()
