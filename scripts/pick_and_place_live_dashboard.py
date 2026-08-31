#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pick-and-place live — meme tableau de bord, mais l'extrinseque est refaite
au lancement, SANS marqueur.

`pick_dashboard.py` charge une extrinseque figee (`arducam_extrinsic_pick.yaml`).
Un YAML gele n'est vrai que tant que la camera n'a pas bouge, et il y a un
precedent : le hand-eye invalide par un deplacement de camera entre juin et
juillet 2026. Tant que personne ne le verifie, la derive est silencieuse — elle
ne se voit que comme un pick qui rate de quelques centimetres.

Ici l'extrinseque est RECALCULEE a chaque demarrage, en utilisant le robot
lui-meme comme mire : pour chaque pose, la FK des encodeurs donne les keypoints
3D en repere base, DREAM les detecte en 2D dans l'image, et on cherche la pose
de camera qui reconcilie les deux. Aucun marqueur, aucune mire imprimee.

Ce que ce fichier n'est PAS : une copie de `pick_dashboard`. Il l'importe et
remplace seulement la source de l'extrinseque. `pick_dashboard.py` n'est jamais
modifie.

Limite a connaitre : la self-calibration robot-comme-mire est CIRCULAIRE vis-a-vis
de DREAM — si DREAM a un biais de detection systematique, l'extrinseque l'absorbe.
Le biais s'annule quand on reprojette des keypoints DREAM, mais PAS quand on
projette un objet detecte autrement (couleur, YOLO). C'est pourquoi le residu de
reprojection est affiche et compare a l'extrinseque en place : un ecart important
doit etre regarde, pas ignore.

Usage :
    # calibration sur la pose courante seulement (le bras ne bouge pas)
    python scripts/pick_and_place_live_dashboard.py

    # calibration sur plusieurs poses — LE BRAS BOUGE, degager la zone
    python scripts/pick_and_place_live_dashboard.py --move

    # reutiliser la derniere self-cal sans la refaire
    python scripts/pick_and_place_live_dashboard.py --reuse
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
# Chaque camera : /dev/videoN, stem d'intrinseque du registre, exposition,
# et le nom du YAML produit.
CAMERAS = {
    'svpro':   {'index': 0, 'intrinsics': 'cam_2', 'exposition': None,
                'stem': 'svpro_extrinsic_selfcal_live'},
    'arducam': {'index': 2, 'intrinsics': 'cam_3', 'exposition': 75,
                'stem': 'arducam_extrinsic_selfcal_live'},
}
# La SVPRO par defaut : elle voit le bras DE COTE. Depuis le zenith de
# l'arducam, base/link1/link2 tombent sur le meme pixel (verifie sur 400 poses,
# 0,000 mm d'ecart horizontal) et il ne reste pas assez de points distincts
# pour un solvePnP. De cote, les 162 mm entre base et link1 sont pleinement
# visibles. Ce n'est pas un reglage : c'est le placement de la camera.
DEFAUT = 'svpro'

for p in (RACINE, RACINE / 'scripts', RACINE / 'training' / 'dream', '/tmp/DREAM'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

KP = ['base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6']
# link2 est retire : c'est le MEME point 3D que link1 (verifie sur 400 poses,
# 0,000 mm d'ecart), le garder compterait deux fois la meme mesure. Tous les
# autres sont pris — mesure du 31/08, bras sur la planche : 7/7 detectes sur
# trois poses, 6/7 sur une quatrieme. Les exclure appauvrissait le solve pour
# rien.
KP_UTILISES = ['base', 'link1', 'link3', 'link4']

# Poses de calibration, en degres. Choisies par recherche sur la FK pour que les
# poses sont DERIVEES des points de travail enregistres (pick_approach, pick,
# observation_clear), en ne faisant tourner que J1 : cette rotation conserve
# exactement l'inclinaison de l'outil, donc la pince reste a 7-12 deg de la
# verticale, POINTEE VERS LE BAS comme pour une prise.
#
# Deux jeux de poses ont ete essayes et RETIRES. Avec J5=90 la pince partait a
# 90 deg — a l'horizontale : orientation qu'on ne prend jamais en travail, donc
# hors de la distribution sur laquelle DREAM a ete affine (real_3cam). Et des
# poses plus depliees sortaient le bras de la planche, au-dessus du tapis et du
# trepied : la detection tombait de 7/7 a 0-1/7. Le fond et l'orientation
# comptent plus que l'etalement des keypoints. Jouees seulement avec --move.
POSES_CALIB = [
    [ 28.53, -118.74,  82.79, -102.56, -17.84, 49.83],
    [ 58.53, -118.74,  82.79, -102.56, -17.84, 49.83],
    [ 88.53, -118.74,  82.79, -102.56, -17.84, 49.83],
    [ 11.92, -128.67,  76.55,  -53.43,  -3.69, 21.79],
    [ 41.92, -128.67,  76.55,  -53.43,  -3.69, 21.79],
    [ 71.92, -128.67,  76.55,  -53.43,  -3.69, 21.79],
    [ 53.05, -120.05,  90.00,  -60.55,  10.28,  6.24],
    [ 83.05, -120.05,  90.00,  -60.55,  10.28,  6.24],
]

RESIDU_MAX_PX = 8.0        # au-dela, la calibration est refusee
ECART_ALERTE_MM = 30.0     # ecart a l'extrinseque en place qui merite un mot


def _reseau():
    """Charge le reseau DREAM (checkpoint de production)."""
    import dream
    ckpt = RACINE / 'training' / 'dream' / 'checkpoints_dream' / \
        'vgg_ultimate_v4_mix_ft_e30' / 'best_network.pth'
    if not ckpt.exists():
        raise SystemExit(f'Checkpoint DREAM introuvable : {ckpt}')
    # La config porte le nom du checkpoint (best_network.yaml), pas config.yaml.
    cfg = ckpt.with_suffix('.yaml')
    if not cfg.exists():
        raise SystemExit(f'Config du reseau introuvable : {cfg}')
    return dream.create_network_from_config_file(str(cfg), str(ckpt))


def _camera(index, exposition):
    """Arducam en 640x480 — l'intrinseque n'est valable que dans CE mode."""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit(f'Camera /dev/video{index} inaccessible')
    if exposition is not None:
        import subprocess
        subprocess.run(['v4l2-ctl', '-d', f'/dev/video{index}',
                        '-c', 'auto_exposure=1',
                        '-c', f'exposure_time_absolute={exposition}'],
                       capture_output=True)
    for _ in range(8):          # purge des trames d'auto-exposition
        cap.read()
    return cap


PAS_MAX_DEG = 30.0     # au-dela, on decoupe le trajet en paliers


def rejoint(pont, cible, vitesse=25):
    """Rejoint `cible` par paliers de PAS_MAX_DEG au plus.

    Un ordre unique depuis une pose eloignee fait partir toutes les
    articulations a fond en meme temps : mesure du 31/08, le bras etait a 99 deg
    de la premiere pose de calibration sur J4 seul. On decoupe.
    """
    depart = np.array(pont.get_angles(), float)
    cible = np.array(cible, float)
    n = max(1, int(np.ceil(np.max(np.abs(cible - depart)) / PAS_MAX_DEG)))
    for i in range(1, n + 1):
        etape = depart + (cible - depart) * (i / n)
        pont.send({'action': 'send_angles',
                   'angles': [round(float(v), 2) for v in etape],
                   'speed': vitesse})
        time.sleep(2.0 if i < n else 3.0)


def collecte(pont, net, cap, poses, bouger):
    """(3D base, 2D image) pour chaque keypoint fiable, sur toutes les poses."""
    from PIL import Image
    from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

    garder = {KP.index(n) for n in KP_UTILISES}
    objs, dets, kidx = [], [], []
    sequence = poses if bouger else [None]

    for i, pose in enumerate(sequence, 1):
        if pose is not None:
            rejoint(pont, pose)
        angles = pont.get_angles()
        # Vider le tampon V4L2 : sans ca on capture une trame ANTERIEURE, prise
        # pendant le deplacement, donc floue — mesure du 31/08, la detection
        # passait de 7/7 a 2/7 pour cette seule raison.
        for _ in range(6):
            cap.read()
        ok, image = cap.read()
        if not ok:
            print(f'  pose {i}: image non lue, ignoree')
            continue

        pos, _ = forward_kinematics(np.radians(angles))
        obj = np.array([pos[n] for n in KEYPOINT_NAMES], float)
        hauteur, largeur = image.shape[:2]
        rgb = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        kps = np.array(net.keypoints_from_image(rgb)['detected_keypoints'], float)

        vus = 0
        for k in range(len(KP)):
            # DREAM signale « non detecte » par [-999.999, -999.999] en repere
            # reseau, qui se remet a l'echelle loin HORS cadre sans jamais etre
            # NaN : il faut un test de cadre, pas seulement un test de NaN.
            x, y = kps[k]
            if (k in garder and np.all(np.isfinite(kps[k]))
                    and 0 <= x < largeur and 0 <= y < hauteur):
                objs.append(obj[k]); dets.append(kps[k]); kidx.append(k)
                vus += 1
        print(f'  pose {i}/{len(sequence)} : {vus}/{len(garder)} keypoints')

    return (np.array(objs, float), np.array(dets, float), np.array(kidx, int))


def resous(objs, dets, K, dist, amorce=None):
    """Extrinseque partagee, moindres carres robustes + rejet des aberrants."""
    def residus(x, masque):
        rvec, tvec = x[:3].reshape(3, 1), x[3:].reshape(3, 1)
        proj, _ = cv2.projectPoints(objs[masque], rvec, tvec, K, dist)
        return (proj.reshape(-1, 2) - dets[masque]).ravel()

    if amorce is not None:
        rvec0, _ = cv2.Rodrigues(amorce[:3, :3])
        x = np.concatenate([rvec0.ravel(), amorce[:3, 3]])
    else:
        ok, rvec0, tvec0 = cv2.solvePnP(objs, dets, K, dist,
                                        flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            raise SystemExit('solvePnP initial a echoue')
        x = np.concatenate([rvec0.ravel(), tvec0.ravel()])

    garde = np.ones(len(objs), bool)
    for _ in range(5):
        r = least_squares(residus, x, args=(garde,), loss='soft_l1', f_scale=5.0,
                          max_nfev=2000)
        x = r.x
        err = np.linalg.norm(residus(x, np.ones(len(objs), bool)).reshape(-1, 2),
                             axis=1)
        neuf = err < max(RESIDU_MAX_PX, 3.0 * np.median(err))
        if neuf.sum() < 12 or (neuf == garde).all():
            break
        garde = neuf

    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(x[:3].reshape(3, 1))[0]
    T[:3, 3] = x[3:]
    return T, err, garde


def calibre(nom, bouger, host):
    from pick_and_place_real import Bridge
    from mycobot_gateway.vision.camera_registry import load_intrinsics

    cam = CAMERAS[nom]
    print(f'Self-calibration markerless sur {nom} '
          f'(/dev/video{cam["index"]}) — le robot est la mire')
    K, dist = load_intrinsics(cam['intrinsics'])

    pont = Bridge(host)
    net = _reseau()
    cap = _camera(cam['index'], cam['exposition'])
    try:
        objs, dets, kidx = collecte(pont, net, cap, POSES_CALIB, bouger)
    finally:
        cap.release()

    if len(objs) < 12:
        raise SystemExit(f'{len(objs)} correspondances seulement — '
                         f'la camera voit-elle le bras ?')

    # Garde-fou de conditionnement. base/link1/link2 partagent la meme verticale
    # (link1 et link2 sont carrement le MEME point : verifie sur 400 poses, 0,000
    # mm d'ecart). Vue d'une camera au zenith ils tombent sur le meme pixel, et
    # les 5 keypoints n'en font plus que 3 distincts — solvePnP en demande 4.
    # Mesure du 31/08 sur l'arducam au zenith : link3 a 32,8 px et 1468 mm
    # d'ecart avec l'extrinseque en place. Ce n'est pas un reglage, c'est le
    # placement de la camera.
    etendue = np.linalg.norm(dets - dets.mean(axis=0), axis=1)
    distincts = len({tuple(np.round(d / 12.0).astype(int)) for d in dets})
    if distincts < 4:
        raise SystemExit(
            f'Seulement {distincts} groupes de keypoints distincts dans l image '
            f'(il en faut 4). La camera voit-elle le bras de biais ? Depuis le '
            f'zenith, base/link1/link2 se superposent et la self-calibration est '
            f'sous-determinee — utiliser une vue de cote.')

    T, err, garde = resous(objs, dets, K, dist)
    residu = float(np.median(err[garde]))
    print(f'\n  {int(garde.sum())}/{len(objs)} correspondances retenues')
    print(f'  residu de reprojection median : {residu:.2f} px')
    for k in sorted(set(kidx)):
        m = (kidx == k) & garde
        if m.any():
            print(f'    {KP[k]:7} {np.median(err[m]):5.2f} px  ({int(m.sum())} pts)')
    pires = {KP[k]: float(np.median(err[(kidx == k) & garde]))
             for k in sorted(set(kidx)) if ((kidx == k) & garde).any()}
    mauvais = {n: v for n, v in pires.items() if v > 2.5 * RESIDU_MAX_PX}
    if mauvais:
        raise SystemExit(
            f'Keypoints aberrants malgre un residu median acceptable : '
            f'{ {n: round(v, 1) for n, v in mauvais.items()} } px. La mediane '
            f'masque le probleme — calibration refusee.')
    if residu > RESIDU_MAX_PX:
        raise SystemExit(f'Residu {residu:.2f} px > {RESIDU_MAX_PX} : '
                         f'calibration refusee, rien n a ete ecrit.')

    ancienne = CALIB / 'arducam_extrinsic_pick.yaml'
    if ancienne.exists():
        Tv = np.array(yaml.safe_load(ancienne.read_text())['T_cam_world'], float)
        ecart = np.linalg.norm(np.linalg.inv(T)[:3, 3] - np.linalg.inv(Tv)[:3, 3])
        mot = ('  <-- a regarder' if ecart * 1000 > ECART_ALERTE_MM else '')
        print(f'\n  ecart avec {ancienne.name} : {ecart * 1000:.0f} mm{mot}')

    sortie = CALIB / f'{cam["stem"]}.yaml'
    sortie.write_text(yaml.safe_dump({
        'source': 'pick_and_place_live_dashboard (self-cal DREAM, sans marqueur)',
        'camera': nom,
        'intrinsics_stem': cam['intrinsics'],
        'resolution': [640, 480],
        'frame_id': 'base_link',
        'keypoints_used': KP_UTILISES,
        'poses': len(POSES_CALIB) if bouger else 1,
        'n_inliers': int(garde.sum()),
        'median_reproj_px': residu,
        'date': time.strftime('%Y-%m-%d %H:%M'),
        'T_cam_world': T.tolist(),
        'T_world_cam': np.linalg.inv(T).tolist(),
    }, sort_keys=False))
    print(f'  ecrit : {sortie.name}\n')
    return sortie


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--camera', default=DEFAUT, choices=sorted(CAMERAS),
                    help='camera a calibrer (defaut : svpro, vue de cote)')
    ap.add_argument('--host', default='10.10.0.224')
    ap.add_argument('--move', action='store_true',
                    help='joue plusieurs poses — LE BRAS BOUGE')
    ap.add_argument('--reuse', action='store_true',
                    help='garde la derniere self-cal sans la refaire')
    ap.add_argument('--calib-only', action='store_true',
                    help='calibre et s arrete, sans ouvrir le tableau de bord')
    args = ap.parse_args()

    stem = CAMERAS[args.camera]['stem']
    fichier = CALIB / f'{stem}.yaml'
    if args.reuse:
        if not fichier.exists():
            raise SystemExit(f'{fichier.name} absent — lancer sans --reuse.')
        d = yaml.safe_load(fichier.read_text())
        print(f'Self-cal reutilisee du {d.get("date")} '
              f'({d.get("median_reproj_px", 0):.2f} px)')
    else:
        calibre(args.camera, args.move, args.host)

    if args.calib_only:
        return

    # Le tableau de bord tel quel, sur l'extrinseque fraiche. `Fenetre` appelle
    # le `Vision` global du module : le remplacer ici suffit, le fichier
    # pick_dashboard.py n'est pas touche.
    import pick_dashboard

    class VisionAutoCalibree(pick_dashboard.Vision):
        def __init__(self, stem=stem):
            super().__init__(stem)

    pick_dashboard.Vision = VisionAutoCalibree
    pick_dashboard.main()


if __name__ == '__main__':
    main()
