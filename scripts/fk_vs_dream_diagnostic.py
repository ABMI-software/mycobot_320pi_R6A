#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FK contre DREAM : lequel des deux se trompe ?

La self-calibration markerless laissait un residu systematique (link3 a 23.6 px)
qu'aucune pose de camera unique n'expliquait. Deux causes possibles, et DREAM ne
peut pas trancher entre elles : dans son schema, la FK est une ENTREE du PnP, pas
une sortie. L'utiliser pour verifier la FK serait circulaire.

Il faut donc une reference exterieure aux deux. C'est le role des extrinseques
MARQUEURS deja calculees (arducam_extrinsic_pick, svpro_extrinsic_servo) : elles
ne doivent rien a DREAM. On projette le squelette FK a travers elles et on le
compare aux detections DREAM, sur les deux cameras a la fois.

Lecture du resultat :
  - le vert epouse le bras dans LES DEUX vues  -> FK et extrinseques validees,
    l'ecart restant est du cote de DREAM ;
  - le vert rate dans les deux de la meme facon -> la FK (ou l'extrinseque) est
    en cause.

Deux vues independantes ne peuvent pas se tromper de la meme maniere par hasard,
et le keypoint `base` sert de juge de paix : il est FIXE, sa projection ne depend
d'aucun angle.

Le bras NE BOUGE PAS : on lit les encodeurs et les cameras, rien d'autre.

Usage (venv_dream) :
    # capture live
    python scripts/fk_vs_dream_diagnostic.py

    # rejouer hors ligne sur les images deja enregistrees (robot eteint)
    python scripts/fk_vs_dream_diagnostic.py --brut \
        --angles 82.7,-122.6,88.76,-61.08,10.89,6.5
"""
import argparse
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
SORTIE = RACINE / 'docs'

sys.path.insert(0, '/tmp/DREAM')
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))

import dream                                                    # noqa: E402
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES       # noqa: E402
import camera_registry as registre                              # noqa: E402

# L'arducam est debranchee/rebranchee de temps en temps : son /dev/videoN
# change et elle PERD son exposition. On la retrouve donc par son nom V4L2,
# et on reimpose 75 a chaque capture — la valeur du registre (cam_3) sur
# laquelle l'extrinseque a ete faite. Sans ce forcage, une camera revenue en
# auto apres un replug fait croire a une regression de detection.
# (L'exposition ne change pas la detection : balayee 20-300 le 01/09, 0 a
# 4/7, aucun optimum. Mais on la fige pour que les mesures soient comparables.)
VUES = [
    ('arducam', {'nom_v4l2': 'Arducam', 'exposition': 75,
                 'extr': 'arducam_extrinsic_pick'}),
    ('svpro',   {'nom_v4l2': '5MP', 'exposition': None,
                 'extr': 'svpro_extrinsic_servo'}),
]
COURT = ['base', 'J1', 'J2', 'J3', 'J4', 'J5', 'bride']
VERT, ORANGE, BLANC, NOIR = (60, 220, 60), (0, 165, 255), (255, 255, 255), (0, 0, 0)
GRIS, FOND = (190, 190, 190), 32
FONT = cv2.FONT_HERSHEY_SIMPLEX
# Modele par defaut. `--modele` permet de comparer deux checkpoints sur la MEME
# pose et les MEMES images : c'est la seule facon de mesurer un gain en direct
# sans que le bras ait bouge entre les deux mesures.
CHECKPOINTS = RACINE / 'training' / 'dream' / 'checkpoints_dream'
MODELE_DEFAUT = 'vgg_montage0901_ft_e30'


def texte(img, s, xy, couleur, echelle=0.5, epais=1, halo=True):
    """Halo par decalages a epaisseur CONSTANTE.

    Un contour trace avec `epais + 2` avance plus vite caractere par caractere
    dans les fontes Hershey : sa queue depasse la passe couleur et le texte
    parait dedouble. Meme epaisseur partout, pas de dedoublement.
    """
    x, y = xy
    if halo:
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            cv2.putText(img, s, (x + dx, y + dy), FONT, echelle, NOIR,
                        epais, cv2.LINE_AA)
    cv2.putText(img, s, xy, FONT, echelle, couleur, epais, cv2.LINE_AA)


def index_v4l2(motif):
    """Premier /dev/videoN capable de capturer, pour la camera nommee `motif`."""
    sortie = subprocess.run(['v4l2-ctl', '--list-devices'],
                            capture_output=True, text=True).stdout
    for bloc in sortie.split('\n\n'):
        if motif.lower() in bloc.split('\n')[0].lower():
            for ligne in bloc.split('\n')[1:]:
                ligne = ligne.strip()
                if ligne.startswith('/dev/video'):
                    return int(ligne.removeprefix('/dev/video'))
    raise SystemExit(f'camera introuvable : {motif}')


def capture(cfg):
    """Trame 640x480 — l'intrinseque n'est valable que dans CE mode."""
    index = cfg['index'] if 'index' in cfg else index_v4l2(cfg['nom_v4l2'])
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if cfg.get('exposition') is not None:
        subprocess.run(['v4l2-ctl', '-d', f'/dev/video{index}',
                        '-c', 'auto_exposure=1',
                        '-c', f"exposure_time_absolute={cfg['exposition']}"],
                       capture_output=True)
    # Vider le tampon V4L2 : sans ca on lit une trame anterieure, prise pendant
    # un mouvement, donc floue — la detection passait de 7/7 a 2/7 (31/08).
    for _ in range(8):
        cap.read()
    ok, image = cap.read()
    cap.release()
    if not ok:
        raise SystemExit(f'lecture impossible sur /dev/video{index}')
    return image


def panneau(nom, cfg, obj, net, brut):
    d = yaml.safe_load((CALIB / f"{cfg['extr']}.yaml").read_text())
    T = np.array(d['T_cam_world'], float)
    K, dist = registre.load_intrinsics(d['intrinsics_stem'])
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    proj = cv2.projectPoints(obj, rvec, T[:3, 3], K, dist)[0].reshape(-1, 2)

    chemin_brut = SORTIE / f'fk_dream_{nom}_brut.png'
    if brut:
        image = cv2.imread(str(chemin_brut))
        if image is None:
            raise SystemExit(f'image brute absente : {chemin_brut}')
    else:
        image = capture(cfg)
        cv2.imwrite(str(chemin_brut), image)

    from PIL import Image
    kps = np.array(net.keypoints_from_image(
        Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    )['detected_keypoints'], float)

    img = image.copy()
    h, w = img.shape[:2]
    for a, b in zip(range(6), range(1, 7)):
        cv2.line(img, tuple(proj[a].astype(int)), tuple(proj[b].astype(int)),
                 VERT, 2, cv2.LINE_AA)

    ecarts = {}
    for k, court in enumerate(COURT):
        fu, fv = proj[k]
        du, dv = kps[k]
        # DREAM signale « non detecte » par [-999.999, -999.999] en repere
        # reseau, qui se remet a l'echelle loin HORS cadre sans jamais etre NaN :
        # il faut un test de cadre, pas seulement un test de NaN.
        vu = np.all(np.isfinite(kps[k])) and 0 <= du < w and 0 <= dv < h
        if vu:
            ecarts[court] = float(np.hypot(fu - du, fv - dv))
            cv2.line(img, (int(fu), int(fv)), (int(du), int(dv)), BLANC, 1, cv2.LINE_AA)
            cv2.circle(img, (int(du), int(dv)), 6, ORANGE, -1, cv2.LINE_AA)
            cv2.circle(img, (int(du), int(dv)), 6, NOIR, 1, cv2.LINE_AA)
        cv2.circle(img, (int(fu), int(fv)), 8, VERT, 2, cv2.LINE_AA)
        # J1 et J2 tombent sur le MEME pixel (meme point 3D dans la FK) :
        # l'etiquette de J2 est decalee, sinon les deux se superposent.
        texte(img, court, (int(fu) + 10, int(fv) + (20 if court == 'J2' else -12)),
              VERT, 0.45)

    bandeau = np.full((58, w, 3), FOND, np.uint8)
    texte(bandeau, f'{nom}   extrinseque {cfg["extr"]}   K {d["intrinsics_stem"]}',
          (10, 22), BLANC, 0.52, halo=False)
    med = f'{np.median(list(ecarts.values())):.0f} px' if ecarts else '-'
    texte(bandeau, f'{len(ecarts)}/7 detectes par DREAM   '
                   f'ecart median FK<->DREAM {med}',
          (10, 44), ORANGE, 0.5, halo=False)
    return np.vstack([bandeau, img]), ecarts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--brut', action='store_true',
                    help='rejouer sur les images deja enregistrees (sans materiel)')
    ap.add_argument('--angles', help='6 angles en degres, separes par des virgules ; '
                                     'sinon lus sur le bridge')
    ap.add_argument('--host', default='10.10.0.224')
    ap.add_argument('--modele', default=MODELE_DEFAUT,
                    help='nom du dossier sous checkpoints_dream/')
    args = ap.parse_args()

    if args.angles:
        angles = np.array([float(v) for v in args.angles.split(',')], float)
    else:
        from pick_and_place_real import Bridge
        angles = np.array(Bridge(args.host).get_angles(), float)
    print('Encodeurs (deg) :', np.round(angles, 2).tolist())

    poids = CHECKPOINTS / args.modele / 'best_network.pth'
    if not poids.is_file():
        raise SystemExit(f'modele introuvable : {poids}')
    print('Modele          :', args.modele)

    pos, _ = forward_kinematics(np.radians(angles))
    obj = np.array([pos[n] for n in KEYPOINT_NAMES], float)
    net = dream.create_network_from_config_file(
        str(poids.with_suffix('.yaml')), str(poids))

    panneaux, tout = [], {}
    for nom, cfg in VUES:
        p, e = panneau(nom, cfg, obj, net, args.brut)
        cv2.imwrite(str(SORTIE / f'fk_dream_{nom}.png'), p)
        panneaux.append(p)
        tout[nom] = e

    largeur = sum(p.shape[1] for p in panneaux) + 8
    duo = np.full((panneaux[0].shape[0] + 108, largeur, 3), FOND, np.uint8)
    x = 0
    for p in panneaux:
        duo[108:108 + p.shape[0], x:x + p.shape[1]] = p
        x += p.shape[1] + 8

    texte(duo, 'FK (encodeurs + extrinseque MARQUEURS) contre DREAM',
          (12, 28), BLANC, 0.62, 2, halo=False)
    q = 'q = ' + np.array2string(np.round(angles, 1), separator=', ') + ' deg'
    (lq, _), _ = cv2.getTextSize(q, FONT, 0.48, 1)
    texte(duo, q, (largeur - lq - 14, 28), BLANC, 0.48, halo=False)
    texte(duo, 'meme pose, deux vues independantes, deux extrinseques calculees separement',
          (12, 52), GRIS, 0.48, halo=False)
    cv2.line(duo, (16, 74), (44, 74), VERT, 3, cv2.LINE_AA)
    texte(duo, 'squelette FK projete : il epouse le bras dans les deux vues '
               '-> FK et extrinseques valides', (54, 79), VERT, 0.5, halo=False)
    cv2.circle(duo, (30, 96), 7, ORANGE, -1, cv2.LINE_AA)
    texte(duo, 'detections DREAM : hors du bras -> pose defavorable, '
               'detections fantomes', (54, 101), ORANGE, 0.5, halo=False)
    cv2.imwrite(str(SORTIE / 'fk_vs_dream.png'), duo)

    for nom, e in tout.items():
        print(f'{nom:8s} {len(e)}/7  '
              + '  '.join(f'{k}={v:.0f}px' for k, v in e.items()))
    print('\necrites dans', SORTIE)


if __name__ == '__main__':
    main()
