#!/usr/bin/env python3
"""DREAM suit-il l'image ? Test de decalage, etape 3 du protocole reel.

Le 02/09, le keypoint `base` de `vgg_montage0901_ft_e30` s'est revele etre une
CONSTANTE par montage : l'image decalee de N px donnait une detection decalee
de 0 px. Une pose camera ajustee sur un tel point restitue le cadrage des
donnees d'affinage, et la comparer aux marqueurs ArUco devient circulaire.
Ce test se fait donc AVANT tout chiffre DREAM sur le banc.

On decale l'image entiere de (du, dv) et on mesure, keypoint par keypoint, la
part du decalage que la detection reproduit :

    suivi = (detection_decalee - detection_origine) . (du, dv) / |(du, dv)|^2

1 = le point suit l'image ; 0 = il ne bouge pas (memorise).

Le bras NE BOUGE PAS : on lit les cameras, rien d'autre. Aucun bridge.

    source ~/ros_jazzy/venv_dream/bin/activate
    python scripts/dream_test_decalage.py                       # capture les deux cameras
    python scripts/dream_test_decalage.py --images a.png b.png  # images deja prises
"""
import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

RACINE = Path(__file__).resolve().parents[1]
# fk_vs_dream_diagnostic pointe encore sur /tmp/DREAM, disparu au redemarrage.
sys.path.insert(0, str(Path.home() / 'DREAM'))
sys.path.insert(0, str(RACINE / 'scripts'))

import dream                                                     # noqa: E402
from fk_vs_dream_diagnostic import VUES, capture, COURT          # noqa: E402

CHECKPOINTS = RACINE / 'training' / 'dream' / 'checkpoints_dream'
MODELE_DEFAUT = 'vgg_tri_mix_ft_e10'
DECALAGES = [(10, 0), (-10, 0), (20, 0), (-20, 0), (40, 0), (-40, 0),
             (0, 10), (0, -10), (0, 20), (0, -20), (0, 40), (0, -40)]


def detecte(net, image):
    """7 keypoints (u, v) en pixels image ; NaN si non detecte.

    DREAM signale « non detecte » par [-999.999, -999.999] en repere reseau, remis
    a l'echelle hors cadre sans etre NaN : d'ou le test de cadre.
    """
    kps = np.array(net.keypoints_from_image(
        Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))['detected_keypoints'], float)
    h, w = image.shape[:2]
    hors = ~(np.isfinite(kps).all(1) & (kps[:, 0] >= 0) & (kps[:, 0] < w)
             & (kps[:, 1] >= 0) & (kps[:, 1] < h))
    kps[hors] = np.nan
    return kps


def decale(image, du, dv):
    # Bord repete et non noir : une bande noire serait un indice de cadrage.
    M = np.float32([[1, 0, du], [0, 1, dv]])
    return cv2.warpAffine(image, M, image.shape[1::-1], borderMode=cv2.BORDER_REPLICATE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--modele', default=MODELE_DEFAUT, help='dossier sous checkpoints_dream/')
    ap.add_argument('--images', nargs='+', help='images 640x480 deja prises, au lieu de capturer')
    ap.add_argument('--seuil', type=float, default=0.9,
                    help='suivi minimal pour dire qu un point suit l image (a trancher)')
    args = ap.parse_args()

    poids = CHECKPOINTS / args.modele / 'best_network.pth'
    if not poids.is_file():
        raise SystemExit(f'modele introuvable : {poids}')
    net = dream.create_network_from_config_file(str(poids.with_suffix('.yaml')), str(poids))

    sortie = (RACINE / 'results' / 'protocole_reel' / 'etape3_decalage'
              / f'{datetime.now():%Y-%m-%d_%H%M}_{args.modele}')
    sortie.mkdir(parents=True, exist_ok=True)

    if args.images:
        sources = [(Path(p).stem, cv2.imread(p)) for p in args.images]
    else:
        sources = [(nom, capture(cfg)) for nom, cfg in VUES]
    for nom, image in sources:
        if image is None:
            raise SystemExit(f'image illisible : {nom}')
        cv2.imwrite(str(sortie / f'{nom}_origine.png'), image)

    lignes, verdicts = [], {}
    for nom, image in sources:
        k0 = detecte(net, image)
        print(f'\n=== {nom} : {int(np.isfinite(k0[:, 0]).sum())}/7 detectes sur l image d origine ===')
        print(f'{"decalage":>10s}  ' + '  '.join(f'{c:>6s}' for c in COURT))
        suivis = {c: [] for c in COURT}
        for du, dv in DECALAGES:
            k1 = detecte(net, decale(image, du, dv))
            s = ((k1 - k0) @ np.array([du, dv], float)) / (du * du + dv * dv)
            print(f'{f"({du:+d},{dv:+d})":>10s}  '
                  + '  '.join('     -' if np.isnan(v) else f'{v:6.2f}' for v in s))
            for c, v, a, b in zip(COURT, s, k0, k1):
                lignes.append([nom, du, dv, c, *a, *b, v])
                if not np.isnan(v):
                    suivis[c].append(v)
        med = {c: (np.median(v) if v else np.nan) for c, v in suivis.items()}
        print(f'{"MEDIANE":>10s}  ' + '  '.join('     -' if np.isnan(v) else f'{v:6.2f}'
                                               for v in med.values()))
        verdicts[nom] = med

    with open(sortie / 'decalage.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['camera', 'du_px', 'dv_px', 'keypoint', 'u_origine', 'v_origine',
                    'u_decale', 'v_decale', 'suivi'])
        w.writerows(lignes)

    print(f'\n=== verdict (seuil {args.seuil:.2f}, a trancher) ===')
    for nom, med in verdicts.items():
        memorises = [c for c, v in med.items() if not np.isnan(v) and v < args.seuil]
        absents = [c for c, v in med.items() if np.isnan(v)]
        etat = 'SUIT L IMAGE' if not memorises else 'NE SUIT PAS : ' + ', '.join(memorises)
        print(f'{nom:10s} {etat}' + (f'   (non detectes : {", ".join(absents)})' if absents else ''))
    print('\necrit dans', sortie)


if __name__ == '__main__':
    main()
