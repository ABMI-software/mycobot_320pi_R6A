#!/usr/bin/env python3
"""Non-regression DREAM : sans pince (rendu du 50K) contre avec pince, memes q, memes cameras.

Lit dream_vs_fk.csv et poses.csv (dream_balayage_pince.py) des deux dossiers ;
ne garde que les images prises pendant la tenue d'une pose ; par camera et par
keypoint : taux de detection et erreur pixel mediane a la projection FK, dans
chaque configuration, puis l'ecart. Les keypoints dont la projection FK tombe
hors de l'image ne comptent pas.

    python3 scripts/dream_pince_compare.py <dossier sans pince> <dossier avec pince>
"""

import csv
from pathlib import Path
import sys

import numpy as np

KEYPOINTS = ('base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6')
# Detecte mais a plus de 20 px de la FK : ce n'est pas le keypoint.
ABERRANT_PX = 20.0


def lignes_tenues(dossier):
    poses = list(csv.DictReader(open(dossier / 'poses.csv')))
    fenetres = [(float(p['t_debut']), float(p['t_fin']), int(p['pose'])) for p in poses]
    out = []
    for r in csv.DictReader(open(dossier / 'dream_vs_fk.csv')):
        if r['fk_in_image'] != 'True':
            continue
        t = float(r['stamp_sim'])
        pose = next((i for a, b, i in fenetres if a <= t <= b), None)
        if pose is not None:
            out.append({**r, 'pose': pose})
    return out, len(poses)


def stats(lignes):
    """{(camera, keypoint): (n, taux de detection, mediane px, taux aberrant)}."""
    groupes = {}
    for r in lignes:
        groupes.setdefault((r['camera_id'], r['keypoint']), []).append(r)
    out = {}
    for cle, g in groupes.items():
        err = np.array([float(r['pixel_error']) for r in g if r['valid'] == 'True'])
        out[cle] = (len(g), len(err) / len(g), float(np.median(err)) if len(err) else np.nan,
                    float(np.mean(err > ABERRANT_PX)) if len(err) else np.nan)
    return out


def main():
    sans, avec = Path(sys.argv[1]), Path(sys.argv[2])
    l_sans, n_sans = lignes_tenues(sans)
    l_avec, n_avec = lignes_tenues(avec)
    s_sans, s_avec = stats(l_sans), stats(l_avec)
    print(f'sans pince : {sans.name}, {n_sans} poses, {len(l_sans)} lignes')
    print(f'avec pince : {avec.name}, {n_avec} poses, {len(l_avec)} lignes\n')
    print(f'{"camera":20s} {"keypoint":8s} | {"detection sans":>14s} {"avec":>6s} | '
          f'{"mediane px sans":>15s} {"avec":>6s} | {"aberrant sans":>13s} {"avec":>6s}')
    tableau = []
    for cam in sorted({c for c, _ in s_sans} | {c for c, _ in s_avec}):
        for kp in KEYPOINTS:
            a, b = s_sans.get((cam, kp)), s_avec.get((cam, kp))
            if a is None or b is None:
                continue
            print(f'{cam:20s} {kp:8s} | {a[1]:14.0%} {b[1]:6.0%} | {a[2]:15.1f} {b[2]:6.1f} | '
                  f'{a[3]:13.0%} {b[3]:6.0%}')
            tableau.append({'camera_id': cam, 'keypoint': kp, 'n_sans': a[0], 'n_avec': b[0],
                            'detection_sans': round(a[1], 4), 'detection_avec': round(b[1], 4),
                            'mediane_px_sans': round(a[2], 2), 'mediane_px_avec': round(b[2], 2),
                            'aberrant_sans': round(a[3], 4), 'aberrant_avec': round(b[3], 4)})
    for nom, lignes in (('sans', l_sans), ('avec', l_avec)):
        valides = [r for r in lignes if r['valid'] == 'True']
        err = np.array([float(r['pixel_error']) for r in valides])
        print(f'\nTOTAL {nom} pince : detection {len(valides) / len(lignes):.1%}, '
              f'mediane {np.median(err):.1f} px, aberrant (> {ABERRANT_PX:.0f} px) '
              f'{np.mean(err > ABERRANT_PX):.1%}')
    sortie = avec / 'dream_pince_compare.csv'
    with open(sortie, 'w', newline='') as f:
        ecrit = csv.DictWriter(f, fieldnames=list(tableau[0]))
        ecrit.writeheader()
        ecrit.writerows(tableau)
    print(f'\n-> {sortie}')


if __name__ == '__main__':
    main()
