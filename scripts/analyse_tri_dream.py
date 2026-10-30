#!/usr/bin/env python3
"""DREAM par keypoint sur un enregistrement reel, en statique et en dynamique.

    ~/ros_jazzy/venv_dream/bin/python scripts/analyse_tri_dream.py results/dream_reel/<session>

Entree : `enregistre_tri_dream.py` (images arducam + SVPRO, codeurs horodates).
Reference de chaque keypoint : FK des codeurs, projetee dans l'image.
  - arducam : a travers l'extrinseque ArUco (`arducam_extrinsic_pick.yaml`),
    reference exterieure a DREAM ;
  - SVPRO : a travers `svpro_extrinsic_labels.yaml` (ArUco, la meme qui a
    etiquete le fine-tuning reel). Sans elle (`--svpro-detection-seule`), taux de
    detection seul : une pose ajustee sur les detections elles-memes a ete
    essayee le 13/10, et selon son point de depart link6 sortait a 3 px ou a
    208 px pour une camera deplacee de 145 mm. Ce chiffre ne mesure rien.

Statique : codeurs immobiles (< 0,3 deg sur chaque joint) dans +/- 0,4 s autour
de l'image. Dynamique : le reste, si les codeurs encadrent l'image a 0,3 s pres.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path.home() / 'DREAM'))
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
import dream  # noqa: E402
import camera_registry as registre  # noqa: E402
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402

CALIB = RACINE / 'training' / 'calibration'
EXTRINSEQUES = {'arducam': 'arducam_extrinsic_pick.yaml', 'svpro': 'svpro_extrinsic_labels.yaml'}
MODELE = 'vgg_tri_reel_ft_e10'
NOMS = [n.replace('mycobot320_', '') for n in KEYPOINT_NAMES]
FENETRE_STATIQUE_S = 0.4
IMMOBILE_DEG = 0.3
ENCADREMENT_S = 0.3
SEUILS_PX = (2, 5, 10, 20, 50)
VERT, ORANGE, BLANC = (60, 220, 60), (0, 165, 255), (255, 255, 255)


def etat_codeurs(t, tc, qc):
    i = np.searchsorted(tc, t)
    if i == 0 or i == len(tc) or tc[i] - t > ENCADREMENT_S or t - tc[i - 1] > ENCADREMENT_S:
        return None, None
    a = (t - tc[i - 1]) / (tc[i] - tc[i - 1])
    q = (1 - a) * qc[i - 1] + a * qc[i]
    fen = (tc >= t - FENETRE_STATIQUE_S) & (tc <= t + FENETRE_STATIQUE_S)
    statique = fen.sum() >= 3 and np.all(np.ptp(qc[fen], axis=0) < IMMOBILE_DEG)
    return q, 'statique' if statique else 'dynamique'


def projette(obj, rvec, tvec, K, dist):
    return cv2.projectPoints(obj, rvec, tvec, K, dist)[0].reshape(-1, 2)


def tableau(lignes):
    err = [[] for _ in range(7)]
    vus = np.zeros(7, int)
    par_image = []
    for l in lignes:
        e = [float(np.hypot(*(l['kp'][k] - l['ref'][k]))) for k in range(7) if l['vu'][k]]
        for k in range(7):
            if l['vu'][k]:
                err[k].append(float(np.hypot(*(l['kp'][k] - l['ref'][k]))))
                vus[k] += 1
        if e:
            par_image.append(np.mean(e))
    n = len(lignes)
    out = []
    for k in range(7):
        e = np.array(err[k])
        out.append((NOMS[k], *(stats(e)), 100.0 * vus[k] / max(n, 1)))
    tout = np.concatenate([np.array(e) for e in err if e]) if any(err) else np.array([])
    out.append(('GLOBAL', *(stats(tout)), 100.0 * vus.sum() / max(7 * n, 1)))
    seuils = [100.0 * np.mean(tout < s) if tout.size else float('nan') for s in SEUILS_PX]
    return out, seuils, np.array(par_image), vus.sum(), 7 * n


def stats(e):
    if e.size == 0:
        return (float('nan'),) * 4
    return float(e.mean()), float(np.median(e)), float(e.std()), float(e.max())


def ecrit_tableau(f, titre, lignes):
    if not lignes:
        f.write(f'\n### {titre}\n\naucune image\n')
        return
    t, seuils, pim, nvus, ntot = tableau(lignes)
    f.write(f'\n### {titre} — {len(lignes)} images\n\n')
    f.write('| Keypoint | Moyenne (px) | Médiane (px) | Écart-type (px) | Max (px) | Détection |\n')
    f.write('|---|---|---|---|---|---|\n')
    for nom, m, med, sd, mx, det in t:
        g = '**' if nom == 'GLOBAL' else ''
        extra = f' ({nvus}/{ntot})' if nom == 'GLOBAL' else ''
        f.write(f'| {g}{nom}{g} | {m:.2f} | {med:.2f} | {sd:.2f} | {mx:.2f} | {det:.1f} %{extra} |\n')
    f.write('\n| ' + ' | '.join(f'< {s} px' for s in SEUILS_PX) + ' |\n')
    f.write('|' + '---|' * len(SEUILS_PX) + '\n')
    f.write('| ' + ' | '.join(f'{v:.1f} %' for v in seuils) + ' |\n')
    if pim.size:
        f.write(f'\nErreur moyenne par image : {pim.mean():.2f} ± {pim.std():.2f} px '
                f'(meilleure {pim.min():.2f}, pire {pim.max():.2f} px).\n')


def ecrit_detection(f, titre, lignes):
    f.write(f'\n### {titre} — {len(lignes)} images\n\n| Keypoint | Détection |\n|---|---|\n')
    n = max(len(lignes), 1)
    vus = np.array([l['vu'] for l in lignes]).reshape(-1, 7)
    for k in range(7):
        f.write(f'| {NOMS[k]} | {100.0 * vus[:, k].sum() / n:.1f} % |\n')
    f.write(f'| **GLOBAL** | {100.0 * vus.sum() / (7 * n):.1f} % ({vus.sum()}/{7 * len(lignes)}) |\n')
    f.write(f'\n7/7 sur {100.0 * np.mean(vus.sum(1) == 7):.1f} % des images, '
            f'≥ 5/7 sur {100.0 * np.mean(vus.sum(1) >= 5):.1f} %.\n')


def superpose(image, l):
    img = image.copy()
    for a in range(6):
        cv2.line(img, tuple(l['ref'][a].astype(int)), tuple(l['ref'][a + 1].astype(int)), VERT, 2)
    for k in range(7):
        cv2.circle(img, tuple(l['ref'][k].astype(int)), 7, VERT, 2)
        if l['vu'][k]:
            cv2.line(img, tuple(l['ref'][k].astype(int)), tuple(l['kp'][k].astype(int)), BLANC, 1)
            cv2.circle(img, tuple(l['kp'][k].astype(int)), 5, ORANGE, -1)
    return img


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('session', type=Path)
    a.add_argument('--modele', default=MODELE)
    a.add_argument('--svpro-detection-seule', action='store_true')
    a.add_argument('--extr-arducam', default=EXTRINSEQUES['arducam'],
                   help='extrinseque valable AU MOMENT de l enregistrement (la camera a pu bouger depuis)')
    a.add_argument('--extr-svpro', default=EXTRINSEQUES['svpro'])
    a.add_argument('--suffixe', default='', help='ajoute aux fichiers de sortie, pour ne pas ecraser')
    args = a.parse_args()

    s = json.loads((args.session / 'session.json').read_text())
    codeurs = sorted(s['codeurs'], key=lambda e: e['t'])
    tc = np.array([e['t'] for e in codeurs])
    qc = np.array([e['q'] for e in codeurs], float)
    poids = RACINE / 'training' / 'dream' / 'checkpoints_dream' / args.modele / 'best_network.pth'
    net = dream.create_network_from_config_file(str(poids.with_suffix('.yaml')), str(poids))

    rapport = args.session / f'rapport_dream{args.suffixe}.md'
    with rapport.open('w') as f:
        f.write(f'# DREAM {args.modele} sur le banc réel — {args.session.name}\n\n'
                f'Tri réel des pièces (`tri_fluide`), images 640×480. Référence : FK des '
                f'codeurs projetée dans l\'image. Erreur L2 en pixels. link1 et link2 sont '
                f'confondus dans le schéma à 7 keypoints.\n')
        for cam, journal in s['cameras'].items():
            nom_extr = {'arducam': args.extr_arducam, 'svpro': args.extr_svpro}[cam]
            extr = yaml.safe_load((CALIB / nom_extr).read_text())
            K, dist = registre.load_intrinsics(extr['intrinsics_stem'])
            lignes = []
            for e in journal:
                q, regime = etat_codeurs(e['t'], tc, qc)
                if q is None:
                    continue
                image = cv2.imread(str(args.session / cam / e['image']))
                h, w = image.shape[:2]
                kp = np.array(net.keypoints_from_image(
                    Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))['detected_keypoints'], float)
                vu = [bool(np.all(np.isfinite(p)) and 0 <= p[0] < w and 0 <= p[1] < h) for p in kp]
                pos, _ = forward_kinematics(np.radians(q))
                obj = np.array([pos[n] for n in KEYPOINT_NAMES], float)
                lignes.append({'image': e['image'], 't': e['t'], 'q': q, 'regime': regime,
                               'kp': kp, 'vu': vu, 'obj': obj})
            if cam != 'arducam' and args.svpro_detection_seule:
                f.write(f'\n## {cam}\n\nSans extrinsèque : détection seule, aucune erreur en pixels.\n')
                for regime in ('statique', 'dynamique'):
                    ecrit_detection(f, regime.capitalize(), [l for l in lignes if l['regime'] == regime])
                continue
            T = np.array(extr['T_cam_world'], float)
            rvec, tvec = cv2.Rodrigues(T[:3, :3])[0], T[:3, 3]
            for l in lignes:
                l['ref'] = projette(l['obj'], rvec, tvec, K, dist)

            f.write(f'\n## {cam}\n\nRéférence : extrinsèque ArUco `{nom_extr}`, '
                    f'extérieure à DREAM.\n')
            for regime in ('statique', 'dynamique'):
                ecrit_tableau(f, regime.capitalize(), [l for l in lignes if l['regime'] == regime])

            with (args.session / f'keypoints_{cam}{args.suffixe}.csv').open('w', newline='') as fc:
                wr = csv.writer(fc)
                wr.writerow(['image', 't', 'regime', *[f'q{j}' for j in range(1, 7)],
                             *[f'{n}_{c}' for n in NOMS for c in ('u', 'v', 'u_ref', 'v_ref', 'err')]])
                for l in lignes:
                    cols = []
                    for k in range(7):
                        err = float(np.hypot(*(l['kp'][k] - l['ref'][k]))) if l['vu'][k] else ''
                        u, v = (l['kp'][k] if l['vu'][k] else ('', ''))
                        cols += [u, v, *l['ref'][k], err]
                    wr.writerow([l['image'], f"{l['t']:.3f}", l['regime'], *np.round(l['q'], 2), *cols])

            for regime in ('statique', 'dynamique'):
                sel = [l for l in lignes if l['regime'] == regime]
                if sel:
                    l = sel[len(sel) // 2]
                    image = cv2.imread(str(args.session / cam / l['image']))
                    cv2.imwrite(str(args.session / f'exemple_{cam}_{regime}{args.suffixe}.png'), superpose(image, l))
    print(rapport.read_text())


if __name__ == '__main__':
    main()
