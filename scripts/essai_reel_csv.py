#!/usr/bin/env python3
"""Un tri reel du dashboard -> les memes CSV qu'un essai Gazebo (07/10).

    # pendant le tri : temperature + version du code, a lancer AVANT le dashboard
    .venv/bin/python scripts/essai_reel_csv.py --journal results/dream_reel/journal_<date>
    # apres le tri (dashboard ferme)
    .venv/bin/python scripts/essai_reel_csv.py results/dream_reel/<session> --journal results/dream_reel/journal_<date> --essai 1

Sortie `results/dream_reel/<date>_tri_essai<k>_dream_yolo_reel/` :

  essai.csv        par piece : YOLO objet (saisie), DREAM pince 3D (au serrage et mediane
                   sur le trajet objet -> bac) ; par bac : YOLO bac (depot)
  trajectoire.csv  chaque image enregistree pendant un cycle, par camera : YOLO sur la
                   cible (la piece jusqu'au serrage, puis son bac), pointe DREAM
  yolo.csv         detection et erreur par element, bras immobile / en mouvement
  dream.csv        detection et erreur 2D par keypoint, bras immobile / en mouvement
  temperature.csv  CPU chaque seconde (pas de gel sur le reel : la colonne reste « non »)
  frequences.csv   frequence de chaque mesure pendant l'essai (images, DREAM, codeurs)
  run.yaml         date, commit git, fichiers modifies, extrinseques, modeles

Ce qui differe de Gazebo, et pourquoi : il n'y a pas de verite terrain. La
reference est la pince lue aux CODEURS (FK) : au serrage pour une piece, a la
pose pour un bac. Les colonnes `gazebo_*` deviennent `codeurs_*`. Les
extrinseques sont les ArUco du dashboard : arducam_extrinsic_pick (YOLO et
DREAM), svpro_extrinsic_servo (YOLO de cote) et svpro_extrinsic_labels (DREAM).

`temps_s` part de 0 au debut du premier cycle (clic sur « demarrer »), pas du
lancement du dashboard. Duree d'un tri : du depart vers la piece a la pose dans
le bac, comme `temps_du_tri_s` en Gazebo (lignes `cible` = piece puis = bac).

YOLO image par image : erreur sur les objets vus par l'arducam seulement (voir
CAMERAS_ERREUR) ; bacs et SVPRO en detection et confiance. Le dashboard n'enregistre que la position retenue pour la
saisie ; ici yolo26 (meme poids, meme seuil que `yolo26_service.py`) repasse
sur les images enregistrees, centre de la boite ramene a la hauteur de la piece
(HAUTEUR_OBJET) ou du bac (HAUTEUR_CARTON) par `Vision.vers_base`. Le dashboard
affine ce centre par le contour couleur : l'ecart image par image est donc celui
de la boite brute, pas exactement celui de la saisie (qui est dans essai.csv).

DREAM 2D : `analyse_tri_dream.py` (venv_dream) est lance s'il n'a pas encore
produit `keypoints_<camera>.csv`.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
import camera_registry as registre  # noqa: E402
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402
import pick_dashboard as pd  # noqa: E402
import pick_fsm as fsm  # noqa: E402
import tri_couleur as tc  # noqa: E402
import yolo26_service  # noqa: E402

VENV_DREAM = Path.home() / 'ros_jazzy' / 'venv_dream' / 'bin' / 'python'
CALIB = RACINE / 'training' / 'calibration'
EXTR_YOLO = {'arducam': 'arducam_extrinsic_pick', 'svpro': 'svpro_extrinsic_labels'}
EXTR_DREAM = {'arducam': 'arducam_extrinsic_pick.yaml', 'svpro': 'svpro_extrinsic_labels.yaml'}
# --sans-marqueurs : session de lancer_tri_extrinseque_dream.py, ou aucune extrinseque ArUco ne sert.
EXTR_SANS_MARQUEURS = {'arducam': 'arducam_extrinsic_dream_tri', 'svpro': 'svpro_extrinsic_dream_tri'}
NOMS_KP = ['base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6']
APPARIEMENT_S = 0.1
ECART_CODEURS_MAX_S = 0.3
FENETRE_STATIQUE_S = 0.4
IMMOBILE_DEG = 0.3
# Comme dream_tri_panneau : hors de ce domaine (pose d'observation) la pointe DREAM est fausse.
J5_DOMAINE_DEG = (45.0, 93.0)
# Erreur YOLO image par image : objets, arducam seule. Mesure du 14/10 16:17 (centre de
# boite contre codeurs) : arducam objets 9,7 mm (= la saisie) ; SVPRO objets 16 mm, bacs
# 87 mm, vue oblique ; arducam bacs 29 mm, alors que le dashboard, qui suit l'ouverture
# du carton, est a 0,3-2,6 mm au depot. Bacs et SVPRO : detection et confiance seules ;
# l'erreur d'un bac est celle du dashboard au depot (essai.csv).
CAMERAS_ERREUR = ('arducam',)


def temperature():
    out = subprocess.run(['sensors'], capture_output=True, text=True).stdout
    for l in out.splitlines():
        if l.startswith('Package id 0:'):
            return int(float(l.split('+')[1].split('°')[0]))
    return None


def journal(dossier):
    """Tourne jusqu'a Ctrl+C : run.yaml une fois, puis une ligne de temperature par seconde."""
    dossier.mkdir(parents=True, exist_ok=True)
    git = lambda *a: subprocess.run(['git', *a], cwd=RACINE, capture_output=True, text=True).stdout.strip()
    (dossier / 'run.yaml').write_text(yaml.safe_dump({
        'date': datetime.now().isoformat(timespec='seconds'),
        'commit': git('rev-parse', 'HEAD'),
        'branche': git('branch', '--show-current'),
        'modified_files': len([l for l in git('status', '--porcelain', '--untracked-files=no').splitlines() if l]),
        'extrinseques_yolo': {c: f'{s}.yaml' for c, s in EXTR_YOLO.items()},
        'extrinseques_dream': EXTR_DREAM,
        'modele_dream': 'vgg_tri_reel_ft_e10',
        'poids_yolo': str(yolo26_service.POIDS_DEFAUT.relative_to(RACINE)),
        'seuil_yolo': tc.SEUIL_MODELE_ENTRAINE}, allow_unicode=True, sort_keys=False))
    with (dossier / 'temperature_brute.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['heure', 't_monotone', 'temperature_c'])
        print(f'journal : {dossier} (Ctrl+C pour arreter)', flush=True)
        try:
            while True:
                w.writerow([datetime.now().strftime('%H:%M:%S'), round(time.monotonic(), 3), temperature()])
                f.flush()
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass


def ecrit(chemin, entete, lignes):
    with chemin.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(entete)
        w.writerows(lignes)


def r(x, n=1):
    return '' if x is None else round(float(x), n)


class Codeurs:
    def __init__(self, s):
        e = sorted(s['codeurs'], key=lambda e: e['t'])
        self.t = np.array([x['t'] for x in e])
        self.q = np.array([x['q'] for x in e], float)

    def q_a(self, t):
        i = int(np.searchsorted(self.t, t))
        if i == 0 or i == len(self.t) or self.t[i] - t > ECART_CODEURS_MAX_S or t - self.t[i - 1] > ECART_CODEURS_MAX_S:
            return None
        a = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return (1 - a) * self.q[i - 1] + a * self.q[i]

    def regime(self, t):
        fen = (self.t >= t - FENETRE_STATIQUE_S) & (self.t <= t + FENETRE_STATIQUE_S)
        return 'statique' if fen.sum() >= 3 and np.all(np.ptp(self.q[fen], axis=0) < IMMOBILE_DEG) else 'dynamique'


BAC_DE = {piece: f'bac_{teinte}' for teinte, piece in tc.NOM_PIECE.items()}


def bac_de(piece):
    return BAC_DE[piece]


def dream_tri_panneau_fusionne(pts, t0, t1, fenetre=1.0, accord=30.0):
    """Meme combinaison que `dream_tri_panneau.fusionne` (le panneau importe PyQt, pas ce venv).
    accord=None : l'ancienne fusion (mediane seule), gardee en colonne pour comparer."""
    derniere, out = {}, []
    for t, cam, p in pts:
        derniere[cam] = (t, p)
        if t0 <= t <= t1:
            vues = {c: q for c, (s, q) in derniere.items() if t - s <= fenetre}
            if accord is not None and len(vues) == 2 and np.linalg.norm(vues['arducam'] - vues['svpro']) > accord:
                out.append((t, vues['arducam']))
            else:
                out.append((t, np.median(list(vues.values()), axis=0)))
    return out


def yolo_z(c, rp):
    """Z de YOLO triangule arducam + SVPRO (lancer_tri_yolo_2cam), ou rien : avec --svpro-dream YOLO est en XY."""
    z = c.get('yolo_z')
    if z is None:
        return ['', '', '']
    e = np.array([*c['yolo'], z]) - rp
    return [r(z), f'{e[2]:+.1f}', r(np.linalg.norm(e), 2)]


APPARIEMENT_DIRECT_S = 0.2
# Repli pour une session d'avant l'enregistrement en direct : YOLO et DREAM refaits sur
# les images, au calme (15/10 : 96-100 degres sinon).
COEURS_REPLI = {0, 1, 2, 3}


def au_calme():
    os.nice(19)
    os.sched_setaffinity(0, COEURS_REPLI)


def plus_proche(ts, t):
    i = int(np.argmin(np.abs(ts - t))) if len(ts) else -1
    return i if i >= 0 and abs(ts[i] - t) <= APPARIEMENT_DIRECT_S else None


def yolo_direct(s, vision, hauteur):
    """(cam, image) -> {classe: (uv, xy, conf)}, depuis les detections du dashboard."""
    par_cam = {}
    for d in s['yolo_direct']:
        par_cam.setdefault(d['camera'], []).append(d)
    out = {}
    for cam, journal_cam in s['cameras'].items():
        dets = par_cam.get(cam, [])
        ts = np.array([d['t'] for d in dets])
        for e in journal_cam:
            i = plus_proche(ts, e['t'])
            det = {}
            for d in ([] if i is None else sorted(dets[i]['det'], key=lambda d: -d['conf'])):
                nom, (x0, y0, x1, y1) = d['classe'], d['boite']
                uv = ((x0 + x1) / 2, (y0 + y1) / 2)
                xy = vision[cam].vers_base(uv, hauteur(nom))[:2]
                if not nom.startswith('bac') and np.hypot(*xy) > fsm.PORTEE_MAX:
                    continue
                det.setdefault(nom, (uv, xy, d['conf']))
            out[(cam, e['image'])] = det
    return out


def keypoints_direct(s, cod):
    """{camera: [ligne au format keypoints_<camera>.csv]} depuis les keypoints du dashboard :
    reference = FK des codeurs projetee par l'extrinseque de DREAM, comme analyse_tri_dream."""
    out = {}
    for cam, nom in EXTR_DREAM.items():
        e = yaml.safe_load((CALIB / nom).read_text())
        K, dist = registre.load_intrinsics(e['intrinsics_stem'])
        T = np.array(e['T_cam_world'], float)
        rvec, tvec = cv2.Rodrigues(T[:3, :3])[0], T[:3, 3]
        images = s['cameras'].get(cam, [])
        ti = np.array([x['t'] for x in images])
        lignes = []
        for x in s['keypoints_direct']:
            if x['camera'] != cam:
                continue
            q = cod.q_a(x['t'])
            if q is None:
                continue
            pos, _ = forward_kinematics(np.radians(q))
            ref = cv2.projectPoints(np.array([pos[n] for n in KEYPOINT_NAMES], float),
                                    rvec, tvec, K, dist)[0].reshape(-1, 2)
            i = plus_proche(ti, x['t'])
            l = {'image': '' if i is None else images[i]['image'], 't': x['t'], 'regime': cod.regime(x['t'])}
            for k, n in enumerate(NOMS_KP):
                p = x['kp'][k]
                l[f'{n}_u'] = '' if p is None else p[0]
                l[f'{n}_err'] = '' if p is None else float(np.hypot(*(np.array(p) - ref[k])))
            lignes.append(l)
        out[cam] = lignes
    return out


def convertit(session, dossier_journal, k):
    s = json.loads((session / 'session.json').read_text())
    cod = Codeurs(s)
    cycles = [c for c in s['cycles'] if c['serre'] is not None]
    if not cycles:
        raise SystemExit('aucune piece serree dans cette session')
    t_debut = cycles[0]['t0']
    sortie = RACINE / 'results' / 'dream_reel' / f'{session.name[:10]}_tri_essai{k}_dream_yolo_reel'
    if sortie.exists():
        raise SystemExit(f'{sortie.name} existe deja : pas d ecrasement')

    direct = 'yolo_direct' in s and 'keypoints_direct' in s
    if direct:
        kps_cam = keypoints_direct(s, cod)
    else:
        au_calme()
        if not all((session / f'keypoints_{cam}.csv').exists() for cam in s['cameras']):
            print('keypoints DREAM absents : analyse_tri_dream.py (au calme)', flush=True)
            subprocess.run([str(VENV_DREAM), str(RACINE / 'scripts' / 'analyse_tri_dream.py'), str(session)],
                           check=True)
        kps_cam = {cam: list(csv.DictReader((session / f'keypoints_{cam}.csv').open())) for cam in s['cameras']}
    fichiers = {}
    j5 = J5_DOMAINE_DEG

    def dans_domaine(x):
        if 'dans_domaine' in x:
            return x['dans_domaine']
        q = cod.q_a(x['t'])
        return q is not None and j5[0] <= q[4] <= j5[1]

    for x in s['pointes_dream']:
        x['dans_domaine'] = dans_domaine(x)

    # Reference de chaque element : la pince aux codeurs, au serrage (piece) ou a la pose (bac).
    ref, fenetre_ref = {}, {}
    for c in cycles:
        bac = bac_de(c['piece'])
        ref[c['piece']] = np.array(c['ref_piece'], float)
        fenetre_ref[c['piece']] = (t_debut, c['serre'])     # avant « demarrer », les pieces ont pu etre placees a la main
        if c['ref_bac'] is not None:
            ref[bac] = np.array(c['ref_bac'], float)
            fenetre_ref[bac] = (t_debut, np.inf)

    vision = {cam: pd.Vision(EXTR_YOLO[cam]) for cam in s['cameras']}
    hauteur = lambda nom: pd.HAUTEUR_CARTON if nom.startswith('bac') else pd.HAUTEUR_OBJET
    yolo = yolo_direct(s, vision, hauteur) if direct else {}
    if not direct:
        from ultralytics import YOLO
        modele = YOLO(str(yolo26_service.POIDS_DEFAUT))
    for cam, journal_cam in ([] if direct else s['cameras'].items()):
        print(f'YOLO {cam} : {len(journal_cam)} images', flush=True)
        for e in journal_cam:
            image = cv2.imread(str(session / cam / e['image']))
            det = {}
            for nom, (x0, y0, x1, y1), conf in yolo26_service.detecte(modele, image, tc.SEUIL_MODELE_ENTRAINE):
                uv = ((x0 + x1) / 2, (y0 + y1) / 2)
                xy = vision[cam].vers_base(uv, hauteur(nom))[:2]
                # Meme regle que lancer_pick_dashboard_final : une piece hors de portee est
                # une fausse detection (cylindre_vert fantome en (10 ; -845) le 14/10).
                if not nom.startswith('bac') and np.hypot(*xy) > fsm.PORTEE_MAX:
                    continue
                if nom not in det:
                    det[nom] = (uv, xy, conf)
            yolo[(cam, e['image'])] = det

    # ---- essai.csv
    lignes = []
    for c in cycles:
        p, rp = c['piece'], ref[c['piece']]
        av = [x for x in s['pointes_dream'] if x['dans_domaine'] and c['serre'] - 2.0 <= x['t'] <= c['serre']]
        dream = np.median([x['pointe_mm'] for x in av], axis=0) if av else None
        confs = [yolo[(cam, e['image'])][p][2] for cam, j in s['cameras'].items() for e in j
                 if c['t0'] - 3.0 <= e['t'] <= c['t0'] and p in yolo[(cam, e['image'])]]
        fin = c['pose'] or c['fin']
        # Comme la courbe DREAM de Gazebo : pointe combinee (derniere vue de chaque camera,
        # moins de 1 s) contre la pointe des codeurs, a chaque instant du trajet.
        vues = sorted((x['t'], x['camera'], np.array(x['pointe_mm'])) for x in s['pointes_dream']
                      if x['dans_domaine'])
        trajet, trajet_mediane = [], []
        for accord, acc in ((30.0, trajet), (None, trajet_mediane)):
            for t, pc in dream_tri_panneau_fusionne(vues, c['t0'], fin, accord=accord):
                q = cod.q_a(t)
                if q is not None:
                    acc.append(np.linalg.norm(pc - fsm.pointe(q)))
        dy = np.asarray(c['yolo'], float) - rp[:2]
        dd = None if dream is None else dream - rp
        lignes.append([p, 'objet', bac_de(p), 'Oui' if c['verdict'] == 'OK' else 'Non',
                       r(c['pose'] - c['t0']) if c['pose'] else '',
                       *[r(v) for v in rp], *[r(v) for v in c['yolo']], r(np.mean(confs), 3) if confs else '',
                       *[r(v, 2) for v in dy], r(np.linalg.norm(dy), 2),
                       *(('',) * 3 if dream is None else [r(v) for v in dream]),
                       *(('',) * 3 if dd is None else [f'{v:+.1f}' for v in dd]),
                       r(np.median(trajet)) if trajet else '',
                       *yolo_z(c, rp),
                       r(np.median(trajet_mediane)) if trajet_mediane else ''])
    for c in cycles:
        b = bac_de(c['piece'])
        if b not in ref:
            continue
        rb = ref[b]
        db = np.asarray(c['yolo_bac'], float) - rb[:2]
        lignes.append([b, 'bac', '', '', '', *[r(v) for v in rb], *[r(v) for v in c['yolo_bac']], '',
                       *[r(v, 2) for v in db], r(np.linalg.norm(db), 2), *[''] * 11])
    fichiers['essai.csv'] = (
          ['element', 'type', 'bac_cible', 'tri_reussi', 'temps_du_tri_s',
           'codeurs_x_mm', 'codeurs_y_mm', 'codeurs_z_mm', 'yolo_x_mm', 'yolo_y_mm', 'yolo_confiance',
           'erreur_yolo_codeurs_x_mm', 'erreur_yolo_codeurs_y_mm', 'erreur_yolo_codeurs_xy_mm',
           'dream_x_mm', 'dream_y_mm', 'dream_z_mm',
           'erreur_dream_codeurs_x_mm', 'erreur_dream_codeurs_y_mm', 'erreur_dream_codeurs_z_mm',
           'erreur_dream_trajet_3d_mm', 'yolo_z_mm', 'erreur_yolo_codeurs_z_mm', 'erreur_yolo_codeurs_3d_mm',
           'erreur_dream_trajet_3d_ancienne_fusion_mm'],
          lignes)

    # ---- trajectoire.csv
    kp = {}
    for cam, lignes_kp in kps_cam.items():
        for l in lignes_kp:
            kp[(cam, l['image'])] = sum(l[f'{n}_u'] != '' for n in NOMS_KP)
    pointes = {}
    for x in s['pointes_dream']:
        if x['dans_domaine']:
            pointes.setdefault(x['camera'], []).append((x['t'], np.array(x['pointe_mm'])))
    pointes = {cam: (np.array([t for t, _ in v]), [p for _, p in v]) for cam, v in pointes.items()}
    lignes = []
    for c in cycles:
        fin = c['pose'] or c['fin']
        for cam, journal_cam in s['cameras'].items():
            for e in journal_cam:
                t = e['t']
                if not c['t0'] <= t <= fin:
                    continue
                cible = c['piece'] if t < c['serre'] else bac_de(c['piece'])
                d = yolo[(cam, e['image'])].get(cible)
                ey = ['', '', ''] if d is None else [r(d[2], 4), '', '']
                if d is not None and cam in CAMERAS_ERREUR and cible == c['piece']:
                    ey[1:] = [r(d[1][0] - ref[cible][0], 2), r(d[1][1] - ref[cible][1], 2)]
                ed = ['', '', '', '']
                if cam in pointes:
                    ts, ps = pointes[cam]
                    i = int(np.argmin(np.abs(ts - t)))
                    q = cod.q_a(ts[i])
                    if abs(ts[i] - t) <= APPARIEMENT_S and q is not None:
                        d3 = ps[i] - fsm.pointe(q)
                        ed = [*[r(v, 2) for v in d3], r(np.linalg.norm(d3), 2)]
                lignes.append([r(t - t_debut, 2), r(t - c['t0'], 2), cam, c['piece'], cible, *ey,
                               kp.get((cam, e['image']), ''), *ed])
    lignes.sort(key=lambda l: (l[0], l[2]))
    fichiers['trajectoire.csv'] = (
          ['temps_s', 'temps_objet_s', 'camera', 'objet', 'cible', 'yolo_confiance',
           'erreur_yolo_codeurs_x_mm', 'erreur_yolo_codeurs_y_mm', 'dream_points',
           'erreur_dream_codeurs_x_mm', 'erreur_dream_codeurs_y_mm', 'erreur_dream_codeurs_z_mm',
           'erreur_dream_3d_mm'], lignes)

    # ---- yolo.csv : chaque element tant qu'il est a sa place, toutes cameras
    stats = {}
    for cam, journal_cam in s['cameras'].items():
        for e in journal_cam:
            if cod.q_a(e['t']) is None:
                continue
            reg = cod.regime(e['t'])
            for nom, rr in ref.items():
                a, b = fenetre_ref[nom]
                if not a <= e['t'] <= b:
                    continue
                st = stats.setdefault((nom, reg), {'n': 0, 'vu': 0, 'px': [], 'mm': []})
                st['n'] += 1
                d = yolo[(cam, e['image'])].get(nom)
                if d is None:
                    continue
                st['vu'] += 1
                if cam not in CAMERAS_ERREUR or nom.startswith('bac'):
                    continue
                uv_ref = vision[cam].vers_pixel([rr[0], rr[1], hauteur(nom)])
                st['px'].append(float(np.hypot(*(np.array(d[0]) - uv_ref))))
                st['mm'].append(float(np.hypot(*(d[1] - rr[:2]))))

    def ligne_yolo(nom, noms):
        out = [nom]
        for reg in ('statique', 'dynamique'):
            n = sum(stats.get((x, reg), {'n': 0})['n'] for x in noms)
            vu = sum(stats.get((x, reg), {'vu': 0})['vu'] for x in noms)
            px = [v for x in noms for v in stats.get((x, reg), {'px': []})['px']]
            mm = [v for x in noms for v in stats.get((x, reg), {'mm': []})['mm']]
            out += [r(100 * vu / n) if n else '', r(np.median(px), 2) if px else '', r(np.median(mm), 2) if mm else '']
        return out

    objets = [c['piece'] for c in cycles]
    bacs = [bac_de(p) for p in objets if bac_de(p) in ref]
    fichiers['yolo.csv'] = (
          ['element', 'statique_detection_pct', 'statique_erreur_mediane_px', 'statique_erreur_mediane_xy_mm',
           'dynamique_detection_pct', 'dynamique_erreur_mediane_px', 'dynamique_erreur_mediane_xy_mm'],
          [ligne_yolo(n, [n]) for n in objets + bacs]
          + [ligne_yolo('objets', objets), ligne_yolo('bacs', bacs), ligne_yolo('global', objets + bacs)])

    # ---- dream.csv : deux cameras ensemble, comme les 4 cameras Gazebo
    kps = [l for lignes_kp in kps_cam.values() for l in lignes_kp]

    def ligne_dream(nom, noms):
        out = [nom]
        for reg in ('statique', 'dynamique'):
            ls = [l for l in kps if l['regime'] == reg]
            n = len(ls) * len(noms)
            vus = [l[f'{x}_err'] for l in ls for x in noms if l[f'{x}_u'] != '']
            errs = [float(v) for v in vus if v != '']
            out += [r(100 * len(vus) / n) if n else '', r(np.median(errs), 2) if errs else '']
        return out

    fichiers['dream.csv'] = (
          ['point', 'statique_detection_pct', 'statique_erreur_mediane_px',
           'dynamique_detection_pct', 'dynamique_erreur_mediane_px'],
          [ligne_dream(n, [n]) for n in NOMS_KP] + [ligne_dream('global', NOMS_KP)])

    # ---- temperature.csv + run.yaml
    t_fin = max(c['fin'] for c in cycles)
    lignes, run = [], None
    if dossier_journal and (dossier_journal / 'temperature_brute.csv').exists():
        brut = [l for l in csv.DictReader((dossier_journal / 'temperature_brute.csv').open())
                if t_debut <= float(l['t_monotone']) <= t_fin]
        for i, l in enumerate(brut):
            evenement = 'départ' if i == 0 else ''
            if i == len(brut) - 1:
                tmax = max(int(x['temperature_c']) for x in brut if x['temperature_c'])
                ok = sum(c['verdict'] == 'OK' for c in cycles)
                evenement = f'fin : {ok}/{len(cycles)} tries, Tmax {tmax} C, 0 gel'
            lignes.append([l['heure'], int(round(float(l['t_monotone']) - t_debut)), l['temperature_c'], 'non',
                           evenement])
        run = (dossier_journal / 'run.yaml').read_text() + f'session: {session.name}\nessai: {k}\n'
        if EXTR_YOLO['arducam'] == EXTR_SANS_MARQUEURS['arducam']:
            # Le journal ecrit run.yaml avant le tri et ne sait pas quel lanceur tourne.
            run += yaml.safe_dump({'sans_marqueurs': True, 'extrinseques_effectives':
                                   {c: f'{s}.yaml' for c, s in EXTR_SANS_MARQUEURS.items()}}, allow_unicode=True)
    else:
        print('pas de journal de temperature pour cette session : temperature.csv vide, run.yaml absent')
    fichiers['temperature.csv'] = (['heure', 'temps_reel_s', 'temperature_c', 'gel', 'evenement'], lignes)
    duree = max((c['pose'] or c['fin']) for c in cycles) - t_debut
    dans = lambda t: t_debut <= t <= t_debut + duree
    lignes = []
    for cam, journal_cam in s['cameras'].items():
        ti = [e['t'] for e in journal_cam if dans(e['t'])]
        td = sorted(x['t'] for x in s['pointes_dream'] if x['camera'] == cam and dans(x['t']))
        dom = [x for x in s['pointes_dream'] if x['camera'] == cam and dans(x['t']) and x['dans_domaine']]
        lignes += [[cam, 'images enregistrees = YOLO (hors ligne) + keypoints DREAM', r(len(ti) / duree, 2)],
                   [cam, 'pointe DREAM (>= 5 keypoints, codeurs encadrants)', r(len(td) / duree, 2)],
                   [cam, 'pointe DREAM dans le domaine J5', r(len(dom) / duree, 2)]]
    for cle, ts in s.get('mesures', {}).items():
        src, cam = cle.split('_', 1)
        lignes.append([cam, f'{src.upper()} dans le dashboard (en direct)', r(sum(map(dans, ts)) / duree, 2)])
    lignes.append(['robot', 'codeurs (get_angles)', r(sum(dans(t) for t in cod.t) / duree, 2)])
    fichiers['frequences.csv'] = (['source', 'mesure', 'frequence_hz'], lignes)
    sortie.mkdir(parents=True)
    for nom, (entete, contenu) in fichiers.items():
        ecrit(sortie / nom, entete, contenu)
    if run:
        (sortie / 'run.yaml').write_text(run)
    print(f'écrit : {sortie.relative_to(RACINE)}')


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('session', type=Path, nargs='?', help='dossier <date>_dashboard du panneau DREAM')
    a.add_argument('--journal', type=Path, help='dossier du journal temperature + version')
    a.add_argument('--essai', type=int, default=1)
    a.add_argument('--sans-marqueurs', action='store_true',
                   help='session de lancer_tri_extrinseque_dream.py : extrinseques DREAM, aucun ArUco')
    args = a.parse_args()
    if args.sans_marqueurs:
        EXTR_YOLO.update(EXTR_SANS_MARQUEURS)
        EXTR_DREAM.update({c: f'{s}.yaml' for c, s in EXTR_SANS_MARQUEURS.items()})
    if args.session is None:
        journal(args.journal)
    else:
        convertit(args.session, args.journal, args.essai)


if __name__ == '__main__':
    main()
