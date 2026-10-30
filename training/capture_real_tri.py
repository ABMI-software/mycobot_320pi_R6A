#!/usr/bin/env python3
"""Images reelles etiquetees par les codeurs, scene de tri, pince montee, deux cameras.

    /usr/bin/python3 training/capture_real_tri.py --jeu essai --poses 5
    /usr/bin/python3 training/capture_real_tri.py --jeu train --poses 300 --graine 1
    /usr/bin/python3 training/capture_real_tri.py --jeu test  --poses 100 --graine 1001

Suite de `capture_real_3cam.py` (memes cameras, meme `labels.csv`, meme
`images/<camera>/NNNNNN.png`, index continu entre sessions d'un meme jeu), avec
trois differences :
  - arducam + SVPRO, sans Astra ;
  - les poses ne sont pas tirees sur 50 % des debattements : pince montee, le
    bras y balaierait les bacs et l'ecran, et ces poses-la ne sont pas celles
    du tri. La pointe est tiree au-dessus de la planche (`BOITE_MM`), outil
    oriente comme dans le tri (inclinaison minimale exigee par l'allonge,
    roulis tire), puis resolue par l'IK du FSM ;
  - les mouvements passent par `pick_fsm.va_vers_par_etapes` (gardes du tri),
    et un trajet dont la pointe passerait sous `Z_TRAJET_MIN_MM` (interpolation
    articulaire) remonte d'abord en pose d'observation. Le coude ne recule pas
    derriere la base (`RECUL_COUDE_MAX_MM`), et chaque pose garde
    `MARGE_COLLISION_MIN_MM` d'auto-collision au sens du FSM.

Labels 2D (`keypoints_2d.csv`) : FK des codeurs projetee par l'extrinseque de
chaque camera — `arducam_extrinsic_pick.yaml`, et pour la SVPRO
`svpro_extrinsic_labels.yaml`, qui ne sert qu'a ca. La pince n'est jamais
commandee.

Jeux : `train` et `test` sont deux dossiers distincts, a graines distinctes ;
`test` ne doit jamais entrer dans un melange d'entrainement.
"""
import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'training'))
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
import camera_registry as registre  # noqa: E402
import capture_real_3cam as c3  # noqa: E402
import pick_fsm as fsm  # noqa: E402
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402

SORTIE = RACINE / 'training' / 'dream' / 'dream_data' / 'real_tri'
CALIB = RACINE / 'training' / 'calibration'
EXTRINSEQUES = {'arducam': CALIB / 'arducam_extrinsic_pick.yaml',
                'svpro': CALIB / 'svpro_extrinsic_labels.yaml'}
BOITE_MM = {'x': (150.0, 400.0), 'y': (-200.0, 250.0), 'z': (90.0, 200.0)}
PORTEE_MM = (150.0, 400.0)
ROULIS_DEG = (-90.0, 90.0)
Z_TRAJET_MIN_MM = 70.0
# Sans cette borne, 9 poses sur 60 repliaient le bras coude DERRIERE la base
# (jusqu'a 117 mm, J2 +60, J3 -129) : posture jamais passee par le tri, vers une
# zone non reconnue (cables, potence).
RECUL_COUDE_MAX_MM = 40.0
# La garde du FSM refuse sous 0 mm (au-dela de ses 5 mm de securite), et teste
# aussi le DEPART. Une pose tiree a 0,5 mm de marge a ete atteinte a -0,03 mm
# (affaissement, 13/10, pose 538, J3 -139,6) : plus aucun mouvement n'etait
# accepte, retour en observation compris. D'ou une marge propre a la capture.
MARGE_COLLISION_MIN_MM = 10.0
REPOS_S = 1.0
# --autour-observation : real_tri/train n'a que J5 46-92 deg ; en pose d'observation
# (J5 ~ 2) et pendant le transit, la pointe DREAM part a 240-390 mm (14/10). Poses
# tirees entre l'observation et une pose de travail, J5 tire sur toute cette plage.
J5_OBSERVATION_DEG = (0.0, 90.0)
J6_ECART_DEG = 45.0
# Hors de la planche (pointe derriere ou a cote de la base : cables, potence, ecran),
# la pointe reste haute.
Z_HORS_PLANCHE_MIN_MM = 250.0
# L'extrinseque arducam est faite sur cam_3 (exposition 75), pas cam_0.
c3.CALIBRATION_MAP['arducam'] = 'cam_3'


def pose_tiree(rng):
    while True:
        xy = np.array([rng.uniform(*BOITE_MM['x']), rng.uniform(*BOITE_MM['y'])])
        portee = float(np.hypot(*xy))
        if not PORTEE_MM[0] <= portee <= PORTEE_MM[1]:
            continue
        z = rng.uniform(*BOITE_MM['z'])
        azimut = float(np.degrees(np.arctan2(xy[1], xy[0])))
        depart = next(t for limite, t in fsm.INCLINAISONS_PAR_PORTEE if portee <= limite)
        theta = float(rng.choice([t for t in fsm.INCLINAISONS if t <= depart]))
        roulis = rng.uniform(*ROULIS_DEG)
        R = fsm.incline(fsm.orientation(xy, roulis), azimut, theta)
        sol = fsm.resout_ik(np.array([xy[0], xy[1], z]), R)
        if (sol is None or fsm.pointe(sol[0])[2] < BOITE_MM['z'][0] - 5.0
                or fsm.marge_collision(sol[0])[0] < MARGE_COLLISION_MIN_MM):
            continue
        pos, _ = forward_kinematics(np.radians(sol[0]))
        coude = np.asarray(pos[KEYPOINT_NAMES[3]])[:2] * 1000.0
        if coude @ (xy / portee) >= -RECUL_COUDE_MAX_MM:
            return np.asarray(sol[0], float)


def pose_autour_observation(rng):
    while True:
        travail = pose_tiree(rng)
        q = fsm.POSE_OBSERVATION + rng.uniform(0.0, 1.0) * (travail - fsm.POSE_OBSERVATION)
        q[4] = rng.uniform(*J5_OBSERVATION_DEG)
        q[5] = fsm.POSE_OBSERVATION[5] + rng.uniform(-J6_ECART_DEG, J6_ECART_DEG)
        if (np.all((q >= fsm.LIMITES[:, 0]) & (q <= fsm.LIMITES[:, 1]))
                and fsm.pointe(q)[2] >= BOITE_MM['z'][0]
                and (BOITE_MM['x'][0] <= fsm.pointe(q)[0] <= BOITE_MM['x'][1]
                     and BOITE_MM['y'][0] <= fsm.pointe(q)[1] <= BOITE_MM['y'][1]
                     or fsm.pointe(q)[2] >= Z_HORS_PLANCHE_MIN_MM)
                and fsm.marge_collision(q)[0] >= MARGE_COLLISION_MIN_MM):
            pos, _ = forward_kinematics(np.radians(q))
            coude = np.asarray(pos[KEYPOINT_NAMES[3]])[:2] * 1000.0
            pointe = fsm.pointe(q)[:2]
            if coude @ (pointe / max(np.linalg.norm(pointe), 1e-6)) >= -RECUL_COUDE_MAX_MM:
                return q


def trajet_haut(qa, qb):
    return all(fsm.pointe(qa + s * (qb - qa))[2] >= Z_TRAJET_MIN_MM for s in np.linspace(0, 1, 21))


def projections():
    out = {}
    for nom, chemin in EXTRINSEQUES.items():
        d = yaml.safe_load(chemin.read_text())
        T = np.array(d['T_cam_world'], float)
        K, dist = registre.load_intrinsics(d['intrinsics_stem'])
        out[nom] = (cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, dist)
    return out


def projette(proj, q_deg):
    rvec, tvec, K, dist = proj
    pos, _ = forward_kinematics(np.radians(q_deg))
    obj = np.array([pos[n] for n in KEYPOINT_NAMES], float)
    return cv2.projectPoints(obj, rvec, tvec, K, dist)[0].reshape(-1, 2)


def index_v4l2(motif):
    return next(s.v4l2_index for s in registre.detect_cameras([motif], probe_capture=False))


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--jeu', choices=('essai', 'train', 'test'), required=True)
    a.add_argument('--poses', type=int, required=True)
    a.add_argument('--graine', type=int, default=0)
    a.add_argument('--plan', action='store_true', help='tirer et resoudre les poses, sans bouger')
    a.add_argument('--autour-observation', action='store_true',
                   help='poses entre l observation et le travail, J5 0-90 deg (voir J5_OBSERVATION_DEG)')
    args = a.parse_args()
    tire = pose_autour_observation if args.autour_observation else pose_tiree

    rng = np.random.default_rng(args.graine)
    if args.plan:
        t0 = time.time()
        qs = [tire(rng) for _ in range(args.poses)]
        p = np.array([fsm.pointe(q) for q in qs])
        print(f'{len(qs)} poses en {time.time() - t0:.0f} s ; pointe X {p[:, 0].min():.0f}-{p[:, 0].max():.0f}'
              f'  Y {p[:, 1].min():.0f}-{p[:, 1].max():.0f}  Z {p[:, 2].min():.0f}-{p[:, 2].max():.0f} mm')
        print('J1-J6 min', np.round(np.min(qs, axis=0)).tolist(), ' max', np.round(np.max(qs, axis=0)).tolist())
        return

    sortie = SORTIE / (f"essai_{datetime.now():%Y-%m-%d_%H-%M-%S}" if args.jeu == 'essai' else args.jeu)
    labels, kp2d, manifeste = sortie / 'labels.csv', sortie / 'keypoints_2d.csv', sortie / 'sessions.json'
    sortie.mkdir(parents=True, exist_ok=True)
    debut = 0
    if labels.exists():
        deja = list(csv.DictReader(labels.open()))
        debut = int(deja[-1]['index']) + 1 if deja else 0
    else:
        with labels.open('w', newline='') as f:
            csv.writer(f).writerow(['index', *[f'j{j}_rad' for j in range(1, 7)],
                                    *[f'j{j}_deg' for j in range(1, 7)], 'camera', 'image_path'])
        with kp2d.open('w', newline='') as f:
            csv.writer(f).writerow(['index', 'camera', *[f'{k}_{c}' for k in KEYPOINT_NAMES
                                                         for c in ('u', 'v')]])
    sessions = json.loads(manifeste.read_text()) if manifeste.exists() else []
    if any(s['graine'] == args.graine for s in sessions):
        raise SystemExit(f'graine {args.graine} deja utilisee dans {sortie.name} : poses repetees')

    cameras = {
        'arducam': c3.LocalCamera(index_v4l2('arducam'), 'arducam', exposure=75),
        'svpro': c3.LocalCamera(index_v4l2('svpro'), 'svpro', fourcc='MJPG'),
    }
    proj = projections()
    ctx = fsm.Contexte()
    ctx.pont = fsm.Pont()
    t0 = time.time()
    faites = 0
    try:
        for i in range(debut, debut + args.poses):
            cible = tire(rng)
            q = ctx.pont.angles()
            if not trajet_haut(q, cible):
                fsm.va_vers_par_etapes(ctx, fsm.POSE_OBSERVATION, nom='degagement', stabilise=False)
            if fsm.va_vers_par_etapes(ctx, cible, nom=f'pose {i}', stabilise=True) is None:
                print(f'[{i}] pose refusee par les gardes, sautee')
                continue
            time.sleep(REPOS_S)
            q = ctx.pont.angles()
            images = {nom: cam.capture() for nom, cam in cameras.items()}
            with labels.open('a', newline='') as fl, kp2d.open('a', newline='') as fk:
                for nom, image in images.items():
                    if image is None:
                        print(f'[{i}] {nom} : pas d image')
                        continue
                    if not (sortie / 'images' / nom / '_camera_settings.json').exists():
                        c3.write_camera_settings(nom, str(sortie), image.shape[1], image.shape[0])
                    rel = f'images/{nom}/{i:06d}.png'
                    c3.save_image(image, str(sortie / rel))
                    csv.writer(fl).writerow([i, *[round(math.radians(v), 4) for v in q],
                                             *np.round(q, 2), nom, rel])
                    csv.writer(fk).writerow([i, nom, *np.round(projette(proj[nom], q).ravel(), 2)])
            faites += 1
            print(f'[{i}] q = {np.round(q, 1).tolist()}  pointe Z {fsm.pointe(q)[2]:.0f} mm', flush=True)
        fsm.va_vers_par_etapes(ctx, fsm.POSE_OBSERVATION, nom='observation', stabilise=False)
    finally:
        for cam in cameras.values():
            cam.close()
        sessions.append({'date': datetime.now().isoformat(timespec='seconds'), 'debut': debut,
                         'poses': faites, 'graine': args.graine, 'cameras': list(cameras),
                         'tirage': 'autour_observation' if args.autour_observation else 'travail',
                         'duree_s': round(time.time() - t0)})
        manifeste.write_text(json.dumps(sessions, indent=1))
        for ligne in ctx.journal:
            print('  ', ligne)
        print(f'{faites} poses dans {sortie}')


if __name__ == '__main__':
    main()
