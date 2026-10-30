#!/usr/bin/env python3
"""Images Gazebo etiquetees automatiquement pour affiner yolo26 (4 cameras DREAM).

Une seule session Gazebo : pour chaque scene, les 8 pieces sont redeplacees
(`set_pose_vector`), puis les 4 cameras sont capturees et etiquetees avec les
boites GT projetees, par le meme calcul que `yolo26_tri_eval.py`. Les cameras ne
bougent pas : ce sont celles du 50K DREAM.

OUTIL DE CONSTITUTION DE DONNEES : il lit la pose Gazebo des pieces pour ecrire
les etiquettes. Ces images servent a l'entrainement, jamais a l'execution.

Graines a partir de 1000 : les graines 1 a 10 sont la scene de TEST, jamais
vues a l'entrainement. Une image ou une piece est cachee a plus de moitie par
une piece plus proche est ecartee : son etiquette amodale mentirait.

    # terminal 1 (brider Gazebo : il charge tout le processeur au demarrage)
    taskset -c 16-23 ros2 launch mycobot_gateway tri_yolo.launch.py seed:=1000 headless:=true yolo:=false
    # terminal 2, une fois le bras en pose d'observation
    /usr/bin/python3 scripts/yolo26_dataset_gazebo.py --sortie training/yolo/gazebo_tri_v1 --scenes 150
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

RACINE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway'))

import tri_couleur as tc                                                # noqa: E402
import yolo26_tri_eval as ev                                            # noqa: E402
from mycobot_gateway.gazebo_ground_truth import read_pose_info          # noqa: E402
from mycobot_gateway.vision import tri_scene as ts                      # noqa: E402
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras   # noqa: E402

MONDE_SOURCE = RACINE / 'mycobot_description/worlds/real_table.sdf'
# Une piece est posee si Gazebo la rapporte a moins de 1 mm de la consigne.
TOLERANCE_M = 0.001
# Temps simule a laisser passer apres le deplacement avant de prendre une image.
RENDU_APRES_S = 0.3
# Monde EN PAUSE entre deux captures : Gazebo qui tourne a vide rend les 4
# cameras sans arret, et a pousse le processeur a 93 °C pendant une simple
# attente (29/09). On avance de 0,5 s simulee (pas de 1 ms) pour chaque scene.
PAS_PAR_SCENE = 500
ATTENTE_IMAGES_S = 15.0
TEMP_PAUSE, TEMP_REPRISE = 78, 65


def temperature():
    out = subprocess.run(['sensors'], capture_output=True, text=True).stdout
    m = re.search(r'Package id 0:\s*\+([\d.]+)', out)
    return float(m.group(1)) if m else 0.0


def garde_temperature():
    if temperature() < TEMP_PAUSE:
        return
    print(f'  pause : {temperature():.0f} °C', flush=True)
    while temperature() >= TEMP_REPRISE:
        time.sleep(5)


def deplace(monde, disposition, essais=3):
    requete = ' '.join(f'pose {{ name: "{nom}" position {{ x: {x:.6f} y: {y:.6f} z: 0 }} '
                       'orientation { w: 1 } }' for nom, (x, y) in disposition.items())
    # Mesure du 29/09 : apres 50 scenes, un appel est reste sans reponse en 4 s.
    for _ in range(essais):
        r = subprocess.run(['gz', 'service', '-s', f'/world/{monde}/set_pose_vector',
                            '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '10000', '--req', requete],
                           capture_output=True, text=True, timeout=20)
        if not r.returncode and re.search(r'data:\s*true', r.stdout):
            return
        print(f'  deplacement non confirme, nouvel essai : {(r.stdout + r.stderr).strip()}', flush=True)
    raise RuntimeError(f'Gazebo n a pas confirme le deplacement apres {essais} essais')


def commande_monde(monde, requete, essais=3):
    # Comme set_pose_vector : un appel sur ~60 scenes reste sans reponse (29/09).
    for _ in range(essais):
        r = subprocess.run(['gz', 'service', '-s', f'/world/{monde}/control',
                            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '10000', '--req', requete],
                           capture_output=True, text=True, timeout=20)
        if not r.returncode and re.search(r'data:\s*true', r.stdout):
            return
        print(f'  « {requete} » non confirme, nouvel essai : {(r.stdout + r.stderr).strip()}', flush=True)
    raise RuntimeError(f'Gazebo a refuse « {requete} » apres {essais} essais')


def pose_confirmee(monde, disposition):
    """(instant simule, {piece: xyz}) une fois toutes les pieces a leur place."""
    for _ in range(20):
        instant, poses = read_pose_info(monde, ev.PIECES, timeout=15)
        xyz = {n: T[:3, 3] for n, T in poses.items()}
        if set(xyz) == set(ev.PIECES) and all(
                np.hypot(*(xyz[n][:2] - disposition[n])) < TOLERANCE_M for n in ev.PIECES):
            return instant, xyz
    raise RuntimeError('les pieces ne sont pas a leur place apres deplacement')


def etiquettes(refs, camera):
    """Lignes YOLO `classe cx cy w h`, normalisees, ordre des classes de pieces_v5."""
    lignes = []
    for nom, r in sorted(refs.items()):
        x0, y0, x1, y1 = r['boite']
        lignes.append(f'{tc.CLASSES_PIECES.index(nom)} {(x0 + x1) / 2 / camera.width:.6f} '
                      f'{(y0 + y1) / 2 / camera.height:.6f} {(x1 - x0) / camera.width:.6f} '
                      f'{(y1 - y0) / camera.height:.6f}')
    return lignes


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--sortie', type=Path, required=True)
    ap.add_argument('--scenes', type=int, default=150)
    ap.add_argument('--graine-depart', type=int, default=1000)
    ap.add_argument('--monde', default='tri_yolo')
    ap.add_argument('--reprise', action='store_true',
                    help='continuer dans un dossier existant (aucun fichier existant n est reecrit)')
    args = ap.parse_args()
    if args.graine_depart <= 10:
        ap.error('les graines 1 a 10 sont la scene de test')
    if args.sortie.exists() and not args.reprise:
        ap.error(f'{args.sortie} existe deja : on n ecrase jamais un jeu de donnees (--reprise)')
    (args.sortie / 'images').mkdir(parents=True, exist_ok=args.reprise)
    (args.sortie / 'labels').mkdir(exist_ok=args.reprise)

    cameras = load_cameras(ev.URDF, {'camera_layout': 'dream50k'})
    empreintes = ts.load_footprints(ev.MODELES)
    planche = ts.load_board(MONDE_SOURCE, ev.MODELES)
    rclpy.init()
    noeud = rclpy.create_node('yolo26_dataset_gazebo')
    pont = CvBridge()
    dernieres = {}
    for nom in ev.CAMERAS:
        noeud.create_subscription(Image, f'/{nom}/image',
                                  lambda m, n=nom: dernieres.__setitem__(n, m), qos_profile_sensor_data)

    gardees = ecartees = 0
    commande_monde(args.monde, 'pause: true')
    try:
        for k in range(args.scenes):
            graine = args.graine_depart + k
            garde_temperature()
            disposition = ts.sample_tri_scene(np.random.default_rng(graine), empreintes, planche,
                                              cameras['synth_camera_top'])
            deplace(args.monde, disposition)
            commande_monde(args.monde, 'pause: true multi_step: 5')
            instant, poses = pose_confirmee(args.monde, disposition)
            commande_monde(args.monde, f'pause: true multi_step: {PAS_PAR_SCENE}')
            fin = time.time() + ATTENTE_IMAGES_S
            relances = 0
            images = {}
            while len(images) < len(ev.CAMERAS):
                if time.time() > fin:
                    manquantes = sorted(set(ev.CAMERAS) - set(images))
                    # 4 coeurs : une camera peut ne pas avoir publie dans les 0,5 s simulees.
                    if relances == 3:
                        raise RuntimeError(f'cameras muettes : {manquantes}')
                    relances += 1
                    print(f'  {manquantes} sans image, on avance encore', flush=True)
                    commande_monde(args.monde, f'pause: true multi_step: {PAS_PAR_SCENE}')
                    fin = time.time() + ATTENTE_IMAGES_S
                rclpy.spin_once(noeud, timeout_sec=0.2)
                for nom, m in dernieres.items():
                    if m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 > instant + RENDU_APRES_S:
                        images.setdefault(nom, m)
            for nom, m in images.items():
                refs = ev.references(cameras[nom], poses, empreintes)
                if any(r['cachee'] for r in refs.values()):
                    ecartees += 1
                    continue
                base = f'{nom}_{graine}'
                if (args.sortie / 'images' / f'{base}.png').exists():
                    raise RuntimeError(f'{base} existe deja : graine deja generee')
                cv2.imwrite(str(args.sortie / 'images' / f'{base}.png'),
                            cv2.cvtColor(pont.imgmsg_to_cv2(m, 'rgb8'), cv2.COLOR_RGB2BGR))
                (args.sortie / 'labels' / f'{base}.txt').write_text(
                    '\n'.join(etiquettes(refs, cameras[nom])) + '\n')
                gardees += 1
            print(f'graine {graine} : {gardees} images gardees, {ecartees} ecartees '
                  f'({temperature():.0f} °C)', flush=True)
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
