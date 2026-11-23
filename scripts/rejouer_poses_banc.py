#!/usr/bin/env python3
"""Rejoue les poses d'un jeu réel NDDS dans la réplique Gazebo du banc (E3.9b).

Les étiquettes simulées sont exactes : FK des angles lus dans Gazebo, projetée
par les intrinsèques simulées (sans distorsion) et l'extrinsèque même qui a
étiqueté le jeu réel. Un écart entre ces images et les images réelles ne peut
donc pas venir d'une erreur d'étiquette. Protocole : docs/ARTICLE_DREAM.md,
« Protocole de E3.9b ».

1. Monde aux caméras d'étiquetage (Python système), la réplique du banc par
   défaut, le monde d'entraînement avec --base (condition C) :
    /usr/bin/python3 scripts/rejouer_poses_banc.py monde \
        --arducam <arducam_extrinsic_pick.avant_1410_1454.yaml> \
        --svpro <svpro_extrinsic_labels.avant_1410.yaml> --sortie <dossier> \
        [--base mycobot_description/worlds/randomized.sdf]
2. Ce monde, une apparence par passe (la commande exacte est affichée par 1) :
    ros2 launch mycobot_gateway banc_realiste.launch.py headless:=true \
        world_file:=<dossier>/<monde>.sdf world_name:=<monde> robot_appearance:=original
3. Le rejeu (conda deactivate, ROS 2 sourcé) :
    /usr/bin/python3 scripts/rejouer_poses_banc.py rejouer --reel <real_tri_test_ndds> \
        --monde <dossier> --apparence original --sortie <dossier>/A

La sortie garde la numérotation du jeu réel (même caméra, même pose sous le
même numéro), ce qui apparie les bootstraps. Refuse un dossier de sortie non vide.
"""
import argparse
import csv
import hashlib
import json
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
import generer_banc_realiste as banc  # noqa: E402
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402

LARGEUR, HAUTEUR = 640, 480
ARTICULATIONS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
# Bras droit entre deux poses : le trajet d'une pose réelle à la suivante
# traverserait les bacs et les pièces, et la physique les déplacerait
# différemment d'une passe à l'autre.
POSE_REFUGE = [0.0] * 6
# Liens dont l'origine est le keypoint de la FK (base, puis l'enfant de chaque articulation).
LIENS = ['base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6']


def md5(chemin):
    return hashlib.md5(Path(chemin).read_bytes()).hexdigest()


def cmd_monde(args):
    args.sortie.mkdir(parents=True, exist_ok=True)
    arbre = ET.parse(args.base)
    monde = arbre.getroot().find('world')
    nom_monde = monde.get('name')
    # La configuration par défaut de Gazebo (server.config) ne charge PAS les
    # capteurs, et banc_realiste.launch.py lance l'URDF sans eux
    # (enable_cameras:=false) : sans ce bloc, aucune image. Un monde sans
    # <plugin> perd aussi la configuration par défaut dès qu'on en déclare un,
    # d'où les trois systèmes par défaut remis explicitement.
    if monde.find("plugin[@filename='gz-sim-sensors-system']") is None:
        if monde.find('plugin') is None:
            for fichier, nom in (('gz-sim-scene-broadcaster-system', 'SceneBroadcaster'),
                                 ('gz-sim-user-commands-system', 'UserCommands'),
                                 ('gz-sim-physics-system', 'Physics')):
                monde.insert(0, ET.Element('plugin', {'filename': fichier,
                                                      'name': f'gz::sim::systems::{nom}'}))
        capteurs = ET.Element('plugin', {'filename': 'gz-sim-sensors-system',
                                         'name': 'gz::sim::systems::Sensors'})
        ET.SubElement(capteurs, 'render_engine').text = args.moteur_rendu
        monde.insert(0, capteurs)
    cameras = {}
    for nom, fichier in (('arducam', args.arducam), ('svpro', args.svpro)):
        ancien = monde.find(f"model[@name='{nom}']")
        if ancien is not None:
            monde.remove(ancien)
        extrinseque = yaml.safe_load(fichier.read_text())
        centre, rpy = banc.pose_camera(extrinseque)
        k = banc.intrinseques_simulees(extrinseque['intrinsics_stem'], LARGEUR, HAUTEUR)
        monde.append(banc.modele_camera(nom, centre, rpy, k, LARGEUR, HAUTEUR, nom))
        cameras[nom] = {'extrinseque': str(fichier.resolve()), 'md5': md5(fichier),
                        'intrinsics_stem': extrinseque['intrinsics_stem'],
                        'T_cam_world': extrinseque['T_cam_world'], 'fxfycxcy': list(k)}
        print(f'{nom:8s} centre {np.round(np.array(centre) * 1000)} mm, fx {k[0]:.1f}')
    ET.indent(arbre, '  ')
    fichier_monde = args.sortie / f'{nom_monde}.sdf'
    arbre.write(fichier_monde, encoding='utf-8', xml_declaration=True)
    (args.sortie / 'cameras.json').write_text(json.dumps(
        {'monde': nom_monde, 'base': str(args.base.resolve()), 'md5_base': md5(args.base),
         'moteur_rendu': monde.find("plugin[@filename='gz-sim-sensors-system']/render_engine").text,
         **cameras},
        indent=2))
    print(f'ros2 launch mycobot_gateway banc_realiste.launch.py headless:=true '
          f'world_file:={fichier_monde.resolve()} world_name:={nom_monde} robot_appearance:=original')


def ecart_fk_mm(nom_monde, q_rad):
    """Plus grand écart entre l'origine Gazebo des liens et la FK de mycobot_fk.py.

    Les poses de /pose/info sont relatives au parent : le modèle du robot, posé
    à l'origine du monde.
    """
    texte = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{nom_monde}/pose/info'],
                           capture_output=True, text=True, timeout=20).stdout
    gazebo = {}
    for bloc in re.findall(r'pose \{(.*?)orientation', texte, re.S):
        nom = re.search(r'name: "([^"]+)"', bloc)
        xyz = [re.search(rf'{a}: (\S+)', bloc.split('position', 1)[-1]) for a in 'xyz']
        if nom and nom.group(1) in LIENS:
            gazebo[nom.group(1)] = np.array([float(m.group(1)) if m else 0.0 for m in xyz])
    if set(gazebo) != set(LIENS):
        return float('nan')
    pos, _ = forward_kinematics(q_rad)
    return max(np.linalg.norm(gazebo[l] - np.array(pos[k])) * 1000 for l, k in zip(LIENS, KEYPOINT_NAMES))


def poses_du_jeu(reel):
    """[(numéro, caméra, angles °)] dans l'ordre des fichiers, et les poses uniques."""
    images = []
    for f in sorted(reel.glob('[0-9]*.json')):
        s = json.loads(f.read_text())['source']
        images.append((int(f.stem), s['camera'], tuple(s['angles_deg'])))
    uniques = list(dict.fromkeys(q for _, _, q in images))
    return images, uniques


def cmd_rejouer(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Image, JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    if args.sortie.exists() and any(args.sortie.iterdir()):
        raise SystemExit(f'{args.sortie} n est pas vide')
    args.sortie.mkdir(parents=True, exist_ok=True)
    config = json.loads((args.monde / 'cameras.json').read_text())
    cameras = {k: v for k, v in config.items() if k in ('arducam', 'svpro')}
    images, uniques = poses_du_jeu(args.reel)
    # rejeu.csv garde l'indice de la pose dans le jeu complet : analyse.py
    # apparie les conditions par cet indice, même sur un sous-ensemble.
    indices = list(range(len(uniques)))[:args.max_poses or None][::args.pas]
    uniques = [uniques[i] for i in indices]
    images = [im for im in images if im[2] in uniques]
    print(f'{len(images)} images, {len(uniques)} poses')

    rclpy.init()
    noeud = Node('rejouer_poses_banc',
                 parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    etat = {'joints': None, 'stamp': 0.0}
    dernieres = {}

    def secondes(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def sur_joints(msg):
        index = {n: i for i, n in enumerate(msg.name)}
        if all(a in index for a in ARTICULATIONS):
            etat['joints'] = np.array([msg.position[index[a]] for a in ARTICULATIONS])
            etat['stamp'] = secondes(msg.header.stamp)

    noeud.create_subscription(JointState, '/joint_states', sur_joints, 50)
    for nom in cameras:
        noeud.create_subscription(
            Image, f'/{nom}/image_raw',
            lambda msg, nom=nom: dernieres.__setitem__(nom, msg), 5)
    pub = noeud.create_publisher(JointTrajectory, '/mycobot_controller/joint_trajectory', 10)

    def attendre(condition, delai_s):
        fin = time.monotonic() + delai_s
        while time.monotonic() < fin:
            rclpy.spin_once(noeud, timeout_sec=0.05)
            if condition():
                return True
        return False

    def aller(q_rad):
        msg = JointTrajectory()
        msg.joint_names = ARTICULATIONS
        msg.points = [JointTrajectoryPoint(
            positions=[float(v) for v in q_rad],
            time_from_start=rclpy.duration.Duration(seconds=args.duree_s).to_msg())]
        depart = etat['stamp']
        pub.publish(msg)
        # Temps simulé : le banc peut tourner plus lentement que le réel.
        atteint = attendre(lambda: etat['stamp'] >= depart + args.duree_s + args.repos_s, 120.0)
        ecart = np.degrees(np.abs(etat['joints'] - q_rad)).max()
        return atteint, ecart

    if not attendre(lambda: etat['joints'] is not None and len(dernieres) == len(cameras), 60.0):
        raise SystemExit('ni /joint_states ni les deux caméras : le banc est-il lancé ?')

    sorties = {}
    lignes = []
    for rang, (p, q_deg) in enumerate(zip(indices, uniques)):
        aller(np.radians(POSE_REFUGE))
        q_cmd = np.radians(q_deg)
        atteint, ecart = aller(q_cmd)
        t_repos = etat['stamp']
        frais = attendre(lambda: all(secondes(dernieres[c].header.stamp) > t_repos + 0.15
                                     for c in cameras), 30.0)
        q_lu = etat['joints'].copy()
        fk_mm = ecart_fk_mm(config['monde'], q_lu) if rang < args.controle_fk else ''
        valide = atteint and frais and ecart <= args.tolerance_deg
        sorties[q_deg] = (q_lu, {c: dernieres[c] for c in cameras}, valide)
        lignes.append({'pose': p, 'ecart_max_deg': round(float(ecart), 4), 'ecart_fk_mm': fk_mm,
                       'atteint': atteint,
                       'image_fraiche': frais, 'valide': valide,
                       **{f'j{j + 1}_cmd_deg': q_deg[j] for j in range(6)},
                       **{f'j{j + 1}_lu_deg': round(float(np.degrees(q_lu[j])), 4) for j in range(6)}})
        print(f'pose {rang + 1:3d}/{len(uniques)} (n° {p}) écart {ecart:.3f}° {f"FK {fk_mm:.2f} mm " if fk_mm != "" else ""}'
              f'{"" if valide else "ÉCARTÉE"}',
              flush=True)

    for numero, nom, q_deg in images:
        q_lu, msgs, valide = sorties[q_deg]
        msg = msgs[nom]
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        cv2.imwrite(str(args.sortie / f'{numero:06d}.rgb.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        T = np.array(cameras[nom]['T_cam_world'], float)
        fx, fy, cx, cy = cameras[nom]['fxfycxcy']
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        pos, _ = forward_kinematics(q_lu)
        points = np.array([pos[k] for k in KEYPOINT_NAMES], float)
        cam = points @ T[:3, :3].T + T[:3, 3]
        uv = cv2.projectPoints(points, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, None)[0].reshape(-1, 2)
        (args.sortie / f'{numero:06d}.json').write_text(json.dumps({'objects': [{
            'class': 'mycobot320',
            'keypoints': [{'name': k, 'location': cam[i].tolist(), 'projected_location': uv[i].tolist()}
                          for i, k in enumerate(KEYPOINT_NAMES)],
        }], 'source': {'camera': nom, 'angles_deg': list(q_deg),
                       'angles_lus_deg': np.degrees(q_lu).tolist(), 'valide': valide,
                       'apparence': args.apparence, 'image_stamp': secondes(msg.header.stamp)}}))

    fx, fy, cx, cy = cameras['arducam']['fxfycxcy']
    (args.sortie / '_camera_settings.json').write_text(json.dumps({'camera_settings': [{
        'name': 'mycobot_camera',
        'intrinsic_settings': {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 's': 0.0},
        'captured_image_size': {'width': LARGEUR, 'height': HAUTEUR}}]}, indent=2))
    with (args.sortie / 'rejeu.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(lignes[0]))
        w.writeheader()
        w.writerows(lignes)
    ecartees = sum(not l['valide'] for l in lignes)
    print(f'{len(images)} images -> {args.sortie} ; {ecartees} pose(s) écartée(s)')
    noeud.destroy_node()
    rclpy.shutdown()


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sous = a.add_subparsers(dest='commande', required=True)
    m = sous.add_parser('monde', help='monde du banc aux caméras d étiquetage')
    m.add_argument('--arducam', type=Path, required=True)
    m.add_argument('--svpro', type=Path, required=True)
    m.add_argument('--sortie', type=Path, required=True)
    m.add_argument('--base', type=Path, default=banc.SORTIE, help='monde de départ (défaut : la réplique du banc)')
    m.add_argument('--moteur-rendu', default=banc.MOTEUR_RENDU, choices=['ogre', 'ogre2'],
                   help='si le monde n a pas de capteurs ; ogre2 pour le monde d entraînement '
                        '(celui de l URDF qui a rendu E3.7)')
    r = sous.add_parser('rejouer', help='rejoue les poses du jeu réel')
    r.add_argument('--reel', type=Path, required=True, help='jeu réel NDDS (source.angles_deg)')
    r.add_argument('--monde', type=Path, required=True, help='dossier écrit par « monde »')
    r.add_argument('--apparence', required=True, choices=['original', 'realistic'],
                   help='celle passée au launch, consignée seulement')
    r.add_argument('--sortie', type=Path, required=True)
    r.add_argument('--duree-s', type=float, default=3.0)
    r.add_argument('--repos-s', type=float, default=1.0)
    r.add_argument('--tolerance-deg', type=float, default=0.5)
    r.add_argument('--max-poses', type=int, default=0, help='essai court : les N premières poses')
    r.add_argument('--pas', type=int, default=1, help='une pose sur N (indices 0, N, 2N…), sous-ensemble fixe')
    r.add_argument('--controle-fk', type=int, default=10, help='poses contrôlées contre la FK (Gazebo /pose/info)')
    args = a.parse_args()
    {'monde': cmd_monde, 'rejouer': cmd_rejouer}[args.commande](args)


if __name__ == '__main__':
    main()
