#!/usr/bin/env python3
"""yolo26 sur les 4 cameras DREAM de la scene de tri, compare a la verite Gazebo.

OUTIL DE VALIDATION : il lit la pose Gazebo des pieces, ce qu'aucun noeud de
perception ni de planification n'a le droit de faire (protocole, invariant I4).

La reference 2D n'existe pas dans Gazebo : c'est la boite 3D de chaque modele
(cotes de `models/*/model.sdf`, pose `/world/<w>/pose/info`) projetee avec les
intrinseques et la pose des cameras dream50k, puis encadree. Boite AMODALE : une
piece cachee derriere une autre garde sa boite entiere ; on la marque « cachee »
si plus de la moitie de sa boite est couverte par une piece plus proche.

    # terminal 1
    ros2 launch mycobot_gateway tri_yolo.launch.py seed:=1
    # terminal 2, une fois le bras en pose d'observation
    /usr/bin/python3 scripts/yolo26_tri_eval.py --sortie results/yolo26_tri/seed1 --seed 1
    # apres plusieurs graines : tableau camera x classe (etape 7)
    /usr/bin/python3 scripts/yolo26_tri_eval.py --tableau results/yolo26_tri
"""
from __future__ import annotations

import argparse
import csv
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

import yolo26_dashboard as y                                         # noqa: E402
from mycobot_gateway.vision import tri_scene as ts                   # noqa: E402
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras  # noqa: E402
from mycobot_gateway.gazebo_ground_truth import read_pose_info  # noqa: E402
from mycobot_gateway.vision.yolo_vs_gt import match_boxes, references  # noqa: E402

URDF = RACINE / 'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf'
MODELES = RACINE / 'mycobot_description/models'
CAMERAS = ('synth_camera', 'synth_camera_right', 'synth_camera_left', 'synth_camera_top')
PIECES = ts.OBJECTS + ts.BINS
ATTENTE_S = 60.0


def poses_gazebo(monde):
    """{modele: (x, y, z)} de TOUS les modeles, statiques compris."""
    _, poses = read_pose_info(monde, PIECES, timeout=15)
    return {nom: T[:3, 3] for nom, T in poses.items()}


def dessine(image, refs, detections, apparies):
    vue = image.copy()
    for nom, r in refs.items():
        x0, y0, x1, y1 = (int(round(v)) for v in r['box'])
        cv2.rectangle(vue, (x0, y0), (x1, y1), (255, 255, 255), 1)
    bonnes = {i for _, i, _ in apparies}
    for i, d in enumerate(detections):
        x0, y0, x1, y1 = (int(v) for v in d['boite'])
        couleur = (0, 200, 0) if i in bonnes else (0, 0, 255)
        cv2.rectangle(vue, (x0, y0), (x1, y1), couleur, 2)
        cv2.putText(vue, f"{d['classe']} {d['conf']:.2f}", (x0, max(10, y0 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, couleur, 1, cv2.LINE_AA)
    return vue


def capture(noeud, pont):
    images = {}
    for nom in CAMERAS:
        noeud.create_subscription(Image, f'/{nom}/image',
                                  lambda m, n=nom: images.setdefault(n, m), qos_profile_sensor_data)
    fin = time.time() + ATTENTE_S
    while len(images) < len(CAMERAS) and time.time() < fin:
        rclpy.spin_once(noeud, timeout_sec=0.5)
    if len(images) < len(CAMERAS):
        raise RuntimeError(f'cameras muettes : {sorted(set(CAMERAS) - set(images))}')
    return {n: (cv2.cvtColor(pont.imgmsg_to_cv2(m, 'rgb8'), cv2.COLOR_RGB2BGR), m.header.stamp)
            for n, m in images.items()}


def tableau(dossier):
    """Camera x classe sur toutes les graines de `dossier` (livrable de l'etape 7).

    Pieces attendues = ni cachees ni tronquees. Taux = bonne classe / attendues.
    """
    lignes = [l for f in sorted(dossier.glob('*/yolo_vs_gt.csv')) for l in csv.DictReader(open(f))]
    print(f'{len({l["seed"] for l in lignes})} graines, {dossier}')
    print(f'{"camera":20s} {"classe":14s} {"detect.":>9s} {"mauv.cl":>7s} {"conf":>5s} '
          f'{"e_px med":>8s} {"e_3d med":>8s} {"e_3d max":>8s}')
    for cam in CAMERAS:
        tp_cam = attendues_cam = 0
        e3_cam = []
        for classe in PIECES:
            l = [x for x in lignes if x['camera'] == cam and x['gt_class'] == classe
                 and x['cachee'] == 'False' and x['tronquee'] == 'False']
            tp = [x for x in l if x['verdict'] == 'TP']
            mauvaise = sum(x['verdict'] == 'mauvaise_classe' for x in l)
            e3 = [float(x['erreur_3d_mm']) for x in tp if x.get('erreur_3d_mm')]
            tp_cam += len(tp)
            attendues_cam += len(l)
            e3_cam += e3
            if not l:
                print(f'{cam:20s} {classe:14s} {"jamais visible":>9s}')
                continue
            conf = np.mean([float(x['confidence']) for x in tp]) if tp else float('nan')
            epx = np.median([float(x['pixel_error']) for x in tp]) if tp else float('nan')
            print(f'{cam:20s} {classe:14s} {len(tp):>4d}/{len(l):<4d} {mauvaise:>7d} {conf:5.2f} '
                  f'{epx:8.1f} {np.median(e3) if e3 else float("nan"):8.1f} '
                  f'{max(e3) if e3 else float("nan"):8.1f}')
        print(f'{cam:20s} {"TOTAL":14s} {tp_cam:>4d}/{attendues_cam:<4d} {"":7s} {"":5s} {"":8s} '
              f'{np.median(e3_cam) if e3_cam else float("nan"):8.1f} '
              f'{max(e3_cam) if e3_cam else float("nan"):8.1f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--tableau', type=Path,
                    help='dossier des graines deja evaluees : imprime le tableau camera x classe')
    ap.add_argument('--sortie', type=Path)
    ap.add_argument('--monde', default='tri_yolo')
    ap.add_argument('--seed', default='')
    ap.add_argument('--poids', type=Path, help='poids yolo26 a evaluer (defaut : ceux du banc)')
    args = ap.parse_args()
    if args.tableau:
        tableau(args.tableau)
        return
    args.sortie.mkdir(parents=True, exist_ok=True)

    cameras = load_cameras(URDF, {'camera_layout': 'dream50k'})
    empreintes = ts.load_footprints(MODELES)
    rclpy.init()
    noeud = rclpy.create_node('yolo26_tri_eval')
    images = capture(noeud, CvBridge())
    poses = poses_gazebo(args.monde)
    if set(poses) != set(PIECES):
        raise RuntimeError(f'pieces absentes de Gazebo : {sorted(set(PIECES) - set(poses))}')

    service = y.ServiceYOLO26(poids=args.poids)
    lignes = []
    try:
        for nom_cam, (image, stamp) in images.items():
            detections = service._aller_retour(nom_cam, image)
            if detections is None:
                raise RuntimeError('yolo26_service.py s est arrete')
            refs = references(cameras[nom_cam], poses, empreintes)
            apparies, fausses, manquees = match_boxes(refs, detections)
            cv2.imwrite(str(args.sortie / f'{nom_cam}.png'), dessine(image, refs, detections, apparies))
            base = {'seed': args.seed, 'stamp': f'{stamp.sec}.{stamp.nanosec:09d}',
                    'camera': nom_cam, 'poids': service.poids, 'seuil': service.seuil}
            for nom, i, score in apparies:
                d, r = detections[i], refs[nom]
                u = (d['boite'][0] + d['boite'][2]) / 2
                v = (d['boite'][1] + d['boite'][3]) / 2
                gu = (r['box'][0] + r['box'][2]) / 2
                gv = (r['box'][1] + r['box'][3]) / 2
                # Meme localisation que yolo_localizer : boite 3D de la classe
                # ANNONCEE par yolo26, recalee sur le centre de la boite 2D.
                try:
                    xyz = ts.locate_from_box(cameras[nom_cam], (u, v), empreintes[d['classe']])
                    erreur_3d = round(float(np.hypot(*(xyz[:2] - poses[nom][:2]))) * 1000, 2)
                except ValueError:
                    xyz, erreur_3d = (np.nan, np.nan), ''
                lignes.append({**base, 'object_id': nom, 'gt_class': nom, 'yolo_class': d['classe'],
                               'verdict': 'TP' if d['classe'] == nom else 'mauvaise_classe',
                               'cachee': r['hidden'], 'tronquee': r['truncated'], 'confidence': round(d['conf'], 4),
                               'iou': round(score, 4), 'u_yolo': u, 'v_yolo': v,
                               'u_gt': round(gu, 2), 'v_gt': round(gv, 2),
                               'pixel_error': round(float(np.hypot(u - gu, v - gv)), 2),
                               'x_yolo': round(float(xyz[0]), 5), 'y_yolo': round(float(xyz[1]), 5),
                               'x_gt': round(float(poses[nom][0]), 5),
                               'y_gt': round(float(poses[nom][1]), 5), 'erreur_3d_mm': erreur_3d,
                               'x_min': d['boite'][0], 'y_min': d['boite'][1],
                               'x_max': d['boite'][2], 'y_max': d['boite'][3]})
            for nom in manquees:
                lignes.append({**base, 'object_id': nom, 'gt_class': nom, 'verdict': 'FN',
                               'cachee': refs[nom]['hidden'], 'tronquee': refs[nom]['truncated']})
            for i in fausses:
                d = detections[i]
                lignes.append({**base, 'yolo_class': d['classe'], 'verdict': 'FP',
                               'confidence': round(d['conf'], 4), 'x_min': d['boite'][0],
                               'y_min': d['boite'][1], 'x_max': d['boite'][2], 'y_max': d['boite'][3]})
    finally:
        service.ferme()
        noeud.destroy_node()
        rclpy.shutdown()

    champs = ['seed', 'stamp', 'camera', 'object_id', 'gt_class', 'yolo_class', 'verdict', 'cachee', 'tronquee',
              'confidence', 'iou', 'u_yolo', 'v_yolo', 'u_gt', 'v_gt', 'pixel_error',
              'x_yolo', 'y_yolo', 'x_gt', 'y_gt', 'erreur_3d_mm',
              'x_min', 'y_min', 'x_max', 'y_max', 'poids', 'seuil']
    with open(args.sortie / 'yolo_vs_gt.csv', 'w', newline='') as f:
        ecrit = csv.DictWriter(f, fieldnames=champs)
        ecrit.writeheader()
        ecrit.writerows(lignes)
    for nom_cam in CAMERAS:
        l = [x for x in lignes if x['camera'] == nom_cam]
        compte = {v: sum(x['verdict'] == v for x in l) for v in ('TP', 'mauvaise_classe', 'FN', 'FP')}
        print(f'{nom_cam:20s} {compte}')


if __name__ == '__main__':
    main()
