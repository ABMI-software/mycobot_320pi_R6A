#!/usr/bin/env python3
"""yolo26 sur les cameras de Gazebo, en millimetres robot.

Meme chaine que le banc reel — meme modele, memes classes, meme
`Vision.vers_base` — mais les trames viennent de la simulation au lieu du V4L2.
Les deux cameras sont a leur pose calibree dans le monde, donc un objet detecte
rend les millimetres ou il se trouve vraiment.

    # terminal 1
    ros2 launch mycobot_gateway banc_realiste.launch.py
    # terminal 2
    /usr/bin/python3 scripts/yolo26_gazebo.py

`--une-passe` rend la main apres un tour, pour un controle rapide.

Les extrinseques utilisees sont les jumelles SIMULEES (`*_extrinsic_sim.yaml`) :
memes fx, fy, cx, cy et meme T_cam_world que le banc reel, mais DISTORSION NULLE.
Gazebo rend une projection pinhole pure ; appliquer a ces images les coefficients
rationnels du vrai objectif (k1 = 5,4, k3 = -48,2) decalerait chaque point de
plusieurs millimetres sans rien signaler.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RACINE / 'scripts'))

import pick_dashboard as pd                                          # noqa: E402
import yolo26_dashboard as y                                         # noqa: E402

CAMERAS = {'arducam': ('/arducam/image_raw', 'arducam_extrinsic_sim'),
           'svpro': ('/svpro/image_raw', 'svpro_extrinsic_sim')}
ATTENTE_TRAME_S = 30.0


def hauteur(classe):
    """Hauteur du dessus de l'objet, pour projeter son centre dans le plan."""
    return y.HAUTEUR_BAC if classe in y.BACS else pd.HAUTEUR_OBJET


COULEURS = {'bac': (60, 200, 60), 'objet': (0, 180, 255)}


def dessine(nom, image, vision, detections):
    """La vue annotee : boite, classe, confiance et millimetres robot."""
    vue = image.copy()
    for d in sorted(detections, key=lambda d: d['conf']):
        x1, y1, x2, y2 = (int(v) for v in d['boite'])
        classe = d['classe']
        couleur = COULEURS['bac' if classe in y.BACS else 'objet']
        xy = y._centre_base(vision, d['boite'], hauteur(classe))
        cv2.rectangle(vue, (x1, y1), (x2, y2), couleur, 2)
        cv2.putText(vue, f"{classe} {d['conf']:.2f}", (x1, max(12, y1 - 16)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, couleur, 1, cv2.LINE_AA)
        cv2.putText(vue, f'({xy[0]:.0f}, {xy[1]:.0f}) mm', (x1, max(24, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, couleur, 1, cv2.LINE_AA)
    cv2.putText(vue, f'{nom} — Gazebo', (8, 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return vue


def montre(nom, vision, detections):
    print(f'\n--- {nom} ({vision.source}) ---')
    if not detections:
        print('  rien')
        return
    meilleures = {}
    for d in detections:
        if d['conf'] > meilleures.get(d['classe'], (0.0,))[0]:
            meilleures[d['classe']] = (d['conf'], d['boite'])
    for classe, (conf, boite) in sorted(meilleures.items()):
        xy = y._centre_base(vision, boite, hauteur(classe))
        print(f'  {classe:16s} conf {conf:.2f}   '
              f'({xy[0]:6.1f}, {xy[1]:7.1f}) mm   allonge {np.hypot(*xy):5.0f}')
    absentes = [c for c in sorted(set(y.OBJETS) | set(y.BACS)) if c not in meilleures]
    if absentes:
        print(f'  non vues : {", ".join(absentes)}')


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--une-passe', action='store_true',
                   help='un seul tour, puis on rend la main')
    a.add_argument('--sans-fenetre', action='store_true',
                   help='pas d affichage, seulement le texte')
    a.add_argument('--periode', type=float, default=2.0, metavar='S')
    args = a.parse_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    visions = {nom: pd.Vision(stem) for nom, (_, stem) in CAMERAS.items()}
    service = y.ServiceYOLO26()
    print(f'modele {service.poids}, seuil {service.seuil}')
    print('classes : ' + ', '.join(sorted(set(y.OBJETS) | set(y.BACS))))

    rclpy.init()
    noeud = Node('yolo26_gazebo')
    pont, trames = CvBridge(), {}
    for nom, (sujet, _) in CAMERAS.items():
        noeud.create_subscription(
            Image, sujet, lambda m, n=nom: trames.__setitem__(n, m), 10)

    limite = time.time() + ATTENTE_TRAME_S
    while time.time() < limite and len(trames) < len(CAMERAS):
        rclpy.spin_once(noeud, timeout_sec=0.5)
    if not trames:
        print(f'\nAUCUNE trame en {ATTENTE_TRAME_S:.0f} s. La simulation tourne-t-elle, '
              f'et le pont d images avec elle ?\n  ros2 launch mycobot_gateway '
              f'banc_realiste.launch.py', file=sys.stderr)
        return 1

    try:
        while True:
            rclpy.spin_once(noeud, timeout_sec=0.2)
            for nom in CAMERAS:
                if nom in trames:
                    service.soumet(nom, pont.imgmsg_to_cv2(trames[nom], 'bgr8'))
            time.sleep(1.0)
            for nom in CAMERAS:
                dernier = service.dernier(nom)
                if dernier is None:
                    continue
                montre(nom, visions[nom], dernier[1])
                if not args.sans_fenetre:
                    cv2.imshow(f'yolo26 — {nom}',
                               dessine(nom, dernier[0], visions[nom], dernier[1]))
            if not args.sans_fenetre and cv2.waitKey(1) == 27:
                break
            if args.une_passe:
                break
            time.sleep(args.periode)
    except KeyboardInterrupt:
        pass
    finally:
        service.ferme()
        cv2.destroyAllWindows()
        noeud.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
