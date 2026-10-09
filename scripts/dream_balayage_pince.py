#!/usr/bin/env python3
"""Non-regression DREAM avec / sans pince (protocole, etape 10) : le balayage de poses.

Le bras parcourt POSES, la meme liste dans les deux configurations, et tient
chaque pose TENUE_S secondes de temps simule une fois arrive. La fenetre de
tenue (temps simule) est ecrite dans <log_dir>/poses.csv : c'est elle, et non un
q approche, qui dit quelles lignes de dream_vs_fk.csv appartiennent a la pose.

    # terminal 1 (une fois par configuration, dossier neuf)
    ros2 launch mycobot_gateway tri_yolo.launch.py seed:=1 piece_reach:=0.28 dream:=true \
        robot:=robot_dream_baseline log_dir:=results/yolo_gazebo/<date>_pince_sans
    # terminal 2
    python3 scripts/dream_balayage_pince.py results/yolo_gazebo/<date>_pince_sans
    # puis les deux dossiers
    python3 scripts/dream_pince_compare.py <dossier sans pince> <dossier avec pince>

Poses verifiees par FK le 05/10 : J5 dans [0, -60] (zone ou DREAM detecte,
mesure du 07/09), 7/7 keypoints dans les 4 images, bride a 0,42-0,50 m et pointe
de pince a 0,29 m au plus bas, loin des bacs (95 mm).
"""

import csv
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ('joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6')
POSES = [(0, 60, -70, 0, 0, 0), (0, 30, -60, 0, -30, 0), (30, 30, -60, 0, -30, 0),
         (-30, 30, -60, 0, -30, 0), (60, 20, -50, 10, -20, 0), (-60, 20, -50, -10, -20, 0),
         (0, 0, -40, 0, -40, 0), (45, 0, -30, 20, -50, 0), (-45, 0, -30, -20, -50, 0),
         (0, -20, -20, 0, -30, 0), (90, 30, -60, 0, -20, 0), (-90, 30, -60, 0, -20, 0),
         (20, 45, -90, 30, -10, 0), (-20, 45, -90, -30, -60, 0)]
MOUVEMENT_S = 4
ARRIVEE_DEG = 0.5
TENUE_S = 6.0
ATTENTE_MAX_S = 60.0


class Balayage(Node):
    def __init__(self):
        super().__init__('dream_balayage_pince', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.q = None
        self.pub = self.create_publisher(JointTrajectory, '/mycobot_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in JOINTS):
            self.q = np.degrees([positions[j] for j in JOINTS])

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def attendre(self, condition, delai_s):
        fin = time.monotonic() + delai_s
        while time.monotonic() < fin:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return True
        return False

    def aller(self, q_deg):
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [math.radians(v) for v in q_deg]
        point.time_from_start.sec = MOUVEMENT_S
        traj.points.append(point)
        self.pub.publish(traj)
        return self.attendre(lambda: self.q is not None
                             and np.abs(self.q - q_deg).max() < ARRIVEE_DEG, ATTENTE_MAX_S)


def main():
    dossier = Path(sys.argv[1])
    if not (dossier / 'run.yaml').exists():
        raise SystemExit(f'{dossier}/run.yaml absent : lancer tri_yolo.launch.py log_dir:={dossier}')
    rclpy.init()
    noeud = Balayage()
    if not noeud.attendre(lambda: noeud.q is not None and noeud.sim_now() > 0, ATTENTE_MAX_S):
        raise SystemExit('pas de /joint_states ni d horloge simulee')
    lignes = []
    try:
        for i, q in enumerate(POSES):
            if not noeud.aller(np.array(q, float)):
                raise SystemExit(f'pose {i} {q} non atteinte (q lu {np.round(noeud.q, 1)})')
            debut = noeud.sim_now()
            noeud.attendre(lambda: noeud.sim_now() - debut >= TENUE_S, ATTENTE_MAX_S)
            lignes.append({'pose': i, **{f'q{k + 1}': v for k, v in enumerate(q)},
                           't_debut': f'{debut:.3f}', 't_fin': f'{noeud.sim_now():.3f}'})
            print(f'pose {i:2d} {q} tenue {debut:.1f}-{noeud.sim_now():.1f} s', flush=True)
    finally:
        with open(dossier / 'poses.csv', 'w', newline='') as f:
            ecrit = csv.DictWriter(f, fieldnames=['pose', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
                                                  't_debut', 't_fin'])
            ecrit.writeheader()
            ecrit.writerows(lignes)
        noeud.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
