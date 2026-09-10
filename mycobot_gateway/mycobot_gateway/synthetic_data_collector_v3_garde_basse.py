#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Collecteur v3 avec la garde au sol abaissee, pour couvrir la zone du pick.

Le v3 rejette toute pose dont un lien descend sous `TABLE_CLEARANCE = 0.13` m.
Mesure du 01/09/2026 sur les 1102 poses reelles de `real_montage_0901` (FK de
`training/dream/mycobot_fk.py`, colonne de base exclue) : le bras y travaille
entre **72,5 et 114,5 mm** au-dessus de la table, donc **1102/1102, soit 100 %,
seraient rejetees**. Le jeu synthetique ne peut structurellement pas contenir la
tache.

C'est ce filtre qui produit le mur sur J2 dans `synthetic_50k` : la distribution
s'arrete net a -103,5 deg (432 poses dans la derniere tranche puis zero, un mur
et non une decroissance) alors que J1 atteint ±167,9 et J3 ±145, leurs limites
pleines. Descendre bas oblige a fermer l'epaule ; interdire le bas coupe J2.

Rien n'est modifie dans le v3 : cette classe n'en change qu'une constante, via
un parametre ROS pour pouvoir la balayer sans toucher au code.

La marge de securite du v3 porte sur le HAUT — le commentaire d'origine demande
de rester SOUS la hauteur d'epaule (~0,162 m), faute de quoi les poses proches
de l'horizontale sont rejetees et la distribution se biaise. Baisser la valeur
n'ouvre donc le domaine que vers le bas, dans le sens sur.

A VERIFIER avant toute collecte longue : que le filtre a 130 mm etait bien
arbitraire et ne masquait pas un artefact de simulation (bras traversant la
table, contacts instables). Regarder les premieres images produites.

Usage :
    ros2 run mycobot_gateway synthetic_data_collector_v3_garde_basse \\
        --ros-args -p table_clearance:=0.05 -p num_samples:=200
"""
import rclpy

from mycobot_gateway.synthetic_data_collector_v3 import SyntheticDataCollectorV3


class CollecteurGardeBasse(SyntheticDataCollectorV3):

    TABLE_CLEARANCE = 0.05

    def __init__(self):
        super().__init__()
        self.declare_parameter('table_clearance', self.TABLE_CLEARANCE)
        self.TABLE_CLEARANCE = self.get_parameter('table_clearance').value
        self.get_logger().warn(
            f'garde au sol = {self.TABLE_CLEARANCE * 1000:.0f} mm '
            f'(v3 d origine : 130 mm — rejetait 100 % des poses du pick)')


def main(args=None):
    rclpy.init(args=args)
    node = CollecteurGardeBasse()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
