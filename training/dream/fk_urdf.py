#!/usr/bin/env python3
"""Cinématique directe de TOUS les liens d'un URDF, pince comprise.

`mycobot_fk.py` ne couvre que les 6 articulations du bras et les 7 keypoints
de DREAM. Les annotations complètes d'une collecte (docs/SYNTHETIC_DATA.md,
« Format de sortie ») veulent la pose de chaque lien — pince, doigts,
caméras fixes —, à partir de l'URDF réellement chargé dans Gazebo (enregistré
par le collecteur dans `robot.urdf`) et des positions MESURÉES de toutes les
articulations (`joints_complets.csv`).

Une articulation absente des mesures prend sa valeur `mimic` si elle en a une,
0 sinon : DART ignore `<mimic>`, donc en simulation la valeur mesurée fait foi.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def _rotation_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _rotation_axe(axe, angle):
    axe = np.asarray(axe, float) / np.linalg.norm(axe)
    k = np.array([[0, -axe[2], axe[1]], [axe[2], 0, -axe[0]], [-axe[1], axe[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _homogene(rotation=np.eye(3), translation=(0.0, 0.0, 0.0)):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rotation, translation
    return t


def _vecteur(element, attribut, defaut):
    if element is None or element.get(attribut) is None:
        return defaut
    return [float(v) for v in element.get(attribut).split()]


class ModeleURDF:
    def __init__(self, texte_urdf):
        racine = ET.fromstring(texte_urdf)
        self.liens = [l.get("name") for l in racine.findall("link")]
        self.articulations = {}
        for j in racine.findall("joint"):
            origine = j.find("origin")
            mimic = j.find("mimic")
            self.articulations[j.get("name")] = {
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "enfant": j.find("child").get("link"),
                "origine": _homogene(_rotation_rpy(*_vecteur(origine, "rpy", [0, 0, 0])),
                                     _vecteur(origine, "xyz", [0, 0, 0])),
                "axe": _vecteur(j.find("axis"), "xyz", [1, 0, 0]),
                "mimic": None if mimic is None else (
                    mimic.get("joint"), float(mimic.get("multiplier", 1)), float(mimic.get("offset", 0))),
            }
        enfants = {a["enfant"] for a in self.articulations.values()}
        self.racine = next(l for l in self.liens if l not in enfants)
        self.mobiles = [n for n, a in self.articulations.items() if a["type"] in ("revolute", "continuous", "prismatic")]

    def valeur(self, nom, positions):
        if nom in positions:
            return positions[nom]
        mimic = self.articulations[nom]["mimic"]
        if mimic is not None and mimic[0] in positions:
            return mimic[1] * positions[mimic[0]] + mimic[2]
        return 0.0

    def poses(self, positions):
        """Lien -> transformation 4x4 dans le repère de la racine (`world`)."""
        poses = {self.racine: np.eye(4)}
        restantes = dict(self.articulations)
        while restantes:
            pretes = [n for n, a in restantes.items() if a["parent"] in poses]
            if not pretes:
                raise ValueError(f"liens non rattachés à {self.racine} : {sorted(restantes)}")
            for nom in pretes:
                a = restantes.pop(nom)
                mouvement = np.eye(4)
                if a["type"] in ("revolute", "continuous"):
                    mouvement = _homogene(_rotation_axe(a["axe"], self.valeur(nom, positions)))
                elif a["type"] == "prismatic":
                    mouvement = _homogene(translation=np.asarray(a["axe"], float) * self.valeur(nom, positions))
                poses[a["enfant"]] = poses[a["parent"]] @ a["origine"] @ mouvement
        return poses
