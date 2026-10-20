"""Les butees articulaires ne doivent plus diverger en silence.

Le depot portait onze declarations pour trois jeux de valeurs differents,
jusqu'a 25,3 deg d'ecart sur J2, sans qu'aucun test ne le voie. Ce fichier
epingle les trois jeux et verifie que chaque URDF et chaque copie Python
reste d'accord avec celui qu'elle est censee porter.

Il ne dit PAS quel jeu est le bon : il dit qu'on ne peut plus en changer un
sans le declarer ici. Les valeurs Python sont lues par `ast`, jamais
importees — la plupart de ces fichiers sont des noeuds ROS2.
"""
import ast
import math
import re
import sys
import unittest
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import numpy as np  # noqa: E402

from diff_ik import (PRACTICAL_JOINT_LIMITS_DEG,  # noqa: E402
                     URDF_GAZEBO_JOINT_LIMITS_DEG, URDF_JOINT_LIMITS_DEG)

URDF = np.array([(-2.96, 2.96), (-2.79, 2.79), (-2.79, 2.79),
                 (-2.79, 2.79), (-2.96, 2.96), (-3.05, 3.05)])
GAZEBO = np.array([(-2.93, 2.93), (-2.35, 2.35), (-2.53, 2.53),
                   (-2.53, 2.53), (-2.93, 2.93), (-3.14, 3.14)])
PRATIQUE_DEG = np.array([(-168.0, 168.0), (-135.0, 135.0), (-150.0, 150.0),
                         (-145.0, 145.0), (-165.0, 165.0), (-180.0, 180.0)])

BRAS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
        'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']

URDFS = {
    'urdf/320_pi/mycobot_pro_320_pi.urdf': URDF,
    'urdf/320_pi/new_mycobot_pro_320_pi_moveit.urdf': URDF,
    'urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf': GAZEBO,
    'urdf/320_pi/mycobot_pro_320_pi_benchmark.urdf': GAZEBO,
}

COPIES_PYTHON = {
    ('mycobot_gateway/mycobot_gateway/dream_validation_dashboard.py',
     '_JOINT_LIMITS'): URDF,
    ('mycobot_gateway/mycobot_gateway/synthetic_data_collector.py',
     'JOINT_LIMITS'): URDF,
    ('mycobot_gateway/mycobot_gateway/synthetic_data_collector_v2.py',
     'JOINT_LIMITS'): GAZEBO,
    ('mycobot_gateway/mycobot_gateway/precision_benchmark_node.py',
     '_JOINT_LIMITS'): GAZEBO,
}


def _butees_urdf(chemin):
    """[(bas, haut)] en radians pour les six joints du bras, dans l'ordre."""
    texte = (RACINE / 'mycobot_description' / chemin).read_text()
    trouve = {}
    for m in re.finditer(r'<joint name="([^"]+)"[^>]*>(.*?)</joint>',
                         texte, re.S):
        lim = re.search(r'<limit[^>]*lower\s*=\s*"([^"]*)"[^>]*'
                        r'upper\s*=\s*"([^"]*)"', m.group(2))
        if lim:
            trouve[m.group(1)] = (float(lim.group(1)), float(lim.group(2)))
    return np.array([trouve[j] for j in BRAS])


def _butees_python(chemin, nom):
    """Valeur litterale d'une affectation, lue sans importer le module."""
    arbre = ast.parse((RACINE / chemin).read_text())
    for noeud in ast.walk(arbre):
        if isinstance(noeud, ast.Assign):
            for cible in noeud.targets:
                if isinstance(cible, ast.Name) and cible.id == nom:
                    return np.array(ast.literal_eval(noeud.value))
    raise AssertionError(f'{nom} introuvable dans {chemin}')


class LesTroisJeuxSontFiges(unittest.TestCase):
    """Changer une de ces valeurs doit etre un geste conscient."""

    def test_le_jeu_urdf_d_origine(self):
        np.testing.assert_allclose(URDF_JOINT_LIMITS_DEG,
                                   np.degrees(URDF), atol=1e-9)

    def test_le_jeu_urdf_gazebo(self):
        np.testing.assert_allclose(URDF_GAZEBO_JOINT_LIMITS_DEG,
                                   np.degrees(GAZEBO), atol=1e-9)

    def test_le_domaine_pratique(self):
        np.testing.assert_allclose(PRACTICAL_JOINT_LIMITS_DEG,
                                   PRATIQUE_DEG, atol=1e-9)

    def test_les_trois_jeux_sont_bien_distincts(self):
        """Sans ca, ce fichier ne protegerait rien."""
        self.assertFalse(np.allclose(URDF, GAZEBO))

    def test_l_ecart_sur_J2_reste_celui_qu_on_a_mesure(self):
        """25,3 deg entre le modele d'origine et le jumeau Gazebo."""
        ecart = math.degrees(URDF[1][1] - GAZEBO[1][1])
        self.assertAlmostEqual(ecart, 25.3, delta=0.1)


class ChaqueURDFPorteLeJeuAttendu(unittest.TestCase):

    def test_les_quatre_urdf(self):
        for chemin, attendu in URDFS.items():
            with self.subTest(urdf=chemin):
                np.testing.assert_allclose(_butees_urdf(chemin), attendu,
                                           atol=1e-9)


class ChaqueCopiePythonSuitUnJeuNomme(unittest.TestCase):
    """Une douzieme table recopiee a la main doit faire echouer la suite."""

    def test_les_copies_restantes(self):
        for (chemin, nom), attendu in COPIES_PYTHON.items():
            with self.subTest(fichier=chemin, nom=nom):
                np.testing.assert_allclose(_butees_python(chemin, nom),
                                           attendu, atol=1e-9)


class LesSitesDeCommandeNeDupliquentPlus(unittest.TestCase):
    """Ils doivent importer diff_ik, pas retranscrire ses valeurs."""

    DEDOUBLONNES = ['mycobot_gateway/mycobot_gateway/sim_sorting_grasp.py',
                    'mycobot_gateway/mycobot_gateway/calibrate_hand_eye_node.py']

    def test_ils_importent_au_lieu_de_recopier(self):
        for chemin in self.DEDOUBLONNES:
            with self.subTest(fichier=chemin):
                texte = (RACINE / chemin).read_text()
                self.assertIn('PRACTICAL_JOINT_LIMITS_DEG', texte)
                self.assertNotIn('(-135.0, 135.0)', texte)
                self.assertNotIn('(-135., 135.)', texte)


if __name__ == '__main__':
    unittest.main()
