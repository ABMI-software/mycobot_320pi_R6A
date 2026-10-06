"""Tout fichier de launch doit se charger.

`sim_grasp.launch.py` est reste inchargeable douze jours sans que personne le
voie : une `PathJoinSubstitution` contenant une liste imbriquee levait une
TypeError avant meme qu'un processus demarre. Rien ne le testait, et le banc de
prehension qui en depend est pourtant designe comme la reference du depot.

Ce test ne lance rien et n'ouvre aucune fenetre : il appelle
`generate_launch_description()`, ce qui suffit a attraper toute erreur de
construction — substitution mal formee, chemin de paquet introuvable, argument
declare deux fois.

Prerequis : espace de travail construit et source (les descriptions resolvent
`get_package_share_directory`).
"""
import importlib.util
import unittest
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
LAUNCH = RACINE / "mycobot_gateway" / "launch"


def fichiers_de_launch():
    return sorted(LAUNCH.glob("*.launch.py"))


class TestFichiersDeLaunch(unittest.TestCase):

    def test_le_repertoire_nest_pas_vide(self):
        self.assertTrue(fichiers_de_launch(),
                        f"aucun fichier de launch trouve dans {LAUNCH}")

    def test_chaque_description_se_construit(self):
        echecs = []
        for chemin in fichiers_de_launch():
            with self.subTest(launch=chemin.name):
                spec = importlib.util.spec_from_file_location(chemin.stem, chemin)
                module = importlib.util.module_from_spec(spec)
                try:
                    spec.loader.exec_module(module)
                    description = module.generate_launch_description()
                except Exception as exc:                      # noqa: BLE001
                    echecs.append(f"{chemin.name}: {type(exc).__name__}: {exc}")
                    continue
                self.assertTrue(
                    description.entities,
                    f"{chemin.name} rend une description vide")
        self.assertEqual([], echecs, "\n".join(echecs))


if __name__ == "__main__":
    unittest.main()
