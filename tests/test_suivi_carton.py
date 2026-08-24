"""Le rectangle du carton doit être stable sans devenir aveugle.

Mesuré le 24/08 : la détection image par image tremble de quelques mm (l'ombre
bouge, pas le carton) et saute de 50 à 170 mm quand le bras passe au-dessus.
Le suivi lisse le premier et ignore le second, mais suit un vrai déplacement.
"""
import sys
import unittest
from pathlib import Path

import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / "scripts"))

import pick_dashboard as tb  # noqa: E402

TAILLE = 101.0 * 135.0          # ouverture mesurée, mm²


class SuiviCartonTests(unittest.TestCase):
    def setUp(self):
        self.suivi = tb.SuiviCarton()
        self.t = 1000.0

    def _voit(self, xy, taille=TAILLE, pas=0.06):
        self.t += pas
        self.suivi.maj(np.asarray(xy, float), taille, None, self.t)

    def test_la_premiere_detection_est_adoptee(self):
        self._voit((320.0, -140.0))
        centre, _ = self.suivi.position(self.t)
        np.testing.assert_allclose(centre, [320.0, -140.0])

    def test_le_tremblement_est_lisse(self):
        self._voit((320.0, -140.0))
        for dx, dy in ((6, -5), (-7, 4), (5, 6), (-4, -6), (7, 3), (-6, 5)):
            self._voit((320.0 + dx, -140.0 + dy))
        centre, _ = self.suivi.position(self.t)
        self.assertLess(float(np.linalg.norm(centre - np.array([320.0, -140.0]))), 4.0)

    def test_un_saut_isole_est_ignore(self):
        """Le bras passe au-dessus : une image aberrante ne doit pas déplacer
        la cible."""
        self._voit((320.0, -140.0))
        self._voit((180.0, 60.0))
        centre, _ = self.suivi.position(self.t)
        self.assertLess(float(np.linalg.norm(centre - np.array([320.0, -140.0]))), 5.0)

    def test_un_deplacement_reel_est_suivi(self):
        self._voit((320.0, -140.0))
        for _ in range(tb.CONFIRMATIONS_CARTON):
            self._voit((180.0, 60.0))
        centre, _ = self.suivi.position(self.t)
        np.testing.assert_allclose(centre, [180.0, 60.0], atol=1.0)

    def test_des_aberrations_dispersees_ne_confirment_rien(self):
        """Six sauts, mais chacun ailleurs : ce n'est pas un déplacement."""
        self._voit((320.0, -140.0))
        for xy in ((180.0, 60.0), (500.0, -200.0), (100.0, 200.0),
                   (450.0, 100.0), (150.0, -250.0), (520.0, 180.0)):
            self._voit(xy)
        centre, _ = self.suivi.position(self.t)
        self.assertLess(float(np.linalg.norm(centre - np.array([320.0, -140.0]))), 5.0)

    def test_un_changement_de_taille_n_est_pas_le_carton(self):
        """Une ombre qui grandit n'est pas l'ouverture : pas de lissage dessus."""
        self._voit((320.0, -140.0))
        self._voit((330.0, -145.0), taille=TAILLE * 3.0)
        centre, _ = self.suivi.position(self.t)
        np.testing.assert_allclose(centre, [320.0, -140.0], atol=0.5)

    def test_la_position_se_perime(self):
        self._voit((320.0, -140.0))
        self.assertIsNone(self.suivi.position(self.t + tb.PEREMPTION_CARTON + 0.1))

    def test_apres_peremption_on_repart_de_la_detection_neuve(self):
        self._voit((320.0, -140.0))
        self.t += tb.PEREMPTION_CARTON + 1.0
        self._voit((180.0, 60.0))
        centre, _ = self.suivi.position(self.t)
        np.testing.assert_allclose(centre, [180.0, 60.0])


if __name__ == "__main__":
    unittest.main()
