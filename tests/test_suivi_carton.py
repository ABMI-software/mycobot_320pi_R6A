"""Le rectangle du carton doit être stable sans devenir aveugle.

Mesuré le 24/08 : la détection image par image tremble de quelques mm (l'ombre
bouge, pas le carton) et saute de 50 à 170 mm quand le bras passe au-dessus.
Le suivi lisse le premier et ignore le second, mais suit un vrai déplacement.
"""
import sys
import unittest
from pathlib import Path

import cv2
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
        centre, _, _ = self.suivi.position(self.t)
        np.testing.assert_allclose(centre, [320.0, -140.0])

    def test_le_tremblement_est_lisse(self):
        self._voit((320.0, -140.0))
        for dx, dy in ((6, -5), (-7, 4), (5, 6), (-4, -6), (7, 3), (-6, 5)):
            self._voit((320.0 + dx, -140.0 + dy))
        centre, _, _ = self.suivi.position(self.t)
        self.assertLess(float(np.linalg.norm(centre - np.array([320.0, -140.0]))), 4.0)

    def test_un_saut_isole_est_ignore(self):
        """Le bras passe au-dessus : une image aberrante ne doit pas déplacer
        la cible."""
        self._voit((320.0, -140.0))
        self._voit((180.0, 60.0))
        centre, _, _ = self.suivi.position(self.t)
        self.assertLess(float(np.linalg.norm(centre - np.array([320.0, -140.0]))), 5.0)

    def test_un_deplacement_reel_est_suivi(self):
        self._voit((320.0, -140.0))
        for _ in range(tb.CONFIRMATIONS_CARTON):
            self._voit((180.0, 60.0))
        centre, _, _ = self.suivi.position(self.t)
        np.testing.assert_allclose(centre, [180.0, 60.0], atol=1.0)

    def test_des_aberrations_dispersees_ne_confirment_rien(self):
        """Six sauts, mais chacun ailleurs : ce n'est pas un déplacement."""
        self._voit((320.0, -140.0))
        for xy in ((180.0, 60.0), (500.0, -200.0), (100.0, 200.0),
                   (450.0, 100.0), (150.0, -250.0), (520.0, 180.0)):
            self._voit(xy)
        centre, _, _ = self.suivi.position(self.t)
        self.assertLess(float(np.linalg.norm(centre - np.array([320.0, -140.0]))), 5.0)

    def test_un_changement_de_taille_n_est_pas_le_carton(self):
        """Une ombre qui grandit n'est pas l'ouverture : pas de lissage dessus."""
        self._voit((320.0, -140.0))
        self._voit((330.0, -145.0), taille=TAILLE * 3.0)
        centre, _, _ = self.suivi.position(self.t)
        np.testing.assert_allclose(centre, [320.0, -140.0], atol=0.5)

    def test_la_position_se_perime(self):
        self._voit((320.0, -140.0))
        self.assertIsNone(self.suivi.position(self.t + tb.PEREMPTION_CARTON + 0.1))

    def test_apres_peremption_on_repart_de_la_detection_neuve(self):
        self._voit((320.0, -140.0))
        self.t += tb.PEREMPTION_CARTON + 1.0
        self._voit((180.0, 60.0))
        centre, _, _ = self.suivi.position(self.t)
        np.testing.assert_allclose(centre, [180.0, 60.0])


if __name__ == "__main__":
    unittest.main()


class ReactionImmediate(unittest.TestCase):
    """Deplacer le carton a la main doit se voir tout de suite, pas dans 1 s."""

    def test_le_carton_deplace_est_adopte_en_moins_de_trois_dixiemes(self):
        suivi = tb.SuiviCarton()
        t = 1000.0
        suivi.maj(np.array([430.0, -80.0]), TAILLE, None, t)
        depart = t
        for _ in range(10):
            t += 0.06
            suivi.maj(np.array([250.0, 120.0]), TAILLE, None, t)
            centre, _, _ = suivi.position(t)
            if float(np.linalg.norm(centre - np.array([250.0, 120.0]))) < 1.0:
                break
        self.assertLess(t - depart, 0.3)
        self.assertEqual(suivi.deplacements, 1)


class TriParCategorie(unittest.TestCase):
    """Chaque categorie a UNE destination, et elle ne change pas en route."""

    def test_les_scotchs_vont_dans_le_petit_carton(self):
        self.assertEqual(tb.DESTINATION["scotch"], "petit")

    def test_la_balle_et_le_robot_vont_dans_le_grand(self):
        self.assertEqual(tb.DESTINATION["balle"], "grand")
        self.assertEqual(tb.DESTINATION["robot"], "grand")

    def test_chaque_categorie_a_une_couleur_de_trace(self):
        self.assertEqual(set(tb.DESTINATION), set(tb.COULEUR_OBJET))

    def test_les_deux_cartons_sont_suivis_separement(self):
        """Deplacer un carton ne doit pas bouger la cible de l'autre."""
        suivis = {c: tb.SuiviCarton() for c in ("grand", "petit")}
        t = 1000.0
        suivis["grand"].maj(np.array([400.0, 175.0]), TAILLE, None, t)
        suivis["petit"].maj(np.array([373.0, -151.0]), TAILLE, None, t)
        for _ in range(tb.CONFIRMATIONS_CARTON + 1):
            t += 0.06
            suivis["petit"].maj(np.array([200.0, -250.0]), TAILLE, None, t)
            suivis["grand"].maj(np.array([400.0, 175.0]), TAILLE, None, t)
        np.testing.assert_allclose(suivis["grand"].position(t)[0], [400.0, 175.0], atol=1.0)
        np.testing.assert_allclose(suivis["petit"].position(t)[0], [200.0, -250.0], atol=1.0)


class RobeDesCartons(unittest.TestCase):
    """Le sens de la regle a ete inverse une fois : il est verrouille ici.

    Le GRAND carton est brun, le PETIT est noir a l'exterieur. Trois mesures du
    24/08 concordent : la consigne d'origine, les ouvertures (138x202 mm pour le
    brun contre 62x113 pour le noir) et l'essai reel, ou le robot dirige vers
    'grand' a atterri dans le petit carton.
    """

    def _cartons(self, part_noire_a, part_noire_b):
        vision = tb.Vision.__new__(tb.Vision)
        candidats = [(20000.0, np.array([300.0, -130.0]), None, part_noire_a,
                      np.array([100.0, 100.0])),
                     (7000.0, np.array([390.0, 210.0]), None, part_noire_b,
                      np.array([200.0, 200.0]))]
        vision._creux_candidats = lambda *a, **k: candidats
        vision.cartons_marques = lambda *a, **k: {}
        return dict((classe, tuple(xy))
                    for classe, xy, _, _ in vision.cartons(None))

    def test_le_carton_brun_est_le_grand(self):
        vus = self._cartons(0.02, 0.59)
        self.assertEqual(vus["grand"], (300.0, -130.0))

    def test_le_carton_noir_est_le_petit(self):
        vus = self._cartons(0.02, 0.59)
        self.assertEqual(vus["petit"], (390.0, 210.0))


class MarqueursDesCartons(unittest.TestCase):
    """Un marqueur colle sur le carton prime sur toute heuristique.

    La robe et le gabarit ont bascule des que les deux cartons changeaient de
    place : l'ouverture du carton lointain se mesure a la hauteur SUPPOSEE du
    rebord, et cette hauteur etait fausse de 23 mm (rebord mesure a 82,9 mm par
    triangulation le 25/08, constante a 60). Le marqueur donne les deux d'un
    coup — le nom et la hauteur.
    """

    def _vision(self, part_noire_a=0.02, part_noire_b=0.59):
        vision = tb.Vision.__new__(tb.Vision)
        vision._creux_candidats = lambda *a, **k: [
            (20000.0, np.array([300.0, -130.0]), None, part_noire_a,
             np.array([100.0, 100.0])),
            (7000.0, np.array([390.0, 210.0]), None, part_noire_b,
             np.array([200.0, 200.0]))]
        vision.vers_base = lambda uv, z: np.array(
            [300.0, -130.0, z] if uv[0] < 150 else [390.0, 210.0, z])
        return vision

    def test_le_marqueur_renomme_contre_la_robe(self):
        vision = self._vision()
        vision.cartons_marques = lambda *a, **k: {
            'petit': (np.array([300.0, -130.0]), 83.0),
            'grand': (np.array([390.0, 210.0]), 71.0)}
        vus = {c: (tuple(xy), z) for c, xy, _, z in vision.cartons(None)}
        self.assertEqual(vus['petit'], ((300.0, -130.0), 83.0))
        self.assertEqual(vus['grand'], ((390.0, 210.0), 71.0))

    def test_un_seul_marqueur_laisse_l_autre_a_la_geometrie(self):
        vision = self._vision()
        vision.cartons_marques = lambda *a, **k: {
            'petit': (np.array([300.0, -130.0]), 83.0)}
        vus = {c: (tuple(xy), z) for c, xy, _, z in vision.cartons(None)}
        self.assertEqual(vus['petit'], ((300.0, -130.0), 83.0))
        self.assertEqual(vus['grand'], ((390.0, 210.0), tb.HAUTEUR_CARTON))

    def test_marqueur_trop_loin_de_toute_ouverture_ignore(self):
        vision = self._vision()
        vision.cartons_marques = lambda *a, **k: {
            'petit': (np.array([300.0 + tb.PORTE_MARQUEUR_CARTON + 50.0, -130.0]),
                      83.0)}
        vus = {c: tuple(xy) for c, xy, _, _ in vision.cartons(None)}
        self.assertEqual(vus['grand'], (300.0, -130.0))
        self.assertEqual(vus['petit'], (390.0, 210.0))


class ContinuiteDesCartons(unittest.TestCase):
    """Deux cartons de meme ouverture ne se separent que par ou ils sont.

    Mesure du 25/08 : poses cote a cote, les deux cartons donnent 160x214 et
    144x205 mm — 5 % d'ecart, sous le bruit. Classes par l'aire, l'etiquette
    basculait d'une image a l'autre ; le tri envoyait l'objet dans le mauvais
    carton une fois sur deux.
    """

    def _vision(self, aires):
        vision = tb.Vision.__new__(tb.Vision)
        vision.cartons_marques = lambda *a, **k: {}
        vision._creux_candidats = lambda *a, **k: [
            (aires[0], np.array([350.0, -170.0]), None, 0.02, np.array([100.0, 100.0])),
            (aires[1], np.array([368.0, 168.0]), None, 0.02, np.array([200.0, 200.0]))]
        return vision

    def test_sans_rien_de_connu_le_plus_grand_est_le_grand(self):
        vus = {c: tuple(xy) for c, xy, _, _ in self._vision((30000.0, 35000.0)).cartons(None)}
        self.assertEqual(vus['grand'], (368.0, 168.0))

    def test_le_nom_survit_a_une_inversion_des_aires(self):
        connus = {'grand': np.array([350.0, -170.0]), 'petit': np.array([368.0, 168.0])}
        # L'aire s'inverse d'une image a l'autre — c'est le bruit mesure.
        for aires in ((30000.0, 35000.0), (35000.0, 30000.0)):
            vus = {c: tuple(xy) for c, xy, _, _
                   in self._vision(aires).cartons(None, connus=connus)}
            self.assertEqual(vus['grand'], (350.0, -170.0))
            self.assertEqual(vus['petit'], (368.0, 168.0))

    def test_un_carton_vraiment_deplace_ne_vole_pas_le_nom_de_l_autre(self):
        connus = {'grand': np.array([350.0, -170.0]),
                  'petit': np.array([368.0 + 3 * tb.CONTINUITE_CARTON, 168.0])}
        vus = {c: tuple(xy) for c, xy, _, _
               in self._vision((30000.0, 35000.0)).cartons(None, connus=connus)}
        self.assertEqual(vus['grand'], (350.0, -170.0))
        self.assertEqual(vus['petit'], (368.0, 168.0))


class MarqueurPoseAPlatSurLaTable(unittest.TestCase):
    """Un marqueur au ras de la table nomme le carton mais ne dit pas sa hauteur.

    Certains cartons n'offrent aucune surface horizontale au niveau du rebord :
    leurs rabats se rabattent a plat sur la table. Prendre la hauteur d'un tel
    marqueur pour celle du rebord decalerait le centre de l'ouverture de 50 mm.
    """

    def _vision(self, z_marqueur):
        vision = tb.Vision.__new__(tb.Vision)
        vision._creux_candidats = lambda *a, **k: [
            (20000.0, np.array([300.0, -130.0]), None, 0.02, np.array([100.0, 100.0]))]
        vision.vers_base = lambda uv, z: np.array([300.0, -130.0, z])
        vision.cartons_marques = tb.Vision.cartons_marques.__get__(vision)
        vision.pose_marqueur = lambda coins: (np.array([300.0, -130.0]), z_marqueur, 0.0)
        return vision

    def test_un_marqueur_sur_le_rebord_donne_sa_hauteur(self):
        vus = {c: z for c, _, _, z in self._vision(84.0).cartons(None, marqueurs={10: None})}
        self.assertEqual(vus['grand'], 84.0)

    def test_un_marqueur_au_ras_de_la_table_ne_la_donne_pas(self):
        vus = {c: z for c, _, _, z in self._vision(3.0).cartons(None, marqueurs={10: None})}
        self.assertEqual(vus['grand'], tb.HAUTEUR_CARTON)


class CoeurSombreDuCreux(unittest.TestCase):
    """L'ombre de la paroi exterieure ne doit pas gonfler l'ouverture.

    Le petit carton mesure 115 x 70 mm au metre ruban. Son creux, colle a
    l'ombre de sa propre paroi, etait mesure 105 x 203 mm — et le point de
    largage se choisit sur ce polygone, donc au-dessus de la paroi plutot que
    dans la boite. Le coeur sombre le ramene a 67 x 115 mm.
    """

    def _vision(self):
        vision = tb.Vision.__new__(tb.Vision)
        vision._cotes_mm = lambda contour, z=None: tuple(
            sorted(cv2.minAreaRect(contour)[1]))
        return vision

    def _creux(self, valeur, contour):
        return self._vision()._coeur_sombre(contour, valeur)

    def test_le_creux_se_separe_de_l_ombre_de_la_paroi(self):
        valeur = np.full((200, 200), 200, np.uint8)
        valeur[40:150, 40:110] = 30        # interieur de la boite
        valeur[40:150, 110:170] = 120      # paroi a l'ombre, sombre aussi
        contour = np.array([[[40, 40]], [[169, 40]], [[169, 149]], [[40, 149]]], np.int32)
        coeur = self._creux(valeur, contour)
        self.assertIsNotNone(coeur)
        petit, grand = sorted(cv2.minAreaRect(cv2.convexHull(coeur))[1])
        self.assertAlmostEqual(petit, 69.0, delta=6.0)
        self.assertAlmostEqual(grand, 109.0, delta=6.0)

    def test_une_ouverture_uniforme_n_est_pas_coupee_en_deux(self):
        valeur = np.full((200, 200), 200, np.uint8)
        valeur[40:150, 40:170] = 30
        contour = np.array([[[40, 40]], [[169, 40]], [[169, 149]], [[40, 149]]], np.int32)
        coeur = self._creux(valeur, contour)
        if coeur is not None:
            aire = cv2.contourArea(coeur) / cv2.contourArea(contour)
            self.assertGreater(aire, 0.9)


class RayonDeLaBase(unittest.TestCase):
    """Rien de ce qui touche la base n'est un carton : c'est le robot.

    Sans ce garde-fou, le bras au repos etait detecte comme un creux de
    70 x 164 mm a 57 mm de la base et prenait le nom de "petit carton" — le
    carton fantome au milieu de la table, qui ne bougeait pas quand on
    deplacait le vrai. Le masque cinematique ne suffit pas : il exige les
    angles, donc le pont vers la Pi, et sans lui il ne masque rien.
    """

    def test_le_seuil_est_sous_la_zone_de_largage(self):
        self.assertLessEqual(tb.RAYON_BASE_MIN, 200.0)

    def test_le_bras_au_pied_du_robot_est_hors_du_seuil(self):
        self.assertLess(float(np.hypot(54.4, -18.4)), tb.RAYON_BASE_MIN)

    def test_les_deux_cartons_mesures_sont_au_dela(self):
        for xy in ((316.4, -144.7), (353.1, 196.3)):
            self.assertGreater(float(np.hypot(*xy)), tb.RAYON_BASE_MIN)
