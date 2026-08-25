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
import pick_fsm as fsm  # noqa: E402

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


class AnneauRompuParLOuverture(unittest.TestCase):
    """L'anneau du rouleau doit etre RECOLLE avant d'etre juge.

    Mesure du 25/08 sur les deux rouleaux reels, arducam a 2,01 mm/px :

                          aire      trou
        masque brut    206/236 px  54/98 px
        apres OPEN     148/132 px   0/0
        OPEN + CLOSE   206/231 px  51/96 px

    L'ouverture qui nettoie le bruit rompt aussi l'anneau, qui perd alors son
    trou et un tiers de son aire. Sans la fermeture, le scotch ne tenait que
    par la regle du blob compact, et un anneau casse en deux arcs passait sous
    AIRE_OBJET_MIN — il disparaissait des qu'on le deplacait.
    """

    ECHELLE = 2.01   # mm/px, mesuree au plan de l'objet

    def _vision(self):
        vision = tb.Vision.__new__(tb.Vision)
        vision.masque_plateau = lambda forme: np.full(forme, 255, np.uint8)
        vision.vers_base = lambda uv, z: np.array([uv[0] * self.ECHELLE,
                                                   uv[1] * self.ECHELLE, z])
        vision._cotes_mm = lambda hull, z: tuple(
            sorted(np.asarray(cv2.minAreaRect(hull)[1], float) * self.ECHELLE))
        return vision

    BOIS = (40, 90, 170)     # BGR

    def _scene(self, rayon, fente=0, perce=True):
        """Fond bois + un anneau bleu, fendu de `fente` pixels."""
        cote = 6 * rayon
        image = np.full((cote, cote, 3), self.BOIS, np.uint8)
        centre = cote // 2
        cv2.circle(image, (centre, centre), rayon, (150, 60, 30), -1)
        if perce:
            cv2.circle(image, (centre, centre), rayon // 2, self.BOIS, -1)
        if fente:
            cv2.line(image, (centre, centre - rayon), (centre, centre + rayon),
                     self.BOIS, fente)
        return image

    def test_un_anneau_fendu_reste_un_scotch(self):
        """Un rouleau de 36 mm, coupe en deux arcs, doit rester un objet."""
        for fente in (0, 1, 2):
            trouves = self._vision().objets(self._scene(9, fente))
            self.assertEqual([c for c, _, _ in trouves], ['scotch'],
                             f'fente de {fente} px')

    def test_un_gros_rouleau_ne_tient_que_par_son_trou(self):
        """Au-dela de COTE_SCOTCH_COMPACT, seul le trou nomme le scotch.

        C'est le cas qui prouve que la fermeture n'a pas bouche l'anneau : a
        70 mm, la regle du blob compact ne s'applique plus, et un disque plein
        de cette taille n'est plus rien du tout.
        """
        rayon = 18           # 36 px de diametre, soit ~72 mm a 2,01 mm/px
        vision = self._vision()
        _, grand = vision._cotes_mm(
            np.array([[[0, 0]], [[2 * rayon, 2 * rayon]]]), 0.0)
        self.assertGreater(grand, tb.COTE_SCOTCH_COMPACT)

        troue = vision.objets(self._scene(rayon, fente=2, perce=True))
        self.assertEqual([c for c, _, _ in troue], ['scotch'])

        plein = vision.objets(self._scene(rayon, fente=0, perce=False))
        self.assertEqual([c for c, _, _ in plein], [])


class UnMarqueurTropPetitNeDitQueSonNom(unittest.TestCase):
    """Sous ~30 px de cote, la hauteur PnP est plus bruitee que ce qu'on mesure.

    La profondeur d'une cible plane se connait a Z * bruit_coin / cote_px pres.
    A 1 m, un coin pointe a 0,3 px, les marqueurs de 30 mm colles le 25/08 font
    ~15 px : leur hauteur est bruitee de ~20 mm, alors que l'ecart cherche entre
    le rebord et la table en fait 83. Le nom, lui, reste exact — c'est un
    identifiant, aucune mesure n'entre dedans.
    """

    def _vision(self):
        vision = tb.Vision.__new__(tb.Vision)
        vision.K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0],
                             [0.0, 0.0, 1.0]])
        vision.dist = np.zeros(5)
        vision.T = np.eye(4)
        vision.vers_base = lambda uv, z: np.array([300.0, -130.0, z])
        return vision

    def _coins(self, distance_m):
        demi = tb.COTE_MARQUEUR_CARTON / 2000.0
        modele = [(-demi, demi), (demi, demi), (demi, -demi), (-demi, -demi)]
        return np.array([[600.0 * x / distance_m + 320.0,
                          600.0 * y / distance_m + 240.0] for x, y in modele])

    def test_a_15_px_la_hauteur_est_refusee(self):
        _, z, cote_px = self._vision().pose_marqueur(self._coins(1.0))
        self.assertLess(cote_px, tb.COTE_MARQUEUR_PX_MIN)
        self.assertIsNone(z)

    def test_a_36_px_la_hauteur_est_rendue(self):
        _, z, cote_px = self._vision().pose_marqueur(self._coins(0.5))
        self.assertGreater(cote_px, tb.COTE_MARQUEUR_PX_MIN)
        self.assertIsNotNone(z)

    def test_le_nom_passe_meme_sans_hauteur(self):
        vision = self._vision()
        vision.pose_marqueur = lambda coins: (np.array([300.0, -130.0]), None, 15.0)
        rendus = tb.Vision.cartons_marques(vision, None, {10: None})
        self.assertEqual(set(rendus), {'grand'})
        self.assertIsNone(rendus['grand'][1])


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


class ObjetDejaDepose(unittest.TestCase):
    """Un objet dans un carton n'est plus une cible.

    La balle deposee etait redetectee au fond de la boite — mesure du 25/08 :
    35,4 mm a l'interieur de l'ouverture du grand carton, et toujours annoncee
    comme cible. Le cycle repartait la chercher indefiniment.
    """

    def _fenetre(self, ouvertures):
        fenetre = tb.Fenetre.__new__(tb.Fenetre)
        fenetre._ouvertures = ouvertures
        return fenetre

    def _carre(self, cx, cy, cote):
        d = cote / 2.0
        return np.array([[cx - d, cy - d], [cx + d, cy - d],
                         [cx + d, cy + d], [cx - d, cy + d]], float)

    def test_la_balle_au_fond_du_carton_ne_compte_plus(self):
        fenetre = self._fenetre([self._carre(320.0, 170.0, 120.0)])
        self.assertTrue(fenetre._depose(np.array([320.5, 188.0])))

    def test_un_objet_sur_la_planche_reste_une_cible(self):
        fenetre = self._fenetre([self._carre(320.0, 170.0, 120.0)])
        self.assertFalse(fenetre._depose(np.array([220.0, -90.0])))

    def test_un_objet_appuye_contre_la_paroi_compte_comme_depose(self):
        fenetre = self._fenetre([self._carre(320.0, 170.0, 120.0)])
        bord = 320.0 + 60.0
        self.assertTrue(fenetre._depose(np.array([bord + tb.MARGE_DEPOSE - 1.0, 170.0])))
        self.assertFalse(fenetre._depose(np.array([bord + tb.MARGE_DEPOSE + 5.0, 170.0])))

    def test_sans_carton_vu_rien_n_est_depose(self):
        self.assertFalse(self._fenetre([])._depose(np.array([320.0, 170.0])))


class PriseParEpaisseur(unittest.TestCase):
    """Le petit robot se prend par le torse, pas par un bras.

    Le centroide de l'enveloppe convexe suit les membres qui depassent : sur une
    silhouette a bras asymetriques il glisse hors du torse, et la pince se
    refermait sur un bras (constate le 25/08). Le point le plus eloigne du bord
    est par construction le plus epais.
    """

    def _silhouette(self):
        """Torse epais a gauche, long bras fin qui part a droite."""
        masque = np.zeros((200, 300), np.uint8)
        masque[70:130, 40:100] = 255      # torse 60 x 60
        masque[95:105, 100:260] = 255     # bras 160 x 10
        return masque

    def test_le_point_epais_tombe_dans_le_torse(self):
        vision = tb.Vision.__new__(tb.Vision)
        u, v = vision.point_le_plus_epais(self._silhouette())
        self.assertTrue(40 <= u <= 100, f'u={u} hors du torse')
        self.assertTrue(70 <= v <= 130, f'v={v} hors du torse')

    def test_le_centroide_convexe_lui_sort_du_torse(self):
        contours, _ = cv2.findContours(self._silhouette(), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        moments = cv2.moments(cv2.convexHull(contours[0]))
        u = moments['m10'] / moments['m00']
        self.assertGreater(u, 100.0)

    def test_une_tache_vide_ne_rend_rien(self):
        vision = tb.Vision.__new__(tb.Vision)
        self.assertIsNone(vision.point_le_plus_epais(np.zeros((50, 50), np.uint8)))

    def test_le_scotch_n_est_pas_concerne(self):
        self.assertNotIn('scotch', tb.PRISE_PAR_EPAISSEUR)
        self.assertIn('robot', tb.PRISE_PAR_EPAISSEUR)


class PlausibiliteDUnCarton(unittest.TestCase):
    """Une designation fausse est pire qu'une designation absente.

    Le 25/08 un fantome a (505,6 ; -34,2) — hors planche, 507 mm — a ete suivi
    puis GRAVE sur le disque comme designation du grand carton. La continuite
    l'imposait ensuite a chaque image contre la regle de taille, qui donnait
    pourtant le bon resultat, et l'erreur survivait aux relances.
    """

    def test_le_fantome_hors_planche_est_refuse(self):
        self.assertFalse(tb.plausible(np.array([505.6, -34.2])))

    def test_le_pied_du_robot_est_refuse(self):
        self.assertFalse(tb.plausible(np.array([54.4, -18.4])))

    def test_les_deux_cartons_mesures_sont_acceptes(self):
        for xy in ((340.2, 178.4), (316.4, -144.4), (352.6, -155.4)):
            self.assertTrue(tb.plausible(np.array(xy)), xy)

    def test_rien_n_est_plausible_sans_position(self):
        self.assertFalse(tb.plausible(None))

    def test_le_plafond_suit_la_portee_de_largage(self):
        self.assertEqual(tb.RAYON_CARTON_MAX, fsm.PORTEE_CARTON_MAX)


class MesuresDeReference(unittest.TestCase):
    """Les gabarits doivent contenir ce qui a ete mesure le 25/08.

    Chacun d'eux avait ete resserre au plus juste sur une scene, puis rejetait
    un objet reel de quelques millimetres a la suivante : scotch blanc rejete
    pour 2,8 mm, petit robot pour 6, carton de gauche pour 10.
    """

    def test_les_deux_scotchs_passent_le_gabarit(self):
        for cote in (41.4, 36.8, 72.8):        # camera blanc, camera bleu, pied a coulisse
            self.assertLessEqual(cote, tb.COTE_SCOTCH_MM[1], cote)
        self.assertGreaterEqual(36.8, tb.COTE_SCOTCH_MM[0])

    def test_le_robot_passe_dans_ses_deux_poses(self):
        for petit, grand in ((71.0, 109.0), (79.0, 146.0)):
            self.assertTrue(tb.COTE_ROBOT_MM[0] <= grand <= tb.COTE_ROBOT_MM[1],
                            f'{petit}x{grand}')
            self.assertLessEqual(petit, tb.LARGEUR_ROBOT_MAX)

    def test_les_deux_cartons_passent_le_gabarit(self):
        for petit, grand in ((113.0, 125.0), (67.0, 115.0)):
            self.assertGreaterEqual(petit, tb.COTE_CARTON_MM[0], f'{petit}x{grand}')
            self.assertLessEqual(grand, tb.COTE_CARTON_MM[1], f'{petit}x{grand}')

    def test_le_rebord_mesure_est_celui_du_code(self):
        self.assertAlmostEqual(tb.HAUTEUR_CARTON, 82.9, delta=1.0)


class DeposeQuandLeCartonEstMasque(unittest.TestCase):
    """Un objet reste depose meme quand sa boite n'est plus visible.

    La balle deposee redevenait une cible des que le bras passait au-dessus de
    son carton : plus de carton detecte a cet instant, donc plus d'ouverture,
    donc plus rien pour la declarer deposee — et le cycle repartait la chercher
    au fond de la boite (25/08). Le suivi, lui, garde le polygone.
    """

    def _carre(self, cx, cy, cote):
        d = cote / 2.0
        return np.array([[cx - d, cy - d], [cx + d, cy - d],
                         [cx + d, cy + d], [cx - d, cy + d]], float)

    def test_le_polygone_du_suivi_survit_a_la_peremption(self):
        suivi = tb.SuiviCarton()
        polygone = self._carre(320.0, 170.0, 120.0)
        suivi.maj(np.array([320.0, 170.0]), 14400.0, polygone,
                  maintenant=0.0, rebord=83.0)
        # Bien apres la peremption : la position n'est plus servie...
        self.assertIsNone(suivi.position(tb.PEREMPTION_CARTON + 10.0))
        # ... mais le polygone reste disponible pour juger d'un depot.
        self.assertIsNotNone(suivi.polygone)
        fenetre = tb.Fenetre.__new__(tb.Fenetre)
        fenetre._ouvertures = [suivi.polygone]
        self.assertTrue(fenetre._depose(np.array([320.5, 188.0])))


class LaContinuitePrimeSurLaTaille(unittest.TestCase):
    """Une boite PLEINE ne se mesure plus : la position tient l'identite.

    Mesure du 25/08 : le grand carton avec la balle et un scotch dedans tombe a
    59 cm2 contre 126 vide, sous le petit reste a 71 — l'aire s'inverse. Elle ne
    vaut donc que pour NOMMER la premiere fois, boites vides.

    L'ordre inverse a ete essaye le meme jour et retire : il corrigeait bien une
    continuite fausse, mais au prix d'inverser les deux cartons des qu'on
    deposait quelque chose dedans, ce qui est le cas normal d'un tri.
    """

    GRAND = np.array([340.0, 178.0])
    PETIT = np.array([337.0, -182.0])

    def _vision(self, aire_grand, aire_petit):
        vision = tb.Vision.__new__(tb.Vision)
        vision.cartons_marques = lambda *a, **k: {}
        vision._creux_candidats = lambda *a, **k: [
            (aire_grand, self.GRAND, None, 0.02, np.array([130.0, 155.0])),
            (aire_petit, self.PETIT, None, 0.02, np.array([300.0, 180.0]))]
        return vision

    def _noms(self, vision, connus=None):
        return {c: tuple(xy) for c, xy, _, _ in vision.cartons(None, connus=connus)}

    def test_la_continuite_tient_meme_quand_l_aire_dit_l_inverse(self):
        # Cas reel : le grand carton, plein, mesure MOINS que le petit.
        connus = {'grand': self.GRAND, 'petit': self.PETIT}
        noms = self._noms(self._vision(5900.0, 7100.0), connus)
        self.assertEqual(noms['grand'], tuple(self.GRAND))
        self.assertEqual(noms['petit'], tuple(self.PETIT))

    def test_sans_continuite_la_taille_nomme_les_boites_vides(self):
        noms = self._noms(self._vision(12600.0, 7700.0))
        self.assertEqual(noms['grand'], tuple(self.GRAND))
        self.assertEqual(noms['petit'], tuple(self.PETIT))

    def test_deux_cartons_de_meme_gabarit_laissent_faire_la_continuite(self):
        connus = {'grand': self.PETIT, 'petit': self.GRAND}
        noms = self._noms(self._vision(12600.0, 12000.0), connus)
        self.assertEqual(noms['grand'], tuple(self.PETIT))

    def test_l_ecart_mesure_est_bien_au_dela_du_seuil(self):
        ecart = (12600.0 - 7700.0) / 12600.0
        self.assertGreater(ecart, tb.ECART_TAILLE_DECISIF)


class UnCartonVuSeulSeMesure(unittest.TestCase):
    """Un carton vu seul doit etre MESURE, pas suppose.

    Le bras masque regulierement l'un des deux. Le code se rabattait alors sur
    "le plus grand des restants est le grand" : le PETIT carton vu seul devenait
    le grand, et la continuite figeait l'erreur pour toute la seance. C'est le
    mecanisme exact de l'inversion constatee le 25/08.
    """

    def _vision(self, aire, xy):
        vision = tb.Vision.__new__(tb.Vision)
        vision.cartons_marques = lambda *a, **k: {}
        vision._creux_candidats = lambda *a, **k: [
            (aire, np.asarray(xy, float), None, 0.02, np.array([130.0, 155.0]))]
        return vision

    def test_le_grand_vu_seul_est_nomme_grand(self):
        noms = {c: tuple(xy) for c, xy, _, _
                in self._vision(12400.0, (340.0, 178.0)).cartons(None)}
        self.assertEqual(noms['grand'], (340.0, 178.0))
        self.assertNotIn('petit', noms)

    def test_le_petit_vu_seul_n_est_PLUS_nomme_grand(self):
        noms = {c: tuple(xy) for c, xy, _, _
                in self._vision(7700.0, (337.0, -182.0)).cartons(None)}
        self.assertEqual(noms['petit'], (337.0, -182.0))
        self.assertNotIn('grand', noms)

    def test_la_frontiere_est_la_moyenne_geometrique(self):
        frontiere = float(np.sqrt(tb.AIRE_CARTON_ATTENDUE['grand']
                                  * tb.AIRE_CARTON_ATTENDUE['petit']))
        self.assertEqual(tb.nom_par_aire(frontiere * 1.5), 'grand')
        self.assertEqual(tb.nom_par_aire(frontiere / 1.5), 'petit')

    def test_une_aire_entre_les_deux_ne_tranche_pas(self):
        frontiere = float(np.sqrt(tb.AIRE_CARTON_ATTENDUE['grand']
                                  * tb.AIRE_CARTON_ATTENDUE['petit']))
        self.assertIsNone(tb.nom_par_aire(frontiere))

    def test_les_deux_aires_mesurees_tombent_du_bon_cote(self):
        self.assertEqual(tb.nom_par_aire(12600.0), 'grand')   # 126 cm2, mesure
        self.assertEqual(tb.nom_par_aire(7700.0), 'petit')    # 77 cm2, mesure


class LeFiltreEstAuPointDeChoix(unittest.TestCase):
    """Aucune source de cible ne doit pouvoir contourner "deja depose".

    Le filtre etait pose a la source, sur la liste des objets vus. Or deux
    chemins ne passent pas par elle : `_detecte_balle`, qui interroge la camera
    directement, et le relais SVPRO. La balle deposee redevenait donc la cible
    cycle apres cycle (25/08). Il est desormais au point de CHOIX.
    """

    def _fenetre(self, ouvertures):
        fenetre = tb.Fenetre.__new__(tb.Fenetre)
        fenetre._ouvertures = ouvertures
        return fenetre

    def test_la_balle_de_la_camera_est_filtree_comme_les_autres(self):
        carton = np.array([[329.4, 95.2], [449.4, 95.2],
                           [449.4, 215.2], [329.4, 215.2]])
        fenetre = self._fenetre([carton])
        # Position reelle relevee sur le tableau de bord : balle (372, 174),
        # carton grand (389,4 ; 155,2) — 25 mm, donc dedans.
        self.assertTrue(fenetre._depose(np.array([372.0, 174.0])))

    def test_un_objet_hors_de_toute_ouverture_reste_choisissable(self):
        carton = np.array([[329.4, 95.2], [449.4, 95.2],
                           [449.4, 215.2], [329.4, 215.2]])
        self.assertFalse(self._fenetre([carton])._depose(np.array([314.0, -34.0])))
