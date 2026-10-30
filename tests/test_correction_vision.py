"""Carte de correction a deux couches, et apprentissage par relache sur place.

Ce qui est tenu : une carte vide ne corrige rien ; la SVPRO n'est pas corrigee ;
une nouvelle extrinseque remet la couche camera a zero et garde la couche robot ;
seule une relache relue apprend, jamais une prise confirmee seule ; un
echantillon remplace ses voisins dans la meme extrinseque, pas dans une autre.
"""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / "scripts"))

import correction_vision as cv  # noqa: E402
import pick_fsm as fsm  # noqa: E402

HAUTEUR_CENTRE_BALLE = 32.07


def _extrinseque(dossier, nom, centre_mm):
    """Camera a la verticale, centre optique en `centre_mm`."""
    R = np.diag([1.0, -1.0, -1.0])
    t = -R @ (np.asarray(centre_mm, float) / 1000.0)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    chemin = Path(dossier) / nom
    chemin.write_text(yaml.safe_dump({"T_cam_world": T.tolist()}))
    return chemin


class FauxPont:
    def __init__(self, q):
        self.q = np.asarray(q, float)
        self.ordres = []

    def angles(self):
        return self.q.copy()

    def envoie(self, action, **kw):
        self.ordres.append(action)
        return "OK"

    def angle_pince(self):
        return 100


class FausseVision:
    source = "arducam_extrinsic_pick.yaml"

    def balle(self, image):
        return np.array([300.0, 20.0]), (1.0, 2.0, 3.0)

    def objets(self, image, angles=None, marqueurs=None):
        return [("scotch", np.array([300.0, 20.0]), None)]


class Fenetre:
    """Ce que `branche` accroche sur la fenetre du dashboard."""
    _camera_balle = "arducam"

    def _detecte_balle(self, echantillons=3, patience=3.0, exige_dessus=False):
        return np.array([300.0, 20.0])

    def show(self):
        pass


class Base(unittest.TestCase):
    def setUp(self):
        self.dossier = tempfile.TemporaryDirectory()
        self.fichier = Path(self.dossier.name) / "carte.json"
        self.ext_a = _extrinseque(self.dossier.name, "a.yaml", (300, 0, 1100))
        self.ext_b = _extrinseque(self.dossier.name, "b.yaml", (250, 150, 1000))
        self.ext_c = _extrinseque(self.dossier.name, "c.yaml", (350, -120, 1050))

    def tearDown(self):
        self.dossier.cleanup()

    def _synthetique(self, extrinseque, translation, robot, points):
        ep = cv.epoque_de(extrinseque)
        for p in points:
            p = np.asarray(p, float)
            reel = p + np.asarray(translation) + robot(p)
            cv.ajoute(p, reel, epoque=ep, z_mm=HAUTEUR_CENTRE_BALLE, fichier=self.fichier)


class TestCouches(Base):
    GRILLE = [(x, y) for x in (220, 300, 380) for y in (-90, 0, 90)]

    @staticmethod
    def robot(p):
        return np.array([2.5, -1.5]) + (p - cv.PIVOT) * np.array([0.01, 0.0])

    def test_carte_vide_neutre(self):
        carte = cv.Carte(self.fichier, self.ext_a, HAUTEUR_CENTRE_BALLE)
        np.testing.assert_allclose(carte.corrige((300.0, 20.0)), [300.0, 20.0])

    def test_interpolateur_exact_dans_son_extrinseque(self):
        self._synthetique(self.ext_a, (1.0, -2.0), self.robot, self.GRILLE)
        carte = cv.Carte(self.fichier, self.ext_a, HAUTEUR_CENTRE_BALLE)
        p = np.array([380.0, 90.0])
        np.testing.assert_allclose(carte.ecart(p), np.array([1.0, -2.0]) + self.robot(p),
                                   atol=0.01)

    def test_nouvelle_extrinseque_garde_le_robot(self):
        self._synthetique(self.ext_a, (1.5, -1.0), self.robot, self.GRILLE)
        self._synthetique(self.ext_b, (-1.5, 1.0), self.robot, self.GRILLE)
        carte = cv.Carte(self.fichier, self.ext_c, HAUTEUR_CENTRE_BALLE)
        self.assertEqual(carte.dans_epoque(), 0)
        for p in [(260.0, -40.0), (340.0, 50.0)]:
            p = np.array(p)
            erreur = np.linalg.norm(carte.ecart(p) - self.robot(p))
            self.assertLess(erreur, 0.6, f"en {p}")
            self.assertLess(erreur, np.linalg.norm(self.robot(p)) / 3)

    def test_deux_prises_rattrapent_la_nouvelle_camera(self):
        self._synthetique(self.ext_a, (1.5, -1.0), self.robot, self.GRILLE)
        self._synthetique(self.ext_b, (-1.5, 1.0), self.robot, self.GRILLE)
        self._synthetique(self.ext_c, (3.0, 2.0), self.robot, [(240, -60), (360, 70)])
        carte = cv.Carte(self.fichier, self.ext_c, HAUTEUR_CENTRE_BALLE)
        p = np.array([300.0, 0.0])
        attendu = np.array([3.0, 2.0]) + self.robot(p)
        sans_camera = np.linalg.norm(self.robot(p) - attendu)
        self.assertLess(np.linalg.norm(carte.ecart(p) - attendu), sans_camera / 2)

    def test_voisin_remplace_dans_la_meme_extrinseque_seulement(self):
        a, b = cv.epoque_de(self.ext_a), cv.epoque_de(self.ext_b)
        cv.ajoute((300, 20), (305, 20), epoque=a, fichier=self.fichier)
        cv.ajoute((306, 22), (310, 22), epoque=a, fichier=self.fichier)
        cv.ajoute((304, 21), (306, 21), epoque=b, fichier=self.fichier)
        self.assertEqual(len(cv.charge(self.fichier)["echantillons"]), 2)

    def test_brut_inverse_corrige(self):
        self._synthetique(self.ext_a, (4.0, -3.0), self.robot, [(300, 20), (200, -80)])
        carte = cv.Carte(self.fichier, self.ext_a, HAUTEUR_CENTRE_BALLE)
        brut = np.array([260.0, -20.0])
        np.testing.assert_allclose(carte.brut(carte.corrige(brut)), brut, atol=0.05)

    def test_validation_par_transfert(self):
        # Le robot domine, la camera varie peu d'une extrinseque a l'autre :
        # c'est le cas ou la couche robot DOIT se transferer.
        self._synthetique(self.ext_a, (0.5, -0.3), self.robot, self.GRILLE)
        self._synthetique(self.ext_b, (-0.5, 0.3), self.robot, self.GRILLE)
        carte = cv.Carte(self.fichier, self.ext_a, HAUTEUR_CENTRE_BALLE)
        transfert = carte.valide_transfert()
        self.assertEqual(set(transfert), {cv.epoque_de(self.ext_a)["id"],
                                          cv.epoque_de(self.ext_b)["id"]})
        for erreurs, brutes in transfert.values():
            self.assertLess(erreurs.mean(), brutes.mean())

    def test_ancien_format_sans_extrinseque(self):
        self.fichier.write_text('{"echantillons": [{"vision": [300, 0], '
                                '"reel": [303, 0], "ecart": [3, 0]}]}')
        carte = cv.Carte(self.fichier, self.ext_a, HAUTEUR_CENTRE_BALLE)
        self.assertEqual(carte.echantillons[0]["epoque"], cv.INCONNUE)
        self.assertEqual(carte.dans_epoque(), 0)


class TestBranche(Base):
    def setUp(self):
        super().setUp()
        (Path(self.dossier.name) / FausseVision.source).write_text(self.ext_a.read_text())
        self.saisie = fsm.ACTIONS["SAISIE"]
        self.transitions = list(fsm.TRANSITIONS)
        self.originaux = {c: dict(vars(c)) for c in (FausseVision, Fenetre)}
        FausseVision.__module__ = __name__
        self.coin = fsm.pointe(fsm.POSE_OBSERVATION)[:2]

    def tearDown(self):
        fsm.ACTIONS["SAISIE"] = self.saisie
        fsm.TRANSITIONS[:] = self.transitions
        for classe, attributs in self.originaux.items():
            for nom in ("balle", "objets", "_detecte_balle", "show"):
                if nom in attributs:
                    setattr(classe, nom, attributs[nom])
        super().tearDown()

    def _branche(self, resultat, coins=None, test=False):
        fsm.ACTIONS["SAISIE"] = lambda ctx: resultat
        return cv.branche(FausseVision, self.fichier, coins, calib=Path(self.dossier.name),
                          test=test)

    def _ctx(self, classe="balle", xy=(300.0, 20.0)):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont(fsm.POSE_OBSERVATION)
        ctx.source_cible, ctx.classe_objet = "arducam", classe
        ctx.balle_xy = np.asarray(xy, float)
        ctx.R_balle = fsm.orientation(np.asarray(xy, float))
        return ctx

    def test_carte_vide_neutre_et_vues_enregistrees(self):
        carte = self._branche("REMONTEE")
        xy, _ = FausseVision().balle(None)
        np.testing.assert_allclose(xy, [300.0, 20.0])
        np.testing.assert_allclose(Fenetre()._detecte_balle(), [300.0, 20.0])
        self.assertEqual(carte.epoque["id"], cv.epoque_de(self.ext_a)["id"])
        self.assertEqual(len(carte.vues_depuis(0.0, "balle")), 1)
        FausseVision().objets(None)
        self.assertEqual(len(carte.vues_depuis(0.0, "scotch")), 1)

    def test_vue_brute_cible_corrigee_quelle_que_soit_la_camera(self):
        carte = self._branche("REMONTEE")
        FausseVision().balle(None)
        carte.apprend((300.0, 20.0), (310.0, 20.0))
        xy, _ = FausseVision().balle(None)
        np.testing.assert_allclose(xy, [300.0, 20.0])
        np.testing.assert_allclose(Fenetre()._detecte_balle(), [310.0, 20.0], atol=0.01)
        fusion = Fenetre()
        fusion._camera_balle = "arducam+svpro"
        self.assertLess(np.linalg.norm(fusion._detecte_balle() - [310.0, 20.0]), 2.0)
        self.assertGreater(np.linalg.norm(fusion._detecte_balle() - [300.0, 20.0]), 8.0)

    def test_autres_objets_corriges_sur_l_arducam_seulement(self):
        carte = self._branche("REMONTEE")
        FausseVision().balle(None)
        carte.apprend((300.0, 20.0), (306.0, 17.0))
        (classe, xy, _), = FausseVision().objets(None)
        self.assertEqual(classe, "scotch")
        self.assertLess(np.linalg.norm(xy - [306.0, 17.0]), 1.0)
        svpro = FausseVision()
        svpro.source = "svpro_extrinsic_servo.yaml"
        np.testing.assert_allclose(svpro.objets(None)[0][1], [300.0, 20.0])

    def test_sans_armement_une_prise_n_apprend_rien(self):
        carte = self._branche("REMONTEE")
        ctx = self._ctx(xy=self.coin)
        self.assertEqual(fsm.ACTIONS["SAISIE"](ctx), "REMONTEE")
        self.assertNotIn("pro_gripper_open", ctx.pont.ordres)
        self.assertEqual(len(carte), 0)

    def _geste(self, carte, vues, xy=None, classe="balle"):
        def va_vers_et_la_balle_reapparait(*a, **kw):
            instant = cv.time.time() + cv.REPOS_BALLE_S + 0.05
            for v in vues:
                carte._vus.append((instant, classe, np.asarray(v, float)))
            return fsm.POSE_OBSERVATION.copy()

        FausseVision().balle(None)
        with mock.patch.object(fsm, "va_vers", return_value=fsm.POSE_OBSERVATION), \
                mock.patch.object(fsm, "resout_ik", return_value=(fsm.POSE_OBSERVATION,)), \
                mock.patch.object(fsm, "va_vers_par_etapes",
                                  side_effect=va_vers_et_la_balle_reapparait), \
                mock.patch.object(cv, "REPOS_BALLE_S", 0.0), \
                mock.patch.object(cv, "PATIENCE_RELECTURE_S", 0.5):
            ctx = self._ctx(classe=classe, xy=self.coin if xy is None else xy)
            return ctx, fsm.ACTIONS["SAISIE"](ctx)

    def test_test_de_saisie_n_importe_ou_une_seule_fois(self):
        carte = self._branche("REMONTEE", test=True)
        # Cible loin de la pointe simulee : le test s'arme n'importe ou. La balle
        # relue, elle, est la ou la pince l'a posee — sous la pointe.
        ailleurs = self.coin + [60.0, 45.0]
        vue = self.coin + [4.0, -1.0]
        ctx, suivant = self._geste(carte, [vue] * cv.VUES_RELECTURE, xy=ailleurs)
        self.assertEqual(suivant, "ATTENTE")
        self.assertIn("pro_gripper_open", ctx.pont.ordres)
        self.assertEqual(len(carte), 1)
        np.testing.assert_allclose(carte.echantillons[0]["vision"], vue, atol=0.01)
        ctx = self._ctx(xy=ailleurs)
        self.assertEqual(fsm.ACTIONS["SAISIE"](ctx), "REMONTEE")
        self.assertNotIn("pro_gripper_open", ctx.pont.ordres)

    def test_test_de_saisie_lit_la_reponse_du_lancement(self):
        import calibration_dialogue
        with mock.patch.object(calibration_dialogue, "TEST_SAISIE", True):
            carte = self._branche("REMONTEE", test=None)
        self.assertIsInstance(carte.armement, cv.TestSaisie)
        self.assertIs(Fenetre.show, self.originaux[Fenetre]["show"])

    def test_coin_appris_puis_cycle_arrete(self):
        carte = self._branche("REMONTEE", coins=[self.coin, (400.0, 100.0)])
        vue = self.coin + [3.0, -2.0]
        ctx, suivant = self._geste(carte, [vue] * cv.VUES_RELECTURE)
        self.assertEqual(suivant, "ATTENTE")
        self.assertIn("pro_gripper_open", ctx.pont.ordres)
        self.assertEqual(len(carte), 1)
        e = carte.echantillons[0]
        np.testing.assert_allclose(e["vision"], vue, atol=0.01)
        np.testing.assert_allclose(e["ecart"], [-3.0, 2.0], atol=0.01)
        self.assertEqual(e["mode"], "relache")
        self.assertIsNone(ctx.balle_xy)
        self.assertEqual(len(carte.tournee.manquants()), 1)
        ctx = self._ctx(xy=self.coin)
        self.assertEqual(fsm.ACTIONS["SAISIE"](ctx), "REMONTEE")
        self.assertNotIn("pro_gripper_open", ctx.pont.ordres)

    def test_balle_hors_coin_ne_relache_pas(self):
        carte = self._branche("REMONTEE", coins=[(400.0, 100.0)])
        FausseVision().balle(None)
        self.assertIsNotNone(carte.epoque)
        ctx = self._ctx(xy=self.coin)
        self.assertEqual(fsm.ACTIONS["SAISIE"](ctx), "REMONTEE")
        self.assertNotIn("pro_gripper_open", ctx.pont.ordres)
        self.assertEqual(len(carte), 0)

    def test_balle_qui_roule_arrete_sans_apprendre(self):
        carte = self._branche("REMONTEE", coins=[self.coin])
        vues = [self.coin + [i * 1.5, 0.0] for i in range(cv.VUES_RELECTURE)]
        _, suivant = self._geste(carte, vues)
        self.assertEqual(suivant, "ATTENTE")
        self.assertEqual(len(carte), 0)
        self.assertEqual(len(carte.tournee.manquants()), 1)

    def test_nouvelle_extrinseque_vide_la_tournee(self):
        carte = self._branche("REMONTEE", coins=[self.coin])
        self._geste(carte, [self.coin] * cv.VUES_RELECTURE)
        self.assertEqual(carte.tournee.manquants(), [])
        carte.regle_extrinseque(self.ext_b)
        self.assertEqual(len(carte.tournee.manquants()), 1)

    def test_echec_ne_relache_pas(self):
        carte = self._branche("DEGAGEMENT", test=True)
        FausseVision().balle(None)
        self.assertIsNotNone(carte.epoque)
        ctx = self._ctx(xy=self.coin)
        self.assertEqual(fsm.ACTIONS["SAISIE"](ctx), "DEGAGEMENT")
        self.assertNotIn("pro_gripper_open", ctx.pont.ordres)
        self.assertEqual(len(carte), 0)

    def test_test_de_saisie_chaque_objet_une_fois(self):
        carte = self._branche("REMONTEE", test=True)
        vue = self.coin + [2.0, 1.0]
        _, suivant = self._geste(carte, [vue] * cv.VUES_RELECTURE, classe="scotch")
        self.assertEqual(suivant, "ATTENTE")
        self.assertEqual(len(carte), 1)
        self.assertIsNone(carte.echantillons[0]["z_mm"])
        self.assertEqual(carte.armement.restant, ["balle", "robot"])
        ctx = self._ctx(classe="scotch", xy=self.coin)
        self.assertEqual(fsm.ACTIONS["SAISIE"](ctx), "REMONTEE")
        self.assertNotIn("pro_gripper_open", ctx.pont.ordres)
        _, suivant = self._geste(carte, [self.coin] * cv.VUES_RELECTURE, classe="balle")
        self.assertEqual(len(carte), 2)
        self.assertIsNotNone(carte.echantillons[-1]["z_mm"])

    def test_tournee_ne_relache_que_la_balle(self):
        carte = self._branche("REMONTEE", coins=[self.coin])
        FausseVision().balle(None)
        ctx = self._ctx(classe="scotch", xy=self.coin)
        self.assertEqual(fsm.ACTIONS["SAISIE"](ctx), "REMONTEE")
        self.assertNotIn("pro_gripper_open", ctx.pont.ordres)

    def test_coins_dessines(self):
        carte = self._branche("REMONTEE", coins=cv.COINS_TOURNEE)

        class Pixels:
            def vers_pixel(self, p_mm):
                return np.array([100.0 + p_mm[0] / 10, 200.0 + p_mm[1] / 10])

        image = np.zeros((480, 640, 3), np.uint8)
        cv.dessine_tournee(image, Pixels(), carte.tournee, carte.z_mm)
        self.assertGreater(int(image.sum()), 0)

    def test_coins_de_la_tournee_atteignables(self):
        for coin in cv.COINS_TOURNEE:
            self.assertTrue(fsm.PORTEE_MIN < np.hypot(*coin) < fsm.PORTEE_MAX, coin)


if __name__ == "__main__":
    unittest.main()
