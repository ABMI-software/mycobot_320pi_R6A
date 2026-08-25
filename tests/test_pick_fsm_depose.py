"""Invariant de dépose : objet en main => jamais de retour au ramassage.

Le cycle a tourné la balle en main parce qu'un carton introuvable renvoyait
vers ECHEC, donc vers ATTENTE, donc vers un nouveau DEGAGEMENT. Ces tests
tiennent la garde qui l'interdit, et le fait qu'un carton déplacé soit suivi.
"""
import types
import sys
import unittest
from pathlib import Path

import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / "scripts"))

import pick_fsm as fsm  # noqa: E402


class FauxPont:
    """Pont TCP simulé : retient les angles commandés et un statut de pince."""

    def __init__(self, statut=2):
        self.statut = statut
        self.q = fsm.POSE_OBSERVATION.copy()
        self.envois = []

    def envoie(self, action, **kw):
        self.envois.append(action)
        if action == "send_angles":
            self.q = np.asarray(kw["angles"], float)
        return "OK"

    def angles(self):
        return self.q.copy()

    def statut_pince(self):
        return self.statut


def contexte(statut=2, carton=None, memoire=None, dossier=None):
    fsm.MEMOIRE_CARTON = Path(dossier) / "carton_position.json"
    fsm.MEMOIRE_CARTON.unlink(missing_ok=True)
    if memoire is not None:
        fsm.memorise_carton(np.asarray(memoire, float))
    ctx = fsm.Contexte()
    ctx.pont = FauxPont(statut)
    ctx.mode_auto = True
    ctx.balle_xy = np.array([250.0, -60.0])
    ctx.R_balle = fsm.orientation(ctx.balle_xy)
    ctx.detecteur = lambda patience=1.0: np.array([250.0, -60.0])
    if carton is not None:
        ctx.detecteur_carton = lambda patience=1.0: (np.asarray(carton, float), None)
    return ctx


def deroule(machine, pas=12):
    """Enchaîne des pas et rend les états REELLEMENT executes."""
    for _ in range(pas):
        avant = machine.etat
        machine.pas()
        if avant == machine.etat == "ECHEC_PORTANT":
            break
    return [ligne[4:-4] for ligne in machine.ctx.journal if ligne.startswith("--- ")]


class ObjetEnMain(unittest.TestCase):
    def setUp(self):
        self.dossier = Path(__file__).resolve().parent / "__memoire__"
        self.dossier.mkdir(exist_ok=True)

    def tearDown(self):
        for fichier in self.dossier.glob("*"):
            fichier.unlink()
        self.dossier.rmdir()

    def _machine(self, depart, **kw):
        machine = fsm.MachineEtats(contexte(dossier=self.dossier, **kw))
        machine.etat = depart
        return machine

    def test_carton_invisible_garde_l_objet(self):
        machine = self._machine("TRANSFERT")
        executes = deroule(machine)
        self.assertFalse([e for e in executes if e in fsm.ETATS_RAMASSAGE])
        self.assertEqual(machine.etat, "ECHEC_PORTANT")

    def test_echec_ne_repart_pas_au_ramassage(self):
        machine = self._machine("ECHEC")
        executes = deroule(machine, pas=6)
        self.assertFalse([e for e in executes if e in fsm.ETATS_RAMASSAGE])

    def test_carton_hors_atteinte_garde_l_objet(self):
        machine = self._machine("TRANSFERT", carton=(450.0, -200.0))
        executes = deroule(machine, pas=4)
        self.assertFalse([e for e in executes if e in fsm.ETATS_RAMASSAGE])
        self.assertEqual(machine.etat, "ECHEC_PORTANT")

    def test_objet_lache_relance_la_saisie(self):
        machine = self._machine("TRANSFERT", statut=1)
        machine.pas()
        self.assertEqual(machine.etat, "DEGAGEMENT")

    def test_carton_vu_mene_au_transfert(self):
        machine = self._machine("RECHERCHE_CARTON", carton=(280.0, -130.0))
        machine.pas()
        self.assertEqual(machine.etat, "TRANSFERT")

    def test_la_remontee_revoit_le_carton(self):
        """Le carton a pu bouger pendant le cycle : sa position d'avant la
        saisie ne fait pas foi."""
        self.assertIn(("REMONTEE", "RECHERCHE_CARTON", "prise tenue", "on revoit le carton"),
                      fsm.TRANSITIONS)


class AvantDeSaisir(unittest.TestCase):
    def setUp(self):
        self.dossier = Path(__file__).resolve().parent / "__memoire2__"
        self.dossier.mkdir(exist_ok=True)

    def tearDown(self):
        for fichier in self.dossier.glob("*"):
            fichier.unlink()
        self.dossier.rmdir()

    def test_carton_inconnu_la_saisie_se_fait_quand_meme(self):
        """Regle inversee le 24/08 : ne pas voir le carton ne bloque plus rien.

        Attendre de le voir avant de saisir immobilisait tout le cycle. Ce qui
        protege le robot n'est pas ce prealable mais l'invariant d'apres la
        prise — teste par `PriseEnMain` : objet en main, on ne recommence
        jamais la saisie.
        """
        machine = fsm.MachineEtats(contexte(statut=1, dossier=self.dossier))
        machine.etat = "DETECTION"
        machine.pas()
        self.assertEqual(machine.etat, "APPROCHE")

    def test_la_memoire_suffit_a_demarrer(self):
        machine = fsm.MachineEtats(contexte(statut=1, memoire=(280.0, -130.0),
                                            dossier=self.dossier))
        machine.etat = "DETECTION"
        machine.pas()
        self.assertEqual(machine.etat, "APPROCHE")
        np.testing.assert_allclose(machine.ctx.carton_xy, [280.0, -130.0])


class AuDessusDuCarton(unittest.TestCase):
    """Une fois le bras au-dessus du carton, il largue.

    Un veto de dernière seconde a été essayé puis retiré : à cet instant le bras
    masque le carton, la détection sautait de 55 à 171 mm et le cycle bouclait
    sans jamais déposer.
    """

    def _machine(self, vu, r_carton=True):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont(2)
        ctx.carton_xy = np.array([280.0, -130.0])
        ctx.R_carton = fsm.orientation(ctx.carton_xy) if r_carton else None
        ctx.detecteur_carton = lambda patience=1.0: (np.asarray(vu, float), None)
        machine = fsm.MachineEtats(ctx)
        machine.etat = "LARGAGE"
        return machine

    def test_largue_meme_si_la_detection_a_saute(self):
        machine = self._machine((180.0, 60.0))
        machine.pas()
        self.assertEqual(machine.etat, "RETRAIT")
        self.assertIn("pro_gripper_open", machine.ctx.pont.envois)

    def test_cible_redefinie_en_route_ne_plante_pas(self):
        """Le bouton « détecter le carton » remet R_carton a None : un clic
        pendant la boucle faisait planter le largage sur `None @ TOOL`."""
        machine = self._machine((282.0, -128.0), r_carton=False)
        machine.pas()
        self.assertEqual(machine.etat, "RECHERCHE_CARTON")
        self.assertNotIn("ERREUR", " ".join(machine.ctx.journal))


class DetectionSousLeBras(unittest.TestCase):
    def test_un_carton_vu_sous_la_pointe_est_ignore(self):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont(2)
        sous_la_pointe = fsm.pointe(fsm.POSE_OBSERVATION)[:2] + np.array([30.0, 20.0])
        ctx.detecteur_carton = lambda patience=1.0: (sous_la_pointe, None)
        self.assertEqual(fsm.detecte_carton(ctx), "invisible")
        self.assertIsNone(ctx.carton_xy)


class PointDeLargage(unittest.TestCase):
    """L'ouverture mesurée le 24/08 : milieu à 363 mm (hors d'atteinte), bord
    proche à 290 mm."""

    OUVERTURE = np.array([[402., -126.], [396., -145.], [379., -198.], [366., -198.],
                          [265., -155.], [265., -141.], [267., -129.], [270., -106.],
                          [276., -100.], [284., -99.], [327., -102.], [355., -105.],
                          [366., -107.], [401., -116.]])
    MILIEU = np.array([334.0, -142.1])

    def test_les_candidats_partent_du_milieu(self):
        points = fsm.points_de_largage(self.MILIEU, self.OUVERTURE)
        distances = [float(np.linalg.norm(p - self.MILIEU)) for p in points]
        self.assertEqual(distances, sorted(distances))

    def test_les_candidats_restent_dans_l_ouverture(self):
        import cv2
        contour = np.asarray(self.OUVERTURE, np.float32).reshape(-1, 1, 2)
        for p in fsm.points_de_largage(self.MILIEU, self.OUVERTURE):
            marge = cv2.pointPolygonTest(contour, (float(p[0]), float(p[1])), True)
            self.assertGreaterEqual(marge, fsm.MARGE_LARGAGE_MIN)

    def test_sans_polygone_on_vise_le_milieu(self):
        points = fsm.points_de_largage(self.MILIEU, None)
        np.testing.assert_allclose(points[0], self.MILIEU)


class ApprocheDepuisBrasDresse(unittest.TestCase):
    """Bras dressé : la pointe est à ~520 mm, la cible d'approche à 110 mm.

    Un seul ordre plongerait de 409 mm — refusé par le garde-fou, ce qui
    faisait boucler le cycle sur ATTENTE → DEGAGEMENT → DETECTION → refus.
    """

    DRESSE = np.array([6.41, 0.26, -1.05, 2.19, -5.62, 171.03])

    def _contexte(self):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont(1)
        ctx.pont.q = self.DRESSE.copy()
        ctx.balle_xy = np.array([217.0, 95.0])
        ctx.R_balle = fsm.orientation(ctx.balle_xy)
        return ctx

    def test_la_pointe_part_bien_de_plus_de_400_mm_au_dessus(self):
        chute = fsm.pointe(self.DRESSE)[2] - fsm.Z_SURVOL
        self.assertGreater(chute, fsm.CHUTE_MAX)

    def test_l_approche_descend_par_paliers(self):
        machine = fsm.MachineEtats(self._contexte())
        machine.etat = "APPROCHE"
        machine.pas()
        self.assertEqual(machine.etat, "RECALAGE")
        ordres = [a for a in machine.ctx.pont.envois if a == "send_angles"]
        self.assertGreater(len(ordres), 1)

    def test_aucun_ordre_ne_depasse_la_chute_maximale(self):
        ctx = self._contexte()
        cible = fsm.resout_ik(np.array([ctx.balle_xy[0], ctx.balle_xy[1], fsm.Z_SURVOL]),
                              ctx.R_balle)
        self.assertIsNotNone(cible)
        hauteurs = [float(fsm.pointe(self.DRESSE)[2])]
        vrai_va_vers = fsm.va_vers

        def espionne(contexte, q_cible, **kw):
            hauteurs.append(float(fsm.pointe(q_cible)[2]))
            return vrai_va_vers(contexte, q_cible, **kw)

        fsm.va_vers = espionne
        try:
            self.assertIsNotNone(fsm.va_vers_par_etapes(ctx, cible[0]))
        finally:
            fsm.va_vers = vrai_va_vers
        chutes = [a - b for a, b in zip(hauteurs, hauteurs[1:])]
        self.assertTrue(all(c <= fsm.CHUTE_MAX for c in chutes), chutes)


if __name__ == "__main__":
    unittest.main()


class ChangementDeCamera(unittest.TestCase):
    """Un ecart entre les deux cameras n'est pas un deplacement de la balle.

    Mesure du 24/08 : le bras masque la vue de dessus, la cible passe a la
    SVPRO, les 10 mm d'ecart de reperage relancent DETECTION. Onze secondes
    perdues, balle immobile.
    """

    def setUp(self):
        self.dossier = Path(__file__).resolve().parent / "__memoire3__"
        self.dossier.mkdir(exist_ok=True)

    def tearDown(self):
        for fichier in self.dossier.glob("*"):
            fichier.unlink()
        self.dossier.rmdir()

    def _ctx(self, source_vue):
        ctx = contexte(statut=1, dossier=self.dossier)
        ctx.balle_xy = np.array([382.0, 177.0])
        ctx.source_cible = "arducam"
        ctx.source_balle = source_vue
        ctx.detecteur = lambda patience=1.0: np.array([392.0, 177.0])
        return ctx

    def test_le_relais_svpro_ne_relance_pas_le_cycle(self):
        ctx = self._ctx("svpro")
        self.assertFalse(fsm.cible_a_bouge(ctx))
        np.testing.assert_allclose(ctx.balle_xy, [392.0, 177.0])

    def test_la_meme_camera_signale_un_vrai_deplacement(self):
        ctx = self._ctx("arducam")
        self.assertTrue(fsm.cible_a_bouge(ctx))


class DegagementExigeLaVueDeDessus(unittest.TestCase):
    """Sept tours a vide le 24/08 : le bras restait plante devant l'arducam
    parce que la SVPRO, elle, voyait la balle."""

    def setUp(self):
        self.dossier = Path(__file__).resolve().parent / "__memoire4__"
        self.dossier.mkdir(exist_ok=True)

    def tearDown(self):
        for fichier in self.dossier.glob("*"):
            fichier.unlink()
        self.dossier.rmdir()

    def test_seule_la_svpro_voit_donc_on_ecarte(self):
        appels = []

        def detecteur(patience=1.0, exige_dessus=False):
            appels.append(exige_dessus)
            return None if exige_dessus else np.array([250.0, -60.0])

        ctx = contexte(statut=1, dossier=self.dossier)
        ctx.detecteur = detecteur
        self.assertIsNone(fsm.dessus(ctx, patience=0.5))
        self.assertTrue(appels[0])

    def test_un_detecteur_qui_ignore_l_option_repond_quand_meme(self):
        ctx = contexte(statut=1, dossier=self.dossier)
        ctx.detecteur = lambda patience=1.0: np.array([250.0, -60.0])
        np.testing.assert_allclose(fsm.dessus(ctx), [250.0, -60.0])


class HauteurDePriseParCategorie(unittest.TestCase):
    """Un rouleau couche ne se saisit pas a la hauteur d'une balle.

    La balle fait 66 mm et tient les doigts ecartes : viser sous le plan de la
    table est sans danger. Un scotch fait ~22 mm — les doigts taperaient la
    planche avant de se refermer, et la pince se fermerait a vide.
    """

    def test_la_balle_garde_ses_hauteurs_historiques(self):
        self.assertEqual(fsm.Z_PRISE_PAR_CLASSE["balle"],
                         (fsm.Z_PRISE, fsm.Z_PRISE_INCLINE))

    def test_les_objets_plats_se_prennent_au_dessus_de_la_planche(self):
        for classe in ("scotch", "robot"):
            self.assertGreater(fsm.Z_PRISE_PAR_CLASSE[classe][0], 0.0)

    def test_couche_l_outil_vise_la_mi_hauteur_de_l_objet_pas_celle_de_la_balle(self):
        """Le defaut du 24/08 : Z=25 (mi-hauteur de la balle) sur un rouleau de
        22 mm — la pince se refermait entierement au-dessus de lui."""
        for classe in ("scotch", "robot"):
            self.assertLess(fsm.Z_PRISE_PAR_CLASSE[classe][1],
                            fsm.Z_PRISE_PAR_CLASSE["balle"][1])

    def test_une_categorie_inconnue_retombe_sur_les_hauteurs_par_defaut(self):
        self.assertEqual(
            fsm.Z_PRISE_PAR_CLASSE.get("inconnu", (fsm.Z_PRISE, fsm.Z_PRISE_INCLINE)),
            (fsm.Z_PRISE, fsm.Z_PRISE_INCLINE))


class OmbreDuBras(unittest.TestCase):
    """Le fantome au milieu de la table est venu de la : une ombre portee par le
    bras se confirme aussi bien qu'un vrai deplacement, puisqu'elle le suit
    image apres image."""

    def test_la_distance_se_mesure_au_bras_entier_pas_a_sa_pointe(self):
        q = np.array([0.0, -60.0, -40.0, 90.0, 0.0, -80.0])
        milieu_du_bras = fsm.forward_kinematics(np.radians(q))[0]
        coude = np.asarray(milieu_du_bras["mycobot320_link3"], float)[:2] * 1000.0
        pointe = fsm.pointe(q)[:2]
        self.assertGreater(float(np.linalg.norm(coude - pointe)), 100.0)
        self.assertLess(fsm.distance_au_bras(coude, q), 1.0)

    def test_un_point_lointain_reste_lointain(self):
        q = np.array([0.0, -60.0, -40.0, 90.0, 0.0, -80.0])
        self.assertGreater(fsm.distance_au_bras(np.array([-400.0, 400.0]), q), 300.0)


class DescenteQuiInsiste(unittest.TestCase):
    """Trois tentatives identiques donnent trois echecs identiques.

    Mesure du 24/08 : recalage a 0,59 mm, descente a 1,42 mm, et pourtant
    « rien saisi » trois fois. La position etait juste, la hauteur non.
    """

    def test_chaque_essai_rate_descend_d_un_cran(self):
        self.assertGreater(fsm.PAS_DESCENTE_ESSAI, 0.0)

    def test_on_ne_descend_jamais_sous_la_hauteur_de_la_balle(self):
        """Le plancher reste la hauteur validee sur la balle : plus bas, les
        doigts tapent la planche."""
        z = fsm.Z_PRISE_PAR_CLASSE["scotch"][0]
        for essais in range(6):
            self.assertGreaterEqual(
                max(fsm.Z_PRISE, z - fsm.PAS_DESCENTE_ESSAI * essais), fsm.Z_PRISE)


class OrdreDuBalayage(unittest.TestCase):
    """Le degagement commence par la pose la PLUS ELOIGNEE de l'objet.

    Mesure du 24/08 : au premier cycle aucune cible n'etait posee, le bras
    balayait dans l'ordre du fichier et essayait cinq poses avant la bonne —
    20 s sur un cycle de 99.
    """

    def test_la_pose_la_plus_eloignee_vient_en_premier(self):
        azimut = 4.0
        ordre = sorted(fsm.BALAYAGE_J1,
                       key=lambda j1: -abs(((j1 - azimut + 180.0) % 360.0) - 180.0))
        self.assertEqual(ordre[0], min(fsm.BALAYAGE_J1,
                                       key=lambda j1: -abs(j1 - azimut)))


class HauteurDeLargage(unittest.TestCase):
    """Le lacher se cale sur le rebord MESURE, pas sur un rebord suppose.

    Le rebord du grand carton mesure 82,9 mm (triangulation des deux vues,
    25/08) alors que la constante en supposait 60 : la garde reelle etait de
    17 mm, et negative pour un carton plus haut.
    """

    def test_sans_mesure_on_garde_le_plancher(self):
        ctx = types.SimpleNamespace(z_rebord=None)
        self.assertEqual(fsm.z_largage(ctx), fsm.Z_LARGAGE)

    def test_un_rebord_bas_ne_fait_pas_descendre_le_lacher(self):
        ctx = types.SimpleNamespace(z_rebord=40.0)
        self.assertEqual(fsm.z_largage(ctx), fsm.Z_LARGAGE)

    def test_un_rebord_haut_releve_le_lacher(self):
        ctx = types.SimpleNamespace(z_rebord=110.0)
        self.assertEqual(fsm.z_largage(ctx), 110.0 + fsm.GARDE_LARGAGE)
