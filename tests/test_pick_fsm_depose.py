"""Invariant de dépose : objet en main => jamais de retour au ramassage.

Le cycle a tourné la balle en main parce qu'un carton introuvable renvoyait
vers ECHEC, donc vers ATTENTE, donc vers un nouveau DEGAGEMENT. Ces tests
tiennent la garde qui l'interdit, et le fait qu'un carton déplacé soit suivi.
"""
import types
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / "scripts"))

import pick_fsm as fsm  # noqa: E402


class FauxPont:
    """Pont TCP simulé : retient les angles commandés et un statut de pince."""

    def __init__(self, statut=2):
        self.statut = statut
        self.q = fsm.POSE_OBSERVATION.copy()
        self.angle = 100          # la pince demarre ouverte, comme au reveil du bras
        self.envois = []

    def envoie(self, action, **kw):
        self.envois.append(action)
        if action == "send_angles":
            self.q = np.asarray(kw["angles"], float)
        if action == "pro_gripper_open":
            self.angle = 100
        if action == "pro_gripper_angle":
            # La pince CALE sur l'objet : elle n'atteint la consigne que si elle
            # se referme sur du vide. `cale` est cette butée mesurée.
            self.angle = max(int(kw["angle"]), getattr(self, "cale", 0))
        return "OK"

    def angles(self):
        return self.q.copy()

    def statut_pince(self):
        return self.statut

    def angle_pince(self):
        return self.angle


def contexte(statut=2, carton=None, memoire=None, dossier=None):
    fsm.MEMOIRE_CARTON = Path(dossier) / "carton_position.json"
    fsm.MEMOIRE_CARTON.unlink(missing_ok=True)
    ctx = fsm.Contexte()
    if memoire is not None:
        # Sous le nom du carton VISE : une position sans nom ne dit pas de
        # quelle boite elle parle, et le repli y envoyait le mauvais objet.
        fsm.memorise_carton(ctx.carton_vise, np.asarray(memoire, float))
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

    def test_la_position_des_cartons_n_est_JAMAIS_persistee(self):
        """Les boîtes se déplacent entre deux séances ; une position écrite sur
        disque, elle, survit à ce déplacement.

        Le 26/08 « grand » valait encore (428, -97) — du côté du PETIT, à
        320 mm du vrai grand — et la balle est allée s'y poser, sur la planche
        entre les deux boîtes. Pendant une séance les boîtes ne bougent pas et
        le suivi caméra tient leur position en mémoire vive : c'est la bonne
        portée pour cette information. Le roulis appris, lui, est une propriété
        du BRAS et non de la scène : il reste persisté.
        """
        fsm.memorise_carton("grand", np.array([280.0, -130.0]),
                            {"carton@375": 0})
        ecrit = __import__("json").loads(fsm.MEMOIRE_CARTON.read_text())
        self.assertNotIn("cartons", ecrit)
        self.assertEqual(ecrit["roulis_appris"], {"carton@375": 0})
        self.assertIsNone(fsm.carton_memorise(classe="grand"))

    def test_le_roulis_appris_est_bien_recharge(self):
        """Il vaut 10 s sur le premier cycle d'une séance : lui reste persisté."""
        ctx = contexte(dossier=self.dossier)          # remet le fichier a zero
        fsm.memorise_carton("grand", np.array([280.0, -130.0]),
                            {"carton@375": 30})
        ctx.roulis_appris.clear()
        fsm.carton_memorise(ctx)
        self.assertEqual(ctx.roulis_appris.get("carton@375"), 30)


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

    def test_la_balle_se_saisit_dans_le_palier_mesure(self):
        """La hauteur COMMANDEE, biais compris, doit tomber dans le palier.

        Balayage du 26/08 : la pince cale a 53 de +10 a -5, a 56 a -10, et a 74
        a -15 — la ou les doigts butent sur la planche et non sur la balle. Le
        critere n'est donc pas « ca saisit » (tout saisissait) mais « l'angle de
        calage ne se degrade pas ».
        """
        commande = fsm.Z_PRISE_PAR_CLASSE["balle"][0] + fsm.BIAIS_Z_PRISE
        self.assertGreaterEqual(commande, -5.0)
        self.assertLessEqual(commande, 10.0)
        self.assertEqual(fsm.Z_PRISE_PAR_CLASSE["balle"][1], fsm.Z_PRISE_INCLINE)

    def test_le_rouleau_ne_vise_PAS_le_bas_du_palier_mesure(self):
        """Le palier -8..-20 a été mesuré à 271-280 mm et NE TRANSFÈRE PAS.

        Descendre au milieu de ce palier (-14) semblait la bonne lecture. Essayé
        le 26/08 sur un rouleau à 312 mm : -14, -18 et -20 se referment sur du
        VIDE, angle 20, et les doigts finissent par toucher la planche. Le biais
        du modèle croît avec l'allonge (5,9 mm à 206, 11,9 à 327, 17,9 à 370),
        donc à allonge plus grande la même consigne descend réellement plus bas.
        On tient donc le HAUT du palier, et ce sont les reprises de `_saisie`
        qui vont chercher plus bas quand il le faut.
        """
        commande = fsm.Z_PRISE_PAR_CLASSE["scotch"][0] + fsm.BIAIS_Z_PRISE
        self.assertEqual(commande, -8.0)
        self.assertGreaterEqual(commande, fsm.Z_PRISE_MIN)

    def test_le_petit_robot_reste_dans_ce_qui_a_ete_execute(self):
        """Son palier n'a PAS été balayé — contrairement à la balle et au
        rouleau. On ne verrouille donc que ce qui a réellement été exécuté :
        saisi à 177 et 220 mm avec cette valeur, et jamais sous le plancher."""
        commande = fsm.Z_PRISE_PAR_CLASSE["robot"][0] + fsm.BIAIS_Z_PRISE
        self.assertEqual(commande, -8.0)
        self.assertGreaterEqual(commande, fsm.Z_PRISE_MIN)

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


class CoupleDeLaPince(unittest.TestCase):
    """On ne serre PAS plus fort le petit robot : c'est une piece imprimee.

    Le couple avait ete monte a 250 pour lui le 25/08, puis redescendu — a
    maillons fins, il casserait. Son lachage pendant la remontee vient de
    l'endroit ou la pince se refermait, pas d'un manque de force.
    """

    def test_le_robot_n_est_pas_serre_plus_fort(self):
        self.assertEqual(fsm.COUPLE_PINCE.get('robot', fsm.COUPLE_PINCE_DEFAUT),
                         fsm.COUPLE_PINCE_DEFAUT)

    def test_le_couple_reste_dans_ce_qu_accepte_le_pont(self):
        for couple in list(fsm.COUPLE_PINCE.values()) + [fsm.COUPLE_PINCE_DEFAUT]:
            self.assertGreaterEqual(couple, 100)
            self.assertLessEqual(couple, 300)

    def test_la_balle_garde_le_defaut(self):
        self.assertEqual(fsm.COUPLE_PINCE.get('balle', fsm.COUPLE_PINCE_DEFAUT),
                         fsm.COUPLE_PINCE_DEFAUT)

    def test_le_rouleau_se_serre_plus_fort_que_le_petit_robot(self):
        """Le rouleau est un objet solide ; le petit robot est une pièce
        imprimée à maillons fins, montée à 250 le 25/08 puis redescendue le même
        jour. Le serrage ne doit pas les traiter pareil."""
        self.assertGreater(fsm.COUPLE_PINCE['scotch'], fsm.COUPLE_PINCE_DEFAUT)
        self.assertEqual(fsm.COUPLE_PINCE.get('robot', fsm.COUPLE_PINCE_DEFAUT),
                         fsm.COUPLE_PINCE_DEFAUT)


class ObjetMisDeCote(unittest.TestCase):
    """Trois saisies a vide => on passe a un autre objet, pas au meme.

    Le 25/08 la machine a repris cycle apres cycle le meme objet — d'abord un
    faux scotch a 252 mm (c'etait le bras), puis un rouleau que la pince ne
    pouvait pas enserrer. A chaque tour elle rechoisissait "le plus proche",
    c'est-a-dire celui qui venait d'echouer, pendant que deux autres objets
    attendaient. Un echec definitif doit ecarter la CIBLE, pas seulement le
    cycle.
    """

    def test_la_position_epuisee_est_mise_de_cote(self):
        ctx = fsm.Contexte()
        xy = np.array([252.0, -40.0])
        self.assertFalse(ctx.est_oublie(xy))
        ctx.oublie(xy)
        self.assertTrue(ctx.est_oublie(xy))

    def test_un_objet_voisin_est_le_meme_objet(self):
        ctx = fsm.Contexte()
        ctx.oublie(np.array([252.0, -40.0]))
        bouge = np.array([252.0 + fsm.RAYON_OUBLI - 5.0, -40.0])
        self.assertTrue(ctx.est_oublie(bouge))

    def test_un_autre_objet_reste_saisissable(self):
        ctx = fsm.Contexte()
        ctx.oublie(np.array([252.0, -40.0]))
        autre = np.array([252.0 + 3 * fsm.RAYON_OUBLI, -40.0])
        self.assertFalse(ctx.est_oublie(autre))

    def test_l_oubli_expire(self):
        """La scene change : l'operateur peut reposer un objet a cet endroit."""
        ctx = fsm.Contexte()
        xy = np.array([252.0, -40.0])
        ctx.oublie(xy)
        ctx.oublies = [(p, t - fsm.DUREE_OUBLI - 1.0) for p, t in ctx.oublies]
        self.assertFalse(ctx.est_oublie(xy))

    def test_la_saisie_epuisee_ecarte_la_cible_et_rouvre_la_pince(self):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont(statut=1)          # 1 = arrete, rien saisi
        ctx.balle_xy = np.array([252.0, -40.0])
        ctx.essais['SAISIE'] = fsm.ESSAIS_MAX - 1
        self.assertEqual(fsm._saisie(ctx), 'RETRAIT')
        self.assertTrue(ctx.est_oublie(ctx.balle_xy))
        self.assertIn('pro_gripper_open', ctx.pont.envois)


class BrasEnAppui(unittest.TestCase):
    """Un bras pose sur la planche doit se relever seul.

    `va_vers` refuse tout trajet qui creuse de plus de 2 mm sous le point le
    plus bas des deux bouts. Depuis le contact, TOUT trajet interpole creuse :
    la machine bouclait sur « creux a -13.9 mm sous le seuil -6.6 » sans que
    rien ne la releve — trois lancements de suite le 25/08.
    """

    def _pose_en_appui(self):
        return np.array([88.8, -18.7, -127.2, -30.7, 27.8, 86.0])

    def test_la_pose_de_depart_est_bien_en_appui(self):
        self.assertLess(fsm.garde_au_sol(self._pose_en_appui()), 0.0)

    def test_le_relevage_degage_franchement(self):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont()
        ctx.pont.q = self._pose_en_appui()
        self.assertTrue(fsm.degage_du_sol(ctx))
        self.assertGreaterEqual(fsm.garde_au_sol(ctx.pont.angles()),
                                fsm.GARDE_DEGAGEE - 1.0)

    def test_le_chemin_du_relevage_ne_descend_jamais(self):
        q0 = self._pose_en_appui()
        ctx = fsm.Contexte()
        ctx.pont = FauxPont()
        ctx.pont.q = q0.copy()
        fsm.degage_du_sol(ctx)
        q1 = ctx.pont.angles()
        creux = min(fsm.garde_au_sol(q0 * (1 - t) + q1 * t)
                    for t in np.linspace(0, 1, 61))
        self.assertGreaterEqual(creux, fsm.garde_au_sol(q0) - 1e-6)

    def test_un_bras_deja_haut_ne_bouge_pas(self):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont()
        avant = ctx.pont.angles()
        self.assertTrue(fsm.degage_du_sol(ctx))
        np.testing.assert_allclose(ctx.pont.angles(), avant)


class InventaireCoche(unittest.TestCase):
    """Un largage reussi est un fait acquis — il ne se redetecte pas.

    Le test geometrique « l'objet est-il dans l'ouverture ? » ne tient que tant
    que le carton reste visible. Le bras qui revient le masque, l'ouverture
    disparait de l'image, et la balle deja deposee redevient une cible : le
    cycle repartait la chercher au fond de la boite. Le compteur, lui, ne
    recule pas.
    """

    LARGAGE = np.array([311.0, 167.0])   # point reel du grand carton, 25/08

    def setUp(self):
        self.dossier = Path(__file__).resolve().parent / "__memoire__"
        self.dossier.mkdir(exist_ok=True)
        # La geometrie du largage est testee ailleurs ; ici seul compte le fait
        # qu'une descente REUSSIE coche la categorie.
        # `va_vers` est court-circuite, mais il DEPLACE le faux bras : depuis le
        # 26/08 le largage verifie qu'il est arrive avant d'ouvrir la pince, et
        # un bras reste a la pose d'observation n'est jamais au-dessus du carton.
        self._ik, self._va = fsm.resout_ik, fsm.va_vers

        def va(ctx, q, **kw):
            ctx.pont.q = np.asarray(q, float)
            return ctx.pont.angles()

        fsm.va_vers = va

    def tearDown(self):
        fsm.resout_ik, fsm.va_vers = self._ik, self._va

    def _contexte_au_largage(self):
        ctx = contexte(carton=tuple(self.LARGAGE), dossier=self.dossier)
        ctx.classe_objet, ctx.carton_vise = 'balle', 'grand'
        # PORTE l'objet : depuis le 27/08 le largage ne coche que ce que la
        # machine a effectivement soulevé (`ctx.en_main`).
        ctx.en_main = 'balle'
        ctx.carton_xy = self.LARGAGE.copy()
        ctx.z_rebord = 83.0
        # L'orientation RESOLUE, pas le roulis nominal : a 353 mm l'outil tenu
        # droit ne se resout pas (il plafonne a 355), c'est le roulis +60 qui
        # passe. Un fixture qui ne se resout pas ne teste que lui-meme.
        _, _, ctx.R_carton = fsm.choisit_pose_largage(ctx, ctx.carton_xy)
        return ctx

    def test_le_largage_coche_la_categorie(self):
        ctx = self._contexte_au_largage()
        self.assertEqual(ctx.deposes.get('balle', 0), 0)
        self.assertEqual(fsm._largage(ctx), 'RETRAIT')
        self.assertEqual(ctx.deposes['balle'], 1)

    def test_deux_largages_comptent_deux(self):
        ctx = self._contexte_au_largage()
        ctx.classe_objet = ctx.en_main = 'scotch'
        fsm._largage(ctx)
        ctx.en_main = 'scotch'            # le largage vide la main, on la remplit
        fsm._largage(ctx)
        self.assertEqual(ctx.deposes['scotch'], 2)

    def test_une_descente_refusee_ne_coche_rien(self):
        ctx = self._contexte_au_largage()
        fsm.va_vers = lambda ctx, q, **k: None
        self.assertEqual(fsm._largage(ctx), 'ECHEC_PORTANT')
        self.assertEqual(ctx.deposes.get('balle', 0), 0)


class LaClasseNeChangePasPinceFermee(unittest.TestCase):
    """Un objet TENU garde sa classe, donc son carton de destination.

    Le 26/08 les deux rouleaux sont partis dans le grand carton : le
    dégagement effaçait `classe_objet` pince fermée, et la destination se
    recalculait ensuite depuis un objet resté sur la planche — le rouleau tenu
    est parti sous le nom « balle » puis sous le nom « robot ».
    """

    def setUp(self):
        self.dossier = __import__('tempfile').mkdtemp()
        self._va, self._degage = fsm.va_vers_par_etapes, fsm.degage_du_sol
        fsm.va_vers_par_etapes = lambda ctx, q, **k: np.asarray(q, float)
        fsm.degage_du_sol = lambda ctx: None

    def tearDown(self):
        fsm.va_vers_par_etapes, fsm.degage_du_sol = self._va, self._degage

    def _contexte(self, statut):
        ctx = contexte(statut=statut, dossier=self.dossier)
        ctx.classe_objet, ctx.carton_vise = 'scotch', 'petit'
        ctx.detecteur = lambda **k: np.array([250.0, -60.0])
        return ctx

    def test_pince_fermee_la_classe_survit_au_degagement(self):
        ctx = self._contexte(statut=2)
        ctx.en_main = 'scotch'
        fsm._degagement(ctx)
        self.assertEqual(ctx.classe_objet, 'scotch')
        self.assertEqual(ctx.carton_vise, 'petit')

    def test_drapeau_perdu_mais_pince_pleine_la_classe_survit(self):
        """`porte_objet` prime : il a rendu un faux négatif à la remontée."""
        ctx = self._contexte(statut=2)
        ctx.en_main = ''
        fsm._degagement(ctx)
        self.assertEqual(ctx.classe_objet, 'scotch')
        self.assertEqual(ctx.en_main, 'scotch')

    def test_pince_vide_la_classe_est_bien_oubliee(self):
        ctx = self._contexte(statut=1)
        ctx.en_main = ''
        fsm._degagement(ctx)
        self.assertEqual(ctx.classe_objet, '')


class UneCibleIntrouvableEstAbandonnee(unittest.TestCase):
    """Sinon ATTENTE la rechoisit et le même dégagement recommence sans fin."""

    def setUp(self):
        self.dossier = __import__('tempfile').mkdtemp()
        self._va, self._degage = fsm.va_vers_par_etapes, fsm.degage_du_sol
        fsm.va_vers_par_etapes = lambda ctx, q, **k: np.asarray(q, float)
        fsm.degage_du_sol = lambda ctx: None

    def tearDown(self):
        fsm.va_vers_par_etapes, fsm.degage_du_sol = self._va, self._degage

    def test_invisible_partout_la_cible_est_oubliee(self):
        ctx = contexte(statut=1, dossier=self.dossier)
        ctx.classe_objet, ctx.en_main = 'balle', ''
        perdue = ctx.balle_xy.copy()
        ctx.detecteur = lambda **k: None
        self.assertEqual(fsm._degagement(ctx), 'ATTENTE')
        self.assertTrue(ctx.est_oublie(perdue))
        self.assertEqual(ctx.classe_objet, '')


class LAngleTemoigneQuandLeStatutMent(unittest.TestCase):
    """Le statut a annoncé « objet lâché » le petit robot dans les doigts.

    La machine est repartie chercher un rouleau en le tenant, puis l'a déposé
    dans le petit carton sous le nom « scotch ». L'angle, lui, ne s'est pas
    trompé : à vide la pince lit exactement la consigne (20), sur un objet elle
    cale plus haut, et ouverte elle lit 100 — d'où une bande, pas un seuil.
    """

    def _pont(self, statut, angle):
        pont = FauxPont(statut)
        pont.angle = angle
        return pont

    def _ctx(self, statut, angle):
        ctx = fsm.Contexte()
        ctx.pont = self._pont(statut, angle)
        # Ces scénarios surviennent tous APRÈS un ordre de fermeture — c'est la
        # seule situation où l'angle a valeur de témoin.
        ctx.pince_fermee = True
        return ctx

    def test_statut_2_suffit(self):
        self.assertTrue(fsm.porte_objet(self._ctx(2, 20)))

    def test_statut_faux_negatif_mais_pince_calee_sur_objet(self):
        for angle in (24, 30, 53, 74):
            with self.subTest(angle=angle):
                self.assertTrue(fsm.porte_objet(self._ctx(3, angle)))

    def test_fermee_a_vide_sur_la_consigne_exacte(self):
        self.assertFalse(fsm.porte_objet(self._ctx(1, 20)))

    def test_pince_ouverte_ne_tient_rien(self):
        """Un simple seuil aurait dit « tenu » : ouverte, elle lit 100."""
        self.assertFalse(fsm.porte_objet(self._ctx(1, 100)))

    def test_une_reponse_ILLISIBLE_ne_fait_pas_lacher(self):
        """Le rouleau blanc est parti en vol pour une lecture parasite.

        27/08, 12 lectures pendant une remontée : deux statuts « 6 » (la pince
        ne rend que 0-3) et un angle « 65535 » (le -1 du registre lu en 16 bits
        non signés). Une seule suffisait à conclure « objet lâché » — la
        remontée s'arrêtait, la pince s'ouvrait, le rouleau était jeté.
        """
        for statut, angle in ((6, 65535), (-1, -1), (6, -1)):
            with self.subTest(statut=statut, angle=angle):
                self.assertTrue(fsm.porte_objet(self._ctx(statut, angle)))

    def test_une_lecture_LISIBLE_tranche_malgre_un_statut_parasite(self):
        """Statut illisible mais angle net : l'angle décide, dans les deux sens."""
        self.assertFalse(fsm.porte_objet(self._ctx(6, 20)))
        self.assertTrue(fsm.porte_objet(self._ctx(6, 26)))


class ApresLeLargageLaCibleNExistePlus(unittest.TestCase):
    """« Il met la balle dans le carton, puis revient à la même position
    prendre du vide » — signalé une semaine durant.

    Le suivi rend la DERNIÈRE position connue dès qu'aucun exemplaire n'est vu
    près d'elle. Après un dépôt, cette position est justement l'endroit vidé :
    le cycle suivant y repartait, y redescendait, et s'y refermait sur rien.
    """

    def setUp(self):
        self.dossier = __import__('tempfile').mkdtemp()
        self._va, self._etapes = fsm.va_vers, fsm.va_vers_par_etapes
        self._ik = fsm.resout_ik
        fsm.va_vers = lambda ctx, q, **k: np.asarray(q, float)
        fsm.va_vers_par_etapes = lambda ctx, q, **k: np.asarray(q, float)
        fsm.resout_ik = lambda p, R, **k: (fsm.POSE_OBSERVATION.copy(), 0.0, 0.0)

    def tearDown(self):
        fsm.va_vers, fsm.va_vers_par_etapes = self._va, self._etapes
        fsm.resout_ik = self._ik

    def test_le_retrait_efface_la_cible_suivie(self):
        ctx = contexte(dossier=self.dossier)
        ctx.classe_objet, ctx.carton_vise = 'balle', 'grand'
        ctx.balle_xy = np.array([196.0, 134.0])
        ctx.R_balle = fsm.orientation(ctx.balle_xy)
        ctx.mode_auto = True
        self.assertEqual(fsm._retrait(ctx), 'DEGAGEMENT')
        self.assertEqual(ctx.classe_objet, '')
        self.assertIsNone(ctx.balle_xy)

    def test_le_second_exemplaire_ne_herite_pas_du_premier(self):
        """Deux rouleaux : la catégorie n'est pas finie, la cible l'est."""
        ctx = contexte(dossier=self.dossier)
        ctx.classe_objet, ctx.carton_vise = 'scotch', 'petit'
        ctx.balle_xy = np.array([303.0, -32.0])
        ctx.R_balle = fsm.orientation(ctx.balle_xy)
        ctx.mode_auto = True
        ctx.deposes['scotch'] = 1
        fsm._retrait(ctx)
        self.assertIsNone(ctx.balle_xy)
        self.assertEqual(ctx.essais, {})


class LaBoucleFermeeNeSEmballePas(unittest.TestCase):
    """26/08 : la pince a buté contre le J3 sur une cible à 151 mm.

    `converge` réinjecte l'écart mesuré dans la consigne à chaque passe. Sur une
    pose que le bras ne peut pas tenir, l'écart ne se referme jamais — 12 passes
    bloquées entre 33 et 37 mm, et CROISSANTES d'un essai à l'autre — donc la
    consigne s'emballe et pousse les servos dans la butée, jusqu'à ce que le
    pont de la Pi cesse de répondre.
    """

    def setUp(self):
        self.dossier = __import__('tempfile').mkdtemp()
        self._va, self._ik = fsm.va_vers, fsm.resout_ik
        fsm.resout_ik = lambda p, R, **k: (fsm.POSE_OBSERVATION.copy(), 0.0, 0.0)

    def tearDown(self):
        fsm.va_vers, fsm.resout_ik = self._va, self._ik

    def test_sans_progres_on_cesse_de_reinjecter(self):
        ctx = contexte(dossier=self.dossier)
        commandes = []

        def va_vers_bloque(ctx, q, **k):
            """Le bras est bloqué : il ne bouge pas, quoi qu'on commande."""
            commandes.append(np.asarray(q, float))
            return fsm.POSE_OBSERVATION.copy() + 5.0

        fsm.va_vers = va_vers_bloque
        _, correction, ok = fsm.converge(ctx, np.array([151.5, 1.5, 5.0]),
                                         fsm.orientation(np.array([151.5, 1.5])))
        self.assertFalse(ok)
        self.assertTrue(np.allclose(correction, 0.0))
        self.assertLessEqual(len(commandes), 2,
                             'la consigne a continué à être réinjectée')

    def test_une_cible_trop_pres_est_refusee_et_oubliee(self):
        ctx = contexte(dossier=self.dossier)
        ctx.balle_xy = np.array([151.5, 1.5])
        ctx.classe_objet = 'robot'
        ctx.detecteur = lambda **k: np.array([151.5, 1.5])
        self.assertEqual(fsm._detecte(ctx), 'ECHEC')
        self.assertTrue(ctx.est_oublie(np.array([151.5, 1.5])))


class LAngleNeTemoigneQueApresUneFermeture(unittest.TestCase):
    """Au réveil après l'incident du 26/08 la pince lisait 73, statut « rien
    saisi » — un état résiduel que la seule lecture de l'angle aurait pris
    pour une prise."""

    def _ctx(self, angle, fermee):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont(1)
        ctx.pont.angle = angle
        ctx.pince_fermee = fermee
        return ctx

    def test_angle_ambigu_sans_ordre_de_fermeture(self):
        self.assertFalse(fsm.porte_objet(self._ctx(73, False)))

    def test_apres_fermeture_le_meme_angle_temoigne(self):
        self.assertTrue(fsm.porte_objet(self._ctx(73, True)))


class LeGardeFouNeCoupePasUneConvergenceQuiMarche(unittest.TestCase):
    """En absolu il coupait 4,43 -> 3,48 mm, un gain de 21 % qui aboutissait.

    L'emballement ne gagne presque rien sur un écart énorme (34,25 -> 33,62,
    soit 1,8 %) ; une convergence lente gagne peu en millimètres mais beaucoup
    en proportion. C'est le rapport qui les sépare, pas la différence.
    """

    def test_les_deux_familles_tombent_de_part_et_d_autre(self):
        emballement = [(34.25, 33.62), (33.62, 33.48), (33.48, 33.22)]
        for avant, apres in emballement:
            with self.subTest(paire=(avant, apres)):
                self.assertGreater(apres, avant * (1.0 - fsm.PROGRES_CONVERGENCE),
                                   'un emballement doit être coupé')
        for avant, apres in [(4.43, 3.48), (11.62, 2.09)]:
            with self.subTest(paire=(avant, apres)):
                self.assertLess(apres, avant * (1.0 - fsm.PROGRES_CONVERGENCE),
                                'une convergence qui progresse doit continuer')


class ApresUneSaisieRateeOnEcarteLeBrasPourRevoir(unittest.TestCase):
    """Doigts au-dessus de l'objet, le bras le cache à l'arducam.

    Le journal du 26/08 affiche « [scotch non vu] » avant chaque reprise : la
    descente suivante repartait donc sur la MÊME position mémorisée, et trois
    essais identiques donnaient trois échecs identiques. On s'écarte d'abord,
    pour revenir avec une position fraîche — y compris si l'objet a été poussé
    par l'essai raté.
    """

    def setUp(self):
        self.dossier = __import__('tempfile').mkdtemp()
        self._va, self._degage = fsm.va_vers_par_etapes, fsm.degage_du_sol
        fsm.va_vers_par_etapes = lambda ctx, q, **k: np.asarray(q, float)
        fsm.degage_du_sol = lambda ctx: None

    def tearDown(self):
        fsm.va_vers_par_etapes, fsm.degage_du_sol = self._va, self._degage

    def test_une_saisie_a_vide_renvoie_au_degagement(self):
        ctx = contexte(statut=1, dossier=self.dossier)
        ctx.classe_objet, ctx.carton_vise = 'scotch', 'petit'
        ctx.z_prise = 2.0
        self.assertEqual(fsm._saisie(ctx), 'DEGAGEMENT')
        self.assertTrue(ctx.garde_cible)

    def test_ce_degagement_la_ne_change_PAS_de_cible(self):
        ctx = contexte(statut=1, dossier=self.dossier)
        ctx.classe_objet, ctx.carton_vise = 'scotch', 'petit'
        ctx.garde_cible = True
        ctx.detecteur = lambda **k: np.array([250.0, -60.0])
        fsm._degagement(ctx)
        self.assertEqual(ctx.classe_objet, 'scotch')
        self.assertFalse(ctx.garde_cible, 'le drapeau ne vaut que pour UN passage')

    def test_le_compteur_d_essais_survit_a_la_reprise(self):
        """Sans lui, la descente ne chercherait jamais un cran plus bas."""
        ctx = contexte(statut=1, dossier=self.dossier)
        ctx.classe_objet, ctx.z_prise = 'scotch', 2.0
        fsm._saisie(ctx)
        fsm._degagement(ctx)
        self.assertEqual(ctx.essais.get('SAISIE'), 1)


class LaPinceNOuvrePasSiLeBrasNEstPasArrive(unittest.TestCase):
    """Le 26/08 la pince s'est ouverte à 256 mm du carton visé.

    `va_vers` rend la main quand le bras ne bouge PLUS, ce qui n'est pas la même
    chose qu'être arrivé : un ordre accepté puis borné par les butées immobilise
    le bras en chemin. Le petit robot est tombé à côté du petit carton.
    """

    class PontQuiNObeitPas(FauxPont):
        """Le bras accepte l'ordre et ne bouge pas — butée, ou cible hors portée."""

        def envoie(self, action, **kw):
            if action == "send_angles":
                self.envois.append(action)
                return "OK"
            return super().envoie(action, **kw)

    def _machine(self, pont):
        ctx = fsm.Contexte()
        ctx.pont = pont
        ctx.carton_xy = np.array([280.0, -130.0])
        ctx.R_carton = fsm.orientation(ctx.carton_xy)
        ctx.classe_objet = "robot"
        ctx.en_main = "robot"
        machine = fsm.MachineEtats(ctx)
        machine.etat = "LARGAGE"
        return machine

    def test_bras_immobilise_en_chemin_objet_garde_en_main(self):
        machine = self._machine(self.PontQuiNObeitPas(2))
        machine.pas()
        self.assertEqual(machine.etat, "ECHEC_PORTANT")
        self.assertNotIn("pro_gripper_open", machine.ctx.pont.envois)
        self.assertEqual(machine.ctx.en_main, "robot")
        self.assertNotIn("robot", machine.ctx.deposes)
        self.assertIn("largage ANNULE", " ".join(machine.ctx.journal))

    def test_le_point_jamais_atteint_n_est_plus_represente(self):
        machine = self._machine(self.PontQuiNObeitPas(2))
        machine.pas()
        rate = machine.ctx.carton_xy if machine.ctx.carton_xy is not None else None
        self.assertTrue(machine.ctx.largages_rates)
        self.assertIsNone(machine.ctx.carton_resolu)
        # Et le même point ne ressort pas de la résolution suivante.
        ctx = machine.ctx
        cible, _, R = fsm.carton_atteignable(ctx, np.array([280.0, -130.0]), None)
        self.assertTrue(cible is None
                        or float(np.linalg.norm(cible - ctx.largages_rates[0])) >= 20.0)
        del rate

    def test_un_bras_qui_arrive_largue_normalement(self):
        machine = self._machine(FauxPont(2))
        machine.pas()
        self.assertEqual(machine.etat, "RETRAIT")
        self.assertIn("pro_gripper_open", machine.ctx.pont.envois)


class UnCartonLoinSeVisePARSonBordProche(unittest.TestCase):
    """Au milieu d'une ouverture à 450 mm, l'IK résout et le bras n'arrive pas.

    Le 26/08 la pince s'est ouverte 256 mm avant la cible. Tous les candidats
    gardent le même recul des parois, donc le bord proche dépose DEDANS lui
    aussi : quand la boîte est loin, le viser d'emblée épargne un aller-retour.
    """

    @staticmethod
    def _ouverture(centre, demi):
        c = np.asarray(centre, float)
        return np.array([[c[0] - demi, c[1] - demi], [c[0] + demi, c[1] - demi],
                         [c[0] + demi, c[1] + demi], [c[0] - demi, c[1] + demi]])

    def test_loin_le_premier_candidat_est_le_plus_proche_du_robot(self):
        centre = np.array([401.0, 203.6])            # grand carton, 26/08
        points = fsm.points_de_largage(centre, self._ouverture(centre, 56.0))
        portees = [float(np.hypot(*p)) for p in points]
        self.assertEqual(portees[0], min(portees))
        self.assertLess(portees[0], float(np.hypot(*centre)))

    def test_pres_le_premier_candidat_reste_le_milieu(self):
        centre = np.array([300.0, -130.0])
        points = fsm.points_de_largage(centre, self._ouverture(centre, 44.0))
        self.assertLess(float(np.linalg.norm(points[0] - centre)), 10.0)

    def test_tous_les_candidats_restent_dans_l_ouverture(self):
        centre = np.array([401.0, 203.6])
        ouverture = self._ouverture(centre, 56.0)
        contour = np.asarray(ouverture, np.float32).reshape(-1, 1, 2)
        for p in fsm.points_de_largage(centre, ouverture):
            marge = cv2.pointPolygonTest(contour, (float(p[0]), float(p[1])), True)
            self.assertGreaterEqual(marge, fsm.MARGE_LARGAGE_MIN)


class LaGardeAuRebordSurvitALAllonge(unittest.TestCase):
    """À 470 mm la pointe arrivait à 82,7 mm pour un rebord mesuré à 82,9.

    Le bras n'atteint pas la consigne sous le couple de gravité : mesuré le
    27/08, il manque 11,9 mm à 340 mm de portée et 25,3 mm à 470. La garde de
    25 mm au-dessus du rebord était donc entièrement mangée dès 410 mm, et
    l'objet lâché AU RAS du bord.
    """

    MESURES = ((340.0, 11.9), (380.0, 14.8), (410.0, 19.7),
               (430.0, 19.1), (450.0, 20.4), (470.0, 25.3))

    def test_le_modele_colle_aux_mesures(self):
        for portee, manque in self.MESURES:
            self.assertAlmostEqual(fsm.affaissement_largage(portee), manque, delta=2.0,
                                   msg=f'portée {portee:.0f} mm')

    def test_la_compensation_grandit_avec_la_portee(self):
        valeurs = [fsm.affaissement_largage(r) for r in (250, 340, 410, 470)]
        self.assertEqual(valeurs, sorted(valeurs))
        self.assertGreaterEqual(valeurs[0], 0.0)

    def test_la_consigne_depasse_la_hauteur_voulue_de_l_affaissement(self):
        ctx = fsm.Contexte()
        ctx.z_rebord = 83.0
        xy = 450.0 * np.array([np.cos(np.radians(27.0)), np.sin(np.radians(27.0))])
        voulue = fsm.z_largage(ctx)
        self.assertEqual(voulue, 83.0 + fsm.GARDE_LARGAGE)
        commandee = fsm.z_largage_commande(ctx, xy)
        self.assertAlmostEqual(commandee - voulue, fsm.affaissement_largage(450.0),
                               places=6)
        # Mesure du 27/08 a l'azimut -18 : consigne 130,0 mm a 450 mm de portee.
        self.assertAlmostEqual(commandee, 130.0, delta=1.0)

    def test_de_pres_la_consigne_reste_sobre(self):
        """Compenser 25 mm partout ferait larguer trop haut de pres."""
        ctx = fsm.Contexte()
        ctx.z_rebord = 83.0
        xy = np.array([250.0, 0.0])
        self.assertLess(fsm.z_largage_commande(ctx, xy) - fsm.z_largage(ctx), 6.0)


class UneDescenteDUnSeulTrait(unittest.TestCase):
    """« Il descend et remonte et redescend » — c'est fini.

    L'ancienne descente mesurait en bas, remontait à Z_SURVOL et recommençait,
    jusqu'à trois fois. Désormais elle descend d'un trait en se recalant en vol
    (la descente est verticale : l'écart XY lu en route EST la dérive latérale),
    puis on saisit. C'est la pince qui dit si c'est pris, pas un seuil.
    """

    def setUp(self):
        self._vrai = fsm.descend_par_paliers
        self.appels = []

        def espion(ctx, p_cible, R, correction, nom='descente'):
            self.appels.append(nom)
            return fsm.POSE_OBSERVATION.copy()      # loin de la cible : gros écart

        fsm.descend_par_paliers = espion

    def tearDown(self):
        fsm.descend_par_paliers = self._vrai

    def _contexte(self):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont(2)
        ctx.classe_objet = 'balle'
        return ctx

    def test_une_seule_descente_meme_si_l_ecart_reste(self):
        ctx = self._contexte()
        cible = np.array([300.0, -120.0, 5.0])
        q, ok = fsm.descente_verticale(ctx, cible, fsm.orientation(cible[:2]),
                                       np.zeros(6))
        self.assertTrue(ok, "un écart résiduel ne doit plus annuler la saisie")
        self.assertEqual(len(self.appels), 1, "une seule descente, pas trois")
        self.assertNotIn('remontee de recalage', ' '.join(ctx.journal))

    def test_un_palier_refuse_arrete_bien_la_descente(self):
        fsm.descend_par_paliers = lambda *a, **k: None
        ctx = self._contexte()
        cible = np.array([300.0, -120.0, 5.0])
        q, ok = fsm.descente_verticale(ctx, cible, fsm.orientation(cible[:2]),
                                       np.zeros(6))
        self.assertIsNone(q)
        self.assertFalse(ok)

    def test_l_ecart_mesure_est_publie(self):
        ctx = self._contexte()
        cible = np.array([300.0, -120.0, 5.0])
        fsm.descente_verticale(ctx, cible, fsm.orientation(cible[:2]), np.zeros(6))
        self.assertIn('descente XY', ctx.resultats)


class LaDeriveApprisePasseALObjetSuivant(unittest.TestCase):
    """Remettre la dérive à zéro à chaque cible faisait repayer la mesure.

    Elle décrit le bras qui s'affaisse, pas l'objet visé.
    """

    def test_repart_a_zero_garde_la_derive_apprise(self):
        ctx = fsm.Contexte()
        ctx.biais_appris = np.array([3.5, -2.0, 0.0])
        ctx.biais_descente = np.array([9.0, 9.0, 0.0])
        ctx.essais['DESCENTE'] = 2
        ctx.repart_a_zero()
        self.assertEqual(ctx.essais, {})
        self.assertTrue(np.allclose(ctx.biais_descente, [3.5, -2.0, 0.0]))

    def test_sans_rien_appris_on_part_bien_de_zero(self):
        ctx = fsm.Contexte()
        ctx.biais_descente = np.array([9.0, 9.0, 0.0])
        ctx.repart_a_zero()
        self.assertTrue(np.allclose(ctx.biais_descente, 0.0))


class LeBrasNeSeTouchePasLuiMeme(unittest.TestCase):
    """Modèle à capsules : un segment par lien, un rayon, une marge.

    Les rayons viennent des maillages de CE bras
    (`mycobot_description/urdf/320_pi/*.dae`), pas d'un schéma générique :
    colonne 58, bras 48, avant-bras 43, poignet 44, bride 29 mm.
    """

    def test_les_paires_voisines_de_fait_sont_exclues(self):
        """poignet/pince : marge de −27,7 à −27,6 mm sur 4000 poses tirées dans
        les butées, soit 0,1 mm d'amplitude. C'est une constante géométrique —
        la bride ne fait que 19 mm — pas une collision."""
        self.assertIn(('poignet', 'pince'), fsm.PAIRES_CAPSULE_EXCLUES)

    def test_la_pose_d_observation_est_largement_libre(self):
        marge, _ = fsm.marge_collision(fsm.POSE_OBSERVATION)
        self.assertGreater(marge, 25.0)

    def test_l_enveloppe_de_travail_n_est_jamais_refusee(self):
        """120 poses balayées le 27/08 : aucune refusée, minimum +2,2 mm.
        Un garde-fou qui refuse du travail normal ne protège rien."""
        for portee in (200.0, 260.0, 320.0, 380.0, 430.0):
            for azimut in (-30.0, 0.0, 30.0):
                xy = np.array([portee * np.cos(np.radians(azimut)),
                               portee * np.sin(np.radians(azimut))])
                sol = fsm.resout_ik(np.array([xy[0], xy[1], 110.0]),
                                    fsm.orientation(xy))
                if sol is None:
                    continue
                marge, paire = fsm.marge_collision(sol[0])
                self.assertGreater(marge, 0.0,
                                   f'portée {portee:.0f} azimut {azimut:.0f} : {paire}')

    def test_le_bras_replie_sur_sa_colonne_est_refuse(self):
        """À 130 mm d'allonge l'avant-bras vient sur la colonne : −5,7 mm."""
        xy = np.array([130.0 * np.cos(np.radians(-10.0)),
                       130.0 * np.sin(np.radians(-10.0))])
        sol = fsm.resout_ik(np.array([xy[0], xy[1], 15.0]), fsm.orientation(xy))
        self.assertIsNotNone(sol, "l'IK résout cette pose — c'est bien le problème")
        marge, paire = fsm.marge_collision(sol[0])
        self.assertLess(marge, 0.0)
        self.assertEqual(paire, 'colonne/avant_bras')

    def test_PORTEE_MIN_reste_plus_severe_que_les_capsules(self):
        """AVERTISSEMENT VOLONTAIREMENT FIGÉ DANS UN TEST.

        L'incident du 26/08 — pince coincée contre J3 — s'est produit à 151 mm.
        Or à 151 mm le modèle à capsules donne une marge POSITIVE (+4 mm) : il
        ne l'aurait pas refusé. La paire qui aurait vu venir la chose est
        avant_bras/pince, et c'est justement celle qu'on a dû exclure faute de
        connaître le volume réel de la pince. `PORTEE_MIN` ne doit donc PAS être
        abaissé sur la foi de ce modèle.
        """
        xy = np.array([151.0 * np.cos(np.radians(-10.0)),
                       151.0 * np.sin(np.radians(-10.0))])
        sol = fsm.resout_ik(np.array([xy[0], xy[1], 15.0]), fsm.orientation(xy))
        marge, _ = fsm.marge_collision(sol[0])
        self.assertGreater(marge, 0.0, 'le modèle croit cette pose sûre')
        self.assertGreater(fsm.PORTEE_MIN, 151.0,
                           'le plancher empirique, lui, la refuse — il doit rester')


class UnObjetLACHENEstPasUnObjetDEPOSE(unittest.TestCase):
    """Le 27/08 l'inventaire a coché « balle ✔ » alors qu'elle n'avait jamais
    quitté la planche. Le journal montre les deux lignes qui se suivent :

        objet lache pendant la remontee — on refait la saisie
        DEGAGEMENT interdit — la pince tient l objet, on va le deposer

    La remontée conclut que l'objet est perdu ; la garde relit la pince une
    seconde plus tard, la trouve « pleine », interdit le ramassage et renvoie
    déposer. Le bras est parti larguer du vide au-dessus du carton.

    Cause physique : quand l'objet glisse des doigts, la pince adaptative RESTE
    à l'angle où elle s'était fermée et son statut bat entre deux valeurs.
    """

    def _contexte(self, statut):
        ctx = fsm.Contexte()
        ctx.pont = FauxPont(statut)
        ctx.pont.angle = 52          # doigts restés là où la balle les tenait
        ctx.pince_fermee = True
        ctx.classe_objet = ctx.carton_vise = 'balle', 'grand'
        ctx.classe_objet, ctx.carton_vise = 'balle', 'grand'
        ctx.carton_xy = np.array([311.0, 167.0])
        ctx.z_rebord = 83.0
        _, _, ctx.R_carton = fsm.choisit_pose_largage(ctx, ctx.carton_xy)
        return ctx

    def test_un_largage_sans_rien_en_main_ne_coche_pas(self):
        ctx = self._contexte(2)
        ctx.en_main = ''                      # la remontée a conclu : perdu
        self._ik, self._va = fsm.resout_ik, fsm.va_vers

        def va(c, q, **kw):
            c.pont.q = np.asarray(q, float)
            return c.pont.angles()

        fsm.va_vers = va
        try:
            fsm._largage(ctx)
        finally:
            fsm.va_vers = self._va
        self.assertEqual(ctx.deposes.get('balle', 0), 0,
                         "cocher un objet resté sur la planche le retire des cibles")

    def test_la_remontee_qui_perd_l_objet_ROUVRE_la_pince(self):
        """Rouvrir remet l'angle à 100 : les deux témoins disent enfin la même
        chose, et le ramassage peut reprendre au lieu d'aller larguer du vide."""
        ctx = self._contexte(1)
        ctx.en_main = 'balle'
        vrai = fsm.monte_par_paliers
        fsm.monte_par_paliers = lambda c, R, **k: (c.pont.angles(), False)
        try:
            suivant = fsm._remontee(ctx)
        finally:
            fsm.monte_par_paliers = vrai
        self.assertEqual(suivant, 'DEGAGEMENT')
        self.assertIn('pro_gripper_open', ctx.pont.envois)
        self.assertFalse(ctx.pince_fermee)
        self.assertEqual(ctx.en_main, '')
        self.assertFalse(fsm.porte_objet(ctx),
                         "apres reouverture, la pince ne doit plus dire qu'elle tient")


class LOutilSeCoucheAVANTQueLeVerticalNeBloque(unittest.TestCase):
    """« Il refuse de descendre » — le rouleau blanc à 324 mm, 27/08.

    La machine y choisissait encore l'outil vertical, et la descente était
    refusée : « palier Z=-8 exige 190 deg — changement de branche refuse ». Le
    seuil valait 325, posé « entre le dernier propre (320) et le premier troué
    (335) » : un milieu raisonné, jamais vérifié.

    Balayage géométrique, descente jusqu'à Z=-8 :

        portée |  300  310  315  320  324  326  330  335  340
        droit  |  ok   ok   ok   ok   NON  NON  NON  NON  NON
        -15    |  ok   ok   ok   ok   ok   ok   ok   ok   ok
    """

    @staticmethod
    def _descend(portee, theta):
        xy = portee * np.array([np.cos(np.radians(33.5)), np.sin(np.radians(33.5))])
        azimut = float(np.degrees(np.arctan2(xy[1], xy[0])))
        for roulis in fsm.ROULIS:
            R = fsm.incline(fsm.orientation(xy, roulis), azimut, theta)
            if fsm.resout_ik(np.array([xy[0], xy[1], fsm.Z_SURVOL]), R) is None:
                continue
            if fsm.colonne_continue(xy, R, -8.0):
                return True
        return False

    def test_le_seuil_vaut_la_derniere_portee_VERIFIEE(self):
        limite = fsm.INCLINAISONS_PAR_PORTEE[0][0]
        self.assertEqual(fsm.INCLINAISONS_PAR_PORTEE[0][1], 0.0)
        self.assertTrue(self._descend(limite, 0.0),
                        'le seuil doit etre une portee ou le vertical descend VRAIMENT')

    def test_le_rouleau_blanc_a_324_mm_n_est_plus_tenu_droit(self):
        theta = next(t for limite, t in fsm.INCLINAISONS_PAR_PORTEE if 324.0 <= limite)
        self.assertNotEqual(theta, 0.0)
        self.assertTrue(self._descend(324.0, theta))

    def test_le_vertical_est_bien_bloque_au_dela(self):
        self.assertFalse(self._descend(324.0, 0.0))


class UnRoulisQuiPOUSSENEstPasRejoue(unittest.TestCase):
    """« Il touche le quart du scotch et il glisse » — mesuré le 27/08.

    Rouleau bleu à 389 mm : seules les inclinaisons -30 et -45 résolvent, et à
    -30 les roulis 0, 30, 60 et 90 passent tous la géométrie. Le roulis 0
    referme la pince à vide et POUSSE le rouleau de 6,4 mm ; le roulis +30 le
    saisit (statut 2, angle 26) et le tient jusqu'à Z=170. La géométrie ne
    sépare pas les deux — seule la prise réelle le fait, et sans mémoire les
    trois essais rejouaient le même roulis raté.
    """

    def _ctx(self):
        ctx = fsm.Contexte()
        ctx.classe_objet = 'scotch'
        return ctx

    def test_le_couple_rate_passe_en_dernier(self):
        ctx = self._ctx()
        xy = np.array([338.0, 193.0])
        cle = fsm.cle_roulis('balle', xy)
        premier = fsm.choisit_pose_prise(ctx, xy)
        self.assertIsNotNone(premier, 'cette allonge doit rester saisissable')
        theta, roulis = premier[1], premier[0]
        ctx.prises_ratees[cle] = [(theta, roulis)]
        ctx.prise_apprise.pop(cle, None)
        second = fsm.choisit_pose_prise(ctx, xy)
        self.assertIsNotNone(second, 'un couple raté ne doit pas rendre l objet insaisissable')
        self.assertNotEqual((second[1], second[0]), (theta, roulis))

    def test_tous_rates_on_en_repropose_quand_meme(self):
        """Écarter n'est pas supprimer : sinon l'objet devient insaisissable."""
        ctx = self._ctx()
        xy = np.array([338.0, 193.0])
        ctx.prises_ratees[fsm.cle_roulis('balle', xy)] = [
            (theta, roulis) for theta in fsm.INCLINAISONS for roulis in fsm.ROULIS]
        self.assertIsNotNone(fsm.choisit_pose_prise(ctx, xy))

    def test_une_prise_REUSSIE_efface_l_ardoise(self):
        ctx = self._ctx()
        xy = np.array([338.0, 193.0])
        cle = fsm.cle_roulis('balle', xy)
        ctx.prises_ratees[cle] = [(-30.0, 0)]
        ctx.balle_xy = xy
        ctx.pont = FauxPont(2)
        ctx.pont.cale = 26
        self.assertEqual(fsm._saisie(ctx), 'REMONTEE')
        self.assertNotIn(cle, ctx.prises_ratees)


class UnStatutSAISISurUnePinceVIDE(unittest.TestCase):
    """« Il pousse la figurine au lieu de la prendre » — mesuré le 28/08.

    Figurine imprimée, roulis 0 : la pince rend statut 2 — « objet saisi » —
    avec un angle de 22, soit deux degrés au-dessus de la pince vide (20), et
    la figurine est retrouvée poussée de 18,3 mm. Rien n'était tenu. Le même
    geste au roulis +30 cale à 46 et tient.

    Conclure sur le statut seul coûte double : le cycle part en REMONTÉE pour
    rien, ET le roulis raté n'est jamais inscrit — la machine rejoue
    indéfiniment l'angle qui pousse l'objet.
    """

    def _ctx(self, statut, angle):
        ctx = fsm.Contexte()
        ctx.classe_objet = 'robot'
        ctx.balle_xy = np.array([233.8, 113.0])
        ctx.pont = FauxPont(statut)
        ctx.pont.cale = angle
        ctx.pont.angle = angle
        ctx.prise_apprise[fsm.cle_roulis('balle', ctx.balle_xy)] = (0.0, 0)
        return ctx

    def test_statut_2_avec_angle_de_pince_vide_nest_pas_une_prise(self):
        ctx = self._ctx(2, 22)
        self.assertNotEqual(fsm._saisie(ctx), 'REMONTEE')
        self.assertEqual(ctx.en_main, '')

    def test_le_roulis_qui_a_POUSSE_est_bien_inscrit(self):
        ctx = self._ctx(2, 22)
        fsm._saisie(ctx)
        cle = fsm.cle_roulis('balle', ctx.balle_xy)
        self.assertIn((0.0, 0), ctx.prises_ratees.get(cle, []))

    def test_une_VRAIE_prise_passe_toujours(self):
        """46 sur la figurine, 26 sur un rouleau, 53 sur la balle."""
        for angle in (26, 46, 53):
            with self.subTest(angle=angle):
                ctx = self._ctx(2, angle)
                self.assertEqual(fsm._saisie(ctx), 'REMONTEE')
                self.assertEqual(ctx.en_main, 'robot')

    def test_un_angle_ILLISIBLE_ne_dement_pas_le_statut(self):
        """65535 ne prouve rien : sur un statut 2, on garde la prise."""
        ctx = self._ctx(2, 65535)
        self.assertEqual(fsm._saisie(ctx), 'REMONTEE')

