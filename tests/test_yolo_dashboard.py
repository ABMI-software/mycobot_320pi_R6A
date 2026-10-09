"""YOLOE dans pick_dashboard : ce que `Vision.objets` de l'arducam rend et ecarte.

Ce qui est tenu : le cylindre va au petit carton, tout autre objet au grand ;
rien hors planche, dans un marqueur detecte, sous le bras, jaune comme la balle,
trop grand, en double ou pres d'un carton ; un objet partiellement vert reste un
objet ; rien tant que YOLO n'a rien rendu ; la SVPRO et un service arrete
retombent sur le detecteur d'origine.
"""
import sys
import threading
import time
import types
import unittest
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / "scripts"))

import yolo_dashboard as yd  # noqa: E402

FORME = (480, 640)


def carre(u, v, cote=30):
    return [[u, v], [u + cote, v], [u + cote, v + cote], [u, v + cote]]


def detection(classe, u, v, cote=30, conf=0.8):
    return {"classe": classe, "nom": "x", "conf": conf, "contour": carre(u, v, cote)}


class FauxService:
    """Rend tout de suite ce qu'on lui a donne ; `None` = service arrete."""
    arrete = False

    def __init__(self, detections):
        self.detections = detections
        self.soumises = 0
        self.vu = None

    def soumet(self, image):
        self.soumises += 1
        if self.detections is None:
            self.arrete = True
        else:
            self.vu = (image, self.detections, time.time())

    def dernier(self):
        return self.vu

    def ferme(self):
        pass


class ServiceMuet(FauxService):
    def soumet(self, image):
        self.soumises += 1


class FausseVision:
    """1 px = 1 mm ; planche = tout sauf la bande u < 100 ; bras = u > 500."""
    source = "arducam_extrinsic_pick.yaml"
    cartons = {}
    jaune = None

    def balle(self, image):
        return self.jaune

    def masque_planche(self, forme):
        return self.masque_plateau(forme)

    def masque_plateau(self, forme):
        m = np.full(forme, 255, np.uint8)
        m[:, :100] = 0
        return m

    def masque_bras(self, forme, angles):
        m = np.zeros(forme, np.uint8)
        m[:, 500:] = 255
        return m

    @staticmethod
    def sans_marqueurs(masque, marqueurs):
        net = masque.copy()
        for coins in (marqueurs or {}).values():
            cv2.fillConvexPoly(net, np.asarray(coins, np.int32), 0)
        return net

    def cartons_marques(self, image, marqueurs):
        return self.cartons

    def _cotes_mm(self, contour, z=None):
        (_, _), (a, b), _ = cv2.minAreaRect(contour)
        return min(a, b), max(a, b)

    def point_le_plus_epais(self, plein):
        v, u = np.nonzero(plein)
        return (float(u.mean()), float(v.mean())) if len(u) else None

    def vers_base(self, uv, z_mm):
        return np.array([uv[0], uv[1], z_mm])

    def objets(self, image, angles=None, marqueurs=None):
        return [("origine", np.zeros(2), None)]


class Base(unittest.TestCase):
    def setUp(self):
        self.image = np.full(FORME + (3,), 128, np.uint8)
        self.module = types.SimpleNamespace(
            Vision=type("V", (FausseVision,), {}), HSV_BALLE=((25, 90, 90), (45, 255, 255)),
            HAUTEUR_OBJET=24.0, HAUTEUR_CENTRE_BALLE=32.0, COTE_ROBOT_MM=(60.0, 200.0), AIRE_OBJET_MIN=90,
            INVENTAIRE={"scotch": 2, "balle": 1, "robot": 1},
            _MARQUEURS={19: [96.8, 203.4, 0.0]},
            Fenetre=type("F", (), {"_detecte_objet": lambda self, *a, **kw: "origine",
                                   "ctx": types.SimpleNamespace(classe_objet="")}))

    def rendu(self, detections, angles=None, marqueurs=None, service=None):
        yd.branche(self.module, service=service or FauxService(detections))
        return self.module.Vision().objets(self.image, angles=angles, marqueurs=marqueurs or {})


class TestClasses(Base):
    def test_cylindre_petit_carton_objet_grand_carton(self):
        rendu = self.rendu([detection("objet", 300, 100), detection("cylindre", 200, 200)])
        self.assertEqual([c for c, _, _ in rendu], ["scotch", "robot"])
        np.testing.assert_allclose(rendu[0][1], [214.5, 214.5], atol=1)

    def test_inventaire_mis_a_jour(self):
        yd.branche(self.module, service=FauxService([]), inventaire={"robot": 4})
        self.assertEqual(self.module.INVENTAIRE, {"scotch": 2, "balle": 1, "robot": 4})

    def test_objet_en_partie_vert_reste_un_objet(self):
        self.image[200:210, 300:330] = (0, 255, 180)
        self.assertEqual(len(self.rendu([detection("cylindre", 300, 200)])), 1)


class TestEcartes(Base):
    def test_hors_planche(self):
        self.assertEqual(self.rendu([detection("objet", 20, 200)]), [])

    def test_dans_un_marqueur_detecte(self):
        marqueur = {19: carre(295, 195, 40)}
        self.assertEqual(self.rendu([detection("objet", 300, 200)], marqueurs=marqueur), [])

    def test_marqueur_de_planche_cache_par_la_main_reste_ecarte(self):
        yd.branche(self.module, service=FauxService([detection("objet", 300, 200)]))
        vision = self.module.Vision()
        self.assertEqual(vision.objets(self.image, marqueurs={19: carre(295, 195, 40)}), [])
        self.assertEqual(vision.objets(self.image, marqueurs={}), [])

    def test_marqueur_de_carton_non_retenu(self):
        yd.branche(self.module, service=FauxService([detection("objet", 300, 200)]))
        vision = self.module.Vision()
        self.assertEqual(vision.objets(self.image, marqueurs={11: carre(295, 195, 40)}), [])
        self.assertEqual(len(vision.objets(self.image, marqueurs={})), 1)

    def test_sous_le_bras_seulement_avec_les_angles(self):
        d = [detection("objet", 520, 200)]
        self.assertEqual(self.rendu(d, angles=np.zeros(6)), [])
        self.assertEqual(len(self.rendu(d, angles=None)), 1)

    def test_jaune_laisse_a_la_balle(self):
        self.image[200:230, 300:330] = (0, 255, 255)
        self.assertEqual(self.rendu([detection("objet", 300, 200)]), [])

    def test_trop_grand(self):
        self.assertEqual(self.rendu([detection("objet", 150, 100, cote=250)]), [])

    def test_trop_petit(self):
        self.assertEqual(self.rendu([detection("objet", 300, 200, cote=5)]), [])

    def test_doublon_garde_le_cylindre(self):
        rendu = self.rendu([detection("objet", 302, 202, conf=0.95),
                            detection("cylindre", 300, 200, conf=0.5)])
        self.assertEqual([c for c, _, _ in rendu], ["scotch"])

    def test_pres_d_un_carton(self):
        self.module.Vision.cartons = {"petit": (np.array([320.0, 220.0]), 80.0, 30.0)}
        self.assertEqual(self.rendu([detection("objet", 300, 200)]), [])
        self.module.Vision.cartons = {"petit": (np.array([320.0, 420.0]), 80.0, 30.0)}
        self.assertEqual(len(self.rendu([detection("objet", 300, 200)])), 1)


class TestBalle(Base):
    """Le jaune d'abord ; YOLO quand il ne voit rien, sur la planche et sur une image recente."""

    def vision_avec(self, detections, age=0.0):
        service = FauxService(detections)
        yd.branche(self.module, service=service)
        service.vu = (self.image, detections, time.time() - age)
        return self.module.Vision()

    def test_le_jaune_d_abord(self):
        vision = self.vision_avec([detection("balle", 300, 200)])
        vision.jaune = (np.array([1.0, 2.0]), (3.0, 4.0, 5.0))
        self.assertEqual(vision.balle(self.image)[1], (3.0, 4.0, 5.0))

    def test_yolo_quand_le_jaune_ne_voit_rien(self):
        xy, (u, v, rayon) = self.vision_avec([detection("balle", 300, 200)]).balle(self.image)
        np.testing.assert_allclose(xy, [315.0, 215.0], atol=0.5)
        self.assertAlmostEqual(rayon, 15 * np.sqrt(2), delta=0.5)

    def test_balle_yolo_hors_planche(self):
        self.assertIsNone(self.vision_avec([detection("balle", 20, 200)]).balle(self.image))

    def test_balle_yolo_perimee(self):
        self.assertIsNone(self.vision_avec([detection("balle", 300, 200)], age=1.0).balle(self.image))

    def test_la_balle_yolo_n_est_pas_un_objet(self):
        self.assertEqual(self.rendu([detection("balle", 300, 200)]), [])

    def test_svpro_garde_le_jaune_seul(self):
        vision = self.vision_avec([detection("balle", 300, 200)])
        vision.source = "svpro_extrinsic_servo.yaml"
        self.assertIsNone(vision.balle(self.image))


class ServiceEnRetard(FauxService):
    """Le resultat d'une image arrive `retard` secondes apres sa soumission."""
    retard = 0.3

    def soumet(self, image):
        self.soumises += 1
        instant = time.time()
        threading.Timer(self.retard, lambda: setattr(self, "vu", (image, [], instant))).start()


class TestImageFraiche(Base):
    """Apres un degagement, le fil du robot ne juge pas une image d'avant."""

    def test_le_fil_du_robot_attend_un_resultat_pris_apres_son_appel(self):
        service = ServiceEnRetard([])
        service.vu = (self.image, [], time.time() - 5.0)
        yd.branche(self.module, service=service)
        fenetre = self.module.Fenetre()
        duree = []

        def robot():
            debut = time.time()
            fenetre._detecte_objet()
            duree.append(time.time() - debut)

        fil = threading.Thread(target=robot)
        fil.start()
        time.sleep(0.05)
        self.module.Vision().objets(self.image)
        fil.join()
        self.assertGreaterEqual(duree[0], service.retard)
        self.assertLess(duree[0], yd.PATIENCE_FRAICHE_S)

    def test_le_fil_graphique_n_attend_jamais(self):
        service = ServiceEnRetard([])
        yd.branche(self.module, service=service)
        debut = time.time()
        self.assertEqual(self.module.Fenetre()._detecte_objet(), "origine")
        self.assertLess(time.time() - debut, 0.05)

    def duree_depuis_le_robot(self, fenetre, **kw):
        duree = []

        def robot():
            debut = time.time()
            fenetre._detecte_objet(**kw)
            duree.append(time.time() - debut)

        fil = threading.Thread(target=robot)
        fil.start()
        fil.join()
        return duree[0]

    def test_pendant_l_approche_pas_d_attente(self):
        yd.branche(self.module, service=ServiceMuet([]))
        fenetre = self.module.Fenetre()
        fenetre.ctx = types.SimpleNamespace(classe_objet="robot")
        self.assertLess(self.duree_depuis_le_robot(fenetre), 0.05)
        self.assertGreaterEqual(self.duree_depuis_le_robot(fenetre, exige_dessus=True),
                                yd.PATIENCE_FRAICHE_S)

    def test_sans_resultat_frais_le_fil_du_robot_abandonne_a_la_patience(self):
        yd.branche(self.module, service=ServiceMuet([]))
        duree = []

        def robot():
            debut = time.time()
            self.module.Fenetre()._detecte_objet()
            duree.append(time.time() - debut)

        fil = threading.Thread(target=robot)
        fil.start()
        fil.join()
        self.assertGreaterEqual(duree[0], yd.PATIENCE_FRAICHE_S)


class TestRepli(Base):
    def test_rien_tant_que_yolo_n_a_rien_rendu(self):
        service = ServiceMuet([])
        self.assertEqual(self.rendu([], service=service), [])
        self.assertEqual(service.soumises, 1)

    def test_svpro_garde_le_detecteur_d_origine(self):
        service = FauxService([detection("objet", 300, 200)])
        yd.branche(self.module, service=service)
        vision = self.module.Vision()
        vision.source = "svpro_extrinsic_servo.yaml"
        self.assertEqual(vision.objets(self.image)[0][0], "origine")
        self.assertEqual(service.soumises, 0)

    def test_service_arrete_retombe_et_n_insiste_pas(self):
        service = FauxService(None)
        yd.branche(self.module, service=service)
        vision = self.module.Vision()
        self.assertEqual(vision.objets(self.image)[0][0], "origine")
        self.assertEqual(vision.objets(self.image)[0][0], "origine")
        self.assertEqual(service.soumises, 1)


if __name__ == "__main__":
    unittest.main()
