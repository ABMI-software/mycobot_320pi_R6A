#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Collecteur v3 a POSE DE CAMERA RANDOMISEE, une pose differente par trame.

Pourquoi ce fichier existe. Mesure du 02/09/2026 : le keypoint `base` de
`vgg_montage0901_ft_e30` n'est pas une detection mais une CONSTANTE PAR
MONTAGE — (211,7 / 342,9) sur l'arducam les deux jours, (384,5 / 333,3) sur la
svpro les deux jours, (250,0 / 256,4) sur real_3cam, ecarts-types 0,00 a 0,21.
Decaler l'image de 30 px deplace `base`, `link1` et `link2` de 0 %. Le reseau
reconnait le montage et recite la position du socle au lieu de la detecter.

La cause est dans les donnees, pas dans l'architecture : DREAM tire sa
generalisation aux points de vue nouveaux de l'entrainement synthetique a poses
de camera RANDOMISEES, or notre synthetique n'a que **4 points de vue fixes**
(front, right, left, top) et l'affinage reel n'a vu que **2 cameras fixes**. Le
socle ne bouge jamais dans l'ensemble d'entrainement : le memoriser est une
strategie gagnante, et le reseau la trouve.

Pourquoi une pose CONTINUE et non un jeu de N points de vue fixes. Avec N poses
figees, le reseau peut memoriser N constantes — c'est le meme defaut, repousse.
Une pose tiree a chaque trame rend la memorisation impossible : il n'existe plus
de constante a apprendre.

Ce que ce fichier NE modifie pas : `synthetic_data_collector_v3.py`,
`randomized.sdf`, l'URDF du robot et ses 4 cameras. La camera libre est un
modele autonome (`worlds/camera_libre.sdf`) spawne par le launch, deplace ici
par `set_pose` — un lien de l'URDF ne serait pas deplacable, `set_pose` ne
s'appliquant qu'a un modele entier.

La garde au sol reste celle de `CollecteurGardeBasse` (50 mm) : a 130 mm, 100 %
des poses reelles du pick sont rejetees.

Usage :
    ros2 launch mycobot_gateway synthetic_data_camera_libre.launch.py \
        num_samples:=20000 output_dir:=/tmp/synth_cam_libre
"""
import csv
import math
import os
import random
import signal
import time

from pathlib import Path

import numpy as np
import rclpy

from rclpy.executors import SingleThreadedExecutor
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from mycobot_gateway.synthetic_data_collector_v3_garde_basse import CollecteurGardeBasse


class CollecteurCameraLibre(CollecteurGardeBasse):

    # Une seule vue, celle de la camera libre. La cle sert de nom de dossier.
    CAMERA_TOPICS = {'libre': '/cam_libre/image'}

    NOM_MODELE = 'cam_libre'

    # Coquille de tirage autour du poste de travail. Les deux cameras reelles y
    # sont : l'arducam a 1,077 m et 80 deg d'elevation, la svpro a 0,98 m et
    # 53 deg. On elargit largement de part et d'autre.
    DIST_MIN, DIST_MAX = 0.70, 1.45
    ELEV_MIN, ELEV_MAX = math.radians(18.0), math.radians(86.0)
    ROULIS_MAX = math.radians(9.0)

    # Point vise : le centre du volume de travail, avec du jeu pour que le robot
    # ne soit pas systematiquement centre dans l'image (le centrage parfait est
    # lui aussi une regularite memorisable).
    VISEE = (0.10, 0.0, 0.20)
    VISEE_JEU = 0.10

    # Le robot doit tenir dans le champ : on exige que la sphere de ce rayon,
    # centree sur la visee nominale, soit entierement vue.
    # 0,34 m et non l'allonge pleine (0,39 m + pince) : c'est le champ VERTICAL
    # qui contraint (demi-angle 0,4525 rad), et exiger 0,45 m imposerait un
    # plancher de 1,03 m — or les deux cameras reelles sont a 0,79 m (svpro) et
    # 0,89 m (arducam). Le domaine synthetique ne les couvrirait pas. A 0,34 m
    # le plancher tombe a 0,78 m et les englobe. Les configurations les plus
    # etendues sortent alors partiellement du cadre aux distances courtes :
    # c'est voulu, les captures reelles ont aussi le bras en bord de champ.
    RAYON_ROBOT = 0.34
    ESSAIS_MAX = 60

    def __init__(self):
        super().__init__()
        self.declare_parameter('dist_min', self.DIST_MIN)
        self.declare_parameter('dist_max', self.DIST_MAX)
        self.DIST_MIN = self.get_parameter('dist_min').value
        self.DIST_MAX = self.get_parameter('dist_max').value

        # Demi-angles du champ, deduits du SDF (hfov 1.15 rad, 640x480).
        hfov = 1.15
        self._demi_h = hfov / 2.0
        self._demi_v = math.atan(math.tan(self._demi_h) * 480.0 / 640.0)

        # Fichier compagnon plutot qu'une colonne de plus dans labels.csv :
        # ecrire la pose ici n'oblige pas a redefinir `_capture_and_save`, donc
        # la logique de garde anti-gel du v3 reste celle du v3, non recopiee.
        # Une pose rejetee (bras non stabilise, rendu gele) laisse une ligne
        # perimee pour cet index : le convertisseur garde la DERNIERE, qui est
        # celle de la trame reellement enregistree.
        self.chemin_poses = os.path.join(self.output_dir, 'camera_poses.csv')
        with open(self.chemin_poses, 'w', newline='') as f:
            csv.writer(f).writerow(
                ['index', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])

        self._n_rejets_cadrage = 0
        self._n_echecs_pose = 0
        self._pose_courante = None
        self._vues_restantes = 0
        self._n_trajets_bloques = 0
        self._n_collisions_mesurees = 0
        self._n_appels_moissonnes = 0
        self._poses_pick = self._charge_poses_pick()
        self._settle_plein = self.settle_time
        self._ouvre_client_pose()
        self.get_logger().warn(
            f'camera LIBRE : pose retiree a chaque trame '
            f'(distance {self.DIST_MIN:.2f}-{self.DIST_MAX:.2f} m, elevation '
            f'{math.degrees(self.ELEV_MIN):.0f}-{math.degrees(self.ELEV_MAX):.0f} deg)')

    # Bout de la pince dans le repere de la bride, pour la garde au sol.
    # URDF `mycobot_pro_320_pi_gazebo.urdf` : `gripper_base` fixe sur link6 en
    # (0, -0.007, 0.056) rpy=(pi/2, 0, 0), doigts ~0,10 m au-dela. Valeur reprise
    # telle quelle de `synthetic_data_collector_v4_gripper.py`, qui l'avait
    # derivee de la meme chaine.
    #
    # Sans ce point, `_is_collision_free` ne surveille que les liens : une pose
    # a 50 mm de la BRIDE enfonce les doigts 60 mm SOUS la table, puisqu'ils
    # descendent 110 mm plus bas. Le v3 pouvait s'en passer, son URDF n'ayant
    # pas de pince ; ici elle est montee.
    _GRIPPER_TIP = np.array([0.0, -0.107, 0.056, 1.0])

    def _fk_points(self, angles):
        """Les 7 origines de repere, plus le bout de la pince en 8e point.

        `_is_collision_free` balaie les couples consecutifs pour la garde au sol
        (R1) : ajouter la pointe en fin de chaine y fait entrer le segment
        bride -> doigts sans toucher au v3. R2 et R3 designent leurs points par
        indice explicite, un 8e element ne les perturbe pas.
        """
        return super()._fk_points(angles) + [
            (self._matrice_bride(angles) @ self._GRIPPER_TIP)[:3].copy()]

    def _matrice_bride(self, angles):
        M = np.eye(4)
        for (xyz, rpy), th in zip(self._JOINT_TF, angles):
            T = np.eye(4)
            T[:3, :3] = self._rpy_mat(*rpy)
            T[:3, 3] = xyz
            c, s = math.cos(th), math.sin(th)
            Rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            M = M @ T @ Rz
        return M

    # Nombre de VUES par pose du bras. Le bras ne bouge pas entre elles, seule
    # la camera est retiree.
    #
    # Pourquoi pas une marche a petits pas facon capture reelle : mesure du
    # 02/09 sur 1950 poses, le pas de 1 deg ramene le plus proche voisin median
    # de 73,5 a 1,7 deg et l'ecart-type moyen des joints de 82,8 a 63,9 deg. Le
    # jeu se remplit de quasi-doublons et couvre moins l'espace articulaire.
    # La densite de trajectoire etait la bonne reponse pour la capture REELLE
    # (`capture_trajectoires.py`), ou le temps de banc est la ressource rare et
    # ou le bras doit se deplacer par petits pas pour sa securite ; en
    # simulation on teleporte, la contrainte n'existe pas.
    #
    # Ce qui compte vraiment contre la memorisation, c'est qu'une MEME pose du
    # bras apparaisse a des endroits differents de l'image : le reseau ne peut
    # alors plus associer le montage a une position fixe du socle. D'ou N vues
    # par pose, comme les 3 cameras de `panda-3cam` chez NVlabs et les 4 vues
    # du v3 — a ceci pres qu'elles ne sont plus figees.
    VUES_PAR_POSE = 3
    # Attente pour une simple vue supplementaire : le bras est deja immobile,
    # seul le rendu doit suivre la camera. La garde anti-gel (CRC32) du v3
    # rattrape une trame qui n'aurait pas ete rafraichie.
    TEMPS_VUE = 0.7


    # Rayons de capsule : les 5 du v3, plus un 6e pour la pince (bride -> bout).
    # 0,050 et non le rayon d'un doigt : il doit couvrir l'enveloppe des DEUX
    # doigts ecartes. Valeurs reprises de `synthetic_data_collector_v4_gripper`.
    LINK_RADII = [0.055, 0.045, 0.040, 0.036, 0.045, 0.050]
    # Paires pince <-> liens ELOIGNES. On ne teste pas (3,5) ni (4,5) : poignet,
    # bride et pince forment une unite naturellement proche, les tester
    # rejetterait les poses tendues (meme raison que l'exclusion (2,4) du v3).
    PAIRES_PINCE = [(0, 5), (1, 5), (2, 5)]
    # Sous-divisions verifiees entre la pose courante et la suivante.
    PAS_TRAJET = 14
    ESSAIS_TRAJET = 40



    # Les listes du v3 ne correspondent PAS au monde `randomized.sdf`, qui ne
    # contient que : ground_plane, back_wall, side_panel, table, clutter_box_1,
    # clutter_box_2, clutter_cylinder.
    #   CLUTTER_MODELS du v3 : 12 noms, dont 10 inexistants (clutter_box_3..6,
    #     clutter_cylinder_1..4, clutter_sphere_1..2) — et le vrai cylindre,
    #     `clutter_cylinder`, n'y figure pas.
    #   SURFACE_MODELS du v3 : `left_wall` et `right_wall` n'existent pas,
    #     `side_panel` manque.
    # Un `gz service visual_config` vers un modele inexistant ne recoit jamais de
    # reponse et son processus reste suspendu ; le v3 les lance en Popen sans
    # jamais les attendre. Mesure du 02/09 : 12 processus fantomes par tour de
    # randomisation, 352 accumules en 5 h, la decouverte gz sature et TOUS les
    # appels finissent par expirer — 2 756 poses de camera perdues.
    # Consequence collaterale, dans le v3 comme dans les jeux deja produits :
    # seuls 2 modeles sur 12 voyaient reellement leur couleur changer.
    CLUTTER_MODELS = ['clutter_box_1', 'clutter_box_2', 'clutter_cylinder']
    SURFACE_MODELS = ['back_wall', 'side_panel', 'table', 'ground_plane']

    # Obstacles fixes du monde `randomized.sdf`, modelises en cylindres
    # VERTICAUX (x, y, z_bas, z_haut, rayon). Le rayon d'une boite est son rayon
    # CIRCONSCRIT en XY : conservateur, il ne depend pas du lacet de la boite.
    #   clutter_box_1   (0.4, 0.5, 0.05)  0.10x0.08x0.10 -> r = hypot(.05,.04)
    #   clutter_box_2   (-0.3, 0.4, 0.075) 0.12x0.06x0.15 -> r = hypot(.06,.03)
    #   clutter_cylinder (0.3, -0.4, 0.1)  r 0.05, h 0.2
    # Les trois sont `static` et ne sont jamais deplaces — la randomisation du
    # v3 ne touche que leur COULEUR. box_1 est a 0,64 m du socle, hors de portee
    # (allonge + pince ~ 0,50 m) ; les deux autres sont a 0,50 m, donc atteignables
    # — et effectivement traverses, observe le 02/09 sur le cylindre bleu.
    OBSTACLES = [
        (0.40, 0.50, 0.0, 0.10, 0.064),
        (-0.30, 0.40, 0.0, 0.15, 0.067),
        (0.30, -0.40, 0.0, 0.20, 0.050),
    ]
    MARGE_OBSTACLE = 0.030

    def _is_collision_free(self, angles):
        """Ajoute la pince a l'auto-collision, que le v3 ne pouvait pas prevoir.

        Le R3 du v3 designe ses points par indice explicite
        (`skel = [pts[0], pts[1], pts[3], pts[4], pts[5], pts[6]]`) : le 8e point
        ajoute par `_fk_points` entre dans la garde au SOL mais reste invisible a
        l'auto-collision. Mesure du 02/09 sur 701 poses collectees : distance
        pince<->bras de 24 mm au minimum et **114 poses sur 701 (16 %)** sous le
        seuil des capsules — la pince traversait reellement le bras.

        Ajoute aussi les obstacles fixes du monde, qu'aucun controle ne voyait :
        la pince traversait le cylindre bleu, a 0,50 m du socle donc a portee.
        """
        if not super()._is_collision_free(angles):
            return False
        pts = self._fk_points(angles)
        skel = [pts[0], pts[1], pts[3], pts[4], pts[5], pts[6], pts[7]]
        segs = [(skel[i], skel[i + 1]) for i in range(6)]
        for i, j in self.PAIRES_PINCE:
            seuil = self.LINK_RADII[i] + self.LINK_RADII[j] + self.SELF_MARGIN
            if self._seg_seg_dist(*segs[i], *segs[j]) < seuil:
                return False

        for x, y, z0, z1, rayon in self.OBSTACLES:
            axe = (np.array([x, y, z0]), np.array([x, y, z1]))
            for i, seg in enumerate(segs):
                if self._seg_seg_dist(*seg, *axe) < self.LINK_RADII[i] + rayon + self.MARGE_OBSTACLE:
                    return False
        return True

    def _trajet_sur(self, a, b):
        """Le CHEMIN entre deux poses, et pas seulement ses extremites.

        `_is_collision_free` protege une pose immobile. Le controleur interpole
        en espace articulaire entre la pose courante et la suivante, et avec des
        tirages independants le saut atteint 80 deg : observe le 02/09, les
        doigts traversent la table EN CHEMIN alors que les deux poses sont
        valides. Meme piege que sur le robot reel, ou `capture_markerless.py` a
        du ajouter le meme controle.
        """
        a, b = np.asarray(a, float), np.asarray(b, float)
        return all(self._is_collision_free(list(a + (b - a) * (k / self.PAS_TRAJET)))
                   for k in range(self.PAS_TRAJET + 1))


    # Fraction des poses tirees dans la distribution REELLE du pick, le reste
    # uniforme pour la couverture generale.
    #
    # Pourquoi. Mesure du 02/09 : le tirage uniforme englobe la plage reelle sur
    # CHAQUE joint pris separement (J1 13,8 % du synthetique dedans, J2 4,8 %,
    # J3 9,2 %) mais **0,0 %** des poses y sont sur les six axes A LA FOIS. En
    # six dimensions, couvrir chaque axe ne veut rien dire : le synthetique ne
    # visite jamais la region ou le robot travaille.
    #
    # `grasp_fraction` du v3 ne repond pas : ses poses d'approche mettent la
    # bride a un rayon de 162 mm et z 339 mm, quand le pick reel travaille a
    # 272-324 mm de rayon et z 116-225 — une autre region, pas une autre branche
    # (J2 y sort a +38..+82 contre -131..-110 en reel).
    #
    # Les 1102 poses reelles passent nos gardes : 98,6 % a 50 mm de garde au
    # sol, 100 % a 30 mm, aucun conflit pince<->bras ni obstacle.
    FRACTION_PICK = 0.4
    JITTER_PICK_DEG = 6.0
    POSES_PICK = 'training/dream/captures/real_montage_0901/labels.csv'

    def _charge_poses_pick(self):
        chemin = Path(__file__).resolve().parents[2] / self.POSES_PICK
        if not chemin.is_file():
            self.get_logger().warn(
                f'{chemin} absent — aucune pose du pick, tirage uniforme seul')
            return []
        vus, poses = set(), []
        with chemin.open() as f:
            for r in csv.DictReader(f):
                if r['index'] in vus:
                    continue
                vus.add(r['index'])
                poses.append([math.radians(float(r[f'j{i}_deg'])) for i in range(1, 7)])
        self.get_logger().warn(
            f'{len(poses)} poses reelles du pick chargees — {self.FRACTION_PICK*100:.0f} % '
            f'du tirage s y fera, jitter +-{self.JITTER_PICK_DEG:.0f} deg')
        return poses

    def _pose_pick(self):
        """Une pose reelle du pick, bruitee, si elle passe les gardes."""
        j = math.radians(self.JITTER_PICK_DEG)
        for _ in range(20):
            base = random.choice(self._poses_pick)
            a = [v + random.uniform(-j, j) for v in base]
            if self._is_collision_free(a):
                return a
        return None

    def _random_joint_angles(self):
        """Meme pose pendant `VUES_PAR_POSE` echantillons, puis une nouvelle.

        `_collect_next` lit `self.settle_time` APRES cet appel : on l'abaisse
        donc ici pour les vues repetees, ou il n'y a rien a stabiliser.
        """
        if self._vues_restantes > 0:
            self._vues_restantes -= 1
            self.settle_time = self.TEMPS_VUE
            return list(self._pose_courante)
        for _ in range(self.ESSAIS_TRAJET):
            candidate = None
            if self._poses_pick and random.random() < self.FRACTION_PICK:
                candidate = self._pose_pick()
            if candidate is None:
                candidate = list(super()._random_joint_angles())
            if self._pose_courante is None or self._trajet_sur(self._pose_courante, candidate):
                self._pose_courante = candidate
                self._vues_restantes = self.VUES_PAR_POSE - 1
                self.settle_time = self._settle_plein
                return list(candidate)
        # Aucun chemin sur : on ne force pas le passage, on prend une vue de plus
        # de la pose courante et on retentera au prochain echantillon.
        self._n_trajets_bloques += 1
        self.settle_time = self.TEMPS_VUE
        return list(self._pose_courante)

    # ------------------------------------------------------------------
    # Tirage de la pose
    # ------------------------------------------------------------------

    # Duree au-dela de laquelle un appel `gz service` lance par le collecteur est
    # considere comme bloque et tue.
    VIE_MAX_APPEL = 10.0
    PERIODE_ECLAIRAGE = 40

    def _moissonne_appels_gz(self):
        """Tue les `gz-transport-service` bloques issus de ce processus.

        `_gz_light_cmd` et `_gz_visual_cmd` du v3 lancent leurs appels en
        `Popen(shell=True)` et ne les attendent JAMAIS. Or un `gz service` se
        bloque regulierement — mesure du 02/09 : 8 `light_config` et 4
        `visual_config` suspendus jusqu'a 680 s, alors meme que `sun`,
        `fill_light` et `warm_point` existent bel et bien. Ils s'accumulent, la
        decouverte gz sature, et ce sont MES `set_pose` qui finissent par
        expirer : 352 processus et 2 756 poses de camera perdues en 5 h.

        On ramasse plutot que de redefinir les deux methodes du v3 : ce filet
        couvre aussi tout appel que le v3 ajouterait plus tard.

        Sans condition d'ascendance, volontairement. Quand les processus
        intermediaires (`sh`, puis `gz`) meurent, le `gz-transport-service` est
        readopte par init : un test « descend-il du collecteur ? » ne le voit
        plus, et c'est exactement ce cas qui laissait 13 processus survivre.
        Aucun autre programme de ce banc ne lance de `gz-transport-service` :
        le serveur Gazebo est `gz sim` (ruby), les ponts sont `parameter_bridge`
        et `image_bridge`.
        """
        maintenant = time.time()
        try:
            ticks = os.sysconf('SC_CLK_TCK')
            btime = next(int(l.split()[1]) for l in open('/proc/stat')
                         if l.startswith('btime '))
        except (OSError, StopIteration, ValueError):
            return
        tues = 0
        for entree in os.scandir('/proc'):
            if not entree.name.isdigit():
                continue
            try:
                champs = open(f'/proc/{entree.name}/stat').read().rsplit(') ', 1)
                if not champs[0].split('(', 1)[1].startswith('gz-transport'):
                    continue
                age = maintenant - (btime + int(champs[1].split()[19]) / ticks)
            except (OSError, IndexError, ValueError):
                continue
            if age < self.VIE_MAX_APPEL:
                continue
            try:
                os.kill(int(entree.name), signal.SIGKILL)
                tues += 1
            except (ProcessLookupError, PermissionError):
                pass
        if tues:
            self._n_appels_moissonnes += tues

    def _tire_pose(self):
        """Position sur une coquille, orientee vers un point vise jitte.

        En SDF la camera regarde le long de +X du lien : le lacet et le tangage
        sont donc calcules pour amener +X sur la direction camera -> visee.
        """
        for _ in range(self.ESSAIS_MAX):
            azimut = random.uniform(-math.pi, math.pi)
            elev = random.uniform(self.ELEV_MIN, self.ELEV_MAX)
            dist = random.uniform(self.DIST_MIN, self.DIST_MAX)
            cx = self.VISEE[0] + dist * math.cos(elev) * math.cos(azimut)
            cy = self.VISEE[1] + dist * math.cos(elev) * math.sin(azimut)
            cz = self.VISEE[2] + dist * math.sin(elev)
            if cz < 0.25:                      # jamais sous le plan de travail
                continue

            visee = [c + random.uniform(-self.VISEE_JEU, self.VISEE_JEU)
                     for c in self.VISEE]
            dx, dy, dz = visee[0] - cx, visee[1] - cy, visee[2] - cz
            n = math.sqrt(dx * dx + dy * dy + dz * dz)
            lacet = math.atan2(dy, dx)
            tangage = -math.asin(max(-1.0, min(1.0, dz / n)))
            roulis = random.uniform(-self.ROULIS_MAX, self.ROULIS_MAX)

            if self._robot_dans_le_champ((cx, cy, cz), (roulis, tangage, lacet)):
                return (cx, cy, cz, roulis, tangage, lacet)
            self._n_rejets_cadrage += 1
        self.get_logger().warn('aucune pose camera cadrante trouvee — pose precedente gardee')
        return None

    def _robot_dans_le_champ(self, pos, rpy):
        """La sphere de travail tient-elle entierement dans l'image ?

        On ne projette pas la FK : la contrainte doit valoir pour TOUTES les
        configurations du bras, pas seulement celle de la trame. Une sphere de
        RAYON_ROBOT centree sur la visee nominale les englobe toutes.
        """
        cx, cy, cz = pos
        roulis, tangage, lacet = rpy
        dx = self.VISEE[0] - cx
        dy = self.VISEE[1] - cy
        dz = self.VISEE[2] - cz
        # passage dans le repere camera (X avant, Y gauche, Z haut) : rotation
        # inverse, roulis sans effet sur la distance angulaire a l'axe.
        cl, sl = math.cos(-lacet), math.sin(-lacet)
        x1, y1, z1 = dx * cl - dy * sl, dx * sl + dy * cl, dz
        ct, st = math.cos(-tangage), math.sin(-tangage)
        avant = x1 * ct + z1 * st
        haut = -x1 * st + z1 * ct
        gauche = y1
        distance = math.sqrt(avant * avant + gauche * gauche + haut * haut)
        if avant <= 0.0 or distance <= self.RAYON_ROBOT:
            return False
        # demi-angle sous lequel la sphere est vue depuis la camera
        ouverture = math.asin(self.RAYON_ROBOT / distance)
        ecart_h = math.atan2(abs(gauche), avant)
        ecart_v = math.atan2(abs(haut), avant)
        return (ecart_h + ouverture <= self._demi_h
                and ecart_v + ouverture <= self._demi_v)

    # ------------------------------------------------------------------
    # Application
    # ------------------------------------------------------------------
    @staticmethod
    def _quaternion(roulis, tangage, lacet):
        cr, sr = math.cos(roulis / 2), math.sin(roulis / 2)
        cp, sp = math.cos(tangage / 2), math.sin(tangage / 2)
        cy, sy = math.cos(lacet / 2), math.sin(lacet / 2)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)

    # Delai d'attente d'une reponse du pont. Genereux : la latence mesuree est
    # de l'ordre de la milliseconde, et une expiration ici coute une trame.
    ATTENTE_POSE = 5.0

    def _ouvre_client_pose(self):
        """Un client de service ROS unique, permanent, pour toute la collecte.

        Le noeud est SEPARE du collecteur et tourne dans son propre executeur :
        `_applique` est appelee depuis un callback de timer du collecteur, donc
        y attendre le futur sur l'executeur principal serait une reentrance.
        Avec un second noeud, `spin_until_future_complete` ne fait tourner que
        celui-la et le blocage reste local — meme comportement que l'attente du
        sous-processus qu'il remplace, mais en millisecondes.
        """
        self._noeud_pose = rclpy.create_node('cam_libre_set_pose_client')
        self._cli_pose = self._noeud_pose.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')
        self._exec_pose = SingleThreadedExecutor()
        self._exec_pose.add_node(self._noeud_pose)
        if not self._cli_pose.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(
                f'/world/{self.world_name}/set_pose absent apres 30 s — le pont '
                f'de service n est pas lance. Sans lui la camera ne bougerait '
                f'pas et toute la collecte serait un point de vue unique, '
                f'exactement le defaut qu on corrige. Collecte interrompue.')
            rclpy.shutdown()
            return
        self.get_logger().warn('set_pose via le pont ROS — aucun processus lance')

    def _applique(self, pose):
        """Applique la pose et ne l'enregistre QUE si l'appel a abouti.

        En cas d'echec on ne sait pas si la pose a ete appliquee ou si seule la
        reponse s'est perdue. On s'abstient donc d'ecrire la ligne : sans pose
        pour cet index, `convert_to_ndds_camera_libre` ECARTE la trame. Perdre
        quelques trames sur 20 000 ne coute rien ; en etiqueter une avec une
        pose de camera qui n'est pas la sienne empoisonnerait l'entrainement,
        precisement le defaut qu'on est en train de corriger.
        """
        x, y, z, roulis, tangage, lacet = pose
        qx, qy, qz, qw = self._quaternion(roulis, tangage, lacet)
        req = SetEntityPose.Request()
        req.entity = Entity(name=self.NOM_MODELE, type=Entity.MODEL)
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
        (req.pose.orientation.x, req.pose.orientation.y,
         req.pose.orientation.z, req.pose.orientation.w) = qx, qy, qz, qw

        futur = self._cli_pose.call_async(req)
        self._exec_pose.spin_until_future_complete(
            futur, timeout_sec=self.ATTENTE_POSE)
        if futur.done() and futur.result() is not None and futur.result().success:
            with open(self.chemin_poses, 'a', newline='') as f:
                csv.writer(f).writerow(
                    [self.sample_idx] + [round(v, 6) for v in pose])
            return True
        if not futur.done():
            futur.cancel()
        self._n_echecs_pose += 1
        self.get_logger().warn(
            f'[{self.sample_idx}] set_pose sans reponse — trame non etiquetee, '
            f'elle sera ecartee a la conversion (total {self._n_echecs_pose})')
        return False

    def _capture_and_save(self, target_angles):
        """Refuse d'enregistrer si `/joint_states` est muet.

        Le v3 retombe alors sur les angles COMMANDES (`angles = self.current_joints
        if self.current_joints else target_angles`). Avec la pince montee le bras
        flechit, commande et mesure different, et le jeu partirait avec des
        etiquettes fausses sans que rien ne le signale. Mesure du 02/09 :
        `joint_state_broadcaster` peut echouer a s'activer sur l'URDF avec pince.
        On s'arrete bruyamment plutot que de produire 20 000 trames a jeter.
        """
        if not self.current_joints:
            self.get_logger().error(
                'AUCUN /joint_states — joint_state_broadcaster n est pas actif. '
                'Les etiquettes seraient les angles COMMANDES et non mesures. '
                'Collecte interrompue.')
            rclpy.shutdown()
            return

        # Le v3 valide la pose COMMANDEE mais enregistre la pose MESUREE : sous
        # la gravite, pince montee, les deux different. Une pose validee peut
        # donc etre enregistree en collision. On revalide donc ce qui va
        # reellement etre ecrit — c'est ce qui rend la garantie « aucune image en
        # collision » vraie par construction, et pas seulement en moyenne.
        # (Le v3 ne fait ce controle que sur les poses forcees, jamais sur les
        # poses stabilisees.)
        if not self._is_collision_free(list(self.current_joints)):
            self.n_skipped += 1
            self._n_collisions_mesurees += 1
            self.get_logger().warn(
                f'[{self.sample_idx}] pose MESUREE en collision alors que la '
                f'commande etait valide — image ecartee '
                f'(total {self._n_collisions_mesurees})')
            self._collect_next()
            return
        return super()._capture_and_save(target_angles)

    def _randomize_scene(self):
        """Point d'accroche : appele en tete de chaque echantillon, AVANT la
        commande articulaire — le temps de stabilisation couvre donc aussi la
        remise a jour du rendu. La garde anti-gel du v3 (CRC32) attrape un rendu
        qui n'aurait pas suivi.
        """
        self._moissonne_appels_gz()
        # `set_pose` passe desormais par le pont ROS, mais l'eclairage et les
        # couleurs du v3 restent des `gz service` : 3 `light_config` + 7
        # `visual_config` par declenchement. C'est la derniere source
        # d'empoisonnement de la decouverte gz (voir le pont dans le launch), et
        # a la periode 3 d'origine cela ferait encore 66 000 appels sur 20 000
        # images — bien au-dela des ~11 000 ou la collecte d'hier etait deja
        # tombee a 50 %. A 40, il en reste 5 000, sous le seuil ou le premier
        # echec apparaissait.
        #
        # La diversite ne souffre pas : ce que l'eclairage fixe pendant 40
        # trames, c'est l'INTENSITE et la couleur, pas la direction apparente —
        # la camera, elle, se replace a chaque trame, donc l'angle d'incidence
        # sur le bras change a chaque image. Restent 500 reglages distincts sur
        # la collecte, la ou un jeu reel n'en a qu'un.
        #
        # Surtout : ces appels sont en tir-et-oublie. Meme si tous echouaient,
        # aucune trame ne serait perdue — l'integrite des etiquettes ne depend
        # plus du tout de gz-transport.
        if self.sample_idx % self.PERIODE_ECLAIRAGE == 0:
            super()._randomize_scene()
        pose = self._tire_pose()
        if pose is not None:
            self._applique(pose)


def main(args=None):
    rclpy.init(args=args)
    node = CollecteurCameraLibre()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
