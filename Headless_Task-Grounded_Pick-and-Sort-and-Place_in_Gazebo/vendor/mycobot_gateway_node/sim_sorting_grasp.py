#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tri des quatre objets par saisie PHYSIQUE dans Gazebo.

Remplace sorting_orchestrator pour le banc `sim_grasp.launch.py` : plus de
teleportation via set_pose, plus de `/model/.../cmd_pos`. Le bras passe par le
JTC `mycobot_controller`, la pince par `gripper_position_controller`, et chaque
prise est VERIFIEE sur la pose Gazebo de l'objet (il monte avec les doigts).

Trois chiffres mesures gouvernent le cycle, tous issus des meshes de la pince
(`pro_adaptive_gripper/*.dae`) et verifies en simulation :

  * le centre des patins est a 166 mm de la bride sur +Z du link6, decale
    de 7.8 mm sur +Y — c'est LUI le point outil, pas l'extremite du doigt ;
  * l'ouverture entre les faces internes vaut 120 mm a l'angle 0 et se ferme
    vers 1.11 rad — d'ou `_SPAN_TABLE`, qui donne l'angle pour une largeur ;
  * l'encombrement EXTERIEUR des doigts refermes (78 a 93 mm) depasse
    l'ouverture utile d'un bac (95 mm de libre pour 100 mm hors-tout). Les
    doigts ne peuvent donc pas entrer dans le bac : on lache a `DROP_TIP_Z`,
    ou l'objet — qui pend sous la pointe — est deja sous le rebord alors que
    les doigts restent au-dessus.

Le bac vert impose la seule vraie contrainte cinematique : son azimut (164.7°)
demande J1 ≈ 187° a l'outil sorti, au-dela de la butee. Il n'est atteignable
que par la branche « par-dessus l'epaule » (J1 ≈ -35°, J3 > 0, J5 < 0), que
`_solve_tip` trouve grace au germe `_seed_over_shoulder`.

Lancement (le banc doit tourner) :
  ros2 launch mycobot_gateway sim_grasp.launch.py
  ros2 run mycobot_gateway sim_sorting_grasp
"""

import math
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_SCRIPTS = Path(__file__).resolve().parents[2] / 'scripts'
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from diff_ik import fk_pose, solve_pose  # noqa: E402


ARM_JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]

# Les quatre joints de la pince, dans l'ordre impose par controller.yaml :
# [servo gauche, servo droit, bout gauche, bout droit]. Fermer = [-a, a, a, -a].
GRIPPER_JOINTS = [
    'gripper_controller', 'gripper_base_to_gripper_right3',
    'gripper_left3_to_gripper_left1', 'gripper_right3_to_gripper_right1',
    'gripper_base_to_gripper_left2', 'gripper_base_to_gripper_right2',
]
GRIPPER_LIMITS = [(-1.20, 0.10), (-0.10, 1.20), (-1.10, 1.10), (-1.10, 1.10),
                  (-1.20, 0.10), (-0.10, 1.20)]
# Butee logicielle : les bouts saturent a 1.10 et les faces se touchent vers
# 1.11. Serrer au-dela n'ajoute aucune force, ne fait que planter les doigts
# l'un dans l'autre.
MAX_CLOSE = 1.02
SQUEEZE_MM = 4.0

# Centre du PATIN de contact dans le repere link6 — pas l'extremite du
# doigt. Les patins ne font que 15 mm de large : viser l'extremite laissait
# 15 mm d'erreur laterale, assez pour que les doigts se referment a cote
# d'un cylindre de 44 mm sans jamais le toucher (mesure du 31/08 : les
# quatre joints atteignaient la consigne au millieme, donc zero contact).
TOOL_OFFSET = np.array([-0.001, 0.0078, 0.166])

# Ouverture entre faces internes (mm) selon l'angle servo (rad), mesuree sur
# les meshes de la pince.
_SPAN_TABLE = [(0.00, 120.0), (0.20, 104.1), (0.40, 84.7), (0.60, 62.7),
               (0.70, 50.9), (0.80, 38.8), (0.90, 26.6), (1.00, 14.2),
               (1.10, 1.9)]

# Demi-encombrement EXTERIEUR des doigts (mm) selon l'angle servo : c'est lui
# qui doit rester sous BIN_INNER_HALF_MM tant que la pince est dans le bac.
_FOOTPRINT_TABLE = [(0.00, 80.8), (0.20, 72.8), (0.40, 63.3), (0.60, 52.3),
                    (0.70, 46.4), (0.80, 40.4), (0.90, 34.2), (1.00, 28.0)]

APPROACH_Z = 0.110     # survol avant descente
TRANSIT_Z = 0.110      # hauteur de transfert : le max atteignable a r=0.28
# L'objet est POSE au fond du bac, pas lache au-dessus : 1 mm de garde sous
# lui, puis on ouvre. Lacher de plus haut le faisait rebondir sur la paroi et
# rester couche sur le rebord (mesure du 31/08 : cube bleu a 44.6 deg).
#
# Les doigts peuvent descendre dans le bac parce que la collision demande DEUX
# conditions simultanees : etre sous le rebord (haut a 30 mm) ET plus ecarte
# que la paroi interne (+-47.5 mm). Refermes sur l'objet ils ne font que
# +-34 a +-44 mm ; c'est en s'OUVRANT qu'ils depassent (+-81 mm grand ouverts).
# D'ou l'ouverture en deux temps : juste de quoi liberer l'objet au fond, puis
# grand ouvert seulement apres etre remonte.
PLACE_CLEARANCE_M = 0.001    # garde sous l'objet au moment de le poser
BIN_INNER_HALF_MM = 47.5     # demi-ouverture utile d'un bac
BIN_FLOOR_Z = 0.002    # dessus du fond du bac
MIN_TRANSIT_Z = 0.060  # la pointe ne doit jamais passer sous ca en transit

IK_ITERATIONS = 60       # 150 ne gagnait rien : la tolerance est a 0.05 mm
JOINT_SPEED_DPS = 75.0   # vitesse articulaire visee, deg/s
SETTLE_TOL_DEG = 0.35    # arrive quand l'ecart passe sous ca

JOINT_LIMITS_DEG = np.array([(-168., 168.), (-135., 135.), (-150., 150.),
                             (-145., 145.), (-165., 165.), (-180., 180.)])


class Target:
    """Un objet a trier : sa taille, la largeur a pincer, son bac."""

    def __init__(self, model, height, grip_mm, bin_xy, phi_deg=None,
                 squeeze_mm=SQUEEZE_MM):
        self.model = model
        self.height = height
        self.grip_mm = grip_mm
        self.bin_xy = bin_xy
        self.phi_deg = phi_deg          # None = laisse l'IK choisir
        self.squeeze_mm = squeeze_mm    # ecrasement commande sous la
                                        # largeur reelle = force de serrage


TARGETS = [
    # yellow_box fait 50x30x40 : on pince les 30 mm, donc doigts sur l'axe Y
    # du monde, phi = 90°. Les autres sont symetriques, phi libre.
    Target('red_cube',       0.040, 40.0, (-0.22, -0.18)),
    Target('blue_cube',      0.050, 50.0, (-0.22, -0.06)),
    # Un cylindre ne touche les patins que sur une ligne : il faut serrer
    # plus fort qu'une face plane pour qu'il ne file pas a la levee.
    Target('green_cylinder', 0.050, 44.0, (-0.22,  0.06), squeeze_mm=6.0),
    Target('yellow_box',     0.040, 30.0, (-0.22,  0.18), phi_deg=90.0),
]


def angle_for_span(span_mm: float) -> float:
    """Angle servo (rad) fermant les doigts a `span_mm` entre faces internes."""
    widths = [w for _, w in _SPAN_TABLE][::-1]
    angles = [a for a, _ in _SPAN_TABLE][::-1]
    return float(min(np.interp(span_mm, widths, angles), MAX_CLOSE))


def footprint_half_mm(angle: float) -> float:
    """Demi-encombrement exterieur des doigts a cet angle de fermeture."""
    angles = [a for a, _ in _FOOTPRINT_TABLE]
    halves = [h for _, h in _FOOTPRINT_TABLE]
    return float(np.interp(angle, angles, halves))


def rotation_top_down(phi_rad: float) -> np.ndarray:
    """Bride outil vers le bas ; `phi` oriente l'axe d'ouverture des doigts."""
    z = np.array([0.0, 0.0, -1.0])
    x = np.array([math.cos(phi_rad), math.sin(phi_rad), 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def tool_tip(q_deg: np.ndarray) -> np.ndarray:
    p_mm, rot = fk_pose(q_deg)
    return p_mm / 1000.0 + rot @ TOOL_OFFSET


def _wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def limit_margin(q_deg: np.ndarray) -> float:
    return float(np.min(np.minimum(q_deg - JOINT_LIMITS_DEG[:, 0],
                                   JOINT_LIMITS_DEG[:, 1] - q_deg)))


class SimSortingGrasp(Node):

    def __init__(self):
        super().__init__('sim_sorting_grasp')
        self.declare_parameter('world_name', 'pick_and_place_sorting')
        self.declare_parameter('move_duration', 0.0)   # 0 = dimensionnee au trajet
        self.declare_parameter('settle_time', 1.5)     # plafond d'attente
        self.declare_parameter('only', '')
        self.declare_parameter('startup_timeout', 60.0)
        self.world = str(self.get_parameter('world_name').value)
        move_dur = float(self.get_parameter('move_duration').value)
        self.move_dur = move_dur if move_dur > 0.0 else None
        self.settle = float(self.get_parameter('settle_time').value)
        only = str(self.get_parameter('only').value)
        self.only = [m.strip() for m in only.split(',') if m.strip()]
        self.startup_timeout = float(self.get_parameter('startup_timeout').value)
        if not math.isfinite(self.startup_timeout) or self.startup_timeout <= 0.0:
            raise ValueError('startup_timeout doit etre positif et fini')
        self.targets = []
        for target in TARGETS:
            param = f'bin_xy.{target.model}'
            self.declare_parameter(param, list(target.bin_xy))
            bin_xy = tuple(self.get_parameter(param).value)
            if len(bin_xy) != 2 or not all(math.isfinite(v) for v in bin_xy):
                raise ValueError(
                    f'{param} doit contenir deux coordonnees finies en metres')
            self.targets.append(Target(
                target.model, target.height, target.grip_mm, bin_xy,
                phi_deg=target.phi_deg, squeeze_mm=target.squeeze_mm))

        self.q_deg: Optional[np.ndarray] = None
        self.grip_pos: Optional[float] = None

        self.pub_arm = self.create_publisher(
            JointTrajectory, '/mycobot_controller/joint_trajectory', 10)
        self.pub_grip = self.create_publisher(
            Float64MultiArray, '/gripper_position_controller/commands', 10)
        self.pub_status = self.create_publisher(String, '/pickplace/status', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.controller_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')

    # ── etat ────────────────────────────────────────────────────────────
    def _joint_cb(self, msg: JointState):
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS):
            self.q_deg = np.degrees(
                [msg.position[names.index(j)] for j in ARM_JOINTS])
        if GRIPPER_JOINTS[0] in names:
            self.grip_pos = msg.position[names.index(GRIPPER_JOINTS[0])]

    def spin_for(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=text))

    def object_poses(self) -> Dict[str, np.ndarray]:
        """Poses Gazebo des modeles mobiles — la seule preuve de saisie."""
        out = subprocess.run(
            ['gz', 'topic', '-e', '-n', '1',
             '-t', f'/world/{self.world}/dynamic_pose/info'],
            capture_output=True, text=True, timeout=10).stdout
        poses: Dict[str, np.ndarray] = {}
        for block in out.split('pose {')[1:]:
            name = re.search(r'name: "([^"]+)"', block)
            pos = re.search(
                r'position \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)',
                block)
            if name and pos:
                poses[name.group(1)] = np.array(
                    [float(pos.group(i)) for i in (1, 2, 3)])
        return poses

    # ── cinematique ─────────────────────────────────────────────────────
    def _seeds(self, tip: np.ndarray) -> List[np.ndarray]:
        """Germes couvrant les deux branches d'epaule et les deux du poignet.

        L'outil sort a ~22° d'azimut de J1, ce qui rend la facade avant
        inatteignable au-dela de 146° : la branche par-dessus l'epaule
        (J1 ≈ az-180) est la seule qui serve le bac vert.
        """
        az = math.degrees(math.atan2(tip[1], tip[0]))
        seeds = [
            [_wrap180(az + 22), -27., -58., -4., 90., 0.],
            [_wrap180(az + 22), 0., -92., 3., 90., 0.],
            [_wrap180(az + 22), -16., -94., 20., 90., 97.],
            [_wrap180(az - 22), -27., -58., -4., -90., 0.],
            [_wrap180(az - 200), 3., 90., -3., -90., 0.],
            [_wrap180(az - 160), 3., 90., -3., -90., 0.],
        ]
        if self.q_deg is not None:
            seeds.insert(0, list(self.q_deg))
        return [np.array(s, dtype=float) for s in seeds]

    def _solve_at_phi(self, tip, phi_deg, q_ref):
        """Meilleure solution pour UN phi donne, ou None."""
        rot = rotation_top_down(math.radians(phi_deg))
        flange_mm = (np.asarray(tip) - rot @ TOOL_OFFSET) * 1000.0
        best = None
        for seed in self._seeds(tip):
            q = solve_pose(seed, flange_mm, rot, iterations=IK_ITERATIONS)
            got_mm, got_rot = fk_pose(q)
            if float(np.linalg.norm(got_mm - flange_mm)) > 2.0:
                continue
            ang = math.degrees(np.arccos(
                np.clip((np.trace(got_rot @ rot.T) - 1) / 2, -1, 1)))
            if ang > 3.0:
                continue
            margin = limit_margin(q)
            if margin < 3.0:
                continue
            travel = (0.0 if q_ref is None
                      else float(np.max(np.abs(q - q_ref))))
            score = travel - 2.0 * min(margin, 30.0)
            if best is None or score < best[0]:
                best = (score, q)
            # Une solution franche (loin des butees, peu de trajet) ne sera pas
            # battue : inutile d'essayer les germes suivants.
            if margin > 20.0 and travel < 60.0:
                break
        return None if best is None else best[1]

    def solve_tip(self, tip, phi_deg=None, q_ref=None):
        """Angles amenant la POINTE des doigts sur `tip`, outil vers le bas."""
        q_ref = self.q_deg if q_ref is None else q_ref
        phis = ([phi_deg] if phi_deg is not None
                else list(range(0, 180, 15)))
        best = None
        for phi in phis:
            q = self._solve_at_phi(tip, phi, q_ref)
            if q is None:
                continue
            travel = (0.0 if q_ref is None
                      else float(np.max(np.abs(q - q_ref))))
            if best is None or travel < best[0]:
                best = (travel, q)
        return None if best is None else best[1]

    def solve_column(self, x, y, heights, phi_deg=None, q_ref=None):
        """Une pile de poses a la MEME orientation de poignet.

        Rebalayer phi a chaque hauteur faisait tourner le poignet entre la
        saisie et la levee, ce qui devissait l'objet des doigts. Un phi qui
        sert toutes les hauteurs d'un meme geste supprime cette rotation.
        """
        q_ref = self.q_deg if q_ref is None else q_ref
        phis = ([phi_deg] if phi_deg is not None
                else list(range(0, 180, 15)))
        best = None
        for phi in phis:
            column, prev = [], q_ref
            for z in heights:
                q = self._solve_at_phi([x, y, z], phi, prev)
                if q is None:
                    column = None
                    break
                column.append(q)
                prev = q
            if not column:
                continue
            travel = max(float(np.max(np.abs(column[0] - q_ref))),
                         *[float(np.max(np.abs(b - a)))
                           for a, b in zip(column, column[1:])]) \
                if len(column) > 1 else float(np.max(np.abs(column[0] - q_ref)))
            if best is None or travel < best[0]:
                best = (travel, phi, column)
            if best[0] < 75.0:      # assez bon, on ne balaie pas les 11 autres
                break
        return (None, None) if best is None else (best[2], best[1])

    def _path_clears_table(self, q_from, q_to, floor=MIN_TRANSIT_Z) -> bool:
        """La pointe reste-t-elle haute sur toute l'interpolation articulaire ?"""
        return all(tool_tip(q_from + t * (q_to - q_from))[2] >= floor
                   for t in np.linspace(0.05, 0.95, 19))

    # ── actionneurs ─────────────────────────────────────────────────────
    def move_to(self, q_deg, duration=None) -> np.ndarray:
        """Commande la pose et rend la main quand le bras y est VRAIMENT.

        Attendre une duree fixe genereuse coutait 6.5 s par mouvement, soit
        l'essentiel des ~3 min du cycle. On dimensionne la duree sur le trajet
        reel (JOINT_SPEED_DPS) puis on sort des que l'ecart passe sous
        SETTLE_TOL_DEG — un petit ajustement se termine en une fraction de
        seconde, un grand pivot prend le temps qu'il faut.
        """
        q_deg = np.asarray(q_deg, dtype=float)
        if duration is None:
            travel = float(np.max(np.abs(q_deg - self.q_deg)))
            duration = float(np.clip(travel / JOINT_SPEED_DPS, 0.6, 4.0))
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(math.radians(v)) for v in q_deg]
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        traj.points = [point]
        self.pub_arm.publish(traj)

        self.spin_for(duration)
        deadline = time.time() + self.settle
        while time.time() < deadline:
            if float(np.max(np.abs(self.q_deg - q_deg))) < SETTLE_TOL_DEG:
                break
            rclpy.spin_once(self, timeout_sec=0.02)
        return tool_tip(self.q_deg)

    def set_gripper(self, angle: float, settle: float = 1.5):
        """Ouvre/ferme les quatre joints, bornes sur les limites de l'URDF."""
        # Les deux dernieres sont les barres du parallelogramme : meme angle
        # que le servo de leur cote, sinon elles restent en croix.
        raw = [-angle, angle, angle, -angle, -angle, angle]
        cmd = [float(np.clip(v, lo, hi))
               for v, (lo, hi) in zip(raw, GRIPPER_LIMITS)]
        self.pub_grip.publish(Float64MultiArray(data=cmd))
        self.spin_for(settle)

    def open_gripper(self):
        self.set_gripper(0.0)

    # ── cycle ───────────────────────────────────────────────────────────
    def wait_until_ready(self):
        """Attendre aussi les actionneurs, pas seulement le broadcaster."""
        self.status('attente des controleurs actifs et de /joint_states…')
        required = {'mycobot_controller', 'gripper_position_controller'}
        deadline = time.monotonic() + self.startup_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if not self.controller_client.service_is_ready():
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            future = self.controller_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(
                self, future,
                timeout_sec=min(2.0, max(0.0, deadline - time.monotonic())))
            if future.done() and future.exception() is None:
                active = {c.name for c in future.result().controller
                          if c.state == 'active'}
                if (required <= active and self.q_deg is not None
                        and self.grip_pos is not None
                        and self.pub_arm.get_subscription_count() > 0
                        and self.pub_grip.get_subscription_count() > 0):
                    return
            elif not future.done():
                self.controller_client.remove_pending_request(future)
                future.cancel()
            rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError(
            'controleurs ou /joint_states indisponibles '
            f'apres {self.startup_timeout:.0f} s — le banc tourne-t-il ?')

    def transit(self, q_goal, label: str) -> bool:
        """Rejoint `q_goal` en garantissant que la pointe ne racle rien."""
        if self._path_clears_table(self.q_deg, q_goal):
            self.move_to(q_goal)
            return True
        # Le chemin direct plonge : on remonte d'abord a la verticale du point
        # de depart, ce qui rend le grand pivot inoffensif.
        tip_now = tool_tip(self.q_deg)
        q_up = self.solve_tip([tip_now[0], tip_now[1], TRANSIT_Z])
        if q_up is None or not self._path_clears_table(q_up, q_goal):
            self.status(f'  ⚠ {label} : aucun chemin sur, segment abandonne')
            return False
        self.move_to(q_up)
        self.move_to(q_goal)
        return True

    def sort_one(self, target: Target, poses: Dict[str, np.ndarray]) -> str:
        name = target.model
        if name not in poses:
            return 'objet absent de la scene'
        x, y, _ = poses[name]
        # La pointe se pose a mi-hauteur de l'objet : les patins mordent la
        # moitie haute et restent a 18 mm de la planche.
        grasp_z = max(0.018, target.height * 0.45)

        self.status(f'▶ {name} : saisie en ({x:+.3f}, {y:+.3f})')
        self.open_gripper()

        # Survol, saisie et levee partagent le meme phi : le poignet ne tourne
        # pas une fois les doigts sur l'objet.
        column, phi = self.solve_column(
            x, y, [APPROACH_Z, grasp_z, TRANSIT_Z], target.phi_deg)
        if column is None:
            return 'aucune orientation de poignet ne sert les trois hauteurs'
        q_above, q_grasp, q_lift = column
        self.status(f'  poignet phi={phi}°')

        if not self.transit(q_above, f'{name} survol'):
            return 'chemin de survol non sur'
        tip_reached = self.move_to(q_grasp)
        self.status(f'  pointe visee ({x:+.3f},{y:+.3f},{grasp_z:.3f}) '
                    f'atteinte {np.round(tip_reached, 4).tolist()}')

        angle = angle_for_span(target.grip_mm - target.squeeze_mm)
        self.status(f'  serrage {target.grip_mm - target.squeeze_mm:.0f} mm '
                    f'→ {angle:.3f} rad')
        self.set_gripper(angle, settle=3.0)

        self.move_to(q_lift)
        held = self.object_poses().get(name)
        if held is None or held[2] < 0.06:
            self.open_gripper()
            return f'prise ratee (objet reste a z={held[2]:.3f} m)'
        self.status(f'  tenu a z={held[2]:.3f} m')

        bx, by = target.bin_xy
        # On descend jusqu'a poser l'objet sur le fond du bac.
        place_z = BIN_FLOOR_Z + PLACE_CLEARANCE_M + target.height / 2.0
        release = angle_for_span(target.grip_mm)
        garde = BIN_INNER_HALF_MM - footprint_half_mm(release)
        if garde < 0.0:
            self.open_gripper()
            return (f'doigts trop larges pour le bac a l ouverture '
                    f'({footprint_half_mm(release):.0f} > {BIN_INNER_HALF_MM} mm)')

        bin_column, bin_phi = self.solve_column(
            bx, by, [TRANSIT_Z, place_z], q_ref=q_lift)
        if bin_column is None:
            self.open_gripper()
            return 'aucune pose de depot au-dessus du bac'
        q_over_bin, q_place = bin_column
        self.status(f'  bac : poignet phi={bin_phi}°, depot a z={place_z:.3f}, '
                    f'garde laterale {garde:.1f} mm')

        if not self.transit(q_over_bin, f'{name} vers bac'):
            self.open_gripper()
            return 'chemin vers le bac non sur'
        self.move_to(q_place)
        # L'objet repose deja sur le fond : rendre sa largeur exacte suffit a
        # annuler la force de serrage, sans que les doigts s'ecartent assez
        # pour toucher la paroi.
        self.set_gripper(release, settle=1.2)
        self.move_to(q_over_bin)
        self.open_gripper()

        landed = self.object_poses().get(name)
        if landed is None:
            return 'objet disparu'
        dx, dy = landed[0] - bx, landed[1] - by
        if abs(dx) < 0.047 and abs(dy) < 0.047 and landed[2] < 0.06:
            return (f'OK — dans le bac, ecart {dx * 1000:+.0f}/{dy * 1000:+.0f} mm '
                    f'du centre')
        return (f'hors du bac : ecart {dx * 1000:+.0f}/{dy * 1000:+.0f} mm, '
                f'z={landed[2]:.3f}')

    def run(self) -> Dict[str, str]:
        self.wait_until_ready()

        results: Dict[str, str] = {}
        for target in self.targets:
            if self.only and target.model not in self.only:
                continue
            poses = self.object_poses()
            try:
                results[target.model] = self.sort_one(target, poses)
            except Exception as exc:                     # noqa: BLE001
                results[target.model] = f'erreur : {exc}'
            self.status(f'  {target.model} : {results[target.model]}')

        self.open_gripper()
        q_home = self.solve_tip([0.25, 0.0, TRANSIT_Z])
        if q_home is not None:
            self.transit(q_home, 'retour')
        return results


def main(args=None):
    rclpy.init(args=args)
    node = SimSortingGrasp()
    try:
        results = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('\n' + '=' * 62)
    print('  TRI DES OBJETS — RESULTAT')
    print('=' * 62)
    for model, verdict in results.items():
        mark = '✔' if verdict.startswith('OK') else '✘'
        print(f'  {mark} {model:16} {verdict}')
    print('=' * 62)


if __name__ == '__main__':
    main()
