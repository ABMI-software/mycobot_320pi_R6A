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
    doigts ne peuvent donc pas entrer dans le bac : le dessous de l'objet
    a 5 mm au-dessus du rebord (`BIN_RIM_Z`), on ecarte les doigts, l'objet
    tombe au fond, puis on remonte.

Le bac vert impose la seule vraie contrainte cinematique : son azimut (164.7°)
demande J1 ≈ 187° a l'outil sorti, au-dela de la butee. Il n'est atteignable
que par la branche « par-dessus l'epaule » (J1 ≈ -35°, J3 > 0, J5 < 0), que
`_solve_tip` trouve grace au germe `_seed_over_shoulder`.

Lancement (le banc doit tourner) :
  ros2 launch mycobot_gateway sim_grasp.launch.py
  ros2 run mycobot_gateway sim_sorting_grasp
"""

import csv
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
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray

from .vision import tri_scene

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
BIN_INNER_HALF_MM = 47.5     # demi-ouverture utile d'un bac
MIN_TRANSIT_Z = 0.060  # la pointe ne doit jamais passer sous ca en transit

IK_ITERATIONS = 60       # 150 ne gagnait rien : la tolerance est a 0.05 mm
JOINT_SPEED_DPS = 75.0   # vitesse articulaire visee, deg/s
SETTLE_TOL_DEG = 0.35    # arrive quand l'ecart passe sous ca

# Perception : trois messages fusionnes successifs, d'accord a 4 mm pres. La
# fusion des 4 cameras tient 0,65 mm de mediane, 2,07 mm au pire (01/10).
PERCEPTION_SAMPLES = 3
# Ouverture d'approche dans la scene de tri : largeur pincee + 24 mm.
APPROACH_MARGIN_MM = 24.0
# Largage incline (bac hors de portee de l'outil vertical) : le dessous de
# l'objet a 5 mm au-dessus du rebord (30 mm), descente lente, ouverture en deux
# temps — l'objet est pose, pas jete. Approche et retrait 50 mm plus haut.
DROP_TILTS_DEG = (15.0, 30.0, 45.0)
DROP_ABOVE_RIM_M = 0.005
DROP_DESCENT_S = 2.0
BIN_RIM_Z = 0.030
DROP_RISE_M = 0.05
# Relacher = ecarter les doigts de 20 mm AVANT de remonter, sinon l'objet
# glisse entre eux pendant la montee (video 4 vues du 02/10).
RELEASE_ABOVE_RIM_EXTRA_MM = 20.0
RELEASE_SETTLE_S = 2.0
PERCEPTION_STABLE_M = 0.004

JOINT_LIMITS_DEG = np.array([(-168., 168.), (-135., 135.), (-150., 150.),
                             (-145., 145.), (-165., 165.), (-180., 180.)])


class Target:
    """Un objet a trier : sa taille, la largeur a pincer, son bac."""

    def __init__(self, model, height, grip_mm, bin_xy, phi_deg=None,
                 squeeze_mm=SQUEEZE_MM, phis=None):
        self.model = model
        self.height = height
        self.grip_mm = grip_mm
        self.bin_xy = bin_xy
        self.phi_deg = phi_deg          # None = laisse l'IK choisir
        self.phis = phis                # orientations permises si phi_deg est libre
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

# Scene de tri (tri_yolo.launch.py) : memes pieces aux noms des classes yolo26.
# Le bac vient de la perception (pose_source:=perception), d'ou bin_xy vide.
TRI_TARGETS = [
    # Un cube se pince par deux faces : a 45° les doigts tombent sur les aretes
    # (71 mm de diagonale pour 50) et le cube file (cube_bleu, graine 4, 01/10).
    # Pieces a lacet nul dans la scene V1, d'ou 0 ou 90°.
    Target('cube_rouge',    0.040, 40.0, None, phis=(0.0, 90.0)),
    Target('pave_jaune',    0.040, 30.0, None, phi_deg=90.0),
    Target('cylindre_vert', 0.050, 44.0, None, squeeze_mm=6.0),
    Target('cube_bleu',     0.050, 50.0, None, phis=(0.0, 90.0)),
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


def _rotation_about(axis, angle):
    """Rotation de `angle` (rad) autour de l'axe unitaire `axis` (Rodrigues)."""
    k = np.asarray(axis, dtype=float)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


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
        self.declare_parameter('pose_source', 'gazebo')
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('csv_path', '')     # un verdict par essai (protocole, etape 11)
        self.pose_source = str(self.get_parameter('pose_source').value)
        self.max_attempts = int(self.get_parameter('max_attempts').value)
        if self.pose_source not in ('gazebo', 'vision', 'perception') or self.max_attempts < 1:
            raise ValueError('pose_source must be gazebo/vision/perception and max_attempts >= 1')
        self.perceived = []
        self.vision_pose = None
        self.vision_received = 0.0
        self.world = str(self.get_parameter('world_name').value)
        move_dur = float(self.get_parameter('move_duration').value)
        self.move_dur = move_dur if move_dur > 0.0 else None
        self.settle = float(self.get_parameter('settle_time').value)
        only = str(self.get_parameter('only').value)
        self.only = [m.strip() for m in only.split(',') if m.strip()]
        if self.pose_source == 'vision' and self.only != ['red_cube']:
            raise ValueError('vision mode currently requires only:=red_cube')
        self.startup_timeout = float(self.get_parameter('startup_timeout').value)
        if not math.isfinite(self.startup_timeout) or self.startup_timeout <= 0.0:
            raise ValueError('startup_timeout doit etre positif et fini')
        self.targets = []
        if self.pose_source == 'perception':
            self.targets = list(TRI_TARGETS)
        for target in ([] if self.pose_source == 'perception' else TARGETS):
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
        if self.pose_source == 'vision':
            self.create_subscription(PoseStamped, '/vision/red_cube/pose', self._vision_cb, 10)
        if self.pose_source == 'perception':
            self.create_subscription(Detection3DArray, '/yolo/objects_3d', self._perception_cb, 10)
        self.controller_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')

    # ── etat ────────────────────────────────────────────────────────────
    def _vision_cb(self, msg):
        point = msg.pose.position
        xyz = np.array([point.x, point.y, point.z])
        if msg.header.frame_id != 'world' or not np.isfinite(xyz).all():
            return
        self.vision_pose = msg
        self.vision_received = time.monotonic()

    def _perception_cb(self, msg):
        self.perceived.append((time.monotonic(), {
            d.results[0].hypothesis.class_id: np.array(
                [d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z])
            for d in msg.detections}))
        self.perceived = self.perceived[-PERCEPTION_SAMPLES:]

    def perceived_poses(self, needed):
        """Pieces AND bins from /yolo/objects_3d only (protocol, step 9).

        Seen from the observation pose, where the arm hides 4.3 % of the board;
        `needed` classes in PERCEPTION_SAMPLES fresh messages that agree within
        PERCEPTION_STABLE_M, median of them.
        """
        q_observe = np.array(tri_scene.OBSERVATION_Q_DEG)
        if not self.transit(q_observe, 'pose d observation'):
            raise RuntimeError('pose d observation inatteignable sans racler')
        requested = time.monotonic()
        deadline = requested + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            fresh = [seen for t, seen in self.perceived if t > requested]
            if len(fresh) < PERCEPTION_SAMPLES or not all(needed <= set(s) for s in fresh):
                continue
            stack = {n: np.array([s[n] for s in fresh]) for n in needed}
            if all(np.max(np.ptp(v[:, :2], axis=0)) < PERCEPTION_STABLE_M for v in stack.values()):
                return {n: np.median(v, axis=0) for n, v in stack.items()}
        missing = needed - set(self.perceived[-1][1]) if self.perceived else needed
        raise RuntimeError(f'perception : pas de position stable (absentes : {sorted(missing)})')

    def target_poses(self, target=None):
        """Use fresh vision for aiming; Gazebo poses are only grasp verification."""
        if self.pose_source == 'gazebo':
            return self.object_poses()
        if self.pose_source == 'perception':
            return self.perceived_poses({target.model, tri_scene.PAIRS[target.model]})
        self.status('localisation du cube par les quatre cameras…')
        deadline = time.monotonic() + 25.0
        # Require a new observation after any previous motion or failed attempt.
        requested = time.monotonic()
        samples = []
        previous_stamp = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            msg = self.vision_pose
            if msg is None or self.vision_received <= requested:
                continue
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not 0 <= age <= 1.5 or stamp == previous_stamp:
                continue
            previous_stamp = stamp
            p = msg.pose.position
            xyz = np.array([p.x, p.y, p.z])
            if (not .14 <= np.linalg.norm(xyz[:2]) <= .32
                    or not .01 <= xyz[2] <= .03):
                samples.clear()
                continue
            samples.append(xyz)
            samples = samples[-3:]
            if len(samples) == 3 and np.max(np.ptp(samples, axis=0)) < .004:
                position = np.median(samples, axis=0)
                self.status(f'vision stable : {np.round(position, 4).tolist()} m')
                return {'red_cube': position}
        raise RuntimeError('pas de position visuelle recente et stable — mouvement annule')

    def _joint_cb(self, msg: JointState):
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS):
            self.q_deg = np.degrees(
                [msg.position[names.index(j)] for j in ARM_JOINTS])
        if GRIPPER_JOINTS[0] in names:
            self.grip_pos = msg.position[names.index(GRIPPER_JOINTS[0])]

    def spin_for(self, seconds: float):
        # Gazebo rendering can run slower than wall time with four cameras.
        # Gripper settling and trajectory durations are simulation seconds.
        end = self.get_clock().now().nanoseconds * 1e-9 + seconds
        watchdog = time.monotonic() + max(15.0, seconds * 15.0)
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end:
            if time.monotonic() > watchdog:
                raise RuntimeError('horloge de simulation arretee ou trop lente')
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
        return self._solve_at_rot(tip, rotation_top_down(math.radians(phi_deg)), q_ref)

    def _solve_at_rot(self, tip, rot, q_ref):
        """Meilleure solution pour UNE orientation d'outil, ou None."""
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

    def solve_column(self, x, y, heights, phi_deg=None, q_ref=None, phis=None):
        """Une pile de poses a la MEME orientation de poignet.

        Rebalayer phi a chaque hauteur faisait tourner le poignet entre la
        saisie et la levee, ce qui devissait l'objet des doigts. Un phi qui
        sert toutes les hauteurs d'un meme geste supprime cette rotation.
        """
        q_ref = self.q_deg if q_ref is None else q_ref
        phis = ([phi_deg] if phi_deg is not None
                else list(phis) if phis is not None else list(range(0, 180, 15)))
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

    def tilted_drop(self, bx, by, z_release, q_ref):
        """Outil incline vers l'exterieur au-dessus du bac, quand la verticale n'y va pas.

        Comme au banc reel (bacs a 0.36-0.45 m, 30/09 et 01/10) : la pointe
        au-dessus du centre du bac, on lache, l'objet tombe dans le bac.
        Rend (q_haut, q_largage, inclinaison) ou None.
        """
        az = math.atan2(by, bx)
        tangent = np.array([-math.sin(az), math.cos(az), 0.0])
        for tilt in DROP_TILTS_DEG:
            for phi in (math.degrees(az) + 90.0, math.degrees(az)):
                rot = _rotation_about(tangent, -math.radians(tilt)) @ \
                    rotation_top_down(math.radians(phi))
                q_release = self._solve_at_rot([bx, by, z_release], rot, q_ref)
                if q_release is None:
                    continue
                q_up = self._solve_at_rot([bx, by, z_release + DROP_RISE_M], rot, q_release)
                if q_up is not None:
                    return q_up, q_release, tilt
        return None

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
        deadline = time.monotonic() + max(5.0, self.settle * 10.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if float(np.max(np.abs(self.q_deg - q_deg))) < SETTLE_TOL_DEG:
                return tool_tip(self.q_deg)
            rclpy.spin_once(self, timeout_sec=0.02)
        raise RuntimeError('le bras simule n a pas atteint la consigne articulaire')

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
        if target.bin_xy is None:
            # Scene de tri : objets a 30 mm des bacs. Grand ouverts, les doigts
            # (+-81 mm) se posaient sur la paroi du bac voisin et se refermaient
            # dans le vide (pave_jaune, graine 5, 01/10).
            self.set_gripper(angle_for_span(target.grip_mm + APPROACH_MARGIN_MM))
        else:
            self.open_gripper()

        # Survol, saisie et levee partagent le meme phi : le poignet ne tourne
        # pas une fois les doigts sur l'objet.
        column, phi = self.solve_column(
            x, y, [APPROACH_Z, grasp_z, TRANSIT_Z], target.phi_deg, phis=target.phis)
        if column is None:
            return 'aucune orientation de poignet ne sert les trois hauteurs'
        q_above, q_grasp, q_lift = column
        self.status(f'  poignet phi={phi}°')

        # Le depot est resolu AVANT de toucher l'objet : un bac hors de portee
        # faisait lacher la piece au hasard sur la planche, contre une autre
        # (graine 4, 01/10).
        bx, by = (target.bin_xy if target.bin_xy is not None
                  else poses[tri_scene.PAIRS[name]][:2])
        # Lacher au-dessus du rebord, doigts hors du bac : dedans, la paroi
        # (95 mm) les empechait de s'ouvrir plus que le cube bleu (50 mm), qui
        # restait pince puis glissait a la remontee (02/10).
        z_release = BIN_RIM_Z + DROP_ABOVE_RIM_M + target.height / 2.0
        bin_column, bin_phi = self.solve_column(
            bx, by, [TRANSIT_Z, z_release], q_ref=q_lift)
        tilted = None
        if bin_column is None:
            tilted = self.tilted_drop(bx, by, z_release, q_lift)
            if tilted is None:
                self.open_gripper()
                return 'aucune pose de depot au-dessus du bac (ni verticale ni inclinee)'
            q_over_bin, q_place, tilt = tilted
            self.status(f'  bac a {math.hypot(bx, by) * 1000:.0f} mm : largage incline '
                        f'{tilt:.0f}°, objet a z={z_release:.3f}')
        else:
            q_over_bin, q_place = bin_column
            self.status(f'  bac : poignet phi={bin_phi}°, largage vertical, '
                        f'objet a z={z_release:.3f}')

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
            return ('prise ratee (pose de verification indisponible)' if held is None
                    else f'prise ratee (objet reste a z={held[2]:.3f} m)')
        self.status(f'  tenu a z={held[2]:.3f} m')


        if not self.transit(q_over_bin, f'{name} vers bac'):
            self.open_gripper()
            return 'chemin vers le bac non sur'
        self.move_to(q_place, duration=DROP_DESCENT_S)
        self.set_gripper(angle_for_span(target.grip_mm + RELEASE_ABOVE_RIM_EXTRA_MM),
                         settle=RELEASE_SETTLE_S)
        dropped = self.object_poses().get(name)
        if dropped is not None:
            self.status(f'  doigts ecartes, objet a z={dropped[2]:.3f} m avant la remontee')
        self.move_to(q_over_bin, duration=DROP_DESCENT_S)
        self.open_gripper()

        landed = self.object_poses().get(name)
        if landed is None:
            return 'objet disparu'
        dx, dy = landed[0] - bx, landed[1] - by
        if abs(dx) < 0.047 and abs(dy) < 0.047 and landed[2] < 0.06:
            return (f'OK — dans le bac, ecart {dx * 1000:+.0f}/{dy * 1000:+.0f} mm '
                    f'du centre, centre de l objet a z={landed[2] * 1000:.0f} mm')
        return (f'hors du bac : ecart {dx * 1000:+.0f}/{dy * 1000:+.0f} mm, '
                f'z={landed[2]:.3f}')

    def write_csv(self, rows):
        path = str(self.get_parameter('csv_path').value)
        if not path:
            return
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', newline='') as f:
            out = csv.DictWriter(f, fieldnames=('stamp_sim', 'object_id', 'attempt', 'ok', 'verdict'))
            out.writeheader()
            out.writerows(rows)

    def run(self) -> Dict[str, str]:
        self.wait_until_ready()

        results: Dict[str, str] = {}
        rows = []
        for target in self.targets:
            if self.only and target.model not in self.only:
                continue
            for attempt in range(self.max_attempts):
                try:
                    poses = self.target_poses(target)
                    results[target.model] = self.sort_one(target, poses)
                except Exception as exc:                 # noqa: BLE001
                    results[target.model] = f'erreur : {exc}'
                self.status(f'  {target.model} (essai {attempt + 1}) : {results[target.model]}')
                rows.append({'stamp_sim': f'{self.get_clock().now().nanoseconds / 1e9:.3f}',
                             'object_id': target.model, 'attempt': attempt + 1,
                             'ok': results[target.model].startswith('OK'),
                             'verdict': results[target.model]})
                # Retry only an empty grasp. Other failures may leave an object
                # in an unknown state and must not start another descent.
                if not results[target.model].startswith('prise ratee'):
                    break
        self.write_csv(rows)

        if self.pose_source == 'vision' and any(
                verdict.startswith('erreur') for verdict in results.values()):
            return results
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
    return 0 if results and all(v.startswith('OK') for v in results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
