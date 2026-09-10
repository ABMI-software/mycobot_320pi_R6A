#!/usr/bin/env python3
"""Nœud de benchmark de précision — MyCobot 320 Pi.

Objectif
--------
Mesurer expérimentalement la précision de positionnement du robot sur une
grille de N cibles dans l'espace de travail, en utilisant uniquement le
retour des encodeurs (cinématique directe).

Architecture
------------
  /aruco/object_pose  →  position détectée de l'objet (optionnel, utilisée
                          comme cible réelle lors du test de saisie finale)
  /fk/ee_pose         →  pose EE actuelle (FK depuis /joint_states)
  /benchmark/joint_commands → commandes joints envoyées au robot

Pipeline par cible
------------------
  1. IK numérique  : cible 3D  →  angles joints
  2. Envoi des commandes joints  (/model/mycobot_320/joint/<jn>/cmd_pos)
  3. Attente de stabilisation  (settle_time secondes)
  4. Lecture /fk/ee_pose  →  position EE réelle
  5. Calcul de l'erreur  ||cible - EE_réel||  en mm
  6. Enregistrement dans le CSV

Grille de test par défaut (9 cibles, grille 3×3)
-------------------------------------------------
  x ∈ {0.15, 0.22, 0.28} m   (portée radiale)
  y ∈ {-0.10, 0.00, 0.10} m  (latéral)
  z = 0.06 m                  (6 cm au-dessus de la table)

Résultats
---------
  /benchmark/status   (std_msgs/String)  — état courant
  ~/benchmark_results_<timestamp>.csv    — fichier résultats

Paramètres ROS 2
----------------
  settle_time  : attente stabilisation en s    (défaut : 2.0)
  grid_z       : hauteur de la grille en m     (défaut : 0.06)
  use_aruco    : inclure cible ArUco en fin    (défaut : true)
  output_dir   : dossier de sortie CSV         (défaut : ~)
"""

from __future__ import annotations

import csv
import enum
import os
import sys
import time
from datetime import datetime
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ── import IK ────────────────────────────────────────────────────────────────
_DREAM_DIR_ALT = '/home/genji/ros_jazzy/src/mycobot_R6A/training/dream'
_DREAM_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '..', '..', 'training', 'dream'
))
for _p in [_DREAM_DIR, _DREAM_DIR_ALT]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from mycobot_fk import forward_kinematics           # type: ignore  # noqa: E402
from mycobot_ik import inverse_kinematics_position  # type: ignore  # noqa: E402

# ── Constantes ───────────────────────────────────────────────────────────────
JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

HOME_ANGLES = np.zeros(6, dtype=np.float64)

# Grille 3×3 de cibles (repère base, mètres)
DEFAULT_GRID: List[Tuple[float, float]] = [
    (x, y)
    for x in (0.15, 0.22, 0.28)
    for y in (-0.10, 0.04, 0.10)
]


class State(enum.Enum):
    INIT          = "INIT"
    HOME          = "HOME"
    WAIT_HOME     = "WAIT_HOME"
    NEXT_TARGET   = "NEXT_TARGET"
    IK_SOLVE      = "IK_SOLVE"
    MOVING        = "MOVING"
    SETTLING      = "SETTLING"
    MEASURE       = "MEASURE"
    ARUCO_TARGET  = "ARUCO_TARGET"
    ARUCO_MOVE    = "ARUCO_MOVE"
    ARUCO_SETTLE  = "ARUCO_SETTLE"
    ARUCO_MEASURE = "ARUCO_MEASURE"
    REPORT        = "REPORT"
    DONE          = "DONE"


class PrecisionBenchmarkNode(Node):
    """Banc de mesure de précision du MyCobot 320 Pi."""

    def __init__(self) -> None:
        super().__init__("precision_benchmark")

        # ── paramètres ──
        self.declare_parameter("settle_time", 2.0)
        self.declare_parameter("grid_z",      0.06)
        self.declare_parameter("use_aruco",   True)
        self.declare_parameter("output_dir",  os.path.expanduser("~"))

        self._settle_time = float(self.get_parameter("settle_time").value)
        self._grid_z      = float(self.get_parameter("grid_z").value)
        self._use_aruco   = bool(self.get_parameter("use_aruco").value)
        self._output_dir  = self.get_parameter("output_dir").value

        # ── état ──
        self._state          = State.INIT
        self._current_target = np.zeros(3, dtype=np.float64)
        self._current_angles : Optional[np.ndarray] = None
        self._settle_deadline: float = 0.0
        self._target_idx     = 0

        # Cibles de la grille
        self._grid_targets: List[np.ndarray] = [
            np.array([x, y, self._grid_z]) for x, y in DEFAULT_GRID
        ]

        # ── mesures en cours ──
        self._ee_pose   : Optional[np.ndarray] = None  # [x, y, z]
        self._joint_pos : Optional[np.ndarray] = None
        self._aruco_target: Optional[np.ndarray] = None
        self._results   : List[dict] = []

        # ── publisher (commandes joints via trajectory controller) ──
        self._traj_pub = self.create_publisher(
            JointTrajectory, "/mycobot_controller/joint_trajectory", 1
        )
        self._pub_status = self.create_publisher(String, "/benchmark/status", 5)

        # ── subscribers ──
        self.create_subscription(PoseStamped, "/fk/ee_pose",          self._ee_cb,    10)
        self.create_subscription(PoseStamped, "/aruco/object_pose",   self._aruco_cb, 5)
        self.create_subscription(JointState,  "/joint_states",        self._js_cb,    10)

        # ── boucle état (10 Hz) ──
        self.create_timer(0.1, self._step)

        # ── CSV ──
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(self._output_dir, f"benchmark_results_{ts}.csv")
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=[
            "trial", "type",
            "target_x", "target_y", "target_z",
            "actual_x",  "actual_y",  "actual_z",
            "error_mm", "error_x_mm", "error_y_mm", "error_z_mm",
            "ik_success", "settle_time_s",
        ])
        self._csv_writer.writeheader()

        self.get_logger().info(
            f"Benchmark démarré | {len(self._grid_targets)} cibles "
            f"| settle={self._settle_time}s | CSV → {self._csv_path}"
        )

    # ── callbacks ────────────────────────────────────────────────────────────

    def _ee_cb(self, msg: PoseStamped) -> None:
        self._ee_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def _aruco_cb(self, msg: PoseStamped) -> None:
        self._aruco_target = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def _js_cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._joint_pos = np.array(
                [name_to_pos[jn] for jn in JOINT_NAMES], dtype=np.float64
            )
        except KeyError:
            pass

    def _now(self) -> float:
        """Retourne le temps courant (sim ou wall) en secondes."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ── machine à états ──────────────────────────────────────────────────────

    def _step(self) -> None:  # noqa: C901  (complexité acceptable pour une FSM)
        s = self._state

        if s == State.INIT:
            self._target_idx = 0
            self._publish_status("Initialisation — retour HOME")
            self._send_joints(HOME_ANGLES)
            self._settle_deadline = self._now() + self._settle_time
            self._state = State.WAIT_HOME

        elif s == State.WAIT_HOME:
            if self._now() >= self._settle_deadline:
                self._state = State.NEXT_TARGET

        elif s == State.NEXT_TARGET:
            if self._target_idx >= len(self._grid_targets):
                if self._use_aruco and self._aruco_target is not None:
                    self._current_target = self._aruco_target.copy()
                    self._publish_status(
                        f"Cible ArUco  x={self._current_target[0]:.3f}"
                        f"  y={self._current_target[1]:.3f}"
                        f"  z={self._current_target[2]:.3f}"
                    )
                    self._state = State.ARUCO_TARGET
                else:
                    self._state = State.REPORT
                return
            self._current_target = self._grid_targets[self._target_idx]
            self._publish_status(
                f"Cible {self._target_idx+1}/{len(self._grid_targets)}  "
                f"x={self._current_target[0]:.3f}  "
                f"y={self._current_target[1]:.3f}  "
                f"z={self._current_target[2]:.3f}"
            )
            self._state = State.IK_SOLVE

        elif s == State.IK_SOLVE:
            angles, ok = self._solve_ik(self._current_target)
            if not ok:
                self.get_logger().warning(
                    f"IK échoué pour cible {self._target_idx+1} "
                    f"({self._current_target}) — sautée"
                )
                self._record_failed_ik()
                self._target_idx += 1
                self._state = State.NEXT_TARGET
                return
            self._current_angles = angles
            self._state = State.MOVING

        elif s == State.MOVING:
            self._send_joints(self._current_angles)
            self._settle_deadline = self._now() + self._settle_time
            self._state = State.SETTLING

        elif s == State.SETTLING:
            remaining = self._settle_deadline - self._now()
            if remaining > 0:
                self._publish_status(
                    f"Stabilisation…  ({remaining:.1f}s restantes)"
                )
                return
            self._state = State.MEASURE

        elif s == State.MEASURE:
            self._measure_and_record(trial_type="grid")
            self._target_idx += 1
            # retour HOME entre chaque cible pour éviter les accumulations d'erreur
            self._send_joints(HOME_ANGLES)
            self._settle_deadline = self._now() + self._settle_time
            self._state = State.WAIT_HOME

        # ── cible ArUco (test de saisie réelle) ───────────────────────────────

        elif s == State.ARUCO_TARGET:
            angles, ok = self._solve_ik(self._current_target)
            if not ok:
                self.get_logger().warning("IK échoué pour cible ArUco")
                self._record_failed_ik(trial_type="aruco")
                self._state = State.REPORT
                return
            self._current_angles = angles
            self._state = State.ARUCO_MOVE

        elif s == State.ARUCO_MOVE:
            self._send_joints(self._current_angles)
            self._settle_deadline = self._now() + self._settle_time
            self._state = State.ARUCO_SETTLE

        elif s == State.ARUCO_SETTLE:
            if self._now() < self._settle_deadline:
                return
            self._state = State.ARUCO_MEASURE

        elif s == State.ARUCO_MEASURE:
            self._measure_and_record(trial_type="aruco")
            self._state = State.REPORT

        elif s == State.REPORT:
            self._print_report()
            self._csv_file.close()
            self._state = State.DONE
            self.get_logger().info(f"Benchmark terminé. Résultats → {self._csv_path}")

        elif s == State.DONE:
            pass  # nœud inactif, résultats enregistrés

    # ── IK ───────────────────────────────────────────────────────────────────

    # Limites joints URDF (rad)
    _JOINT_LIMITS = [
        (-2.93, 2.93),   # joint2_to_joint1
        (-2.35, 2.35),   # joint3_to_joint2
        (-2.53, 2.53),   # joint4_to_joint3
        (-2.53, 2.53),   # joint5_to_joint4
        (-2.93, 2.93),   # joint6_to_joint5
        (-3.14, 3.14),   # joint6output_to_joint6
    ]

    def _angles_in_limits(self, angles: np.ndarray) -> bool:
        return all(lo <= a <= hi for a, (lo, hi) in zip(angles, self._JOINT_LIMITS))

    def _solve_ik(self, target: np.ndarray) -> Tuple[np.ndarray, bool]:
        """Lance l'IK numérique avec plusieurs seeds. Retourne (angles, success)."""
        # Seeds : position courante, HOME, et plusieurs configs alternatives
        seeds = [
            self._joint_pos if self._joint_pos is not None else HOME_ANGLES,
            HOME_ANGLES.copy(),
            np.array([ 0.5,  0.5,  0.5, -0.5,  0.5,  0.0]),
            np.array([-0.5, -0.5, -0.5,  0.5, -0.5,  0.0]),
            np.array([ 1.0,  0.3,  0.5, -0.3,  1.0,  0.0]),
            np.array([ 0.0,  0.5,  1.0, -1.0,  0.0,  0.0]),
        ]
        best_angles: Optional[np.ndarray] = None
        best_residual = float("inf")

        for seed in seeds:
            try:
                success, angles, residual = inverse_kinematics_position(
                    target_xyz=target,
                    q0=seed,
                )
                if not success:
                    continue
                angles = np.array(angles, dtype=np.float64)
                if self._angles_in_limits(angles):
                    if residual < best_residual:
                        best_residual = residual
                        best_angles = angles
            except Exception:
                continue

        if best_angles is not None:
            self.get_logger().debug(
                f"IK réussi (résidu={best_residual:.6f}): "
                f"{[f'{a:.3f}' for a in best_angles]}"
            )
            return best_angles, True

        self.get_logger().warning(
            f"IK: aucune solution dans les limites pour {target}"
        )
        return HOME_ANGLES, False

    # ── commandes joints ─────────────────────────────────────────────────────

    def _send_joints(self, angles: np.ndarray) -> None:
        """Envoie une commande de position via JointTrajectory."""
        self.get_logger().debug(
            f"_send_joints: {[f'{a:.3f}' for a in angles]}"
        )
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()  # obligatoire avec sim time
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in angles]
        # time_from_start = settle_time pour laisser le contrôleur arriver à la cible
        secs = int(self._settle_time)
        nsecs = int((self._settle_time - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
        traj.points = [pt]
        self._traj_pub.publish(traj)

    # ── mesure ───────────────────────────────────────────────────────────────

    def _measure_and_record(self, trial_type: str = "grid") -> None:
        if self._ee_pose is None:
            self.get_logger().warning("Pas de /fk/ee_pose disponible — mesure sautée")
            return

        err_vec = self._current_target - self._ee_pose
        err_mm  = np.linalg.norm(err_vec) * 1000.0

        row = {
            "trial":        self._target_idx + 1,
            "type":         trial_type,
            "target_x":     round(float(self._current_target[0]), 4),
            "target_y":     round(float(self._current_target[1]), 4),
            "target_z":     round(float(self._current_target[2]), 4),
            "actual_x":     round(float(self._ee_pose[0]), 4),
            "actual_y":     round(float(self._ee_pose[1]), 4),
            "actual_z":     round(float(self._ee_pose[2]), 4),
            "error_mm":     round(err_mm, 2),
            "error_x_mm":   round(float(err_vec[0] * 1000), 2),
            "error_y_mm":   round(float(err_vec[1] * 1000), 2),
            "error_z_mm":   round(float(err_vec[2] * 1000), 2),
            "ik_success":   True,
            "settle_time_s": self._settle_time,
        }
        self._csv_writer.writerow(row)
        self._csv_file.flush()
        self._results.append(row)

        self.get_logger().info(
            f"[{trial_type.upper()}] cible {self._target_idx+1}  "
            f"erreur = {err_mm:.2f} mm  "
            f"(Δx={err_vec[0]*1000:.1f}  Δy={err_vec[1]*1000:.1f}  "
            f"Δz={err_vec[2]*1000:.1f} mm)"
        )

    def _record_failed_ik(self, trial_type: str = "grid") -> None:
        row = {
            "trial":        self._target_idx + 1,
            "type":         trial_type,
            "target_x":     round(float(self._current_target[0]), 4),
            "target_y":     round(float(self._current_target[1]), 4),
            "target_z":     round(float(self._current_target[2]), 4),
            "actual_x":     None, "actual_y": None, "actual_z": None,
            "error_mm":     None,
            "error_x_mm":   None, "error_y_mm": None, "error_z_mm": None,
            "ik_success":   False,
            "settle_time_s": self._settle_time,
        }
        self._csv_writer.writerow(row)
        self._csv_file.flush()

    # ── rapport final ─────────────────────────────────────────────────────────

    def _print_report(self) -> None:
        valid = [r for r in self._results if r["error_mm"] is not None]
        if not valid:
            self.get_logger().warning("Aucun résultat valide à rapporter.")
            return

        errors = np.array([r["error_mm"] for r in valid])
        ex     = np.array([abs(r["error_x_mm"]) for r in valid])
        ey     = np.array([abs(r["error_y_mm"]) for r in valid])
        ez     = np.array([abs(r["error_z_mm"]) for r in valid])

        sep = "─" * 58
        lines = [
            "",
            sep,
            " RÉSULTATS BENCHMARK DE PRÉCISION — MyCobot 320 Pi",
            sep,
            f"  Essais réussis    : {len(valid)} / {len(self._results)}",
            f"  Erreur moyenne    : {errors.mean():.2f} mm",
            f"  Écart-type        : {errors.std():.2f} mm",
            f"  Erreur max        : {errors.max():.2f} mm",
            f"  Erreur min        : {errors.min():.2f} mm",
            f"  RMSE              : {np.sqrt(np.mean(errors**2)):.2f} mm",
            sep,
            f"  Δx moyen (abs)    : {ex.mean():.2f} mm  (max {ex.max():.2f})",
            f"  Δy moyen (abs)    : {ey.mean():.2f} mm  (max {ey.max():.2f})",
            f"  Δz moyen (abs)    : {ez.mean():.2f} mm  (max {ez.max():.2f})",
            sep,
            f"  CSV               : {self._csv_path}",
            sep,
            "",
        ]
        report = "\n".join(lines)
        self.get_logger().info(report)
        self._publish_status(f"DONE — RMSE={np.sqrt(np.mean(errors**2)):.2f}mm")

    def _publish_status(self, msg: str) -> None:
        self._pub_status.publish(String(data=msg))
        self.get_logger().info(f"[BENCHMARK] {msg}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PrecisionBenchmarkNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
