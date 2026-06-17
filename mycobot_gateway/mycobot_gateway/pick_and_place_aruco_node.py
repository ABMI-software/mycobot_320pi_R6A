#!/usr/bin/env python3
"""Pick-and-place ArUco — orchestrateur sim + robot réel.

Reçoit la position de l'objet depuis /aruco/object_pose (fourni par
gz_sim_localizer en simulation ou aruco_localizer_node sur robot réel),
planifie un cycle pick-and-place complet via IK numérique et l'exécute.

Deux modes (paramètre ROS2 `mode`) :
  sim  — mouvement via ros2_control (mycobot_controller/joint_trajectory)
          grasp émulé par gz set_pose (téléportation Gazebo)
  real — mouvement via trajectory_to_robot_bridge → bridge_tour → Pi
          gripper via JSON sur /to_robot (no-op si pas de pince physique)

Interface d'entrée commune (sim / réel) :
  /aruco/object_pose     (geometry_msgs/PoseStamped) — position objet
  /aruco/workspace_valid (std_msgs/Bool)              — localizer prêt
  /joint_states          (sensor_msgs/JointState)     — retour encodeurs
  /fk/ee_pose            (geometry_msgs/PoseStamped)  — pose EE (FK)

Topics publiés :
  /mycobot_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
  /to_robot  (std_msgs/String, JSON)   — gripper + bridge_tour (mode real)
  /pickplace/status  (std_msgs/String) — état courant

Séquence de mouvement par cycle :
  HOME → approach_pick → grasp_pos → [GRASP] → lift →
  approach_place → place_pos → [RELEASE] → retreat → HOME

Paramètres ROS2 :
  mode           : "sim" ou "real"                       (défaut : "sim")
  place_x/y/z    : position de dépose en m               (défaut : 0.20/-0.18/0.04)
  approach_height: hauteur au-dessus pick/place en m      (défaut : 0.12)
    object_diameter: diamètre de l'objet en m               (défaut : 0.06)
    grasp_z_offset : marge de prise au-dessus de la base m  (défaut : 0.005)
    min_pick_z     : borne basse de sécurité pour pick_z m  (défaut : 0.0)
  settle_time    : attente stabilisation par segment (s)  (défaut : 2.0)
  gz_world       : nom du monde Gazebo                    (défaut : precision_benchmark)
  gz_object      : nom du modèle à téléporter            (défaut : target_cube)
  pose_timeout   : délai max attente /aruco/object_pose   (défaut : 30.0)
  speed          : vitesse pymycobot pour robot réel      (défaut : 40)
"""

from __future__ import annotations

import enum
import json
import os
import subprocess
import sys
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ── Import IK/FK depuis le module DREAM du dépôt ─────────────────────────────
_DREAM_DIR_ALT = "/home/genji/ros_jazzy/src/mycobot_R6A/training/dream"
_DREAM_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "training", "dream",
))
for _p in [_DREAM_DIR, _DREAM_DIR_ALT]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from mycobot_fk import forward_kinematics           # type: ignore  # noqa: E402
from mycobot_ik import inverse_kinematics_position  # type: ignore  # noqa: E402

# ── Noms de joints dans l'ordre de la chaîne URDF ────────────────────────────
JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

# Position initiale stable au-dessus du workspace (évite l'instabilité à [0,0,0,0,0,0]).
# [j1=0°, j2=-46°, j3=80°, j4=-46°, j5=0°, j6=0°]
HOME_ANGLES = np.array([0.0, -0.8, 1.4, -0.8, 0.0, 0.0], dtype=np.float64)


class State(enum.Enum):
    INIT            = "INIT"
    WAIT_POSE       = "WAIT_POSE"
    IK_PLAN         = "IK_PLAN"
    MOVING          = "MOVING"
    SETTLING        = "SETTLING"
    GRASP           = "GRASP"
    RELEASE         = "RELEASE"
    DONE            = "DONE"
    ERROR           = "ERROR"


class PickAndPlaceArucoNode(Node):
    """Orchestrateur pick-and-place ArUco — simulation et robot réel."""

    def __init__(self) -> None:
        super().__init__("pick_and_place_aruco")

        # ── Paramètres ────────────────────────────────────────────────────────
        self.declare_parameter("mode",            "sim")
        self.declare_parameter("place_x",          0.20)
        self.declare_parameter("place_y",         -0.18)
        self.declare_parameter("place_z",          0.04)
        self.declare_parameter("approach_height",  0.12)
        self.declare_parameter("object_diameter",  0.06)
        self.declare_parameter("grasp_z_offset",   0.005)
        self.declare_parameter("min_pick_z",       0.0)
        self.declare_parameter("settle_time",      2.0)
        self.declare_parameter("gz_world",        "precision_benchmark")
        self.declare_parameter("gz_object",       "target_cube")
        self.declare_parameter("pose_timeout",    30.0)
        self.declare_parameter("speed",           40)

        self._mode          = self.get_parameter("mode").value
        self._place         = np.array([
            self.get_parameter("place_x").value,
            self.get_parameter("place_y").value,
            self.get_parameter("place_z").value,
        ], dtype=np.float64)
        self._approach_h    = float(self.get_parameter("approach_height").value)
        self._obj_diam      = float(self.get_parameter("object_diameter").value)
        self._grasp_z_off   = float(self.get_parameter("grasp_z_offset").value)
        self._min_pick_z    = float(self.get_parameter("min_pick_z").value)
        self._settle        = float(self.get_parameter("settle_time").value)
        self._gz_world      = self.get_parameter("gz_world").value
        self._gz_object     = self.get_parameter("gz_object").value
        self._pose_timeout  = float(self.get_parameter("pose_timeout").value)
        self._speed         = int(self.get_parameter("speed").value)

        # ── État ──────────────────────────────────────────────────────────────
        self._state         : State             = State.INIT
        self._plan          : List[dict]        = []
        self._plan_idx      : int               = 0
        self._current_seg   : Optional[dict]    = None  # segment en cours d'exécution
        self._settle_deadline: float            = 0.0
        self._carrying      : bool              = False

        # Données capteurs
        self._joint_pos   : Optional[np.ndarray] = None
        self._ee_pos      : Optional[np.ndarray] = None
        self._object_pos  : Optional[np.ndarray] = None
        self._ws_valid    : bool                  = False

        # ── Publishers ────────────────────────────────────────────────────────
        self._traj_pub = self.create_publisher(
            JointTrajectory, "/mycobot_controller/joint_trajectory", 1
        )
        self._robot_pub = self.create_publisher(String, "/to_robot", 10)
        self._status_pub = self.create_publisher(String, "/pickplace/status", 5)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(JointState,   "/joint_states",          self._js_cb,     10)
        self.create_subscription(PoseStamped,  "/fk/ee_pose",            self._fk_cb,     10)
        self.create_subscription(PoseStamped,  "/aruco/object_pose",     self._obj_cb,     5)
        self.create_subscription(Bool,         "/aruco/workspace_valid", self._ws_cb,      5)

        # ── Boucle principale 10 Hz ───────────────────────────────────────────
        self.create_timer(0.1, self._step)

        self.get_logger().info(
            f"[pick_and_place_aruco] mode={self._mode} "
            f"place={np.round(self._place, 3)} "
            f"approach_h={self._approach_h}m obj_diam={self._obj_diam}m "
            f"settle={self._settle}s"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._joint_pos = np.array(
                [name_to_pos[j] for j in JOINT_NAMES], dtype=np.float64
            )
        except KeyError:
            pass

    def _fk_cb(self, msg: PoseStamped) -> None:
        self._ee_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def _obj_cb(self, msg: PoseStamped) -> None:
        self._object_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def _ws_cb(self, msg: Bool) -> None:
        self._ws_valid = msg.data

    # ── Horloge ───────────────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── Envoi commande joints ─────────────────────────────────────────────────

    def _send_joints(self, angles: np.ndarray, duration: float | None = None) -> None:
        dur = duration if duration is not None else self._settle
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in angles]
        secs = int(dur)
        nsecs = int((dur - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
        traj.points = [pt]
        self._traj_pub.publish(traj)

    # ── Status ────────────────────────────────────────────────────────────────

    def _pub_status(self, detail: str = "") -> None:
        msg = String()
        msg.data = f"{self._state.value}|{detail}"
        self._status_pub.publish(msg)

    def _transition(self, new: State, detail: str = "") -> None:
        self.get_logger().info(
            f"[FSM] {self._state.value} → {new.value}"
            + (f" ({detail})" if detail else "")
        )
        self._state = new

    # ── IK helper ─────────────────────────────────────────────────────────────

    def _solve_ik(
        self, target: np.ndarray, q0: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], bool]:
        ok, q, err = inverse_kinematics_position(target, q0=q0)
        if not ok:
            self.get_logger().warning(
                f"IK échoué pour {np.round(target, 3)} — err={err*1000:.1f} mm"
            )
            return None, False
        ee = self._fk_pos(q)
        dist_mm = np.linalg.norm(ee - target) * 1000.0
        self.get_logger().info(
            f"  IK {np.round(np.degrees(q), 1)}° → err={dist_mm:.1f} mm"
        )
        return q, True

    def _fk_pos(self, angles: np.ndarray) -> np.ndarray:
        positions, _ = forward_kinematics(angles)
        return np.array(positions["mycobot320_link6"])

    # ── Position TCP (gripper_base) en frame monde ────────────────────────────

    def _gripper_base_world(self) -> Optional[np.ndarray]:
        """Retourne la position de gripper_base en frame monde via FK complète.

        La jonction joint6output_to_gripper_base est fixe : xyz=[0,-0.007,0.056]
        dans le frame link6. On applique la translation après la rotation de link6.
        """
        if self._joint_pos is None:
            return self._ee_pos
        _, transforms = forward_kinematics(self._joint_pos)
        T6 = transforms[6]                             # world→link6 (4×4)
        p_local = np.array([0.0, -0.007, 0.056, 1.0]) # gripper_base origin in link6
        gb_world = (T6 @ p_local)[:3]
        return gb_world

    # ── Grasp simulation via gz service ───────────────────────────────────────

    def _gz_set_pose(self, model: str, x: float, y: float, z: float) -> None:
        req = f'name: "{model}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}'
        try:
            subprocess.run(
                [
                    "gz", "service", "-s",
                    f"/world/{self._gz_world}/set_pose",
                    "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "500",
                    "--req", req,
                ],
                check=False, capture_output=True, timeout=2.0,
            )
        except Exception as exc:
            self.get_logger().warning(f"gz set_pose failed: {exc}")

    # ── Gripper (robot réel) ──────────────────────────────────────────────────

    def _gripper(self, action: str) -> None:
        msg = String()
        msg.data = json.dumps({"command": action})
        self._robot_pub.publish(msg)
        self.get_logger().info(f"[gripper] → {action}")

    # ── Machine à états principale ────────────────────────────────────────────

    def _step(self) -> None:  # noqa: C901
        self._pub_status()

        if self._state == State.INIT:
            self._transition(State.WAIT_POSE, "attente /aruco/object_pose")
            self._settle_deadline = self._now() + self._pose_timeout

        elif self._state == State.WAIT_POSE:
            if self._object_pos is not None and self._joint_pos is not None:
                self.get_logger().info(
                    f"Pose objet reçue : {np.round(self._object_pos, 3)}"
                )
                self._transition(State.IK_PLAN)
                return
            if self._now() > self._settle_deadline:
                self.get_logger().error(
                    f"Timeout ({self._pose_timeout}s) — "
                    "/aruco/object_pose jamais reçu"
                )
                self._transition(State.ERROR)

        elif self._state == State.IK_PLAN:
            self._build_plan()

        elif self._state == State.MOVING:
            # _current_seg est fixé par _advance_plan() avant la transition MOVING
            seg = self._current_seg
            self._send_joints(seg["angles"])
            self._settle_deadline = self._now() + self._settle * 1.1
            self._transition(State.SETTLING, seg["label"])

        elif self._state == State.SETTLING:
            if self._now() < self._settle_deadline:
                # En mode sim, suivre le gripper_base avec l'objet pendant le transport.
                # Le cube est placé légèrement au-dessus du gripper_base (cube demi-hauteur).
                if self._carrying and self._mode == "sim":
                    gb = self._gripper_base_world()
                    if gb is not None:
                        self._gz_set_pose(
                            self._gz_object,
                            float(gb[0]),
                            float(gb[1]),
                            float(gb[2]) + 0.025,   # demi-hauteur cube 4cm
                        )
                return
            self._advance_plan()

        elif self._state == State.GRASP:
            self._do_grasp()

        elif self._state == State.RELEASE:
            self._do_release()

        elif self._state == State.DONE:
            pass

        elif self._state == State.ERROR:
            pass

    # ── Construction du plan ──────────────────────────────────────────────────

    def _build_plan(self) -> None:
        obj = self._object_pos.copy()

        # Pose ArUco objet = sommet de l'objet (marqueur collé dessus).
        # On descend donc d'un diamètre complet pour viser la base de l'objet,
        # puis on ajoute une petite marge verticale de sécurité.
        pick_z = obj[2] - self._obj_diam + self._grasp_z_off
        pick_z = max(self._min_pick_z, pick_z)
        self.get_logger().info(
            f"Pick height computed from object geometry: "
            f"obj_z={obj[2]:.3f}m -> pick_z={pick_z:.3f}m"
        )

        # Waypoints cartésiens
        approach_pick  = obj.copy();  approach_pick[2]  += self._approach_h
        grasp_pos      = obj.copy();  grasp_pos[2]      = pick_z
        lift_pos       = obj.copy();  lift_pos[2]       += self._approach_h

        place          = self._place.copy()
        approach_place = place.copy(); approach_place[2] += self._approach_h
        place_pos      = place.copy()
        retreat        = place.copy(); retreat[2]        += self._approach_h

        waypoints = [
            ("home",           HOME_ANGLES,    None),   # angles directs
            ("approach_pick",  approach_pick,  None),
            ("grasp_pos",      grasp_pos,      None),
            ("lift",           lift_pos,       None),
            ("approach_place", approach_place, None),
            ("place_pos",      place_pos,      None),
            ("retreat",        retreat,        None),
            ("home_end",       HOME_ANGLES,    None),
        ]

        self.get_logger().info("Planification IK...")
        q_prev: Optional[np.ndarray] = self._joint_pos
        plan: List[dict] = []
        ik_cache: dict[str, np.ndarray] = {}

        for label, target, _ in waypoints:
            if isinstance(target, np.ndarray) and target.shape == (6,):
                # Angles directs (HOME)
                plan.append({"type": "move", "label": label, "angles": target})
                q_prev = target
            else:
                q, ok = self._solve_ik(target, q0=q_prev)
                if not ok:
                    self.get_logger().error(f"IK échoué pour waypoint '{label}'")
                    self._transition(State.ERROR)
                    return
                plan.append({"type": "move", "label": label, "angles": q})
                ik_cache[label] = q
                q_prev = q

        # Insertion des segments spéciaux GRASP et RELEASE
        full_plan: List[dict] = []
        for seg in plan:
            full_plan.append(seg)
            if seg["label"] == "grasp_pos":
                full_plan.append({"type": "GRASP", "label": "GRASP"})
            if seg["label"] == "place_pos":
                full_plan.append({"type": "RELEASE", "label": "RELEASE"})

        self._plan = full_plan
        self._plan_idx = 0

        self.get_logger().info(
            f"Plan établi : {len(full_plan)} segments\n"
            + "\n".join(f"  {i:2d}. {s['label']}" for i, s in enumerate(full_plan))
        )
        self._advance_plan()

    # ── Exécution du plan ─────────────────────────────────────────────────────

    def _advance_plan(self) -> None:
        if self._plan_idx >= len(self._plan):
            self.get_logger().info("Cycle pick-and-place terminé.")
            self._transition(State.DONE)
            return

        seg = self._plan[self._plan_idx]
        self._plan_idx += 1

        if seg["type"] == "GRASP":
            self._transition(State.GRASP, "fermeture gripper")
        elif seg["type"] == "RELEASE":
            self._transition(State.RELEASE, "ouverture gripper")
        else:
            self.get_logger().info(f"[{self._plan_idx}/{len(self._plan)}] {seg['label']}")
            self._current_seg = seg   # mémorise le segment AVANT la transition
            self._transition(State.MOVING, seg["label"])

    def _do_grasp(self) -> None:
        if not hasattr(self, "_grasp_done"):
            self._grasp_done = False

        if not self._grasp_done:
            if self._mode == "sim":
                # Téléporte l'objet au niveau du gripper_base (point de saisie réel).
                gb = self._gripper_base_world()
                target = gb if gb is not None else self._ee_pos
                if target is not None:
                    self._gz_set_pose(
                        self._gz_object,
                        float(target[0]),
                        float(target[1]),
                        float(target[2]) + 0.025,   # demi-hauteur cube (4cm/2 + marge)
                    )
            else:
                self._gripper("gripper_close")
            self._carrying = True
            self._grasp_done = True
            self._settle_deadline = self._now() + 1.0

        if self._now() >= self._settle_deadline:
            del self._grasp_done
            self.get_logger().info("Grasp effectué.")
            self._advance_plan()

    def _do_release(self) -> None:
        if not hasattr(self, "_release_done"):
            self._release_done = False

        if not self._release_done:
            if self._mode == "sim":
                self._gz_set_pose(
                    self._gz_object,
                    float(self._place[0]),
                    float(self._place[1]),
                    float(self._place[2]),
                )
            else:
                self._gripper("gripper_open")
            self._carrying = False
            self._release_done = True
            self._settle_deadline = self._now() + 1.0

        if self._now() >= self._settle_deadline:
            del self._release_done
            self.get_logger().info("Release effectué.")
            self._advance_plan()


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = PickAndPlaceArucoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
