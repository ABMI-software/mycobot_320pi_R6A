#!/usr/bin/env python3
"""Contrôleur d'asservissement visuel — fusion, suivi, commande, sûreté (§11.1).

Regroupe dans UN processus `multi_camera_fusion_node`, `robot_state_node`,
`visual_servo_controller`, `safety_supervisor` et `state_machine_node`. Le §11.3
demande ≥10 Hz de bout en bout avec une latence faible et stable : ces cinq
étages échangent des vecteurs de six flottants à chaque période, les répartir en
cinq processus ajouterait cinq allers-retours DDS pour aucun bénéfice.

La boucle, une fois par période (§4.1) :

    1. relever les détections de chaque caméra (déjà en base_link)
    2. les fusionner par confiance                              §7.2
    3. mettre à jour le Kalman, prédire la cible à t+τ          §8.1-8.2
    4. lire les codeurs -> FK -> pose de la pince               §4.1
    5. calculer e = P_EE − P_d                                   §8.3
    6. demander son ordre à la machine à états                   §10
    7. faire valider par le superviseur                          §12
    8. saturer, résoudre l'IK différentielle, envoyer            §8.3

Ce qui rend la boucle FERMÉE, c'est l'étape 1 à chaque tour : la cible est
remesurée, jamais rejouée. Si les détections s'arrêtent, le superviseur arrête
le robot — il ne continue pas vers la dernière position connue, sauf sur cible
explicitement déclarée immobile et pour la durée courte du §7.1.

⚠ ORIENTATION — l'asservissement ne commande que XYZ ; le poignet garde
l'orientation qu'il a au démarrage (`solve_step` la tient). Amener le bras dans
l'orientation-démo (J5 ≈ −15°) AVANT de lancer. Le top-down [180,0,0] envoie J5
vers 90° et met le bras en auto-collision — constaté sur le pick du 31/07.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

_PKG_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _PKG_DIR.parents[2]
for _extra in ('scripts', 'training/dream'):
    _path = str(_REPO_ROOT / _extra)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from diff_ik import fk_pose, solve_pose                                  # noqa: E402

from .fusion import CameraMeasurement, FusionConfig, MultiCameraFusion   # noqa: E402
from .object_tracker import ObjectTracker, TargetLock, TrackerConfig     # noqa: E402
from .safety import (JOINT_LIMITS_DEG, SafetyLimits, SafetyState,
                     SafetySupervisor)                                  # noqa: E402
from .servo_law import (ConvergenceMonitor, ServoGains, compute_command)  # noqa: E402
from .state_machine import (Action, Context, MissionConfig, PickStateMachine,
                            State)                                       # noqa: E402

GRIPPER_ID = 14
GRIPPER_STATUS_TEXT = {0: 'en mouvement', 1: 'rien saisi', 2: 'objet saisi',
                       3: 'objet tombé'}
CONTROL_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class VisualServoNode(Node):

    def __init__(self):
        super().__init__('visual_servo_controller')
        self.declare_parameter('cameras', ['arducam', 'svpro'])
        self.declare_parameter('primary_camera', 'arducam')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('mobile_target', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('robot_speed', 25)
        self.declare_parameter('table_z', 0.0)
        self.declare_parameter('pregrasp_height', 0.065)
        self.declare_parameter('stop_at_pregrasp', False)
        self.declare_parameter('freeze_xy_on_descend', True)
        self.declare_parameter('descend_step', 0.005)
        self.declare_parameter('verify_lift_height', 0.005)
        self.declare_parameter('grasp_firm', True)
        self.declare_parameter('grasp_height', 0.012)
        self.declare_parameter('lift_height', 0.12)
        self.declare_parameter('place_position', [0.24, -0.20, 0.10])
        self.declare_parameter('grasp_angle', 20)
        # Serrage : angle de la 2e passe (échelle 0-100, plus bas = plus serré).
        # 20 est la valeur qui a donné une prise confirmée lors du pick manuel du
        # 17/08 — elle n'est donc pas « trop lâche » pour venir au contact. Les 8
        # points de serrage s'appliquent APRÈS confirmation du contact : les
        # doigts ne parcourent presque rien, ils appuient. La butée de couple
        # interne de la pince Pro reste la limite physique.
        self.declare_parameter('grasp_firm_angle', 12)
        self.declare_parameter('open_angle', 100)
        self.declare_parameter('kp', [1.2, 1.2, 0.9])
        self.declare_parameter('max_speed', 0.08)
        self.declare_parameter('max_step', 0.012)
        self.declare_parameter('feedforward', 1.0)
        self.declare_parameter('extra_latency', 0.05)
        self.declare_parameter('auto_start', False)

        p = self.get_parameter
        self.cameras = list(p('cameras').value)
        self.dry_run = bool(p('dry_run').value)
        self.robot_speed = int(p('robot_speed').value)
        self.grasp_angle = int(p('grasp_angle').value)
        self.grasp_firm_angle = int(p('grasp_firm_angle').value)
        self.open_angle = int(p('open_angle').value)
        if not 0 <= self.grasp_firm_angle <= self.grasp_angle:
            raise ValueError(
                f'grasp_firm_angle={self.grasp_firm_angle} doit être dans '
                f'[0, grasp_angle={self.grasp_angle}] — au-dessus, la « passe de '
                f'serrage » RELÂCHERAIT la pince.')
        self.extra_latency = float(p('extra_latency').value)

        self.gains = ServoGains(
            kp=tuple(float(x) for x in p('kp').value),
            feedforward=float(p('feedforward').value),
            max_speed=float(p('max_speed').value),
            max_step=float(p('max_step').value))
        self.mission = MissionConfig(
            pregrasp_height=float(p('pregrasp_height').value),
            grasp_height=float(p('grasp_height').value),
            lift_height=float(p('lift_height').value),
            place_position=tuple(float(x) for x in p('place_position').value),
            mobile_target=bool(p('mobile_target').value),
            stop_at_pregrasp=bool(p('stop_at_pregrasp').value),
            freeze_xy_on_descend=bool(p('freeze_xy_on_descend').value),
            descend_step=float(p('descend_step').value),
            verify_lift_height=float(p('verify_lift_height').value),
            grasp_firm=bool(p('grasp_firm').value),
            lateral_camera=next((c for c in self.cameras
                                 if c != p('primary_camera').value), 'svpro'))

        self.fusion = MultiCameraFusion(p('primary_camera').value, FusionConfig(
            max_age=0.25, base_std=0.006))
        self.tracker = ObjectTracker(TrackerConfig())
        self.lock = TargetLock()
        self.safety = SafetySupervisor(SafetyLimits(table_z=float(p('table_z').value)))
        self.convergence = ConvergenceMonitor(
            threshold=self.mission.fine_threshold,
            required_frames=self.mission.fine_frames)
        self.machine = PickStateMachine(self.mission)

        self.tool_offset_mm = self._load_tool_offset()
        self._detections: dict[str, dict] = {}
        self._joint_angles_deg: np.ndarray | None = None
        self._joint_stamp: float | None = None
        self._gripper_status: int | None = None
        self._status_unsupported = False
        self._last_gripper_cmd = 0.0
        self._last_cycle: float | None = None
        self._last_object_position: np.ndarray | None = None
        self._latency = self.extra_latency
        self._running = bool(p('auto_start').value)
        self._commanded_angles: np.ndarray | None = None

        for camera in self.cameras:
            self.create_subscription(
                String, f'/visual_servo/{camera}/detection',
                self._on_detection(camera), 1)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 1)
        self.create_subscription(String, '/from_robot', self._on_robot_response, 10)
        self.create_subscription(String, '/visual_servo/command', self._on_command, 1)

        self.pub_robot = self.create_publisher(String, '/to_robot', CONTROL_QOS)
        self.pub_status = self.create_publisher(String, '/visual_servo/status', 1)
        self.pub_target = self.create_publisher(PoseStamped, '/visual_servo/target', 1)
        self._tf = TransformBroadcaster(self)

        rate = float(p('control_rate').value)
        self.create_timer(1.0 / rate, self.control_cycle)
        self.get_logger().info(
            f'asservissement visuel — caméras={self.cameras} {rate:.0f}Hz '
            f'{"DRY-RUN (aucune commande envoyée)" if self.dry_run else "ROBOT ARMÉ"} '
            f'cible={"mobile" if self.mission.mobile_target else "immobile"}')
        self.get_logger().info(
            'en attente : ros2 topic pub --once /visual_servo/command '
            'std_msgs/String \'{data: start}\'' if not self._running else 'démarrage auto')

    # — entrées ———————————————————————————————————————————————————————

    def _load_tool_offset(self) -> np.ndarray:
        """Déport bride -> bout des doigts (mm, repère bride), depuis la calib TCP."""
        path = _REPO_ROOT / 'scripts' / 'tool_offset.json'
        if not path.is_file():
            self.get_logger().warn(
                'tool_offset.json absent : la pose asservie est celle de la BRIDE, '
                'pas des doigts — la saisie sera décalée du déport de la pince.')
            return np.zeros(3)
        return np.asarray(json.loads(path.read_text())['tool_offset_mm'], np.float64)

    def _on_detection(self, camera: str):
        def callback(msg: String):
            self._detections[camera] = json.loads(msg.data)
        return callback

    def _on_joints(self, msg: JointState):
        if len(msg.position) < 6:
            return
        self._joint_angles_deg = np.degrees(np.asarray(msg.position[:6], np.float64))
        self._joint_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_robot_response(self, msg: String):
        if 'PRO_GRIPPER_STATUS:' in msg.data:
            m = re.search(r'-?\d+', msg.data.split('PRO_GRIPPER_STATUS:', 1)[1])
            if m:
                self._gripper_status = int(m.group())
        elif 'Action inconnue' in msg.data and 'gripper_status' in msg.data:
            # bridge_pi_simple.py n'implémente pas get_pro_gripper_status : le
            # statut resterait inconnu tout du long, et « vérifier objet saisi »
            # n'aurait rien à vérifier. Seul gripper_bridge.py le fournit.
            if not self._status_unsupported:
                self._status_unsupported = True
                self.get_logger().error(
                    'Le bridge de la Pi ne connaît pas get_pro_gripper_status : '
                    'aucune saisie ne pourra être confirmée ni vérifiée. Lancer '
                    'scripts/gripper_bridge.py sur la Pi, pas bridge_pi_simple.py.')

    def _on_command(self, msg: String):
        command = msg.data.strip().lower()
        now = self._now()
        if command == 'start':
            self._reset(now)
            self._running = True
            self.get_logger().info('démarrage de l\'asservissement')
        elif command in ('stop', 'abort'):
            self._running = False
            self.machine.stop_reasons = ('arrêt demandé par l\'opérateur',)
            self.machine._enter(State.SAFE_STOP, now)
            self.get_logger().warn('arrêt demandé')
        elif command == 'reset':
            self._reset(now)
            self.get_logger().info('réinitialisation')

    def _reset(self, now: float):
        self.machine.reset(now)
        self.tracker.reset()
        self.lock.reset()
        self.fusion.reset()
        self.safety.reset()
        self.convergence.reset()
        self._last_object_position = None
        self._last_cycle = None
        self._gripper_status = None

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # — pose de la pince ———————————————————————————————————————————————

    def _end_effector_position(self) -> np.ndarray | None:
        """Position des doigts en mètres, repère base — codeurs + FK + déport TCP."""
        if self._joint_angles_deg is None:
            return None
        flange_mm, rotation = fk_pose(self._joint_angles_deg)
        tip_mm = np.asarray(flange_mm) + rotation @ self.tool_offset_mm
        return tip_mm / 1000.0

    # — boucle ————————————————————————————————————————————————————————

    def control_cycle(self):
        now = self._now()
        dt = 1.0 / float(self.get_parameter('control_rate').value) \
            if self._last_cycle is None else now - self._last_cycle
        self._last_cycle = now

        measurements = self._collect_measurements(now)
        result = self.fusion.fuse(measurements, now)
        measurement_valid = result is not None and result.mode != 'PERDU'

        if measurement_valid:
            self._update_latency(measurements, now)
            self.tracker.update(result.position[:2], now, result.confidence)
        else:
            self.tracker.predict(now)

        ee = self._end_effector_position()
        object_position = self._current_object_position(result, measurement_valid)
        context = self._build_context(now, ee, object_position, result,
                                      measurement_valid)
        directive = self.machine.step(context)

        report = self.safety.check(SafetyState(
            now=now,
            time_since_measurement=context.time_since_measurement,
            image_age=self._worst_image_age(measurements, now),
            position_jump=self._position_jump(object_position),
            reprojection_px=max((m.reprojection_px for m in measurements), default=0.0),
            target_speed=self.tracker.speed,
            joint_angles_deg=self._joint_angles_deg,
            ee_position=ee,
            commanded_position=directive.target,
            diverging=self.convergence.diverging,
            allow_table_contact=directive.allow_table_contact,
        ))
        # Le superviseur a vu la cible que la machine vient de choisir : si elle
        # est refusée, on refait un tour de machine avec l'arrêt en entrée, pour
        # que l'état bascule en SAFE_STOP au lieu d'exécuter un ordre invalidé.
        # Tant que l'opérateur n'a pas armé la boucle, la sûreté est évaluée et
        # publiée mais ne fait pas basculer la machine : sinon le contrôleur
        # démarre en SAFE_STOP parce qu'aucune caméra n'a encore publié, et
        # crache une erreur avant même que les caméras soient initialisées.
        if self._running and report.stop and \
                self.machine.state not in (State.SAFE_STOP, State.DONE):
            context.safety_stop = True
            context.safety_reasons = report.reasons
            directive = self.machine.step(context)
            self.get_logger().error('ARRÊT — ' + ' | '.join(report.reasons))
        elif self._running and report.verdict.name == 'SLOW':
            self.get_logger().warn('ralentissement — ' + ' | '.join(report.reasons))

        if self._running:
            self._execute(directive, context, ee, dt, report.speed_scale, now)
        self._publish_status(now, context, directive, report, result)
        self._publish_frames(now, object_position, directive)

        self._last_object_position = (None if object_position is None
                                      else np.asarray(object_position).copy())

    def _collect_measurements(self, now: float) -> list[CameraMeasurement]:
        out = []
        for camera in self.cameras:
            d = self._detections.get(camera)
            if d is None or not d.get('valid'):
                continue
            out.append(CameraMeasurement(
                camera=camera,
                position=np.asarray(d['position'], np.float64),
                stamp=float(d['stamp']),
                detector_score=float(d.get('detector_score', 1.0)),
                reprojection_px=float(d.get('reprojection_px', 0.0)),
                pixel_area=float(d.get('pixel_area', 0.0)),
                sharpness=float(d.get('sharpness', 1.0)),
                valid=True))
        return out

    def _update_latency(self, measurements, now: float):
        """τ mesuré = âge de l'image la plus fraîche + latence de commande (§11.3).

        Lissé, parce qu'une τ qui saute déplace la cible prédite d'un coup. La
        constante 0.2 laisse la mesure dominer sans transmettre le bruit.
        """
        if not measurements:
            return
        freshest = min(now - m.stamp for m in measurements)
        measured = max(0.0, freshest) + self.extra_latency
        self._latency = 0.8 * self._latency + 0.2 * measured

    def _current_object_position(self, result, valid) -> np.ndarray | None:
        """Position de l'objet compensée en latence (§8.2), z sur le plan objet."""
        if not valid:
            return None
        predicted_xy = self.tracker.predict_position(self._latency)
        return np.array([predicted_xy[0], predicted_xy[1], float(result.position[2])])

    def _position_jump(self, position) -> float:
        if position is None or self._last_object_position is None:
            return 0.0
        return float(np.linalg.norm(np.asarray(position) - self._last_object_position))

    def _worst_image_age(self, measurements, now: float) -> float:
        """Âge de l'image la plus FRAÎCHE : une caméra lente ne doit pas tout bloquer."""
        if not measurements:
            return 0.0
        return max(0.0, min(now - m.stamp for m in measurements))

    def _build_context(self, now, ee, object_position, result, valid) -> Context:
        target = None
        if object_position is not None:
            target = np.asarray(object_position, np.float64).copy()
            target[2] += self.mission.pregrasp_height
        elif self.lock.locked is not None:
            target = self.lock.locked.copy()
            target[2] += self.mission.pregrasp_height

        error_norm = float('inf')
        if ee is not None and target is not None:
            error_norm = float(np.linalg.norm(ee - target))
            self.convergence.push(error_norm)

        if valid and not self.mission.mobile_target and \
                self.machine.state in (State.TRACK, State.SEARCH):
            self.lock.add(object_position)

        return Context(
            now=now,
            ee_position=ee,
            object_position=object_position,
            object_velocity=self.tracker.velocity,
            measurement_valid=valid,
            confidence=result.confidence if result else 0.0,
            contributing=result.contributing if result else (),
            time_since_measurement=self.fusion.time_since_valid(now),
            primary_visible=self.fusion.primary in (result.contributing if result else ()),
            lateral_visible=self.mission.lateral_camera in (
                result.contributing if result else ()),
            error_norm=error_norm,
            converged=self.convergence.converged,
            locked_position=self.lock.locked,
            gripper_closed=(None if self._gripper_status is None
                            else self._gripper_status == 2),
            safety_stop=False)

    # — sorties ———————————————————————————————————————————————————————

    def _execute(self, directive, context, ee, dt, speed_scale, now):
        if directive.action is Action.STOP:
            self._running = False
            return
        if directive.action is Action.GRIP:
            self._send_gripper(
                self.grasp_firm_angle if directive.firm else self.grasp_angle, now)
            return
        if directive.action is Action.RELEASE:
            self._send_gripper(self.open_angle, now)
            return
        if directive.action is not Action.SERVO or directive.target is None or ee is None:
            return

        command = compute_command(ee, directive.target, self.tracker.velocity,
                                  dt, self.gains, fine=directive.fine)
        if command.idle:
            return
        step = command.step * speed_scale
        self._send_cartesian_step(ee, ee + step, now)

    def _send_cartesian_step(self, ee, target, now):
        """Incrément cartésien -> angles par IK différentielle -> send_angles (§8.3).

        `solve_pose` part des angles COURANTS et tient l'orientation courante :
        le poignet ne dérive pas et la solution reste sur la même branche IK que
        la pose de départ — c'est ce qui évite le basculement de coude qui a mis
        le bras en auto-collision lors des essais top-down.
        """
        if self._joint_angles_deg is None:
            return
        flange_mm, rotation = fk_pose(self._joint_angles_deg)
        flange_target_mm = np.asarray(flange_mm) + (np.asarray(target) - ee) * 1000.0
        angles = solve_pose(self._joint_angles_deg, flange_target_mm, rotation)

        # Défense finale avant le bridge : aucune commande hors du domaine réel,
        # même si un autre solveur ou une future modification contourne le clip IK.
        outside = ((angles < JOINT_LIMITS_DEG[:, 0]) |
                   (angles > JOINT_LIMITS_DEG[:, 1]))
        if np.any(outside):
            joints = ', '.join(f'J{i + 1}={angles[i]:.1f}°'
                               for i in np.flatnonzero(outside))
            self.get_logger().error(
                f'Commande refusée avant bridge : limite articulaire ({joints})')
            return

        reached_mm, _ = fk_pose(angles)
        residual = float(np.linalg.norm(reached_mm - flange_target_mm))
        if residual > 3.0:
            self.get_logger().warn(
                f'IK à {residual:.1f}mm de la cible — pose hors d\'atteinte, '
                f'incrément ignoré')
            return

        # Une seule commande en vol : attendre que les encodeurs confirment le
        # mouvement précédent avant d'en publier une autre. Sans ce verrou, la
        # boucle 10 Hz empile des consignes plus vite que le retour robot 5 Hz.
        if not self.safety.register_command(ee, target, now):
            return
        self._commanded_angles = angles
        if self.dry_run:
            return
        self.pub_robot.publish(String(data=json.dumps({
            'action': 'send_angles', 'angles': [float(a) for a in angles],
            'speed': self.robot_speed})))

    def _send_gripper(self, angle: int, now: float):
        """La pince Pro exige ~1,6 s entre deux ordres — sinon elle les ignore."""
        if now - self._last_gripper_cmd < 1.6:
            # La machine croit l'ordre parti : le dire, sinon un serrage avalé
            # ressemble à un serrage inefficace.
            self.get_logger().warn(
                f'ordre pince angle={angle} ignoré — {now - self._last_gripper_cmd:.2f}s '
                f'depuis le précédent (< 1,6 s)')
            return
        self._last_gripper_cmd = now
        if self.dry_run:
            self.get_logger().info(f'[dry-run] pince angle={angle}')
            self._gripper_status = 2 if angle <= self.grasp_angle else 1
            return
        self.pub_robot.publish(String(data=json.dumps({
            'action': 'pro_gripper_angle', 'angle': angle, 'gripper_id': GRIPPER_ID})))
        self.pub_robot.publish(String(data=json.dumps({
            'action': 'get_pro_gripper_status', 'gripper_id': GRIPPER_ID})))

    def _publish_status(self, now, context, directive, report, result):
        payload = {
            'stamp': now,
            'state': self.machine.state.value,
            'action': directive.action.value,
            'reason': directive.reason,
            'running': self._running,
            'dry_run': self.dry_run,
            'error_mm': (None if not np.isfinite(context.error_norm)
                         else context.error_norm * 1000.0),
            'confidence': context.confidence,
            'mode': result.mode if result else 'PERDU',
            'cameras': list(result.contributing) if result else [],
            'per_camera_confidence': result.per_camera if result else {},
            'latency_ms': self._latency * 1000.0,
            'object_speed_mm_s': self.tracker.speed * 1000.0,
            'time_since_measurement': context.time_since_measurement,
            'locked': self.lock.locked is not None,
            'gripper_status': (None if self._gripper_status is None else
                               GRIPPER_STATUS_TEXT.get(self._gripper_status, '?')),
            'safety': report.verdict.value,
            'safety_reasons': list(report.reasons),
        }
        self.pub_status.publish(String(data=json.dumps(payload)))

    def _publish_frames(self, now, object_position, directive):
        """base_link -> object_filtered -> pregrasp_target (§11.2)."""
        stamp = self.get_clock().now().to_msg()
        if object_position is not None:
            self._tf.sendTransform(_transform(
                stamp, 'base_link', 'object_filtered', object_position))
        if directive.target is not None:
            offset = (np.asarray(directive.target) - np.asarray(object_position)
                      if object_position is not None else np.asarray(directive.target))
            self._tf.sendTransform(_transform(
                stamp, 'object_filtered' if object_position is not None else 'base_link',
                'pregrasp_target', offset))
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = float(directive.target[0])
            pose.pose.position.y = float(directive.target[1])
            pose.pose.position.z = float(directive.target[2])
            pose.pose.orientation.w = 1.0
            self.pub_target.publish(pose)


def _transform(stamp, parent: str, child: str, translation) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(translation[0])
    t.transform.translation.y = float(translation[1])
    t.transform.translation.z = float(translation[2])
    t.transform.rotation.w = 1.0
    return t


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
