#!/usr/bin/env python3
"""Asservissement visuel en boucle fermée — pick-and-place adaptatif (§11).

Auto-détection des caméras comme dream_multicam : une branche par caméra
branchée, plus le contrôleur unique.

    camera_publisher(cam) → <image> → object_pose_node(cam) → /visual_servo/<cam>/detection ┐
    joint_sync (/joint_states) · bridge_tour (↔ Pi TCP 5005)                                ├→ visual_servo_controller
                                                                                            ┘

Le contrôleur démarre **désarmé** : `dry_run:=true` par défaut, et il attend un
ordre explicite avant de bouger. Deux verrous, volontairement :

    ros2 launch mycobot_gateway visual_servo.launch.py              # observe, ne commande rien
    ros2 launch mycobot_gateway visual_servo.launch.py dry_run:=false
    ros2 topic pub --once /visual_servo/command std_msgs/String '{data: start}'

Suivre la boucle en direct :
    ros2 topic echo /visual_servo/status

⚠ Le bridge de la Pi doit être `scripts/gripper_bridge.py`, PAS bridge_pi_simple.py :
seul le premier répond à `get_pro_gripper_status`. Sans ce retour, la saisie n'est
jamais confirmée (GRASP expire) et surtout la vérification de prise au levage
n'a rien à vérifier. Le contrôleur le signale explicitement s'il tombe dessus.

⚠ Préalables : `conda deactivate`, gripper_bridge.py côté Pi, `ping 10.10.0.221`,
extrinsèques calibrées et VALIDÉES (training/calibration/calibrate_camera_base_extrinsic.py
--camera arducam, puis --camera svpro).

⚠ Amener le bras dans l'orientation-démo (J5 ≈ −15°) avant de démarrer : la boucle
n'asservit que XYZ et tient l'orientation de départ. Le top-down [180,0,0] met le
bras en auto-collision.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mycobot_gateway.vision.camera_registry import detect_cameras

# camera_registry connaît l'intrinsèque de chaque caméra ; l'extrinsèque et le
# détecteur sont propres à l'asservissement.
EXTRINSIC_SUFFIX = '_extrinsic_servo.yaml'


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'cameras', default_value='',
            description="Liste forcée ('arducam' ou 'arducam,svpro'). Vide = auto."),
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='true = calcule et publie tout, n\'envoie AUCUNE commande.'),
        DeclareLaunchArgument(
            'mobile_target', default_value='false',
            description='true = objet susceptible de bouger : pas de verrou §7.1.'),
        DeclareLaunchArgument(
            'detector', default_value='color',
            description="'color' (balle HSV) ou 'aruco' (marqueur collé sur l'objet)."),
        DeclareLaunchArgument(
            'control_rate', default_value='10.0',
            description='Hz de la boucle. Le §11.3 demande ≥10 Hz de bout en bout.'),
        DeclareLaunchArgument(
            'aruco_id', default_value='40',
            description="ID du marqueur objet — NE PAS réutiliser 19/23/25/26."),
        DeclareLaunchArgument('table_z', default_value='0.0'),
        DeclareLaunchArgument(
            'stop_at_pregrasp', default_value='true',
            description='true = terminer au pré-grasp sans descendre ni fermer la pince.'),
        DeclareLaunchArgument(
            'object_height', default_value='0.071',
            description="Diamètre de l'objet (m). Défaut = la balle du dépôt."),
        DeclareLaunchArgument(
            'freeze_xy_on_descend', default_value='true',
            description='true = XY figé au pré-grasp, descente purement verticale.'),
        DeclareLaunchArgument(
            'descend_step', default_value='0.005',
            description='Palier de descente (m) — une réobservation par palier.'),
        DeclareLaunchArgument(
            'verify_lift_height', default_value='0.005',
            description='Levage de contrôle (m) avant de monter au transport.'),
        DeclareLaunchArgument(
            'grasp_angle', default_value='20',
            description='Fermeture (0-100, plus bas = plus serré). 20 = pick validé 17/08.'),
        DeclareLaunchArgument(
            'grasp_firm_angle', default_value='12',
            description='Serrage après contact confirmé. Doit rester ≤ grasp_angle.'),
        DeclareLaunchArgument(
            'grasp_firm', default_value='true',
            description='false = une seule fermeture, sans passe de serrage.'),
        DeclareLaunchArgument('auto_start', default_value='false'),
    ]

    forced = os.environ.get('SERVO_CAMERAS', '').strip()
    names = [c.strip() for c in forced.split(',') if c.strip()] or None
    specs = detect_cameras(names=names, probe_capture=False)

    actions = list(arguments)
    if not specs:
        actions.append(LogInfo(msg='❌ Aucune caméra connue détectée (arducam/svpro).'))
        return LaunchDescription(actions)

    calibrated = [s for s in specs if s.calib_ok]
    detected = ', '.join(f'{s.name}(/dev/video{s.v4l2_index})' for s in specs)
    mode = 'FUSION' if len(calibrated) >= 2 else 'MONO'
    warning = ('' if len(calibrated) >= 2 else
               " — une seule vue : aucune redondance contre l'occultation (§7.3).")
    actions.append(LogInfo(msg=f'📷 {detected} → {mode}{warning}'))

    detector = LaunchConfiguration('detector')
    table_z = LaunchConfiguration('table_z')

    # `direct_capture` : object_pose_node ouvre lui-même le périphérique V4L2 et
    # aucune image ne transite par le middleware. Mesuré le 18/08, le passage par
    # un topic plafonnait la perception à 1-8 Hz de façon erratique — sous la
    # fenêtre de fraîcheur de la fusion, qui déclarait la cible perdue alors que
    # les détections étaient bonnes. C'est la « communication intra-processus »
    # du §11.3. Contrepartie : /camera/image_raw n'existe pas dans CE launch ;
    # la chaîne DREAM garde le sien (dream_multicam.launch.py), inchangée.
    direct = os.environ.get('SERVO_IMAGE_TOPIC', '').strip() == ''

    for s in specs:
        if not direct:
            actions.append(Node(
                package='mycobot_gateway', executable='camera_publisher',
                name=f'camera_publisher_{s.name}', output='screen',
                parameters=[{
                    'camera_index': s.v4l2_index,
                    'manual_exposure': s.manual_exposure,
                    'manual_focus': s.manual_focus,
                    'output_topic': s.image_topic.lstrip('/'),
                }],
            ))
        actions.append(Node(
            package='mycobot_gateway', executable='object_pose_node',
            name=f'object_pose_{s.name}', output='screen',
            parameters=[{
                'camera': s.name,
                'image_topic': s.image_topic,
                'camera_device': s.v4l2_index if direct else -1,
                'manual_exposure': s.manual_exposure,
                'capture_rate': 15.0,
                'calib_stem': _calib_stem(s.name),
                'extrinsic': '',          # défaut : <camera>_extrinsic_servo.yaml
                'output_prefix': f'/visual_servo/{s.name}',
                'detector': detector,
                'table_z': table_z,
                'object_height': LaunchConfiguration('object_height'),
                'aruco_id': LaunchConfiguration('aruco_id'),
            }],
        ))

    actions.append(Node(package='mycobot_gateway', executable='joint_sync',
                        name='joint_sync', output='screen'))
    actions.append(Node(package='mycobot_gateway', executable='bridge_tour',
                        name='bridge_tour', output='screen'))
    actions.append(Node(
        package='mycobot_gateway', executable='visual_servo_controller',
        name='visual_servo_controller', output='screen',
        parameters=[{
            'cameras': [s.name for s in specs],
            'primary_camera': specs[0].name,
            'control_rate': LaunchConfiguration('control_rate'),
            'dry_run': LaunchConfiguration('dry_run'),
            'mobile_target': LaunchConfiguration('mobile_target'),
            'stop_at_pregrasp': LaunchConfiguration('stop_at_pregrasp'),
            'freeze_xy_on_descend': LaunchConfiguration('freeze_xy_on_descend'),
            'descend_step': LaunchConfiguration('descend_step'),
            'verify_lift_height': LaunchConfiguration('verify_lift_height'),
            'grasp_angle': LaunchConfiguration('grasp_angle'),
            'grasp_firm_angle': LaunchConfiguration('grasp_firm_angle'),
            'grasp_firm': LaunchConfiguration('grasp_firm'),
            'auto_start': LaunchConfiguration('auto_start'),
            'table_z': table_z,
        }],
    ))
    return LaunchDescription(actions)


def _calib_stem(camera: str) -> str:
    from mycobot_gateway.vision.camera_registry import KNOWN_BY_NAME
    return KNOWN_BY_NAME[camera].calib_stem
