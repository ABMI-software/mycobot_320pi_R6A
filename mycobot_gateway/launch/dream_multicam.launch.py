#!/usr/bin/env python3
"""Chaîne de validation DREAM multi-caméras — auto-détection 1 ou 2 caméras.

Sonde les caméras USB connues (arducam, SVPRO — voir camera_registry.py) au
lancement, puis spawne AUTOMATIQUEMENT une branche parallèle par caméra
branchée :

    camera_publisher(cam)  →  <image_topic>  →  dream_inference(cam)  →  <keypoints_topic>

plus joint_sync (/joint_states), bridge_tour (↔ Pi TCP 5005) et le dashboard,
à qui on passe la LISTE des caméras détectées. Le dashboard fusionne les vues
calibrées (≥2 → fusion des angles ; 1 → mono, comportement historique).

Chaque caméra part avec SON intrinsèque et SON exposition (arducam=75, SVPRO
normale) — rien à éditer à la main : brancher/débrancher la SVPRO suffit.

    ros2 launch mycobot_gateway dream_multicam.launch.py
    ros2 launch mycobot_gateway dream_multicam.launch.py model_name:=vgg_ultimate_v4_mix_ft_e30
    ros2 launch mycobot_gateway dream_multicam.launch.py cameras:=arducam   # forcer mono

⚠ Préalables inchangés : `deactivate` si (.venv), bridge_pi_simple.py côté Pi,
`ping 10.10.0.221`. Voir docs/DREAM_VALIDATION_LAUNCH.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mycobot_gateway.vision.camera_registry import detect_cameras


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        'model_name', default_value='vgg_ultimate_v4_mix_ft_e30',
        description='Checkpoint DREAM (dossier training/dream/checkpoints_dream/<name>)')
    cameras_arg = DeclareLaunchArgument(
        'cameras', default_value='',
        description="Liste forcée (ex. 'arducam' ou 'arducam,svpro'). Vide = auto-détection.")

    model_name = LaunchConfiguration('model_name')

    # Auto-détection au moment de construire la description (probe_capture=False :
    # on n'ouvre PAS les caméras ici — camera_publisher s'en charge ensuite).
    import os
    forced = os.environ.get('DREAM_CAMERAS', '').strip()  # échappatoire hors CLI
    names = [c.strip() for c in forced.split(',') if c.strip()] or None
    specs = detect_cameras(names=names, probe_capture=False)

    actions = [model_name_arg, cameras_arg]

    if not specs:
        actions.append(LogInfo(msg='❌ Aucune caméra connue détectée (arducam/svpro) — '
                                   'branche une caméra puis relance.'))
        return LaunchDescription(actions)

    detected = ', '.join(f'{s.name}(/dev/video{s.v4l2_index})' for s in specs)
    mode = 'FUSION' if len([s for s in specs if s.calib_ok]) >= 2 else 'MONO'
    actions.append(LogInfo(msg=f'📷 Caméras détectées : {detected} → mode {mode}'))

    for s in specs:
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
            package='mycobot_gateway', executable='dream_inference',
            name=f'dream_inference_{s.name}', output='screen',
            parameters=[{
                'camera_topic': s.image_topic,
                'model_name': model_name,
                'output_prefix': s.output_prefix,
                'visualize': False,   # l'overlay est dessiné par le dashboard
            }],
        ))

    # Nœuds partagés
    actions.append(Node(package='mycobot_gateway', executable='joint_sync',
                        name='joint_sync', output='screen'))
    actions.append(Node(package='mycobot_gateway', executable='bridge_tour',
                        name='bridge_tour', output='screen'))
    actions.append(Node(
        package='mycobot_gateway', executable='dream_validation_dashboard',
        name='dream_validation_dashboard', output='screen',
        parameters=[{'cameras': ','.join(s.name for s in specs)}],
    ))

    return LaunchDescription(actions)
