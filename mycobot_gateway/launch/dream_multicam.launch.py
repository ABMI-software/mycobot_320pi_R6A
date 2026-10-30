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
`ping 10.10.0.224`. Voir docs/DREAM_VALIDATION_LAUNCH.md.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mycobot_gateway.vision.camera_registry import detect_cameras



def _pythonpath_venv_dream():
    """PYTHONPATH courant + le site-packages de venv_dream, AJOUTE EN FIN.

    En fin et non au debut : les entrees de PYTHONPATH precedent la bibliotheque
    standard dans `sys.path`, donc un paquet du venv portant le nom d'un module
    standard la masque pour tout l'interpreteur. C'est ce qui s'est produit le
    03/09 avec `enum34` (retroportage Python 2, desinstalle depuis) : l'import de
    `re` echouait sur `enum has no attribute global_enum` avant meme le demarrage
    du noeud.
    """
    venv = os.path.expanduser(
        '~/ros_jazzy/venv_dream/lib/python3.12/site-packages')
    courant = os.environ.get('PYTHONPATH', '')
    return f'{courant}:{venv}' if courant else venv


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        'model_name', default_value='vgg_ultimate_v4_mix_ft_e30',
        description='Checkpoint DREAM (dossier training/dream/checkpoints_dream/<name>)')
    # L'IP de la Pi n'est pas fixe (bail DHCP) : sans cet argument bridge_tour
    # retombe sur son defaut et se tait, /joint_states reste vide et le
    # dashboard n'a plus d'encodeurs a comparer.
    pi_ip_arg = DeclareLaunchArgument(
        'pi_ip', default_value='10.10.0.224',
        description='IP de la Raspberry Pi portant le bridge TCP 5005')
    cameras_arg = DeclareLaunchArgument(
        'cameras', default_value='',
        description="Liste forcée (ex. 'arducam' ou 'arducam,svpro'). Vide = auto-détection.")

    model_name = LaunchConfiguration('model_name')
    pi_ip = LaunchConfiguration('pi_ip')

    # Auto-détection au moment de construire la description (probe_capture=False :
    # on n'ouvre PAS les caméras ici — camera_publisher s'en charge ensuite).
    import os
    forced = os.environ.get('DREAM_CAMERAS', '').strip()  # échappatoire hors CLI
    names = [c.strip() for c in forced.split(',') if c.strip()] or None
    specs = detect_cameras(names=names, probe_capture=False)

    actions = [model_name_arg, cameras_arg, pi_ip_arg]

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
            # `torch` n'existe que dans venv_dream, et le Python systeme de ROS2
            # ne le voit pas : sans cet ajout le noeud meurt sur
            # `ModuleNotFoundError: No module named 'torch'`.
            #
            # L'ajout est PAR NOEUD, jamais global. Mesure du 03/09 : exporter ce
            # PYTHONPATH pour tout le lancement impose aussi le cv2 5.0 du venv a
            # `cv_bridge`, compile contre le 4.6 du systeme, et `camera_publisher`
            # meurt sur `KeyError: 16` — plus aucune image publiee. Or
            # `dream_inference` importe cv2 mais PAS cv_bridge, donc lui seul
            # supporte le cv2 du venv.
            additional_env={'PYTHONPATH': _pythonpath_venv_dream()},
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
                        name='bridge_tour', output='screen',
                        parameters=[{'pi_ip': pi_ip}]))
    actions.append(Node(
        package='mycobot_gateway', executable='dream_validation_dashboard',
        name='dream_validation_dashboard', output='screen',
        parameters=[{'cameras': ','.join(s.name for s in specs)}],
    ))

    return LaunchDescription(actions)
