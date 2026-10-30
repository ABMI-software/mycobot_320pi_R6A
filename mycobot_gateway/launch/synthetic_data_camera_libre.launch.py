#!/usr/bin/env python3
"""Collecte synthetique a POSE DE CAMERA RANDOMISEE (une par trame).

Meme topologie que `synthetic_data_v3.launch.py`, qui n'est pas modifie, avec
deux ajouts : le modele autonome `worlds/camera_libre.sdf` est spawne dans le
monde, et son image est pontee vers ROS. Le collecteur
`synthetic_data_collector_v3_camera_libre` deplace ce modele par `set_pose`
avant chaque echantillon.

Pourquoi : mesure du 02/09/2026, le keypoint `base` du reseau est une constante
par montage (il suit 0 % d'un decalage d'image). Le synthetique n'ayant que 4
points de vue FIXES et l'affinage reel 2 cameras fixes, memoriser la position du
socle est une strategie gagnante. Une pose de camera continue, differente a
chaque trame, supprime la constante a apprendre.

Les 4 cameras d'origine restent dans l'URDF mais ne sont ni pontees ni
enregistrees ici : `multi_view` vaut true et la seule vue declaree par le
collecteur est `libre`.

Usage :
  ros2 launch mycobot_gateway synthetic_data_camera_libre.launch.py \
      num_samples:=20000 output_dir:=/tmp/synth_cam_libre
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'randomized'


def generate_launch_description():
    desc_pkg = get_package_share_directory('mycobot_description')
    gz_pkg = get_package_share_directory('ros_gz_sim')

    # URDF AVEC la pince Pro adaptative, contrairement au v3 qui prend la
    # variante `_nogripper`. Le v3 la justifiait par « pour que le joint 6 / la
    # bride soit entierement visible » — mais a l'inference la bride est
    # TOUJOURS occultee par la pince reellement montee : entrainer sur une bride
    # nue apprend une situation qui n'existe jamais. `pro_adaptive_gripper` est
    # bien le modele monte sur le robot.
    urdf_path = os.path.join(
        desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf')
    world_path = os.path.join(desc_pkg, 'worlds', f'{WORLD_NAME}.sdf')
    camera_sdf = os.path.join(desc_pkg, 'worlds', 'camera_libre.sdf')

    gz_resource_path = os.path.dirname(desc_pkg)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    set_gz_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f'{gz_resource_path}:{existing}' if existing else gz_resource_path)

    args = [
        DeclareLaunchArgument('num_samples', default_value='20000'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/synth_cam_libre'),
        # 2,5 s et non 1,5 : les poses sont tirees independamment, donc le bras
        # saute de ~80 deg, et avec la pince montee il n'arrivait pas dans le
        # delai — 20 relances pour 18 images le 02/09. Chaque relance coute une
        # attente entiere, il est moins cher d'attendre une fois assez longtemps.
        DeclareLaunchArgument('settle_time', default_value='2.5'),
        # 1,0 s demandait au controleur d'avaler 80 deg en une seconde : il
        # saturait et restait en retard.
        DeclareLaunchArgument('move_time', default_value='2.0'),
        DeclareLaunchArgument('domain_randomize', default_value='true'),
        DeclareLaunchArgument('noise_stddev', default_value='0.0'),
        DeclareLaunchArgument('grasp_fraction', default_value='0.0'),
        DeclareLaunchArgument('table_clearance', default_value='0.05'),
        DeclareLaunchArgument('dist_min', default_value='0.70'),
        DeclareLaunchArgument('dist_max', default_value='1.45'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description}], output='screen')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}',
                          'on_exit_shutdown': 'true'}.items())

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'mycobot_320', '-z', '0.0'])

    # La camera est spawnee ICI et non par le noeud : le pont ros_gz doit
    # trouver le topic gz au demarrage, sinon il ne le relaie jamais.
    spawn_camera = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-file', camera_sdf, '-name', 'cam_libre',
                   '-x', '1.1', '-y', '0.0', '-z', '0.9',
                   '-R', '0.0', '-P', '0.6', '-Y', '3.1416'])

    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/world/{WORLD_NAME}/model/mycobot_320/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Le collecteur deplace la camera par CE pont, et non par un
            # `gz service` par echantillon. Mesure du 02-03/09 sur 7 245 images :
            # chaque appel CLI enregistre un noeud gz-transport ephemere aupres
            # du serveur, qui garde ensuite son adresse morte (« Host
            # unreachable » en boucle dans le log). La table de decouverte
            # s'empoisonne de facon MONOTONE et les set_pose finissent par ne
            # plus repondre : 100 % de poses obtenues sur les 500 premieres
            # images, 83 % a la 2 500e, 38 % a la 4 500e, 7,8 % a la 7 000e — et
            # la cadence tombe de 1,8 a 7,3 s/image. Relancer ne fait que
            # remettre le compteur a zero. Le pont, lui, est un processus unique
            # et permanent : un seul noeud gz-transport pour toute la collecte.
            f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        remappings=[(f'/world/{WORLD_NAME}/model/mycobot_320/joint_state',
                     '/gz/joint_states')])

    controller_cfg = os.path.join(desc_pkg, 'config', 'controller.yaml')
    # Spawners ECHELONNES, cadence reprise de `sim_grasp.launch.py` — le seul
    # launch ecrit pour cet URDF-ci. Lances immediatement (comme dans le v3, qui
    # n'a pas de pince), `joint_state_broadcaster` echoue a s'activer : mesure du
    # 02/09, « Failed to activate controller » a 5 s, et alors `/joint_states`
    # reste MUET. Le collecteur retombe alors sur les angles COMMANDES au lieu
    # des mesures — or la pince fait flechir le bras, donc les deux different :
    # 40 relances pour 10 poses et des etiquettes fausses.
    joint_state_broadcaster = TimerAction(period=4.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')])
    mycobot_controller_spawner = TimerAction(period=5.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['mycobot_controller', '--controller-manager', '/controller_manager',
                   '--param-file', controller_cfg],
        output='screen')])

    # La pince a SIX articulations pilotees, jamais `mimic` : DART ignore les
    # contraintes <mimic> de l'URDF. Sans ce controleur, les deux barres
    # exterieures du parallelogramme restent immobiles pendant que les doigts
    # tournent — « les deux bras noirs a l'horizontale, la pince cassee sur les
    # cotes », defaut corrige le 31/08 et documente dans `controller.yaml`.
    # Cadence de spawn reprise de `sim_grasp.launch.py`.
    gripper_controller_spawner = TimerAction(period=6.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_position_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controller_cfg],
        output='screen')])

    gz_image_bridge = Node(
        package='ros_gz_image', executable='image_bridge', name='gz_image_bridge',
        output='screen', arguments=['/cam_libre/image'])

    collector = TimerAction(
        period=14.0,
        actions=[Node(
            package='mycobot_gateway',
            executable='synthetic_data_collector_v3_camera_libre',
            name='synthetic_data_collector_v3_camera_libre',
            output='screen',
            parameters=[{
                'num_samples': LaunchConfiguration('num_samples'),
                'output_dir': LaunchConfiguration('output_dir'),
                'settle_time': LaunchConfiguration('settle_time'),
                'move_time': LaunchConfiguration('move_time'),
                'multi_view': True,
                'domain_randomize': LaunchConfiguration('domain_randomize'),
                'noise_stddev': LaunchConfiguration('noise_stddev'),
                'grasp_fraction': LaunchConfiguration('grasp_fraction'),
                'table_clearance': LaunchConfiguration('table_clearance'),
                'dist_min': LaunchConfiguration('dist_min'),
                'dist_max': LaunchConfiguration('dist_max'),
                'world_name': WORLD_NAME,
            }])])

    return LaunchDescription([
        set_gz_resource, *args,
        rsp, gz_sim, spawn_robot, spawn_camera, gz_bridge,
        joint_state_broadcaster, mycobot_controller_spawner,
        gripper_controller_spawner, gz_image_bridge, collector,
    ])
