import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # 1. Chemins des fichiers
    model_path = os.path.join(
        get_package_share_directory("mycobot_description"),
        "urdf/320_pi/mycobot_pro_320_pi.urdf"
    )
    
    rviz_config_path = os.path.join(
        get_package_share_directory("mycobot_320pi"),
        "config/mycobot_320_pi.rviz"
    )

    return LaunchDescription([
        # Robot State Publisher : Gère le modèle URDF et publie l'arbre TF du robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[model_path]
        ),

        # Joint State Publisher : Lit les angles réels des servomoteurs
        Node(
            package='mycobot_320pi',
            executable='follow_display',
            name='follow_display',
        ),

        # CORRECTION : Lien statique entre la base du robot et la caméra
        # On définit la position physique : x=0.6m, y=0.0m, z=0.43m
        # On applique une rotation pour que le 'devant' de la caméra soit le 'devant' du robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['1.60', '0.05', '0.40', '3.14159', '-1.5708', '3.14159', 'base', 'camera_link']
        ),

        # Nœuds de Vision
        Node(
            package="mycobot_320pi", 
            executable="opencv_camera", 
            name="opencv_camera"
        ),
        Node(
            package="mycobot_320pi", 
            executable="detect_marker", 
            name="detect_marker"
        ),

        # Nœud de Contrôle : Le script de suivi que nous avons mis à jour
        Node(
            package="mycobot_320pi", 
            executable="following_marker", 
            name="following_marker"
        ),

        # RViz2 pour visualiser la scène en 3D
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])