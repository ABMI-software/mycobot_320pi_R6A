from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Détection des marqueurs ArUco
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='aruco_marker_publisher',
            parameters=[{
                'image_is_rectified': True,
                'marker_size': 0.05,  # Taille réelle de ton marqueur en mètres
                'reference_frame': 'camera_link',
            }],
            remappings=[
                ('/camera_info', '/camera/camera_info'),
                ('/image', '/camera/image_raw'),
            ]
        ),

        # 2. Transformation statique (Ajustée selon tes derniers logs)
        # On place la caméra à 40cm devant le robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['-0.4', '0.0', '0.43', '-1.5708', '0.0', '-1.5708', 'base', 'camera_link']
        ),

        # 3. Ton script de suivi
        Node(
            package='ton_package_name', # Remplace par le nom de ton package
            executable='relative_follower',
            name='relative_follower_node',
            output='screen'
        )
    ])