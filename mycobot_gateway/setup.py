from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mycobot_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install scripts to lib/<package_name>/ for ros2 run to find them
        (os.path.join('lib', package_name), glob('scripts/*')),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Gazebo world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='genji',
    maintainer_email='genji@todo.todo',
    description='Bridge réseau et vision pour MyCobot - Tour (PC) side',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Bridge nodes
            'bridge_tour = mycobot_gateway.bridge_tour:main',
            
            # Vision nodes (complex computation on Tour)
            'marker_detector = mycobot_gateway.vision.marker_detector:main',
            'camera_publisher = mycobot_gateway.vision.camera_publisher:main',
            'orbbec_camera_publisher = mycobot_gateway.orbbec_camera_publisher:main',
            'camera_live_view = mycobot_gateway.camera_live_view:main',
            'camera_web_view = mycobot_gateway.camera_web_view:main',
            
            # Robot command interface
            'robot_commander = mycobot_gateway.robot_commander:main',
            
            # Joint synchronization for RViz
            'joint_sync = mycobot_gateway.joint_sync:main',
            
            # GUI and control interfaces (NEW - Tour side)
            'simple_gui = mycobot_gateway.simple_gui:main',
            'slider_control = mycobot_gateway.slider_control:main',
            'teleop_keyboard = mycobot_gateway.teleop_keyboard:main',
            'marker_follower = mycobot_gateway.marker_follower:main',
            
            # Synthetic data collection (Gazebo)
            'synthetic_data_collector = mycobot_gateway.synthetic_data_collector:main',
            
            # DREAM inference + Pick-and-place (Gazebo)
            'dream_inference = mycobot_gateway.dream_inference_node:main',
            'pick_and_place = mycobot_gateway.pick_and_place_node:main',

            # DREAM vs encoders real-time validation dashboard (PyQt)
            'dream_validation_dashboard = mycobot_gateway.dream_validation_dashboard:main',

            # Asservissement visuel en boucle fermée (pick-and-place adaptatif)
            'object_pose_node = mycobot_gateway.visual_servo.object_pose_node:main',
            'visual_servo_controller = mycobot_gateway.visual_servo.visual_servo_node:main',

            # Multi-object color sorting (Gazebo)
            'color_object_detector = mycobot_gateway.color_object_detector:main',
            'sorting_orchestrator = mycobot_gateway.sorting_orchestrator:main',
            'sim_sorting_grasp = mycobot_gateway.sim_sorting_grasp:main',

            # Hand teleoperation (trajectory → JSON bridge for real robot)
            'trajectory_to_robot_bridge = mycobot_gateway.trajectory_to_robot_bridge:main',
            'gripper_to_robot_bridge = mycobot_gateway.gripper_to_robot_bridge:main',

            # ── Benchmark de précision ──────────────────────────────────────
            # Localizer ArUco (robot réel) : détecte workspace + objet
            'aruco_localizer = mycobot_gateway.aruco_localizer_node:main',
            # Localizer Gazebo (simulation) : publie pose GT Gazebo
            'gz_sim_localizer = mycobot_gateway.gz_sim_localizer_node:main',
            # FK EE pose (commun sim + réel) : /joint_states → /fk/ee_pose
            'fk_ee_pose = mycobot_gateway.fk_ee_pose_node:main',
            # Orchestrateur benchmark : grille 9 cibles + rapport CSV
            'precision_benchmark = mycobot_gateway.precision_benchmark_node:main',

            # ── Pick-and-place ArUco (sim + robot réel) ─────────────────────
            # mode=sim  : gz set_pose emulation + mycobot_controller
            # mode=real : trajectory_to_robot_bridge + bridge_tour + gripper
            'pick_and_place_aruco = mycobot_gateway.pick_and_place_aruco_node:main',
            # one-shot reach test (no gripper): /aruco/object_pose -> robot target
            'reach_target_aruco = mycobot_gateway.reach_target_aruco_node:main',

            # ── Calibration (§3.2 extrinsèque, §3.3 hand-eye) ───────────────
            'calibrate_extrinsic = mycobot_gateway.calibrate_extrinsic_node:main',
            'calibrate_hand_eye = mycobot_gateway.calibrate_hand_eye_node:main',
        ],
    },
)
