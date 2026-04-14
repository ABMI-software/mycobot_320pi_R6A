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
        ],
    },
)
