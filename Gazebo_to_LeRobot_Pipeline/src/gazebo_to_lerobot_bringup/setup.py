import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gazebo_to_lerobot_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tomislav Ilic",
    maintainer_email="tomislav_tole_ilic@yahoo.com",
    description="Sim bring-up launch and joint-trajectory test script for the Gazebo_to_LeRobot_Pipeline sandbox.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "send_test_trajectory = gazebo_to_lerobot_bringup.send_test_trajectory:main",
            "send_moveit_goal = gazebo_to_lerobot_bringup.send_moveit_goal:main",
            "teleop_arm_keyboard = gazebo_to_lerobot_bringup.teleop_arm_keyboard:main",
        ],
    },
)
