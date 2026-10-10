# Image for recording Osama's four-object sort (tri_yolo) with rosetta and
# converting it to LeRobot. Built on the Gazebo_to_LeRobot_Pipeline image
# (ROS 2 Jazzy, Gazebo Harmonic, MoveIt, ros2_control), which setup_workspace.sh
# builds first from Gazebo_to_LeRobot_Pipeline/docker/Dockerfile.
#
# Adds what the 2026-10-10 acquisition needed and the base image lacked:
#   ros-jazzy-moveit-py   RLDS extraction (FK) and the RLDS replay (IK)
#   python3-venv/pip      the LeRobot environment below
#   python3-vcstool       cloning the pinned rosetta packages
#   /workspace/venv_lerobot  lerobot 0.6.1 + CPU torch 2.11, the exact versions
#       that ported and reloaded the datasets (requirements_lerobot.txt). It sees
#       ROS's Python (--system-site-packages) without changing it: numpy 2,
#       recent protobuf/grpcio/scipy live only in the venv.
FROM gazebo_to_lerobot:jazzy-harmonic

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit-py \
        python3-venv \
        python3-pip \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

COPY requirements_lerobot.txt /tmp/requirements_lerobot.txt
RUN python3 -m venv --system-site-packages /workspace/venv_lerobot \
    && /workspace/venv_lerobot/bin/pip install --no-cache-dir --upgrade pip \
    && /workspace/venv_lerobot/bin/pip install --no-cache-dir -r /tmp/requirements_lerobot.txt \
    && touch /workspace/venv_lerobot/COLCON_IGNORE

CMD ["sleep", "infinity"]
