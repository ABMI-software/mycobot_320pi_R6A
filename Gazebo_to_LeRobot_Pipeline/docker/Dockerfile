# Gazebo_to_LeRobot_Pipeline — ROS2 Jazzy + Gazebo Harmonic learning sandbox.
# Ubuntu 24.04 (the officially supported combo) inside a container, since
# the host OS (26.04) has no ROS2 Jazzy / Gazebo Harmonic binaries.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Desktop tools: rviz2, rqt framework
    ros-jazzy-desktop \
    # Gazebo Harmonic + ros2_control stack
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-position-controllers \
    ros-jazzy-controller-manager \
    # URDF / robot state
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    # Camera path (confirmed missing from every package.xml in the source repo)
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-rqt-image-view \
    # MoveIt2 (net-new — nothing to reuse from the source repo)
    ros-jazzy-moveit \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-moveit-visual-tools \
    # Build tooling
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    # Minimal GL/EGL loader userland; the actual WSLg driver is bind-mounted
    # at runtime from /usr/lib/wsl on the host.
    libgl1 \
    libglx-mesa0 \
    libegl1 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc

CMD ["sleep", "infinity"]
