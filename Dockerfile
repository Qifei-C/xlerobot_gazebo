FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y \
    git \
    libgl1-mesa-dri \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-jazzy-controller-manager \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-omni-wheel-drive-controller \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces/ws

COPY src /workspaces/ws/src
COPY ros_entrypoint_overlay.sh /ros_entrypoint_overlay.sh

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

RUN chown -R "${USER_UID}:${USER_GID}" /workspaces/ws /ros_entrypoint_overlay.sh \
    && chmod +x /ros_entrypoint_overlay.sh

USER ${USER_UID}:${USER_GID}

ENTRYPOINT ["/ros_entrypoint_overlay.sh"]
CMD ["bash"]
