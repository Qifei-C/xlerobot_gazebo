FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=ros

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

RUN mkdir -p /workspaces/ws/src "/home/${USERNAME}" \
    && chown -R "${USER_UID}:${USER_GID}" /workspaces "/home/${USERNAME}"

WORKDIR /workspaces/ws

COPY --chown=${USER_UID}:${USER_GID} src /workspaces/ws/src
COPY --chown=${USER_UID}:${USER_GID} ros_entrypoint_overlay.sh /ros_entrypoint_overlay.sh

RUN chmod +x /ros_entrypoint_overlay.sh

ENV HOME=/home/${USERNAME}
ENV USER=${USERNAME}

USER ${USER_UID}:${USER_GID}

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build

RUN echo 'source /opt/ros/jazzy/setup.bash' >> "${HOME}/.bashrc" \
    && echo '[ -f /workspaces/ws/install/setup.bash ] && source /workspaces/ws/install/setup.bash' >> "${HOME}/.bashrc"

ENTRYPOINT ["/ros_entrypoint_overlay.sh"]
CMD ["bash"]
