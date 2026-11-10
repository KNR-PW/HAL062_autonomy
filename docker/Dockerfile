FROM cached_isaac_run_dev_image_local:latest
SHELL ["/bin/bash", "-c"]

# OpenCV with ArUco support
RUN apt-get update -y --allow-releaseinfo-change && apt-get install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*


# Additional ROS stuff
RUN apt-get update -y && apt-get install -y \
    libboost-all-dev \
    v4l-utils \
    python3-pykdl \
    ros-lyrical-robot-localization \
    ros-lyrical-rclcpp \
    ros-lyrical-tf-transformations \
    tilix \
    nano \
    && rm -rf /var/lib/apt/lists/*


# Isaac ROS packages missing from the base image (built with keys zed+cuvslam only):
# nvblox (for hal_vslam nvblox.launch.py + mesh_to_pointcloud), launch utils, ZED launch fragments, foxglove.
# Deliberately NOT ros-lyrical-isaac-ros-nvblox / nvblox-examples-bringup: they pull detectnet/peoplenet/Triton (GBs).
RUN apt-get update -y && apt-get install -y \
    ros-lyrical-nvblox-ros \
    ros-lyrical-nvblox-msgs \
    ros-lyrical-nvblox-ros-python-utils \
    ros-lyrical-nvblox-rviz-plugin \
    ros-lyrical-nvblox-nav2 \
    ros-lyrical-isaac-ros-launch-utils \
    ros-lyrical-isaac-ros-zed \
    ros-lyrical-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update -y && apt-get install -y \
    ros-lyrical-ros-gz \
    ros-lyrical-sdformat-urdf \
    ros-lyrical-gz-ros2-control \
    ros-lyrical-ros2-control \
    ros-lyrical-ros2-controllers \
    ros-lyrical-xacro \
    ros-lyrical-robot-state-publisher \
    ros-lyrical-joint-state-publisher \
    ros-lyrical-joint-state-publisher-gui \
    ros-lyrical-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*


ENV PATH="/opt/venv/bin:$PATH"
RUN python3 -m venv --system-site-packages /opt/venv \
    && pip3 install --no-cache-dir \
        ultralytics \
        pygame \
        pyproj \
        pyserial \
        pynmea2 \
        nicegui \
        scipy \
    && chmod -R a+rwX /opt/venv


# Workspace. Outside /root because the container runs as a non-root user ('hal062-admin')
# whose UID/GID match the host user - see entrypoint.sh / launch_docker.sh.
ENV HAL062_WS=/workspaces
WORKDIR ${HAL062_WS}
COPY ./workspace ${HAL062_WS}/src

# Shell setup for every user (ROS itself is already sourced in /etc/bash.bashrc by the base image)
RUN echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /etc/bash.bashrc \
    && echo '[ -f "${HAL062_WS}/install/setup.bash" ] && source "${HAL062_WS}/install/setup.bash"' >> /etc/bash.bashrc

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
