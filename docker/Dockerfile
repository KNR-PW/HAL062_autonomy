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


# Python dependencies
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
# RUN pip3 uninstall -y setuptools-scm
RUN pip3 install ultralytics \
    pygame \
    pyproj \
    pyserial \
    pynmea2 \
    nicegui \
    scipy


# Nav2 1.5 for hal_navigation (the base image only has nav2_costmap_2d/msgs/util as nvblox deps).
# Not ros-lyrical-nav2-bringup: it pulls ros_gz_sim + slam_toolbox + TB3/TB4 sims and its launch starts
# servers we do not configure (docs/NAV2_DESCRIPTION_PORT_PLAN.md §8.1-8.2).
RUN apt-get update -y && apt-get install -y \
    ros-lyrical-navigation2 \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /root/workspace
RUN mkdir -p src
COPY ./workspace /root/workspace/src

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

