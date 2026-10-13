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


# Python dependencies
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
# RUN pip3 uninstall -y setuptools-scm
RUN pip3 install ultralytics \
    pygame \
    pyproj \
    pyserial \
    pynmea2 \
    nicegui


WORKDIR /root/workspace
RUN mkdir -p src
COPY ./workspace /root/workspace/src

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

