#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAL062_AUTONOMY_WS="$(cd "$SCRIPT_DIR/../workspace" && pwd)"

CONTAINER_NAME="hal062_autonomy_container"
IMAGE_NAME="hal062_autonomy:latest"
CONTAINER_USER="admin"
CONTAINER_WS="/workspaces"

xhost +local:docker

if [ -n "$(docker ps -q -f name="^${CONTAINER_NAME}$")" ]; then
    echo "Attaching to running container: ${CONTAINER_NAME}"
    docker exec -it \
        -u "${CONTAINER_USER}" \
        -w "${CONTAINER_WS}" \
        -e DISPLAY \
        "${CONTAINER_NAME}" bash
else
    if [ -n "$(docker ps -aq -f name="^${CONTAINER_NAME}$")" ]; then
        docker rm "${CONTAINER_NAME}"
    fi

    echo "Starting new container: ${CONTAINER_NAME}"
    docker run -it --rm \
        --name "${CONTAINER_NAME}" \
        --runtime nvidia --gpus all \
        --privileged --network host --ipc host \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e DISPLAY -e ROS_DOMAIN_ID \
        -e USERNAME="${CONTAINER_USER}" \
        -e HOST_USER_UID="$(id -u)" \
        -e HOST_USER_GID="$(id -g)" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /dev:/dev \
        -v /dev/shm:/dev/shm \
        -v "$ISAAC_ROS_WS":/workspaces/isaac_ros-dev \
        -v "$HAL062_AUTONOMY_WS":"${CONTAINER_WS}/src" \
        -v hal062_zed_resources:/usr/local/zed/resources \
        -w "${CONTAINER_WS}" \
        "${IMAGE_NAME}"
fi
