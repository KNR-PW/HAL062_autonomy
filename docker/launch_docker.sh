set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAL062_AUTONOMY_WS="$(cd "$SCRIPT_DIR/../workspace" && pwd)"

xhost +local:docker
docker run -it --rm \
    --name hal062_autonomy_container \
    --runtime nvidia --gpus all \
    --privileged --network host --ipc host \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY -e ROS_DOMAIN_ID \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    -v '$ISAAC_ROS_WS':/workspaces/isaac_ros-dev \
    -v '$HAL062_AUTONOMY_WS':/root/workspace/src \
    -w /root \
    hal062_autonomy:latest