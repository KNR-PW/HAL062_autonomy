# HAL062_autonomy
![OS](https://img.shields.io/badge/OS-Ubutu_24.04-orange)
![ROS](https://img.shields.io/badge/ROS-Lyrical_Luth-cyan)


## Quick Description
HAL062 autonomy software repository. ROS2 based packages, mainly responsible for navigation, perception, mapping, planning and traversal.


## Setup Guide
### 1. General environment setup guide
**Expected result:** Working NVIDIA docker environment.  
Visit our Wiki, and follow instructions on ["How To Setup"](https://wiki.knr.edu.pl/doc/how-to-setup-Ri1gZDAwQM) page.
### 2. Codebase setup guide
**Expected result:** Custom NVIDIA docker environment based development environment for HAL062.
```bash
git clone git@github.com:KNR-PW/HAL062_autonomy.git
cd HAL062_autonomy
docker build -f docker/Dockerfile -t hal062_autonomy .
```
## Launch guide
Simply run one bash script.
```bash
cd HAL062_autonomy/docker
chmod +x launch_docker.sh
./launch_docker.sh
```

## Packages (`workspace/`)
| Package | Content |
|---|---|
| `hal_interfaces` | custom msg/srv/action |
| `hal_vslam` | ZED 2i + Isaac ROS cuVSLAM + nvblox launch/config, `odom_manager`, `anti_teleport_node` (see `docs/VSLAM_PORT_PLAN.md`) |
| `hal_description` | rover URDF (xacro, ZED links from the `zed_wrapper` macro), meshes, `description.launch.py` |
| `hal_navigation` | Nav2 1.5 config, maps, `navigation.launch.py`, `autonomy.launch.py`, `planar_tf_broadcaster.py` (see `docs/NAV2_DESCRIPTION_PORT_PLAN.md`) |

### Build & test (inside the container)
```bash
cd /root/workspace
colcon build --packages-select hal_interfaces hal_description hal_vslam hal_navigation
source install/setup.bash
colcon test --packages-select hal_description hal_navigation && colcon test-result --verbose
```
`hal_navigation`'s launch test runs the whole Nav2 stack with a fake robot (no camera needed).
Nav2's controller uses `use_realtime_priority: true`, which needs `CAP_SYS_NICE`: `launch_docker.sh` runs
`--privileged`, so this is fine there; a plain `docker run` without `--privileged`/`--cap-add SYS_NICE` makes
FollowPath hang silently (see `docs/NAV2_DESCRIPTION_PORT_PLAN.md` §9).

### Run the autonomy stack (ZED 2i on USB 3.0, ZED Studio closed on the host)
```bash
ros2 launch hal_navigation autonomy.launch.py location:=blank_large      # description + VSLAM (+nvblox) + Nav2 + rviz
# or step by step:
ros2 launch hal_description description.launch.py
ros2 launch hal_vslam bringup.launch.py publish_urdf:=false publish_base_to_camera_tf:=false
ros2 launch hal_navigation navigation.launch.py location:=blank_large
```
Nav2 publishes `geometry_msgs/Twist` on `/cmd_vel`; the rover moves only when `hal_communication` `wheel_mux` relays it.
