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
## Launch and attach guide
Simply run one bash script.
```bash
cd HAL062_autonomy/docker
chmod +x launch_docker.sh
./launch_docker.sh
```
If you want to attach to the exisiting container (like `docker exec`)
```bash
./launch_docker.sh
```

## Use Guide
Visit ["Code Documentation"](https://wiki.knr.edu.pl/doc/code-documentation-MxS5VXgT31) page for more precise info about packages, utilities, tools, code, etc.

### Most Used Commands
**Launch simulation**
```bash
ros2 launch hal_simulation simulation.launch.py 
```
World and sensor parameters (GPS origin lat/lon, rates, noise) are in `workspace/hal_simulation/config/simulation.yaml`
(or `sim_config:=<your copy>`). Simulated sensors bridged to ROS, same topics/frames as on the rover:

| Sensor | Topic | Type | frame_id |
|---|---|---|---|
| ZED 2i IMU | `/zed/zed_node/imu/data` | `sensor_msgs/Imu` (200 Hz, orientation in ENU) | `zed_imu_link` (= `zed_left_camera_frame`) |
| ZED 2i magnetometer | `/zed/zed_node/imu/mag` | `sensor_msgs/MagneticField` (50 Hz, Tesla) | `zed_imu_link` |
| GPS | `/fix` | `sensor_msgs/NavSatFix` (10 Hz) | `gps_link` |
| ZED left/right gray | `/zed/zed_node/{left,right}/gray/rect/image` + `camera_info` | `mono8`, 1280x720 @ 30 Hz | `zed_{left,right}_camera_frame_optical` |
| ZED left color | `/zed/zed_node/left/color/rect/image` + `camera_info` | `rgb8` (rover: `bgra8`), 640x360 @ 10 Hz | `zed_left_camera_frame_optical` |
| ZED depth | `/zed/zed_node/depth/depth_registered` + `depth/camera_info` | `32FC1` [m], rendered (ideal) depth | `zed_left_camera_frame_optical` |
| ZED point cloud | `/zed/zed_node/point_cloud/cloud_registered` | `PointCloud2`, 320x180 @ 10 Hz (own small depth sensor) | `zed_left_camera_frame` |

Camera topics are bridged lazily (only rendered while something subscribes).
`/mux/cmd_vel` drives the simulated rover; with `relay_cmd_vel:=true` also `/cmd_vel` (Nav2), bypassing `wheel_mux`.

**Whole autonomy stack (cuVSLAM + nvblox + Nav2) on the simulation**
```bash
ros2 launch hal_navigation autonomy.launch.py simulation:=true [headless:=true] [nvblox:=false]
```
Same launch as on the rover: `simulation:=true` starts `hal_simulation` instead of the bare description, skips the ZED
wrapper and runs everything with `use_sim_time`. Needs an image with Nav2 (rebuild after the `ros-lyrical-navigation2`
line in `docker/Dockerfile`).

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
