# Port plan: Nav2 -> `hal_navigation`, URDF -> `hal_description` (Isaac ROS 5.0 / ROS 2 Lyrical)

State as of 2026-11-11. Every fact below was checked on this machine, in the `hal062_autonomy:latest`
image (throwaway `--rm` containers only, image not modified) or in the old stack under `ros2_docker_ws/`.
Companion document: `docs/VSLAM_PORT_PLAN.md` (hal_vslam port, phases 0-5 done).

---

## 1. What the old stack (`ros2_docker_ws/src`) contains that is relevant

| Old location | What it is | Goes to |
|---|---|---|
| `hal_main/config/nav2_params_erc.yaml` | Nav2 config actually used by the launch (default `nav2_params_file`) | `hal_navigation/config/nav2_params.yaml` |
| `hal_main/config/nav2_params.yaml`, `nav2_params_spd.yaml` | older tuning / "speed" variant (diff = inflation radii, DWB speeds/accels, rolling global costmap 400x400) | `nav2_params_spd.yaml` kept as variant; the old `nav2_params.yaml` is superseded by `_erc` (decision, see §8) |
| `hal_main/config/ekf_*.yaml` | robot_localization EKF, header says `NOT USED` | dropped |
| `hal_main/launch/navigation.launch.py` | map_server + `nav2_bringup/navigation_launch.py` + planar_tf_broadcaster + tf_limiter + manual lifecycle for map_server via `ros2 lifecycle set` TimerActions | `hal_navigation/launch/navigation.launch.py` (rewritten, see §4) |
| `hal_main/launch/robot_state_publisher.launch.py` | xacro -> robot_state_publisher + static TF `camera_frame -> zed2_camera_link` | `hal_description/launch/description.launch.py` |
| `hal_main/launch/full_autonomy.launch.py` | top-level orchestrator: rsp (or Gazebo sim), hal_vslam visual_slam (+1 s), nvblox (+8 s, commented out), navigation (+13 s), ArUco detector + global_pos_from_aruco, GOD detector (commented), GNSS watchdog + gnss_support (+10 s, `gps:=False`), navsat_transform (commented) | `hal_navigation/launch/autonomy.launch.py` – only the parts that exist in the new workspace (description, hal_vslam bringup, navigation); ArUco / GOD / GNSS stay as TODO args until those packages are ported |
| `hal_main/description/*.xacro` (+ generated `hal_062_vis.urdf`) | rover URDF: base_footprint -> chassis -> rockers -> wheels, camera at `camera_frame`, Gazebo plugins | `hal_description/urdf/` |
| `hal_main/maps/*.yaml + *.png` | map_server maps: `blank`, `blank_large` (both use `costmap_blank.png`), `erc` (`costmap_erc_26.png`), `mel` (`costmap_mel.png`) | `hal_navigation/maps/` – only the 4 yaml + 3 referenced png (the other 6 png are unreferenced history) |
| `hal_main/rviz/full.rviz` | rviz config for the full stack | `hal_navigation/rviz/navigation.rviz` with topic/frame renames |
| `hal_main/scripts/planar_tf_broadcaster.py` | publishes `map_2d`, `odom_2d`, `rover_2d` (roll/pitch removed, z = footprint z). **Hard dependency of the Nav2 config** (`robot_base_frame: rover_2d`, costmap frames `map_2d`/`odom_2d`) and of `hal_automata` | `hal_navigation/scripts/` (only node in the package) |
| `hal_main/scripts/tf_limiter.py` | republishes `map -> base_footprint` on `/limited_tf` at 10 Hz for the GUI (`hal_application`) | not navigation; port together with hal_application (decision §8) |
| `hal_main/src/z_offset_updater.cpp` | adjusts `mesh_layer.*obstacle_height` from TF `odom_low -> base_footprint`; `mesh_layer` is commented out in every config, `odom_low` frame exists nowhere, node was disabled in the launch (`delay_get` commented) | dropped (dead code) |
| `hal_main/scripts/vslam_initial_pose_from_imu.py` | initial SLAM yaw from IMU; commented out in launch, replaced by `anti_teleport_node` (already ported in hal_vslam) | dropped |
| `hal_main/scripts/{follow_waypoints,urc_automata,gnss_support,vslam_gnss_watchdog,aruco_obstacle,slam_pose_extractor,fake_odom,mock_slam}.py` | mission logic / GNSS / ArUco helpers (Nav2 *clients*: `FollowWaypoints`, `NavigateToPose` in frame `map`) | out of scope for hal_navigation (mission layer); `mock_slam.py` idea reused for tests (§6) |
| `nav2_tilt_costmap_plugin/` | C++ costmap layer `nav2_tilt_costmap_plugin::TiltLayer` (slope + step cost from a PointCloud2). Used by **both** costmaps in every config | **not ported now** (decision §8.9); replaced by built-in `ObstacleLayer`. Compile fix for later in §3.2 |
| `meshes_gazebo/` | 300 MB of meshes + Gazebo models; the rover URDF uses 7 `.dae` (91 MB total: kadlub 39 MB, 2x zawieszenie_dol 11.6 MB, zawieszenie_lewo/prawo 7.9/7.7 MB, kolo_lewe/prawe 7 MB) + `ZED2.STL` (49 kB) | 7 `.dae` into `hal_description/meshes/` (size decision §8); ZED mesh replaced by the ZED wrapper's own URDF macro |
| `hal_simulation/`, `plugins.xacro` Gazebo sensors | Ignition Gazebo simulation | **not portable now**: the image has no `ros_gz` / `gz` (only `gz_*_vendor` cmake stubs). Keep the `simulation` xacro arg so the URDF still generates, but no sim launch |
| `hal_communication/wheel_mux.cpp` | consumes `/cmd_vel` as `geometry_msgs/Twist` and republishes `/mux/cmd_vel` | not ported here, but it fixes the cmd_vel **type** Nav2 must publish (§3.1) |

## 2. What the image has / lacks (checked 2026-11-11)

| Item | State |
|---|---|
| Nav2 | **not installed**. Only `nav2_costmap_2d`, `nav2_msgs`, `nav2_util`, `nav2_ros_common`, `nav2_voxel_grid` **1.5.0** are present (pulled in by `nvblox_nav2`). No controller/planner/bt_navigator/map_server/lifecycle_manager |
| Nav2 in apt (Isaac ROS repo) | `ros-lyrical-navigation2` 1.5.0 (metapackage: all servers, smac, dwb, mppi, rpp, route, docking, rviz plugins, simple_commander), `ros-lyrical-nav2-bringup` 1.5.0 (depends on `ros-gz-sim`, `ros-gz-bridge`, `slam-toolbox`, `nav2-minimal-tb3/tb4-sim`, `diff-drive-controller` – heavy, all unused by us) |
| `nvblox_nav2` | installed (`nvblox::nav2::NvbloxCostmapLayer`, consumes `/nvblox_node/static_map_slice`); usable as an alternative costmap source later (ESDF publishing is off in `nvblox_base.yaml`, `update_esdf_rate_hz: 0.0`) |
| URDF tooling | `xacro` 2.1.1 (CLI + python module), `robot_state_publisher` 3.5.5, `urdf`, `tf2_tools`, `rviz2` – all present. No `joint_state_publisher` (not needed: on the real rover every joint is `fixed`) |
| ZED URDF | `zed_wrapper/urdf/zed_macro.urdf.xacro`: macro `zed_camera(name, model, …)` -> `<name>_camera_link` (root, mount point) -> `<name>_camera_center` -> `<name>_{left,right}_camera_frame` -> `…_frame_optical`, plus `mag_link`, `baro_link`, `temp_*`. `zed_camera.launch.py` runs its own `robot_state_publisher` from `zed_descr.urdf.xacro` unless `publish_urdf:=false` |
| Gazebo | none (`gz`/`ign` binaries absent, no `ros_gz_*`) |
| Test tooling | `ament_cmake_pytest`, `ament_cmake_gtest`, `launch_testing(_ros, _ament_cmake)`, `ament_lint_*`, pytest 7.4.4 – all present |
| scipy | in `/opt/venv` (needed by `planar_tf_broadcaster.py`) |

## 3. Breaking changes found (old stack = Nav2 Humble-era; target = Nav2 1.5.0)

### 3.1 Nav2 parameter / launch changes (from the 1.5.0 debs, `nav2_bringup/params/nav2_params.yaml` + headers)

| Topic | Old (`nav2_params_erc.yaml`) | Nav2 1.5.0 | Action |
|---|---|---|---|
| cmd_vel type | implicit `Twist` | `enable_stamped_cmd_vel` **defaults to `true`** (TwistStamped) in `controller_server`, `velocity_smoother`, `behavior_server` (spin/backup/drive_on_heading/wait/assisted_teleop), `collision_monitor` | set `enable_stamped_cmd_vel: false` in each of those nodes – `wheel_mux` subscribes `geometry_msgs/Twist` on `/cmd_vel` |
| behavior_server frames | `global_frame: odom_2d`, `robot_base_frame: rover_2d` | behaviours use `local_frame` (default `odom`) and `global_frame` (default `map`) | add `local_frame: odom_2d`, `global_frame: map_2d` |
| behavior plugin names | `nav2_behaviors/Spin` etc. | pluginlib names are `nav2_behaviors::Spin`, `::BackUp`, `::DriveOnHeading`, `::Wait` | rename to `::` |
| planner plugin name | `nav2_smac_planner/SmacPlannerHybrid` | `nav2_smac_planner::SmacPlannerHybrid` | rename to `::` |
| behavior_server topics | `costmap_topic`, `footprint_topic` | `local_costmap_topic`, `global_costmap_topic`, `local_footprint_topic`, `global_footprint_topic` | rename |
| bt_navigator | 50-entry `plugin_lib_names` list of Humble names | built-in BT plugins are loaded automatically; `plugin_lib_names` is only for custom plugins; new required-ish keys `navigators`, `error_code_name_prefixes` (defaults exist) | delete the list; keep `global_frame: map_2d`, `robot_base_frame: rover_2d`, `odom_topic` |
| `*_rclcpp_node` / `*_client` sections | `global_costmap_client`, `global_costmap_rclcpp_node`, `bt_navigator_navigate_*_rclcpp_node` | those helper nodes no longer exist | delete (harmless but noise) |
| costmap params `lethal_cost_threshold`, `trinary_costmap`, `map_subscribe_transient_local`, `footprint_padding`, `track_unknown_space`, `always_send_full_costmap`, `transform_tolerance` | used | all still declared in `libnav2_costmap_2d_core` / layers 1.5 | keep |
| `controller_server.use_realtime_priority`, `SimpleProgressChecker`, `SimpleGoalChecker`, `dwb_core::DWBLocalPlanner` + critics incl. `PreferForward`, `nav2_smoother::SimpleSmoother`, `nav2_waypoint_follower::InputAtWaypoint`, `nav2_costmap_2d::{StaticLayer,InflationLayer}` | used | unchanged names | keep |
| `nav2_bringup/navigation_launch.py` | started: controller, smoother, planner, behaviors, bt_navigator, waypoint_follower, velocity_smoother | 1.5 additionally starts **`route_server`, `collision_monitor`, `docking_server`, `following_server`** as lifecycle nodes (plus `use_keepout_zones`/`use_speed_zones` defaults `True` -> costmap filter servers expected). Without their params the lifecycle manager fails to bring the stack up | **do not include `navigation_launch.py`**; `hal_navigation/launch/navigation.launch.py` starts exactly our node set with one `nav2_lifecycle_manager` whose `node_names` includes `map_server` (removes the old `ros2 lifecycle set` TimerAction hack) |
| point-cloud topic (old `tilt_layer.topic`, now `ObstacleLayer` source) | `/zed2/zed_node/point_cloud/cloud_registered` | ZED 5.2: `/zed/zed_node/point_cloud/cloud_registered` (`depth.publish_point_cloud: true`, `point_cloud_freq: 10`, published only with a subscriber, requires depth != NONE -> `zed2i_override.yaml` already has NEURAL_LIGHT) | rename; expose as launch arg `cloud_topic` so `/nvblox/mesh_pointcloud` (hal_vslam, frame `odom`, ~5 Hz) can be selected |
| odometry topic | `/visual_slam/tracking/odometry` (bt_navigator, velocity_smoother) | unchanged in cuVSLAM 5.0; `odom_manager` republishes it on `/odom_manager/odometry` with a `/odom_manager/switch` | keep `/visual_slam/tracking/odometry` (minimal change); switching to `/odom_manager/odometry` is a one-line follow-up |

### 3.2 (deferred, see §8.9) `nav2_tilt_costmap_plugin` does not compile against nav2_costmap_2d 1.5 (verified with `colcon build` in the image)

```
tilt_layer.cpp:95: error: no matching function for call to
  nav2::LifecycleNode::create_subscription<PointCloud2>(std::string&, rclcpp::SensorDataQoS, <callback>)
```

- `nav2_costmap_2d::Layer::node_` is now `nav2::LifecycleNode::WeakPtr` (from `nav2_ros_common`), whose
  `create_subscription(topic, callback, qos)` has **callback before QoS**. Fix: swap the two arguments (one line).
- `CMakeLists.txt`: `cxx_std_14` -> 17 (rclcpp Lyrical), drop `find_package(nvblox_msgs)` + `nvblox_msgs` in
  `ament_target_dependencies` (never included in the code), add `nav_msgs` (GridCells debug topic) and
  `tf2_sensor_msgs`/`tf2_geometry_msgs` to `package.xml`.
- Everything else (`layered_costmap_->isRolling()`, `useExtraBounds`, `tf2::doTransform` on PointCloud2,
  `pluginlib` export via `tilt_layer.xml`) is unchanged API.

### 3.3 Description / TF changes forced by ZED wrapper 5.2 (see also `VSLAM_PORT_PLAN.md` §3)

| Old | New |
|---|---|
| link `camera_frame` + static TF `camera_frame -> zed2_camera_link` (identity), `camera_frame` also used as IMU frame | ZED macro root is `zed_camera_link`; IMU frame is `zed_imu_link`, published dynamically by the wrapper (`publish_imu_tf:=true`, `zed_left_camera_frame -> zed_imu_link`) |
| sim-only links `zed2_left/right_camera_optical_frame` | `zed_left/right_camera_frame_optical` come from the ZED macro |
| two URDFs (rover + ZED wrapper's own) | one `robot_description`: `hal_062.xacro` includes `$(find zed_wrapper)/urdf/zed_macro.urdf.xacro` and instantiates `<xacro:zed_camera name="$(arg camera_name)" model="$(arg camera_model)">`; ZED launched with `publish_urdf:=false` |
| `visual_slam.launch.py` publishes static `base_footprint -> zed_camera_link` as a stop-gap | `publish_base_to_camera_tf:=false` once `hal_description` runs |
| camera mount pose: `chassis -> camera_frame` at xyz `0.342 0 0.1` (chassis is 0.3564 above `base_footprint`) | reuse as `chassis -> zed_camera_link` (mount-screw point in Stereolabs' convention; `zed_camera_center` is +0.015 m above it for zed2i). **Must be re-measured on the rover** – old value was an approximation and the ZED 2 -> ZED 2i bracket may differ |

## 4. Target package layout

```
workspace/
  hal_description/                      (exists, empty skeleton, ament_cmake)
    package.xml, CMakeLists.txt          install(DIRECTORY urdf meshes launch rviz)
    urdf/hal_062.urdf.xacro              args: simulation (default false), camera_name (zed), camera_model (zed2i)
    urdf/suspension.xacro, wheels.xacro  $(find hal_main) -> $(find hal_description); file://$(find meshes_gazebo) -> package://hal_description/meshes
    urdf/camera.xacro                    ZED macro include + fixed joint chassis -> ${camera_name}_camera_link
    urdf/gazebo_plugins.xacro            old plugins.xacro, included only when simulation=true (unused now)
    meshes/*.dae (7 files)
    launch/description.launch.py         robot_state_publisher(robot_description = xacro(...)), args above + use_sim_time
    rviz/description.rviz                RobotModel + TF only (for the description test)
    test/test_urdf.py, test/test_description_launch.py
  hal_navigation/                        (new, ament_cmake, launch+config+maps+rviz+1 script)
    config/nav2_params.yaml              = old nav2_params_erc.yaml migrated per §3.1
    config/nav2_params_spd.yaml          same migration applied to the speed variant
    maps/costmap_{blank,blank_large,erc,mel}.yaml + costmap_{blank,erc_26,mel}.png
    launch/navigation.launch.py          map_server + 7 Nav2 servers + lifecycle_manager + planar_tf_broadcaster; args:
                                         params_file, location (blank|blank_large|erc|mel), use_sim_time, autostart,
                                         cloud_topic, odom_topic, rviz
    launch/autonomy.launch.py            description.launch.py + hal_vslam bringup.launch.py(publish_urdf:=false,
                                         publish_base_to_camera_tf:=false) + navigation.launch.py (delayed until
                                         /visual_slam/tracking/odometry is alive instead of fixed 13 s timers)
    rviz/navigation.rviz
    scripts/planar_tf_broadcaster.py
    test/test_params.py, test/test_navigation_launch.py
```

Dependencies to declare: `hal_navigation` -> `nav2_bringup`? **no**; `nav2_map_server`, `nav2_controller`, `nav2_planner`,
`nav2_smoother`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_waypoint_follower`, `nav2_velocity_smoother`,
`nav2_lifecycle_manager`, `nav2_smac_planner`, `dwb_core`/`dwb_critics`, `nav2_costmap_2d`, `tf2_ros`,
`python3-scipy`, `hal_description` (exec), `hal_vslam` (exec, autonomy launch). `hal_description` -> `robot_state_publisher`,
`xacro`, `zed_wrapper` (urdf macro), `launch_ros`, `urdf`.

## 5. Docker changes required (need approval before touching the Dockerfile – rebuild of a 68 GB image)

Add one apt layer after the nvblox layer:

```
ros-lyrical-navigation2
```

That is the whole Nav2 stack (incl. `nav2-rviz-plugins`, `nav2-simple-commander`). **Not** `ros-lyrical-nav2-bringup`
(pulls Gazebo sim + slam_toolbox, and we no longer include its launch). Nothing else is missing: xacro,
robot_state_publisher, scipy, tf2_tools are already in the image.

For development before the rebuild: `apt-get install ros-lyrical-navigation2` inside the running container (lost on
`--rm`), or build/test in a `--rm` container that installs it at start (what the tests in §6 do).

## 6. Tests

### T0 – build / static (no hardware, `--rm` container)
1. `colcon build --packages-select hal_interfaces hal_description hal_navigation hal_vslam` – zero warnings from our CMake.
2. `xacro hal_062.urdf.xacro simulation:=false camera_name:=zed camera_model:=zed2i | check_urdf` – valid tree, single root `base_footprint`.
3. `ros2 launch hal_navigation navigation.launch.py --show-args` / `-p` (print) for every launch file – imports and substitutions resolve.
4. `ament_flake8`/`ament_pep257` on the python scripts (same style as hal_vslam/test).

### T1 – `hal_description` (pytest + launch_testing)
- `test_urdf.py`: process xacro for `simulation` false/true; assert link set == expected (`base_footprint`, `chassis`,
  4 suspension links, 6 wheels, `zed_camera_link`, `zed_camera_center`, `zed_left/right_camera_frame(_optical)`), no
  `camera_frame`, no `zed2_*`, every `package://hal_description/meshes/*.dae` resolves to an existing file, all joints
  `fixed` when simulation=false.
- `test_description_launch.py`: start `description.launch.py`, with a `tf2_ros.Buffer` assert `base_footprint -> zed_left_camera_frame_optical`
  and `base_footprint -> wheel_link_RF` are available within 5 s, and that `/robot_description` is latched (transient_local).

### T2 – (deferred with the plugin) `nav2_tilt_costmap_plugin` unit test (pytest, no camera)
- Launch a bare `nav2_costmap_2d` node (`costmap_2d_cloud`? no – `nav2_costmap_2d/nav2_costmap_2d` executable) with
  `plugins: [tilt_layer]`, static TFs `odom_2d -> rover_2d`, `rolling_window: true`, 0.1 m resolution.
- Publish a synthetic `PointCloud2` in frame `odom_2d`: flat ground z=0 for x in [0,4], a 0.3 m step at x≈2, a 40° ramp at x≈3.
- Assert on `/costmap/costmap`: cells on flat ground == 0, cells at the step >= 200, ramp cells in (0, 254), and
  `/costmap/tilt_layer_debug` GridCells non-empty. This pins the slope/step semantics (`alpha_min/max`, `min/max_height`, `dh_radius`).

### T3 – `hal_navigation` without hardware (pytest + launch_testing, the key regression test)
Fixtures (test-only, live in `hal_navigation/test/`):
- `fake_localization.py`: publishes TF `map -> odom -> base_footprint` (identity, 30 Hz) and `nav_msgs/Odometry` on
  `/visual_slam/tracking/odometry`; integrates received `/cmd_vel` so the "robot" actually moves (replaces the old
  `fake_odom.py`/`mock_slam.py` idea).
- `fake_cloud.py`: flat ground `PointCloud2` + one 0.5 m box on the configured `cloud_topic` in frame `zed_left_camera_frame_optical`
  -> requires `description.launch.py` in the test (exercises the real TF chain).
Checks:
1. `test_params.py` (pure python, fast): every plugin name uses `::`; `enable_stamped_cmd_vel: false` present in
   controller_server, velocity_smoother, behavior_server; frames consistent (`map_2d`/`odom_2d`/`rover_2d`) across
   costmaps, bt_navigator, behavior_server; `odom_topic` identical in bt_navigator and velocity_smoother; every map yaml's
   `image` exists; no `plugin_lib_names`, no `*_rclcpp_node` sections. Same for `nav2_params_spd.yaml`.
2. `test_navigation_launch.py`: `navigation.launch.py location:=blank` + fixtures. Assert within 60 s:
   `lifecycle_manager_navigation/is_active` returns true; `ros2 lifecycle get` == `active` for map_server, controller_server,
   planner_server, smoother_server, behavior_server, bt_navigator, waypoint_follower, velocity_smoother; TF `map_2d`, `odom_2d`,
   `rover_2d` exist (planar_tf_broadcaster); `/global_costmap/costmap` and `/local_costmap/costmap` publish; the local costmap
   marks the fake box as lethal and the flat ground as free (ObstacleLayer height band 0.2–1.2 m works through the real TF chain).
3. Same test, continued: send `NavigateToPose` (frame `map`, 3 m ahead) -> `/cmd_vel` is received as **`geometry_msgs/Twist`**
   (type check via `ros2 topic type`), `linear.x > 0` within 10 s, `/plan` non-empty, goal reaches SUCCEEDED (fake robot
   integrates cmd_vel). Then `FollowWaypoints` with 2 poses and `input_at_waypoint` -> reaches waypoint 1 and waits for
   `/input_at_waypoint/input` (old ERC behaviour).
4. `autonomy.launch.py --show-args` + a dry run with `hal_vslam` launches disabled (`vslam:=false`) to check composition.

### T4 – on the rover / with the ZED 2i (manual, needs USB 3.0 – see §7)
1. `ros2 launch hal_navigation autonomy.launch.py location:=blank_large` (no wheel_mux -> rover does not move).
2. `ros2 run tf2_tools view_frames`: **one** tree `map -> odom -> base_footprint -> chassis -> … -> zed_camera_link -> … -> zed_imu_link`;
   no duplicate publishers for `zed_camera_link` (would be the case if `publish_urdf` or `publish_base_to_camera_tf` were left on).
3. `ros2 topic hz /zed/zed_node/point_cloud/cloud_registered` ≈ 10 Hz once the local costmap subscribes; a person standing 3 m ahead appears as lethal cells in `/local_costmap/costmap`.
4. rviz `navigation.rviz`: RobotModel aligned with the nvblox mesh (camera height check), costmaps, send a 2D goal, observe `/cmd_vel`.
5. Repeat with `cloud_topic:=/nvblox/mesh_pointcloud` and compare costmaps.
6. Only then enable `wheel_mux` and drive a short goal.

### T5 – replay (after VSLAM Phase 6 produces an SVO/rosbag)
`zed.launch.py svo_path:=…` + full autonomy launch -> T4 checks 2-4 become scriptable and run without the camera.

## 7. Side finding: ZED 2i status on this host (2026-11-11)

- 11:20: camera attached but enumerated as **USB 2.0 (480 Mbps)** behind hub `1-7`; opening it in the container failed
  (`CAMERA STREAM FAILED TO START` x6 -> `Camera detection timeout`). The ZED 2i needs USB 3.0.
- 11:31: after re-plugging into a SuperSpeed port (`2-4.4`, **5000 Mbps**) the camera opens in a `--rm` container:
  `left/gray/rect/image` 1280x720 mono8 at 30.1 Hz, `imu/data` at 100 Hz (HD720@30, depth NONE, no pos. tracking).
  Hardware test T4 is unblocked. Check `cat /sys/bus/usb/devices/*/speed` for the `2b03` device before every hardware session.

## 8. Decisions (taken 2026-11-11)

1. **Dockerfile**: add `ros-lyrical-navigation2` – **approved**. `nav2-bringup` excluded.
2. **Own launch** (not `nav2_bringup/navigation_launch.py`) – **approved**. Reason: the 1.5 bringup launch hard-codes 11 lifecycle
   nodes (adds route_server, collision_monitor, docking_server, following_server) that fail without params, and the deb pulls
   ros_gz_sim + slam_toolbox. Own launch = Node blocks for map_server + 7 servers + one lifecycle manager (~80 lines).
3. **Meshes**: copy the 7 `.dae` (91 MB) into `hal_description/meshes/` – **approved**.
4. **Config variants**: port `nav2_params_erc.yaml` as `nav2_params.yaml` and `nav2_params_spd.yaml`; drop the plain
   `nav2_params.yaml` (earlier iteration of the same tuning, its values are quoted as "old" in the ERC file).
5. **`tf_limiter.py`**: out of `hal_navigation`. Its only consumer is `hal_application/AutonomousNode.cpp` (`/limited_tf`);
   port it with hal_application.
6. **Camera mount offset**: keep `chassis -> zed_camera_link` = `0.342 0 0.1` as default, expose as xacro args
   (`camera_x/y/z/pitch`), verify in T4 with the rover on flat ground: ZED point-cloud ground plane at z≈0 in `base_footprint`
   and not tilting with distance (pitch is the value most likely to be wrong).
7. **No `camera_frame` alias.** ArUco poses from the left image belong in `zed_left_camera_frame_optical`; when hal_aruco is
   ported change `aruco_obstacle.py` FRAME_ID and `global_pos_from_aruco.py` lookup (2 strings). An alias would freeze the
   old 6 cm / axis-convention error.
8. **Odometry topic**: keep `/visual_slam/tracking/odometry` (30 Hz raw). Nav2 only reads it for velocity (bt_navigator stuck
   detection; velocity_smoother is OPEN_LOOP). `/odom_manager/odometry` is a 10 Hz held copy meant for the reset/re-seed switch.
9. **No custom costmap plugins in this port** (user, 2026-11-11): `nav2_tilt_costmap_plugin` is **not** ported now. Base Nav2 only:
   - global costmap: `static_layer` + `obstacle_layer` + `inflation_layer` (first version had no obstacle layer here and the
     planner routed straight through live obstacles - seen in rviz with the fake box; fixed 2026-11-11)
   - local costmap: built-in `nav2_costmap_2d::ObstacleLayer` on the ZED point cloud (the commented-out `mesh_layer` block of the
     old config: `/zed/zed_node/point_cloud/cloud_registered`, `min_obstacle_height: 0.2`, `max_obstacle_height: 1.2`, marking +
     clearing, raytrace/obstacle range 1000 m is excessive for a 16x16 m rolling window -> use 10 m) + `inflation_layer`
   - `planar_tf_broadcaster.py` stays: obstacle heights are relative to the costmap frame's z, so `odom_2d` (z = footprint z)
     is what makes 0.2–1.2 m mean "above the ground under the rover"; `hal_automata` also needs `map_2d`/`rover_2d`.
   - §3.2 (tilt plugin compile fix) and test T2 are kept as a later phase.

## 9. Order of work / progress

Progress 2026-11-11:
- [x] 1. `hal_description` written (urdf with ZED macro, 7 meshes, `description.launch.py`, rviz) + hal_vslam `zed.launch.py`
      got a `publish_urdf` arg. **T0/T1 green in the image: 23 tests (8 URDF structure, 3 launch/TF, lint), 0 failures.**
      Note: apt `zed_msgs` in the image ships no meshes -> the ZED's own visual is missing in rviz (TF unaffected).
- [x] 2. `hal_navigation` written: `config/nav2_params.yaml` (+`_spd`) migrated per §3.1/§8.9 by a reproducible script,
      `launch/navigation.launch.py` (own node list + lifecycle manager + yaml rewrite instead of nav2_common),
      `launch/autonomy.launch.py`, maps, rviz, `scripts/planar_tf_broadcaster.py`, `test/test_params.py`,
      `test/test_navigation_launch.py` with fixtures `fake_localization.py` / `fake_cloud.py`.
      **T3 green in a `--rm --cap-add SYS_NICE` container with `ros-lyrical-navigation2` installed: 43 tests, 0 failures**
      (lint, 22 params checks, launch: all 8 lifecycle nodes active, map_2d/odom_2d/rover_2d, global costmap_raw 1024x1024,
      fake box marked lethal in the local costmap, NavigateToPose 2 m -> geometry_msgs/Twist on /cmd_vel, goal SUCCEEDED,
      fake robot within 0.6 m of the goal).
- [x] 3. Dockerfile: `ros-lyrical-navigation2` layer added (after the pip layer for cache reuse) + root `.dockerignore`
      (context was 420 MB of ros2_docker_ws + .git); **image rebuilt 2026-11-11 (hal062_autonomy:latest 82d5eb924ccc, 68.7 GB)**.
- [x] 4. `README` section added.
- [x] 5. T4 bench runs with the real ZED 2i (camera on the desk, no rover) via `autonomy.launch.py nvblox:=false`:
      **one TF tree** rooted at `map` (23 frames: map -> odom -> base_footprint -> chassis -> zed_camera_link -> zed_camera_center
      -> zed_left_camera_frame -> {zed_left_camera_frame_optical, zed_imu_link}, wheels, rover_2d/odom_2d/map_2d), no legacy frames;
      cuVSLAM odometry 28-30 Hz, ZED cloud 10 Hz, local costmap 3 Hz with the desk marked lethal, all Nav2 nodes active,
      NavigateToPose accepted. Found and fixed: (a) `blank_large` map out of bounds (above), (b) `anti_teleport_node.py`
      crashed on `get_logger().warn()` - rclpy Lyrical removed the deprecated alias -> `warning()`. Second run: cuVSLAM lost
      tracking on the static, textureless desk scene (odometry jumped 147 m; ZED logged "Self-calibration was skipped due to
      low texture"); the watchdog detected it and died on the logger bug. Third run (after the fix): watchdog alive, 194 jump
      warnings, 15 successful `set_slam_pose` corrections, odometry stayed near the origin; NavigateToPose accepted, controller
      produced /cmd_vel Twist at ~27 Hz until "Failed to make progress" (camera cannot move - expected). The drive test needs
      the rover (T4 steps 4-6 remain: camera mount check on flat ground, nvblox cloud variant, wheel_mux on).

Findings from T3 debugging (2026-11-11):
- `controller_server.use_realtime_priority: true` (old config, kept) calls `sched_setscheduler(SCHED_FIFO)` in the
  FollowPath worker thread. Without `CAP_SYS_NICE` (plain `docker run`, no `--privileged`) it throws before any log
  line: FollowPath is accepted, never executes, never returns, later goals land in the "pending slot" and the BT waits
  forever. `launch_docker.sh` runs `--privileged`, so the rover container is fine; **tests must run with
  `--cap-add SYS_NICE` or `--privileged`**.
- `controller_server` had no `odom_topic` in the old config -> default `odom`, which nothing publishes -> DWB saw zero
  velocity. Now `/visual_slam/tracking/odometry` like bt_navigator / velocity_smoother (deliberate deviation).
- `global_costmap.always_send_full_costmap: False` (old) -> the OccupancyGrid goes out only on size changes; rviz and the
  test read `/global_costmap/costmap_raw`, which is published whenever subscribed.
- `maps/costmap_blank_large.yaml` was broken in the old stack: it pointed at the 1024-cell `costmap_blank.png` with origin
  -1000 m, so the map covered [-1000, -795] m and the robot at (0,0) was out of bounds (planner error 203 on the bench).
  The old 10000x10000 `costmap_blank_large.png` (100 MB OccupancyGrid, 100M-cell costmap) was not used; `blank_large` is now a
  generated 2048x2048 blank image (409.6 m x 409.6 m, origin -204.8). `test_params.py` asserts (0,0) lies inside every map.
- Planner (SmacPlannerHybrid on the 1024x1024 blank map): 11 ms per path. Large /map messages are delivered fine with
  rmw_fastrtps (checked 1 MB and 2 MB maps).


1. `hal_description` (urdf with ZED macro, meshes, launch) + hal_vslam args `publish_urdf:=false`, `publish_base_to_camera_tf:=false`; T1.
2. `hal_navigation`: config migration (§3.1 + §8.9) + `navigation.launch.py` + `planar_tf_broadcaster.py`; T3 in a `--rm`
   container with `apt-get install ros-lyrical-navigation2` at start.
3. Dockerfile layer `ros-lyrical-navigation2` + rebuild; rerun T0, T1, T3 in the new image.
4. `autonomy.launch.py`, rviz config, README section; T4 (ZED is on USB 3.0 again).
5. Later phase: `nav2_tilt_costmap_plugin` port (§3.2) + T2, then switch the costmaps back to `tilt_layer`.
