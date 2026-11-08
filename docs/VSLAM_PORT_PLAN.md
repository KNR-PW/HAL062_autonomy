# Plan portu `hal_vslam` na Isaac ROS 5.0 / ROS 2 Lyrical

Stan na 2026-10-27. Wszystkie fakty poniżej zostały sprawdzone na tej maszynie i w obrazie `hal062_autonomy:latest`
(kontenery testowe `--rm`, obraz nie był modyfikowany).

---

## 1. Co jest na hoście

| Element | Stan |
|---|---|
| OS / kernel | Ubuntu 24.04.5, kernel 6.8.0-147, x86_64 |
| GPU | RTX 4070 Ti 12 GB, sterownik 595.99.02 (CUDA 13.2) |
| Docker | 29.9.2, nvidia-container-toolkit 1.20.1, runtime `nvidia` + CDI |
| ROS na hoście | brak (`/opt/ros` nie istnieje), jest tylko `colcon` i `isaac-ros` CLI |
| `~/workspaces/isaac_ros-dev` | pusty `src/`, konfiguracja CLI z image keys `zed` + `cuvslam` |
| ZED SDK na hoście | zainstalowany (ZED Studio) — **ZED Studio blokuje `/dev/video2`, musi być zamknięte przed startem kontenera** |
| Kamera | **ZED 2i** (S/N 31610881, FW 1523) na USB 3.0 (5000 Mbps), udev `99-slabs.rules` OK |

## 2. Co jest w obrazie `hal062_autonomy:latest`

Baza: `cached_isaac_run_dev_image_local` = obraz Isaac ROS CLI z kluczami `zed` + `cuvslam` (release-5.0).

| Element | Stan |
|---|---|
| OS / ROS | Ubuntu 24.04.3, **ROS 2 Lyrical**, CUDA 13.2 |
| ZED SDK | **5.2.3** w `/usr/local/zed` |
| zed-ros2-wrapper | **v5.2.0**, zbudowany ze źródeł w `/opt/ros_ws` (`ISAAC_ROS_WS=/opt/ros_ws`), sourcowany z `/etc/bash.bashrc` |
| Isaac ROS (apt, 5.0.0) | `isaac_ros_cuvslam`, `isaac_ros_visual_slam_interfaces`, `isaac_ros_image_proc`, `isaac_ros_examples`, `isaac_ros_common`, dnn-image-encoder, tensor-rt, triton, unet |
| ROS (apt) | `robot_localization`, `rviz2`, `tf2_ros`, `cv_bridge`, `rclcpp_components`, `lifecycle_msgs`, ~250 pakietów ros-lyrical |
| Python | venv `/opt/venv` (numpy 2.5.4, opencv 5.0, ultralytics) — **brak `scipy`**, `tf_transformations` nie importuje się (transforms3d vs numpy 2) |
| **Brak** (stan przed Fazą 4) | nvblox, `isaac_ros_launch_utils`, `nvblox_ros_python_utils`, `isaac_ros_zed`, `foxglove_bridge` — **dodane do Dockerfile w Fazie 4** |
| Dostępne w apt (repo release-5.0) | `ros-lyrical-isaac-ros-nvblox` (meta), `nvblox-ros`, `nvblox-msgs`, `nvblox-ros-python-utils`, `nvblox-rviz-plugin`, `nvblox-nav2`, `nvblox-examples-bringup`, `isaac-ros-launch-utils`, `isaac-ros-zed`, `foxglove-bridge` |

Montowanie (`docker/launch_docker.sh`): `./workspace` → `/root/workspace/src`. Katalog `ros2_docker_ws/` (gdzie leży `hal_vslam`)
**nie jest montowany** do kontenera.

## 3. Wynik smoke-testu ZED 2i w kontenerze (zed_camera.launch.py, camera_model:=zed2i)

Kamera otwiera się poprawnie. Zmierzone:

- obraz `left/color/rect/image`: `bgra8`, **960x540**, ~15 Hz (zed2i.yaml: grab HD1080@15, `pub_downscale_factor: 2`)
- obraz `left/gray/rect/image`: `mono8` (gotowy input dla cuVSLAM, bez konwersji)
- IMU `imu/data`: ~100 Hz, `frame_id: zed_imu_link`
- QoS obrazów: RELIABLE / VOLATILE
- depth (`NEURAL_LIGHT`): przy **pierwszym** starcie ~2 min (download + optymalizacja modelu do `/usr/local/zed/resources`).
  W kontenerze `--rm` dzieje się to **za każdym razem** → trzeba wolumen na `/usr/local/zed/resources`.
- topiki `left/*` i `right/*` są publikowane **tylko** gdy `video.publish_left_right: true` (domyślnie false w 5.2).

Nazwy topików (namespace `/zed`, node `zed_node`, `camera_name` domyślnie `zed`):

| Stary kod (wrapper 4.x, ZED2) | Wrapper 5.2 (ZED 2i) |
|---|---|
| `/zed2/zed_node/left/image_rect_color` | `/zed/zed_node/left/color/rect/image` (bgra8) |
| — | `/zed/zed_node/left/gray/rect/image` (mono8) |
| `/zed2/zed_node/left/camera_info` | `/zed/zed_node/left/color/rect/camera_info` (lub `gray/rect/camera_info`) |
| `/zed2/zed_node/right/...` | analogicznie `right/...` |
| `/zed2/zed_node/imu/data` | `/zed/zed_node/imu/data` |
| `/zed2/zed_node/imu/mag` | **nie widać w domyślnej liście** — do sprawdzenia (param `sensors.*`) |
| `/zed2/zed_node/depth/depth_registered` | `/zed/zed_node/depth/depth_registered` |
| `/zed2/zed_node/depth/camera_info` | `/zed/zed_node/depth/camera_info` |

Nazwy frame'ów (konwencja 5.x: `..._frame_optical`, nie `..._optical_frame`):

| Stary kod | Wrapper 5.2 |
|---|---|
| `zed2_left_camera_optical_frame` | `zed_left_camera_frame_optical` |
| `zed2_right_camera_optical_frame` | `zed_right_camera_frame_optical` |
| `camera_frame` (jako IMU) | `zed_imu_link` |
| `zed2_camera_link` | `zed_camera_link` (root URDF), `zed_camera_center` |

## 4. Zmiany API cuVSLAM 3.1 → 5.0

Plugin bez zmian: `nvidia::isaac_ros::visual_slam::VisualSlamNode`, ale pakiet to teraz **`isaac_ros_cuvslam`**.

| Stary parametr | Nowy |
|---|---|
| `enable_imu_fusion: True` | `tracking_mode: 1` (0 = multicamera stereo, 1 = VIO z IMU, 2 = RGBD) |
| `input_imu_frame` | `imu_frame` |
| `publish_odom_to_rig_tf` | `publish_odom_to_base_tf` |
| obrazy rgb8 | **obrazy grayscale (mono8)** — konwertery rgb8 do usunięcia |
| `image_jitter_threshold_ms: 67` | zostaje (15 Hz = 66.7 ms), domyślnie 34 ms |
| `calibration_frequency: 200` | IMU ZED leci ~100 Hz; ustawić `sensors.sensors_pub_rate: 200` w ZED albo 100 tutaj |

Topiki/serwisy cuVSLAM (`visual_slam/image_N`, `camera_info_N`, `imu`, `tracking/odometry`, `reset`, `set_slam_pose`,
`get_all_poses`, `save_map`, `load_map`, `localize_in_map`) — bez zmian względem starego kodu.

Referencje w obrazie: `/opt/ros/lyrical/share/isaac_ros_cuvslam/launch/isaac_ros_visual_slam_core.launch.py`,
`/opt/ros/lyrical/share/isaac_ros_cuvslam/rviz/`, a po instalacji `isaac_ros_zed`:
`/opt/ros/lyrical/share/isaac_ros_zed/launch/isaac_ros_zed_stereo_rect_core.launch.py` (fragment NVIDIA dla ZED).

## 5. Problemy w samym `hal_vslam` (niezależne od portu)

- `package.xml`: literówka `sensor_msg`; brak `nav_msgs`, `std_srvs`, `tf2_ros`, `isaac_ros_visual_slam_interfaces`,
  `nvblox_msgs` jest, ale brak `isaac_ros_launch_utils`, `nvblox_ros_python_utils`, `launch_ros`, `python3-numpy`.
- `CMakeLists.txt`: `src/odom_manager.cpp` **w ogóle nie jest budowany**; `include_directories(include)` bez katalogu;
  `cxx_std_14` (rclcpp Lyrical wymaga C++17).
- `anti_teleport_node.py`: import `scipy` (brak w venv) i `hal_interfaces.msg.ArucoPose` (pakietu nie ma w tym repo).
- Śmieci: `src/.anti_teleport_node.py.swp`, `hal_vslam/__init__.py` + `resource/` (pozostałość po ament_python).
- `config/nvblox_*.yaml` pochodzą z nvblox 3.2 — nazwy parametrów trzeba zdiffować z nvblox 5.0.

---

## 6. Plan — fazy (kolejność wg zależności, nie wg ważności)

### Faza 0 — baseline (ZROBIONE 2026-10-27)
- [x] inwentaryzacja hosta i obrazu, ZED 2i działa w kontenerze, spisane topiki/frame'y/encodingi
- [x] potwierdzone: cuvslam 5.0 jest, nvblox nie ma (jest w apt)
- [x] potwierdzone: `odom_manager` i `anti_teleport` da się sportować bez brakujących paczek

### Faza 1 — porządki w pakiecie (ZROBIONE 2026-10-27)
- pakiety przeniesione: `ros2_docker_ws/{hal_vslam,hal_interfaces}` → `workspace/` (katalog montowany do kontenera)
- usunięte: `.anti_teleport_node.py.swp`, `hal_vslam/__init__.py`, `resource/`
- `package.xml`: literówka `sensor_msg`, pełna lista zależności (w tym `hal_interfaces`, `python3-scipy`, `zed_wrapper`)
- `CMakeLists.txt`: C++17, target `odom_manager`, `mesh_to_pointcloud` budowany warunkowo (`find_package(nvblox_msgs QUIET)`)
  dopóki nvblox nie trafi do obrazu
- `hal_interfaces` buduje się na Lyrical bez zmian

### Faza 2 — port `odom_manager` + `anti_teleport_node` (ZROBIONE 2026-10-27)
- `odom_manager.cpp`: jedyna zmiana wymuszona przez rclcpp 32 (Lyrical): `create_client(..., rmw_qos_profile_services_default, cb_group)`
  → `create_client(..., rclcpp::ServicesQoS(), cb_group)` (stare przeciążenie z `rmw_qos_profile_t` usunięte)
- `anti_teleport_node.py`: tylko nazwy topików ZED (`/zed/zed_node/imu/data`, `/zed/zed_node/imu/mag`); scipy i import
  `hal_interfaces.msg.ArucoPose` bez zmian (scipy: tymczasowo `pip install scipy` w kontenerze, na stałe w Dockerfile — Faza 4)
- zweryfikowane z kamerą: lifecycle `active`, watchdog woła `/visual_slam/set_slam_pose` (korekta orientacji po starcie i po
  resecie SLAM — zachowanie jak w starym kodzie), `odom_manager` 10 Hz passthrough, `/odom_manager/switch false/true` = reset + set_pose OK

### Faza 3 — port launcha VSLAM (ZROBIONE 2026-10-27, bez nvblox)
- `config/zed2i_override.yaml`: HD720@30 (NATIVE, jak stare 1280x720), `publish_left_right`+`publish_gray`, depth NONE,
  ZED pos. tracking OFF, `sensors_pub_rate: 200`, `publish_mag: true`
- `launch/zed.launch.py`: include `zed_camera.launch.py` (zed2i, `publish_tf:=false`, `publish_map_tf:=false`,
  **`publish_imu_tf:=true`** — mimo opisu w launchu wrappera działa przy `publish_tf:=false` i daje tylko
  `zed_left_camera_frame → zed_imu_link`, którego cuVSLAM wymaga dla `imu_frame`; `publish_tf:=true` wprowadzałoby `odom → zed_camera_link` od ZED)
- `launch/visual_slam.launch.py`: `isaac_ros_cuvslam`, `tracking_mode: 1`, `imu_frame`, `publish_odom_to_base_tf`,
  remapy na topiki gray, nowe frame'y, bez konwerterów, static TF `base_footprint → zed_camera_link`, argumenty
  `enable_anti_teleport`, `enable_odom_manager`, `standalone`
- **cuVSLAM domyślnie ładowany do kontenera ZED (`/zed/zed_container`) z IPC**: pomiar na RTX 4070 Ti, 1280x720@30 —
  osobny kontener: ~170 ostrzeżeń jitter / 40 s, odometria ~23 Hz (DDS gubi klatki, rmw_fastrtps, host `rmem_max`=1 MB);
  wspólny kontener: ~4–8 ostrzeżeń, odometria 30 Hz. `standalone:=true` przywraca osobny kontener.
- wynik testu: `vo_state: 1`, `/visual_slam/tracking/odometry` 30 Hz (`odom → base_footprint`), TF `map → base_footprint` OK
- uruchomienie (w kontenerze, ZED Studio na hoście zamknięte):
  ```bash
  cd /root/workspace && colcon build --packages-select hal_interfaces hal_vslam && source install/setup.bash
  ros2 launch hal_vslam zed.launch.py
  ros2 launch hal_vslam visual_slam.launch.py
  ros2 topic hz /visual_slam/tracking/odometry ; ros2 topic echo /visual_slam/status
  ```
- do zrobienia później: porównać `tracking_mode` 1 (VIO) vs 0 (stereo) pod kątem driftu (wymaga przejazdu)

### Faza 4 — Dockerfile / launch_docker.sh (ZROBIONE 2026-10-27, obraz przebudowany)
- `docker/Dockerfile`: nowa warstwa apt `ros-lyrical-nvblox-ros nvblox-msgs nvblox-ros-python-utils nvblox-rviz-plugin nvblox-nav2
  isaac-ros-launch-utils isaac-ros-zed foxglove-bridge` (bez metapakietu `isaac-ros-nvblox` i `nvblox-examples-bringup`,
  które ciągną detectnet/peoplenet/Triton); `scipy` dopisane do `pip3 install`.
- `docker/launch_docker.sh`: wolumen `hal062_zed_resources:/usr/local/zed/resources` (cache modeli AI ZED).
- `docker/entrypoint.sh`: `source /root/workspace/install/setup.bash` tylko gdy plik istnieje.
- zweryfikowane w nowym obrazie: `mesh_to_pointcloud` buduje się i rejestruje jako komponent, `nvblox_ros_python_utils`
  importuje się (`NvbloxCamera` ma `zed2` i `zedx`, nie ma `zed2i` — do Fazy 5), VSLAM działa jak w Fazie 3.

### Faza 5 — nvblox (ZROBIONE 2026-10-27)
- `config/nvblox_base.yaml`: usunięte 4 parametry, których nvblox 5.0 już nie deklaruje (`lidar_max_valid_range_m`,
  `static_mapper.exclude_last_view_from_decay`, `static_mapper.layer_streamer_exclusion_{height,radius}_m`); pozostałe 97 zgodne.
- `config/nvblox_zed.yaml`: **`input_qos: "SENSOR_DATA"` → `"SYSTEM_DEFAULT"`** — z SENSOR_DATA subskrypcje nvblox 5.0 nie dopasowują się
  do publisherów ZED 5.2 (RELIABLE), licznik subskrybentów 0, zero integracji.
- `launch/nvblox.launch.py`: topiki ZED 5.x, argumenty `camera_name`, `robot_frame` (domyślnie `base_footprint`, nadpisuje
  `pose_frame` i wszystkie `*_attachment_frame_id`, bo `rover_2d` nie istnieje), `enable_mesh_to_pointcloud`. Kontener standalone jak
  w starym launchu; `run_standalone:=False container_name:=/zed/zed_container` też działa (trochę więcej klatek depth).
- `config/zed2i_override.yaml` ma teraz `depth_mode: NEURAL_LIGHT` (nvblox); wariant bez depth: `zed2i_override_nodepth.yaml`
  (`zed.launch.py override_config:=...`). nvblox przyjmuje kolor `bgra8` z ZED bez konwersji, depth `32FC1`.
- wynik (2 niezależne przebiegi): depth callback 13–18 Hz, kolor 3 Hz (limit `integrate_color_rate_hz: 5`), `/nvblox_node/mesh` ~7 Hz,
  `/nvblox/mesh_pointcloud` ~5 Hz (frame `odom`, ~15k pkt), VSLAM 30 Hz, GPU ~10 %.
- UWAGA (stare zachowanie, nie zmieniane): `nvblox_base.yaml` ma `update_esdf_rate_hz: 0.0`, więc `static_map_slice`/ESDF
  nie są publikowane; costmap w starym stacku szedł z `mesh_to_pointcloud`. Jeśli Nav2 ma brać ESDF slice, ustawić np. 5.0.
- Jeden wczesny przebieg testu nie integrował depth mimo poprawnego QoS i nie dał się odtworzyć (4 kolejne przebiegi OK);
  gdyby wróciło: `print_queue_drops_to_console: true` i sprawdzić linię "Subscribing input topics with QoS".
- uruchomienie: `ros2 launch hal_vslam zed.launch.py` → `visual_slam.launch.py` → `nvblox.launch.py`;
  rviz: `/nvblox_node/mesh` (plugin nvblox_rviz_plugin) lub `/nvblox/mesh_pointcloud`.

### Faza 6 — powtarzalne testy i dokumentacja
- nagrać SVO (`/zed/zed_node/start_svo_rec`) albo rosbag (gray L/R + camera_info + imu + tf_static) z przejazdu z pętlą,
  argument `svo_path` w `zed.launch.py` → testy bez kamery.
- sekcja w README: jak odpalić VSLAM i co ma być widoczne.

## 7. Pytania otwarte (do Ciebie)
1. Zgoda na zmiany w Dockerfile/launch_docker.sh z Fazy 1.3–1.4?
2. Skąd ma brać się `base_footprint` / `rover_2d` — `hal_description` jest pusty; na testy wystarczy static TF.
3. `hal_interfaces` (ArucoPose) — jest w innym repo i ma wrócić, czy anti_teleport ma działać bez ArUco?
4. Nazwa kamery: zostawić domyślne `zed` (namespace `/zed/zed_node`) czy ustawić `camera_name:=zed2i` jak w configu NVIDIA?

## 8. Źródła
- isaac_ros_cuvslam 5.0 (parametry, topiki): https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html
- ZED setup Isaac ROS 5.0 (ZED 2i = SQA tested): https://nvidia-isaac-ros.github.io/getting_started/sensors/zed_setup.html
- nvblox ZED tutorial: https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_zed.html
- Pliki w obrazie: `/opt/ros/lyrical/share/isaac_ros_cuvslam/`, `/opt/ros_ws/install/zed_wrapper/share/zed_wrapper/{launch,config,urdf}`
