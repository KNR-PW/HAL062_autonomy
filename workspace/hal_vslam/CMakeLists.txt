cmake_minimum_required(VERSION 3.8)
project(hal_vslam)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
# nvblox is not part of the base image yet (Isaac ROS 5.0 image built with keys zed+cuvslam only).
# Until it is added to the Dockerfile, build the nvblox-dependent component only when nvblox_msgs is present.
find_package(nvblox_msgs QUIET)

# Build component library (requires nvblox_msgs)
if(nvblox_msgs_FOUND)
  add_library(mesh_to_pointcloud SHARED src/mesh_to_pointcloud.cpp)
  target_compile_features(mesh_to_pointcloud PRIVATE cxx_std_17)
  ament_target_dependencies(mesh_to_pointcloud
    rclcpp
    rclcpp_components
    nvblox_msgs
    sensor_msgs
  )

  rclcpp_components_register_node(mesh_to_pointcloud
    PLUGIN "hal_vslam::MeshToPointCloudComponent"
    EXECUTABLE mesh_to_pointcloud_exec
  )

  install(TARGETS mesh_to_pointcloud
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  message(WARNING "hal_vslam: nvblox_msgs not found - skipping mesh_to_pointcloud component (nvblox not installed in this image yet)")
endif()

# odom_manager node (SLAM -> OdomManager -> NAV2 passthrough with switch)
add_executable(odom_manager src/odom_manager.cpp)
target_compile_features(odom_manager PRIVATE cxx_std_17)
ament_target_dependencies(odom_manager
  rclcpp
  nav_msgs
  std_srvs
  isaac_ros_visual_slam_interfaces
)

# Install
install(TARGETS odom_manager
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/anti_teleport_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
