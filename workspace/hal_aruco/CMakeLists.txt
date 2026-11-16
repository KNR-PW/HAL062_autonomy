cmake_minimum_required(VERSION 3.8)
project(hal_aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoPose.msg"
  "srv/ArucoPose.srv"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(detector src/main.cpp)

target_include_directories(detector PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${OpenCV_INCLUDE_DIRS}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(detector
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  cv_bridge::cv_bridge
  rclcpp::rclcpp
  ${OpenCV_LIBS}
  "${cpp_typesupport_target}"
)

install(TARGETS
  detector
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/global_pos_from_aruco.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
