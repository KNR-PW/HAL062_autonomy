cmake_minimum_required(VERSION 3.20)
project(hal_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/planar_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # static checks of config/*.yaml and maps (no ROS graph)
  ament_add_pytest_test(test_params test/test_params.py TIMEOUT 60)
  # full Nav2 stack with fake localization / point cloud: needs ros-lyrical-navigation2 installed
  add_launch_test(test/test_navigation_launch.py TIMEOUT 300)
endif()

ament_package()
