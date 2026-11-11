cmake_minimum_required(VERSION 3.20)
project(hal_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # xacro -> URDF structure checks (no ROS graph needed)
  ament_add_pytest_test(test_urdf test/test_urdf.py TIMEOUT 120)
  # robot_state_publisher launch -> TF chain check
  add_launch_test(test/test_description_launch.py TIMEOUT 120)
endif()

ament_package()
