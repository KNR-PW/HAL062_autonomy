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

# urdf/ launch/ config/ are installed once they exist
foreach(dir meshes urdf launch config rviz)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

# GZ_SIM_RESOURCE_PATH += <prefix>/share  ->  Gazebo resolves package://hal_description/meshes/...
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv")

ament_package()
