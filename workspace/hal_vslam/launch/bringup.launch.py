#!/usr/bin/env python3
"""
INFO:
- One-shot bringup of the HAL062 perception stack on ZED 2i (Isaac ROS 5.0 / ROS 2 Lyrical):
    zed.launch.py          -> camera (depth on), ZED pos. tracking off
    visual_slam.launch.py  -> cuVSLAM (loaded into the ZED container), anti_teleport_node, odom_manager, base_footprint TF
    nvblox.launch.py       -> nvblox + mesh_to_pointcloud (own container)
    rviz2                  -> config/hal_vslam.rviz (rviz:=false to skip)

USAGE (inside the container, after colcon build + source):
    ros2 launch hal_vslam bringup.launch.py
    ros2 launch hal_vslam bringup.launch.py nvblox:=false        # VSLAM only (still with depth on; see zed.launch.py override_config)
    ros2 launch hal_vslam bringup.launch.py rviz:=false
    ros2 launch hal_vslam bringup.launch.py simulation:=true                # on hal_simulation (no ZED wrapper)

SIMULATION (simulation:=true):
- zed.launch.py is not started: hal_simulation publishes the same /<camera_name>/zed_node/... topics and the
  ZED frames come from hal_description (simulation:=true, incl. <camera_name>_imu_link).
- cuVSLAM runs in its own container (standalone:=true, there is no ZED container), use_sim_time everywhere,
  no stop-gap base_footprint -> camera TF (the URDF provides it).

ADDITIONAL:
- Close ZED Studio on the host first.
- First start with depth downloads/optimizes the ZED neural depth model (~2 min) into the hal062_zed_resources volume.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hal_vslam')
    launch_dir = os.path.join(share, 'launch')
    camera_name = LaunchConfiguration('camera_name')
    simulation = LaunchConfiguration('simulation')
    # 'true'/'false' -> python bool expression helper
    in_sim = ["'", simulation, "'.lower() == 'true'"]

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'zed.launch.py')),
        launch_arguments={'camera_name': camera_name,
                          'svo_path': LaunchConfiguration('svo_path'),
                          'publish_urdf': LaunchConfiguration('publish_urdf')}.items(),
        condition=UnlessCondition(simulation))

    visual_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'visual_slam.launch.py')),
        launch_arguments={'camera_name': camera_name,
                          'use_sim_time': simulation,
                          # no ZED container in simulation -> own container
                          'standalone': PythonExpression(
                              ["'true' if "] + in_sim + [" else '", LaunchConfiguration('standalone'), "'"]),
                          # the simulation URDF always provides base_footprint -> camera
                          'publish_base_to_camera_tf': PythonExpression(
                              ["'false' if "] + in_sim
                              + [" else '", LaunchConfiguration('publish_base_to_camera_tf'), "'"]),
                          'enable_anti_teleport': LaunchConfiguration('enable_anti_teleport'),
                          'enable_odom_manager': LaunchConfiguration('enable_odom_manager')}.items())

    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nvblox.launch.py')),
        launch_arguments={'camera_name': camera_name,
                          'use_sim_time': simulation}.items(),
        condition=IfCondition(LaunchConfiguration('nvblox')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'config', 'hal_vslam.rviz')],
        parameters=[{'use_sim_time': simulation}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')
        
    aruco = Node(
        package='hal_aruco',
        executable='detector',
        name='aruco_node',
        condition=IfCondition(LaunchConfiguration('aruco')),
        output='screen')

    aruco_global_pos = Node(
        package='hal_aruco',
        executable='global_pos_from_aruco.py',
        name='global_pos_from_aruco',
        condition=IfCondition(LaunchConfiguration('aruco')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument('simulation', default_value='false',
                              description='Run on hal_simulation instead of the ZED (no zed_wrapper, use_sim_time)'),
        DeclareLaunchArgument('publish_urdf', default_value='true',
                              description='ZED wrapper publishes its own URDF/TF (false when hal_description runs)'),
        DeclareLaunchArgument('standalone', default_value='false',
                              description='cuVSLAM in its own container (forced true in simulation)'),
        DeclareLaunchArgument('publish_base_to_camera_tf', default_value='true',
                              description='Stop-gap static base_footprint -> camera TF (false when hal_description '
                                          'runs; forced false in simulation)'),
        DeclareLaunchArgument('svo_path', default_value='live'),
        DeclareLaunchArgument('nvblox', default_value='true', description='Start nvblox + mesh_to_pointcloud'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start rviz2 with config/hal_vslam.rviz'),
        DeclareLaunchArgument('enable_anti_teleport', default_value='true'),
        DeclareLaunchArgument('enable_odom_manager', default_value='true'),
        zed,
        visual_slam,
        nvblox,
        rviz,
        DeclareLaunchArgument('aruco', default_value='true', description='Start hal_aruco detector'),
        aruco,
        aruco_global_pos
    ])
