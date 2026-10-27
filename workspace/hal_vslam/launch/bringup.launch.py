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

ADDITIONAL:
- Close ZED Studio on the host first.
- First start with depth downloads/optimizes the ZED neural depth model (~2 min) into the hal062_zed_resources volume.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hal_vslam')
    launch_dir = os.path.join(share, 'launch')
    camera_name = LaunchConfiguration('camera_name')

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'zed.launch.py')),
        launch_arguments={'camera_name': camera_name,
                          'svo_path': LaunchConfiguration('svo_path')}.items())

    visual_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'visual_slam.launch.py')),
        launch_arguments={'camera_name': camera_name,
                          'enable_anti_teleport': LaunchConfiguration('enable_anti_teleport'),
                          'enable_odom_manager': LaunchConfiguration('enable_odom_manager')}.items())

    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nvblox.launch.py')),
        launch_arguments={'camera_name': camera_name}.items(),
        condition=IfCondition(LaunchConfiguration('nvblox')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'config', 'hal_vslam.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument('svo_path', default_value='live'),
        DeclareLaunchArgument('nvblox', default_value='true', description='Start nvblox + mesh_to_pointcloud'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start rviz2 with config/hal_vslam.rviz'),
        DeclareLaunchArgument('enable_anti_teleport', default_value='true'),
        DeclareLaunchArgument('enable_odom_manager', default_value='true'),
        zed,
        visual_slam,
        nvblox,
        rviz,
    ])
