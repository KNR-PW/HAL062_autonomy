#!/usr/bin/env python3
# INFO:
# - robot_state_publisher for the HAL-062 URDF (urdf/hal_062.urdf.xacro), ZED included via the
#   zed_wrapper URDF macro.
# - Port of ros2_docker_ws/src/hal_main/launch/robot_state_publisher.launch.py (the extra static TF
#   camera_frame -> zed2_camera_link is gone: the ZED links are part of the URDF now).
#
# ARGUMENTS:
# - simulation (false): revolute rockers/wheels + Gazebo plugins in the URDF, use_sim_time
# - camera_name (zed), camera_model (zed2i): must match hal_vslam zed.launch.py
# - camera_x/y/z/pitch: ZED mount pose in the chassis frame (defaults = old hal_main values)
#
# USAGE:
#     ros2 launch hal_description description.launch.py
#     ros2 launch hal_vslam zed.launch.py publish_urdf:=false   # no second URDF/TF tree from ZED
#     ros2 launch hal_vslam visual_slam.launch.py publish_base_to_camera_tf:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('hal_description')
    xacro_file = os.path.join(share, 'urdf', 'hal_062.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' simulation:=', LaunchConfiguration('simulation'),
            ' camera_name:=', LaunchConfiguration('camera_name'),
            ' camera_model:=', LaunchConfiguration('camera_model'),
            ' camera_x:=', LaunchConfiguration('camera_x'),
            ' camera_y:=', LaunchConfiguration('camera_y'),
            ' camera_z:=', LaunchConfiguration('camera_z'),
            ' camera_pitch:=', LaunchConfiguration('camera_pitch'),
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('simulation'),
        }])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'description.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation', default_value='false',
            description='Simulation URDF variant (movable joints, Gazebo plugins), use_sim_time'),
        DeclareLaunchArgument(
            'camera_name', default_value='zed',
            description='ZED TF prefix, must match hal_vslam zed.launch.py camera_name'),
        DeclareLaunchArgument('camera_model', default_value='zed2i'),
        DeclareLaunchArgument('camera_x', default_value='0.342',
                              description='ZED mount x in chassis frame [m]'),
        DeclareLaunchArgument('camera_y', default_value='0.0',
                              description='ZED mount y in chassis frame [m]'),
        DeclareLaunchArgument('camera_z', default_value='0.1',
                              description='ZED mount z in chassis frame [m]'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0',
                              description='ZED mount pitch [rad], positive = looking down'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start rviz2 with rviz/description.rviz'),
        robot_state_publisher,
        rviz,
    ])
