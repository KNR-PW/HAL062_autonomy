#!/usr/bin/env python3
# INFO:
# - One launch for the whole HAL-062 autonomy stack on the ZED 2i. Port of
#   ros2_docker_ws/src/hal_main/launch/full_autonomy.launch.py:
#     hal_description/description.launch.py   robot_state_publisher (URDF incl. ZED links)
#     hal_vslam/bringup.launch.py             ZED (publish_urdf:=false) + cuVSLAM (without the
#                                             stop-gap base->camera TF) + anti_teleport +
#                                             odom_manager + nvblox (nvblox:=false to skip)
#     hal_navigation/navigation.launch.py     Nav2 + planar_tf_broadcaster, after `vslam_delay` s
# - Not ported yet (were in full_autonomy.launch.py): hal_aruco detector + global_pos_from_aruco,
#   hal_god detector, GNSS watchdog / gnss_support / navsat_transform. Add them here when those
#   packages are ported.
# - The rover only moves when hal_communication wheel_mux relays /cmd_vel.
#
# ARGUMENTS:
# - location (blank_large), params_file, cloud_topic -> navigation.launch.py
# - camera_name (zed), nvblox (true), rviz (true) -> hal_vslam bringup / rviz of hal_navigation
# - vslam (true): false = do not start hal_vslam (description + Nav2 only, e.g. with fake odometry)
# - vslam_delay (10.0): seconds between hal_vslam bringup and Nav2 start

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch(pkg, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(pkg), 'launch', name))


def generate_launch_description():
    description = IncludeLaunchDescription(
        _launch('hal_description', 'description.launch.py'),
        launch_arguments={'camera_name': LaunchConfiguration('camera_name')}.items())

    vslam = IncludeLaunchDescription(
        _launch('hal_vslam', 'bringup.launch.py'),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'publish_urdf': 'false',
            'publish_base_to_camera_tf': 'false',
            'nvblox': LaunchConfiguration('nvblox'),
            'rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('vslam')))

    navigation = IncludeLaunchDescription(
        _launch('hal_navigation', 'navigation.launch.py'),
        launch_arguments={
            'location': LaunchConfiguration('location'),
            'params_file': LaunchConfiguration('params_file'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'rviz': LaunchConfiguration('rviz'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('location', default_value='blank_large'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('hal_navigation'), 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('cloud_topic',
                              default_value='/zed/zed_node/point_cloud/cloud_registered'),
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument('nvblox', default_value='true'),
        DeclareLaunchArgument('vslam', default_value='true',
                              description='Start hal_vslam bringup (ZED + cuVSLAM [+ nvblox])'),
        DeclareLaunchArgument('vslam_delay', default_value='10.0',
                              description='Seconds to wait after hal_vslam before starting Nav2'),
        DeclareLaunchArgument('rviz', default_value='true'),
        description,
        vslam,
        TimerAction(period=LaunchConfiguration('vslam_delay'), actions=[navigation]),
    ])
