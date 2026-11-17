#!/usr/bin/env python3
# INFO:
# - One launch for the whole HAL-062 autonomy stack on the ZED 2i. Port of
#   ros2_docker_ws/src/hal_main/launch/full_autonomy.launch.py:
#     hal_description/description.launch.py   robot_state_publisher (URDF incl. ZED links)
#       or, with simulation:=true,
#     hal_simulation/simulation.launch.py     Gazebo + robot (same URDF, simulation variant) +
#                                             bridge publishing the ZED / IMU / GPS topics
#     hal_vslam/bringup.launch.py             ZED (publish_urdf:=false) + cuVSLAM (without the
#                                             stop-gap base->camera TF) + anti_teleport +
#                                             odom_manager + nvblox (nvblox:=false to skip)
#                                             simulation:=true -> no ZED wrapper, use_sim_time
#     hal_navigation/navigation.launch.py     Nav2 + planar_tf_broadcaster, after `vslam_delay` s
# - Not ported yet (were in full_autonomy.launch.py): hal_aruco detector + global_pos_from_aruco,
#   hal_god detector, GNSS watchdog / gnss_support / navsat_transform. Add them here when those
#   packages are ported.
# - The rover only moves when hal_communication wheel_mux relays /cmd_vel. In simulation /cmd_vel
#   drives the simulated rover directly (simulation.launch.py relay_cmd_vel:=true).
# - Every include is a scoped GroupAction: IncludeLaunchDescription does not scope launch
#   arguments, so e.g. 'rviz': 'false' given to hal_vslam would otherwise also switch off the
#   Nav2 rviz started later.
#
# ARGUMENTS:
# - simulation (false): true = start hal_simulation (Gazebo) instead of the bare description,
#   run hal_vslam / Nav2 on the simulated sensors with use_sim_time
# - headless (false): simulation only, Gazebo without GUI
# - location (blank_large), params_file, cloud_topic -> navigation.launch.py
# - camera_name (zed), nvblox (true), rviz (true) -> hal_vslam bringup / rviz of hal_navigation
# - vslam (true): false = do not start hal_vslam (description + Nav2 only, e.g. with fake odometry)
# - vslam_delay (10.0): seconds between hal_vslam bringup and Nav2 start

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch(pkg, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(pkg), 'launch', name))


def _include(pkg, name, arguments, condition=None):
    """Include a launch file in its own launch-configuration scope."""
    return GroupAction(
        [IncludeLaunchDescription(_launch(pkg, name), launch_arguments=arguments.items())],
        scoped=True, condition=condition)


def generate_launch_description():
    simulation = LaunchConfiguration('simulation')

    description = _include(
        'hal_description', 'description.launch.py',
        {'camera_name': LaunchConfiguration('camera_name')},
        condition=UnlessCondition(simulation))

    gazebo = _include(
        'hal_simulation', 'simulation.launch.py',
        {'headless': LaunchConfiguration('headless'),
         'relay_cmd_vel': 'true',  # Nav2 /cmd_vel drives the simulated rover (no wheel_mux)
         'rviz': 'false'},
        condition=IfCondition(simulation))

    vslam = _include(
        'hal_vslam', 'bringup.launch.py',
        {'camera_name': LaunchConfiguration('camera_name'),
         'simulation': simulation,
         'publish_urdf': 'false',
         'publish_base_to_camera_tf': 'false',
         'nvblox': LaunchConfiguration('nvblox'),
         'rviz': 'false'},
        condition=IfCondition(LaunchConfiguration('vslam')))

    navigation = _include(
        'hal_navigation', 'navigation.launch.py',
        {'location': LaunchConfiguration('location'),
         'params_file': LaunchConfiguration('params_file'),
         'cloud_topic': LaunchConfiguration('cloud_topic'),
         'use_sim_time': simulation,
         'rviz': LaunchConfiguration('rviz')})

    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false',
                              description='true: Gazebo simulation (hal_simulation) instead of '
                                          'the rover; whole stack with use_sim_time'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Simulation only: Gazebo server without the GUI'),
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
        gazebo,
        vslam,
        TimerAction(period=LaunchConfiguration('vslam_delay'), actions=[navigation]),
    ])
