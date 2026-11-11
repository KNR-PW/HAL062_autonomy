#!/usr/bin/env python3
"""
INFO:
- Launchfile that fires up the ZED 2i camera only (zed-ros2-wrapper 5.x, ZED SDK 5.2)
- Compatible with Isaac ROS 5.0 / ROS 2 Lyrical image
- Thin wrapper around zed_wrapper/launch/zed_camera.launch.py with HAL062 overrides
  (config/zed2i_override.yaml): left/right gray topics on, ZED pos. tracking off, mag on.

APPLICATION:
- Produces the inputs for visual_slam.launch.py:
    /zed/zed_node/{left,right}/gray/rect/image        (mono8)
    /zed/zed_node/{left,right}/gray/rect/camera_info
    /zed/zed_node/imu/data, /zed/zed_node/imu/mag
  and the static camera TF tree rooted at zed_camera_link (robot_state_publisher from ZED URDF).

ADDITIONAL:
- Close ZED Studio / any other ZED SDK app on the host first, otherwise the camera will not open.
- svo_path:=<file.svo2> replays a recording instead of the live camera.
- publish_urdf:=false when hal_description/description.launch.py runs (otherwise two robot_state_publishers claim the ZED links).
- override_config:=<yaml> selects the ZED parameter override (default: with depth for nvblox).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_override_config = os.path.join(
        get_package_share_directory('hal_vslam'), 'config', 'zed2i_override.yaml')
    zed_camera_launch = os.path.join(
        get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_camera_launch),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
            'svo_path': LaunchConfiguration('svo_path'),
            # cuVSLAM owns map->odom->base; ZED must not publish its own odom TF.
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            # false when hal_description provides the robot_description (ZED links are part of the rover URDF).
            'publish_urdf': LaunchConfiguration('publish_urdf'),
            # Camera->IMU calibration TF (zed_left_camera_frame -> zed_imu_link), needed by cuVSLAM imu_frame.
            # Works with publish_tf:=false in wrapper 5.2 despite the launch-arg description.
            'publish_imu_tf': 'true',
            'ros_params_override_path': LaunchConfiguration('override_config'),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='zed2i'),
        DeclareLaunchArgument('camera_name', default_value='zed',
                              description='Topic namespace and TF frame prefix (/<camera_name>/zed_node/..., <camera_name>_camera_link)'),
        DeclareLaunchArgument('publish_urdf', default_value='true',
                              description='Let the ZED wrapper publish its own URDF/static TF tree. Set false when '
                                          'hal_description description.launch.py is running (single robot_description)'),
        DeclareLaunchArgument('svo_path', default_value='live',
                              description='Path to an SVO file to replay instead of the live camera'),
        DeclareLaunchArgument('override_config', default_value=default_override_config,
                              description='ZED ros_params_override_path (zed2i_override.yaml = with depth for nvblox, '
                                          'zed2i_override_nodepth.yaml = VSLAM only)'),
        zed,
    ])
