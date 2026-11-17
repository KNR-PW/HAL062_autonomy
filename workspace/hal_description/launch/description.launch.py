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
# - gps_x/y/z: GNSS antenna (gps_link) pose in the chassis frame (placeholder defaults)
# - sim_config: YAML with simulated sensor parameters (hal_simulation/config/simulation.yaml)
#
# USAGE:
#     ros2 launch hal_description description.launch.py
#     ros2 launch hal_vslam zed.launch.py publish_urdf:=false   # no second URDF/TF tree from ZED
#     ros2 launch hal_vslam visual_slam.launch.py publish_base_to_camera_tf:=false
#
# TEMPORARY: the zed_wrapper macro uses file://<zed_msgs>/meshes/<model>.stl as the ZED visual and
# collision, but the apt zed_msgs in the image ships no meshes (Gazebo errors, nothing in rviz).
# While that file is missing, the mesh is replaced by a box of the ZED 2i size
# (replace_missing_zed_mesh); once the mesh exists the URDF is used unchanged.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

XACRO_ARGS = ['simulation', 'camera_name', 'camera_model', 'camera_x', 'camera_y', 'camera_z',
              'camera_pitch', 'gps_x', 'gps_y', 'gps_z', 'sim_config']

# ZED 2i housing 175.25 x 30.25 x 43.10 mm (width x height x depth) -> x (depth), y, z [m]
ZED_BOX_SIZE = '0.0431 0.1753 0.0303'


def replace_missing_zed_mesh(doc):
    """Replace ZED meshes from zed_msgs that do not exist on disk with a box (in place)."""
    for mesh in list(doc.getElementsByTagName('mesh')):
        filename = mesh.getAttribute('filename')
        path = filename[len('file://'):] if filename.startswith('file://') else None
        if path and '/zed_msgs/meshes/' in path and not os.path.isfile(path):
            box = doc.createElement('box')
            box.setAttribute('size', ZED_BOX_SIZE)
            mesh.parentNode.replaceChild(box, mesh)
    return doc


def robot_state_publisher(context, xacro_file):
    mappings = {arg: LaunchConfiguration(arg).perform(context) for arg in XACRO_ARGS}
    doc = replace_missing_zed_mesh(xacro.process_file(xacro_file, mappings=mappings))
    simulation = mappings['simulation'].lower() == 'true'
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': doc.toxml(),
            'use_sim_time': simulation,
        }])]


def generate_launch_description():
    share = get_package_share_directory('hal_description')
    xacro_file = os.path.join(share, 'urdf', 'hal_062.urdf.xacro')

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
        DeclareLaunchArgument('gps_x', default_value='0.0',
                              description='GNSS antenna (gps_link) x in chassis frame [m]'),
        DeclareLaunchArgument('gps_y', default_value='0.0',
                              description='GNSS antenna (gps_link) y in chassis frame [m]'),
        DeclareLaunchArgument('gps_z', default_value='0.3',
                              description='GNSS antenna (gps_link) z in chassis frame [m]'),
        DeclareLaunchArgument('sim_config', default_value='',
                              description='YAML with simulated sensor parameters (only with '
                                          'simulation:=true; empty = xacro defaults)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start rviz2 with rviz/description.rviz'),
        OpaqueFunction(function=robot_state_publisher, args=[xacro_file]),
        rviz,
    ])
