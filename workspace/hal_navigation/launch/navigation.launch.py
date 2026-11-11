#!/usr/bin/env python3
# INFO:
# - HAL-062 Nav2 bringup (Nav2 1.5.0 / ROS 2 Lyrical).
#   Port of ros2_docker_ws/src/hal_main/launch/navigation.launch.py.
# - Starts map_server, controller/smoother/planner/behavior servers, bt_navigator,
#   waypoint_follower, velocity_smoother and ONE lifecycle manager that also owns map_server.
#   The old launch included nav2_bringup/navigation_launch.py and activated map_server by hand
#   with timed `ros2 lifecycle set`. Nav2 1.5's bringup launch additionally starts
#   route/collision_monitor/docking/following servers that we do not configure, hence this own
#   node list (docs/NAV2_DESCRIPTION_PORT_PLAN.md §8.2).
# - planar_tf_broadcaster publishes map_2d / odom_2d / rover_2d (frames used by the Nav2 config
#   and by hal_automata).
# - Expects: TF map -> odom -> base_footprint (cuVSLAM, hal_vslam), robot description
#   (hal_description), odometry on /visual_slam/tracking/odometry, ZED point cloud for the local
#   costmap obstacle layer.
#
# ARGUMENTS:
# - params_file   Nav2 yaml (default config/nav2_params.yaml; nav2_params_spd.yaml = fast variant)
# - location      map: blank | blank_large | erc | mel  (maps/costmap_<location>.yaml)
# - use_sim_time  false
# - cloud_topic   PointCloud2 for the local costmap obstacle layer
#                 (default /zed/zed_node/point_cloud/cloud_registered,
#                  alternative /nvblox/mesh_pointcloud)
# - autostart     true -> lifecycle manager configures + activates everything
# - log_level     info | debug ... applied to all Nav2 nodes
# - rviz          false
#
# Nav2 publishes geometry_msgs/Twist on /cmd_vel (enable_stamped_cmd_vel: False in the yaml) for
# hal_communication wheel_mux.

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

VALID_LOCATIONS = ['blank', 'blank_large', 'erc', 'mel']

LIFECYCLE_NODES = [
    'map_server',
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def _rewrite_params(params_file, use_sim_time, map_yaml, cloud_topic):
    """
    Return a temp copy of params_file with use_sim_time / map / cloud topic applied.

    Replaces nav2_common.RewrittenYaml (nav2_common is not in the navigation2 metapackage).
    """
    with open(params_file) as f:
        params = yaml.safe_load(f)

    def visit(node):
        if isinstance(node, dict):
            if 'ros__parameters' in node and isinstance(node['ros__parameters'], dict):
                node['ros__parameters']['use_sim_time'] = use_sim_time
            for v in node.values():
                visit(v)
    visit(params)

    params['map_server']['ros__parameters']['yaml_filename'] = map_yaml
    for costmap in ('local_costmap', 'global_costmap'):
        layer = params[costmap][costmap]['ros__parameters'].get('obstacle_layer')
        if layer:
            layer['cloud']['topic'] = cloud_topic

    out = tempfile.NamedTemporaryFile(mode='w', prefix='hal_nav2_params_', suffix='.yaml',
                                      delete=False)
    yaml.safe_dump(params, out, default_flow_style=False)
    out.close()
    return out.name


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('hal_navigation')
    location = LaunchConfiguration('location').perform(context)
    if location not in VALID_LOCATIONS:
        raise RuntimeError(f'Invalid location "{location}". Must be one of: {VALID_LOCATIONS}')
    map_yaml = os.path.join(share, 'maps', f'costmap_{location}.yaml')
    if not os.path.exists(map_yaml):
        raise RuntimeError(f'Map file not found: {map_yaml}')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    params_file = _rewrite_params(
        LaunchConfiguration('params_file').perform(context),
        use_sim_time,
        map_yaml,
        LaunchConfiguration('cloud_topic').perform(context))
    autostart = LaunchConfiguration('autostart').perform(context).lower() in ('true', '1')
    log_level = LaunchConfiguration('log_level').perform(context)

    def nav2_node(package, executable, name, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=list(remappings))

    nodes = [
        nav2_node('nav2_map_server', 'map_server', 'map_server'),
        # controller -> cmd_vel_nav -> velocity_smoother -> /cmd_vel (no collision monitor in this
        #   stack)
        nav2_node('nav2_controller', 'controller_server', 'controller_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server'),
        nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'node_names': LIFECYCLE_NODES}]),
    ]
    return nodes


def generate_launch_description():
    share = get_package_share_directory('hal_navigation')

    planar_tf_broadcaster = Node(
        package='hal_navigation',
        executable='planar_tf_broadcaster.py',
        name='planar_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml'),
                              description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument('location', default_value='blank_large',
                              description='Map for map_server: ' + ', '.join(VALID_LOCATIONS)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic',
                              default_value='/zed/zed_node/point_cloud/cloud_registered',
                              description='PointCloud2 source of the local costmap obstacles'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Lifecycle manager configures + activates Nav2 nodes'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start rviz2 with rviz/navigation.rviz'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log level for the Nav2 nodes'),
        planar_tf_broadcaster,
        OpaqueFunction(function=launch_setup),
        rviz,
    ])
