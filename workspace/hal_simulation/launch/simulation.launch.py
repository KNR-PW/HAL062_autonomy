from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_gazebo(context):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot = LaunchConfiguration('robot').perform(context).lower() == 'true'

    # -r: start simulation immediately (not paused), -s: server only (no GUI)
    gz_args = f'{world} -r -v {LaunchConfiguration("verbosity").perform(context)}'
    if headless:
        gz_args += ' -s'
    else:
        # GUI layout + theme (dark, red accents); overrides any <gui> config from the world SDF
        gz_args += f' --gui-config {LaunchConfiguration("gui_config").perform(context)}'

    gz_sim = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',  # closing the Gazebo window stops the whole launch
        }.items(),
    )
    if not robot:
        return [gz_sim]

    # robot_state_publisher (+ optional RViz) from hal_description: xacro -> /robot_description + TF.
    # use_sim_time: joint states and TF are stamped with the Gazebo clock (/clock from the bridge).
    robot_state_publisher = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('hal_description'), 'launch', 'robot_state_publisher.launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    # Spawn the robot from the /robot_description topic published by robot_state_publisher.
    # No -world: 'create' asks Gazebo for the running world, so any world file works.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-topic', 'robot_description',
            '-name', LaunchConfiguration('robot_name'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )
    return [gz_sim, robot_state_publisher, spawn_robot]


def generate_launch_description():
    return LaunchDescription([
        # Run every process (also Gazebo from the included gz_sim.launch.py) as if attached to a terminal,
        # otherwise they detect a pipe and print logs without colors.
        SetLaunchConfiguration('emulate_tty', 'true'),

        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('hal_simulation'), 'worlds', 'mars.sdf']),
            description='Path to the world .sdf file',
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run only the Gazebo server, without the GUI',
        ),
        DeclareLaunchArgument(
            'verbosity', default_value='3',
            description='Gazebo log verbosity (0-4)',
        ),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('hal_simulation'), 'config', 'ros_gz_bridge.yaml']),
            description='Path to the ros_gz_bridge YAML config',
        ),
        DeclareLaunchArgument(
            'gui_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('hal_simulation'), 'config', 'gui.config']),
            description='Path to the Gazebo GUI config (layout, theme)',
        ),
        DeclareLaunchArgument(
            'robot',
            default_value='true',
            description='Spawn robot in the world'
        ),
        DeclareLaunchArgument(
            'robot_name', default_value='hal_062',
            description='Name of the spawned robot model in Gazebo',
        ),
        DeclareLaunchArgument('x', default_value='-2.5', description='Spawn position x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn position y [m]'),
        DeclareLaunchArgument('z', default_value='0.5', description='Spawn position z [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn orientation yaw [rad]'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz (hal_description/rviz/default.rviz) together with the robot',
        ),

        OpaqueFunction(function=launch_gazebo),

        # ROS <-> Gazebo topic bridge, topics listed in config/ros_gz_bridge.yaml
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'config_file': LaunchConfiguration('bridge_config')}],
            output='screen',
        ),
    ])
