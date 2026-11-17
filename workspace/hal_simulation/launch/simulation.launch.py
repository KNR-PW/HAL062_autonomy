import tempfile

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def process_world(world, sim_config):
    """Return an SDF path for gz: *.xacro worlds are expanded with sim_config to a temp file."""
    if not world.endswith('.xacro'):
        return world
    sdf = xacro.process_file(world, mappings={'sim_config': sim_config}).toprettyxml(indent='  ')
    with tempfile.NamedTemporaryFile('w', prefix='hal_world_', suffix='.sdf', delete=False) as f:
        f.write(sdf)
    return f.name


def launch_gazebo(context):
    sim_config = LaunchConfiguration('sim_config').perform(context)
    world = process_world(LaunchConfiguration('world').perform(context), sim_config)
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

    # robot_state_publisher (+ optional RViz) from hal_description:
    # xacro -> /robot_description + TF.
    # simulation:=true -> movable joints + Gazebo plugins/sensors in the URDF and use_sim_time
    # (joint states and TF stamped with the Gazebo clock, /clock from the bridge).
    robot_state_publisher = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('hal_description'), 'launch', 'description.launch.py']),
        launch_arguments={
            'simulation': 'true',
            'sim_config': sim_config,  # sensor parameters (section "sensors")
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
        # Run every process (also Gazebo from the included gz_sim.launch.py) as if attached to a
        # terminal, otherwise they detect a pipe and print logs without colors.
        SetLaunchConfiguration('emulate_tty', 'true'),

        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('hal_simulation'), 'worlds', 'mars.sdf.xacro']),
            description='Path to the world .sdf file (*.sdf.xacro is expanded with sim_config)',
        ),
        DeclareLaunchArgument(
            'sim_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('hal_simulation'), 'config', 'simulation.yaml']),
            description='YAML with world (GPS origin, ...) and sensor parameters',
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
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Spawn orientation yaw [rad]'),
        DeclareLaunchArgument(
            'relay_cmd_vel', default_value='false',
            description='Drive the robot also from /cmd_vel (Nav2), bypassing wheel_mux',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz (hal_description/rviz/description.rviz) with the robot',
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

        # Optional: Nav2 /cmd_vel straight to the drive (gz /mux/cmd_vel). On the rover the
        # hal_communication wheel_mux does this; it is not part of the simulation.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=['/mux/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
            remappings=[('/mux/cmd_vel', '/cmd_vel')],
            condition=IfCondition(LaunchConfiguration('relay_cmd_vel')),
            output='screen',
        ),
    ])
