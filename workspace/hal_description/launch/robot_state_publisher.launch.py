from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    hal_description_pkg = get_package_share_directory('hal_description')
    hal062_urdf = PathJoinSubstitution([hal_description_pkg, 'urdf', 'hal062.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([hal_description_pkg, 'rviz', 'default.rviz'])

    robot_description_content = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', hal062_urdf]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(rviz) 
    )



    return LaunchDescription([
        SetLaunchConfiguration('emulate_tty', 'true'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='If true, turn on rviz with hardocded config'
        ),


        robot_state_publisher,
        rviz_node

    ])


