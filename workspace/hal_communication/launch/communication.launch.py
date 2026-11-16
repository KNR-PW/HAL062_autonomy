import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    # Path to the YAML configuration file
    config_file = os.path.join(
        get_package_share_directory('hal_communication'),
        'config',
        'comm.yaml'
    )

    return LaunchDescription([
        # Declare the parameter file
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the communication configuration file'
        ),

        # Communication server node
        Node(
            package='hal_communication',
            executable='hal_communication_node',
            name='communication',
            namespace="hal",
            parameters=[config_file],
            output='screen',
            respawn=True
        ),

        # Message sender node (Python)
        Node(
            package='hal_communication',
            executable='message_sender.py',
            name='message_sender',
            namespace="hal",
            output='screen'
        )
    ])
