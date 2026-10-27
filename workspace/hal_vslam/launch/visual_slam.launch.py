#!/usr/bin/env python3
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

"""
INFO:
- Launchfile that fires up 'isaac_ros_cuvslam' only (package renamed from 'isaac_ros_visual_slam' in Isaac ROS 5.0)
- Compatible with Isaac ROS 5.0 / ROS 2 Lyrical
- Configured for ZED 2i via zed-ros2-wrapper 5.x (start zed.launch.py first)

APPLICATION:
- Calculates VIO
- Calculates SLAM PATH and solves loop-closure problem
- Generates nice pointcloud map - can be used for after-task-presentation

DEVELOPMENT:
- Originates from https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-2.0.0/isaac_ros_visual_slam/launch/isaac_ros_visual_slam_zed.launch.py
- Ported to Isaac ROS 5.0 using (inside the image):
    /opt/ros/lyrical/share/isaac_ros_cuvslam/launch/isaac_ros_visual_slam_core.launch.py
    /opt/ros/lyrical/share/isaac_ros_zed/launch/isaac_ros_zed_stereo_rect_core.launch.py  (after apt install ros-lyrical-isaac-ros-zed)
- Params: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html
- Changes vs release 3.1 version of this file:
    enable_imu_fusion -> tracking_mode: 1 ; input_imu_frame -> imu_frame ; publish_odom_to_rig_tf -> publish_odom_to_base_tf
    cuVSLAM 5.0 takes grayscale images -> ImageFormatConverter (rgb8) nodes removed, ZED 5.x gray topics used directly
    ZED 5.x topics (/zed/zed_node/left/gray/rect/image) and frames (zed_left_camera_frame_optical, zed_imu_link)

ADDITIONAL:
- Be carefull! It can bite
- By default the cuVSLAM component is LOADED INTO THE ZED CONTAINER (/<camera_name>/zed_container, created by
  zed.launch.py) with intra-process comms. Measured on RTX 4070 Ti @ 1280x720x30fps: separate container = ~170 frame
  jitter warnings / 40 s and odometry ~23 Hz (DDS drops); same container + IPC = ~4 warnings and 30 Hz.
  standalone:=true restores the old behaviour (own component container).
- base_footprint does not exist in TF yet (hal_description is empty) -> this launch publishes a static
  base_footprint -> <camera_name>_camera_link TF (disable with publish_base_to_camera_tf:=false once a URDF provides it).
"""


"""THIS LAUNCHFILE IS FOR ZED-ONLY. DOESNT SUPPORT SIMULATION FOR NOW"""


def generate_launch_description():

    camera_name = LaunchConfiguration('camera_name')
    zed_topics = ['/', camera_name, '/zed_node/']

    """ISAAC_ROS_VISUAL_SLAM  <1>"""
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_cuvslam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_image_denoising': False,
                    'rectified_images': True,
                    'tracking_mode': 1,  # 0 = multicamera (stereo only), 1 = VIO (IMU fusion), 2 = RGBD
                    'image_jitter_threshold_ms': 35.00,  # ZED publishes at 30 fps (33.3 ms)
                    'base_frame': 'base_footprint',
                    'imu_frame': [camera_name, '_imu_link'],
                    'gyro_noise_density': 0.00279,  #SPEC
                    'gyro_random_walk': 0.00022,    #EST
                    'accel_noise_density': 0.0314,  #SPEC
                    'accel_random_walk': 0.0450,    #EST
                    'calibration_frequency': 200.0,
                    'enable_slam_visualization': True, #VISUALIZATION ONLY
                    'enable_landmarks_view': False, #VISUALIZATION ONLY
                    'enable_observations_view': False, #VISUALIZATION ONLY
                    'path_max_size': 36000,
                    'camera_optical_frames': [
                        [camera_name, '_left_camera_frame_optical'],
                        [camera_name, '_right_camera_frame_optical'],
                    ],
                    'publish_map_to_odom_tf': LaunchConfiguration('publish_map_to_odom_tf'),
                    'publish_odom_to_base_tf': LaunchConfiguration('publish_odom_to_base_tf')
                    }],
        remappings=[('visual_slam/image_0', zed_topics + ['left/gray/rect/image']),
                    ('visual_slam/camera_info_0', zed_topics + ['left/gray/rect/camera_info']),
                    ('visual_slam/image_1', zed_topics + ['right/gray/rect/image']),
                    ('visual_slam/camera_info_1', zed_topics + ['right/gray/rect/camera_info']),
                    ('visual_slam/imu', zed_topics + ['imu/data'])],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    """ISAAC_ROS_VISUAL_SLAM  <2a> default: load into the ZED container (shared process, IPC)"""
    load_into_zed_container = LoadComposableNodes(
        target_container=['/', camera_name, '/', LaunchConfiguration('zed_container_name')],
        composable_node_descriptions=[visual_slam_node],
        condition=UnlessCondition(LaunchConfiguration('standalone'))
    )

    """ISAAC_ROS_VISUAL_SLAM  <2b> standalone:=true -> own container (old behaviour)"""
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        condition=IfCondition(LaunchConfiguration('standalone')),
        output='screen'
    )

    "Temporary robot TF: base_footprint -> camera (until hal_description provides a URDF)"
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_camera_tf',
        arguments=['--x', LaunchConfiguration('camera_x'),
                   '--y', LaunchConfiguration('camera_y'),
                   '--z', LaunchConfiguration('camera_z'),
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', [camera_name, '_camera_link']],
        condition=IfCondition(LaunchConfiguration('publish_base_to_camera_tf')),
        output='screen'
    )

    "Teleportation solution v1.0"
    anti_teleport_node = Node(
        package='hal_vslam',
        executable='anti_teleport_node.py',
        name='anti_teleport_node',
        condition=IfCondition(LaunchConfiguration('enable_anti_teleport')),
        output='screen'
    )

    "SLAM -> OdomManager -> NAV2 passthrough with /odom_manager/switch"
    odom_manager_node = Node(
        package='hal_vslam',
        executable='odom_manager',
        name='odom_manager',
        condition=IfCondition(LaunchConfiguration('enable_odom_manager')),
        output='screen'
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='Must match camera_name of zed.launch.py (topic namespace and TF prefix)'
        ),
        DeclareLaunchArgument(
            'standalone',
            default_value='false',
            description='true: run cuVSLAM in its own component container; false: load into the ZED container (IPC, no DDS frame drops)'
        ),
        DeclareLaunchArgument(
            'zed_container_name',
            default_value='zed_container',
            description='Name of the container created by zed_camera.launch.py (used when standalone:=false)'
        ),
        DeclareLaunchArgument(
            'publish_map_to_odom_tf',
            default_value='true',
            description='Whether to publish the map to odom transform'
        ),
        DeclareLaunchArgument(
            'publish_odom_to_base_tf',
            default_value='true',
            description='Whether to publish the odom to base_footprint transform'
        ),
        DeclareLaunchArgument(
            'publish_base_to_camera_tf',
            default_value='true',
            description='Publish static base_footprint -> <camera_name>_camera_link TF (set false when a URDF provides it)'
        ),
        DeclareLaunchArgument('camera_x', default_value='0.0', description='Camera offset from base_footprint [m]'),
        DeclareLaunchArgument('camera_y', default_value='0.0', description='Camera offset from base_footprint [m]'),
        DeclareLaunchArgument('camera_z', default_value='0.0', description='Camera offset from base_footprint [m]'),
        DeclareLaunchArgument(
            'enable_anti_teleport',
            default_value='true',
            description='Start anti_teleport_node watchdog'
        ),
        DeclareLaunchArgument(
            'enable_odom_manager',
            default_value='true',
            description='Start odom_manager node'
        ),
        base_to_camera_tf,
        anti_teleport_node,
        odom_manager_node,
        load_into_zed_container,
        visual_slam_launch_container
    ])
