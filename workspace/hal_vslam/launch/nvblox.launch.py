from typing import List, Tuple

from launch import Action, LaunchDescription
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

"""
INFO:
- Launchfile that fires up 'isaac_ros_nvblox' only
- Compatible with Isaac ROS 5.0 / ROS 2 Lyrical (nvblox_ros 5.0.0)
- Configured for ZED 2i via zed-ros2-wrapper 5.x (start zed.launch.py with depth + visual_slam.launch.py first)

APPLICATION:
- Based on VIO calculated by VSLAM, calculates local_costmap, that is integrated with NAV2

DEVELOPMENT:
- Modified version of https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/release-3.2/nvblox_examples/nvblox_examples_bringup/launch/perception/nvblox.launch.py
- If you wanna develop it, plase check ^^^^^ for detailed information
- Changes vs release 3.2 version of this file:
    ZED 5.x topics: /zed/zed_node/depth/depth_registered, /zed/zed_node/left/color/rect/image (+ camera_info)
    config/nvblox_base.yaml: 4 params removed by nvblox 5.0 dropped (see comment in the yaml), rest unchanged
    nvblox 5.0 has pose_frame (default base_link); 'rover_2d' from the configs does not exist in the new stack yet
      -> launch arg robot_frame (default base_footprint) overrides pose_frame + the *_attachment_frame_id params
    launch arg camera_name (must match zed.launch.py)
- If you wanna chenge more advanced stuff check:
    TODO

ADDITIONAL:
- Be carefull! It can bite
"""


def get_remappings(mode: NvbloxMode, camera_name: str) -> List[Tuple[str, str]]:
    assert mode is NvbloxMode.static, 'Nvblox only supports static mode for ZED cameras.'
    zed = f'/{camera_name}/zed_node/'
    remappings = []
    """MAGICALLY STILL NOT WORKS WITH SIM..."""
    remappings.append(('camera_0/depth/image', zed + 'depth/depth_registered'))
    remappings.append(('camera_0/depth/camera_info', zed + 'depth/camera_info'))
    remappings.append(('camera_0/color/image', zed + 'left/color/rect/image'))
    remappings.append(('camera_0/color/camera_info', zed + 'left/color/rect/camera_info'))
    return remappings



def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:

    mode = NvbloxMode[args.mode]
    camera = NvbloxCamera[args.camera]

    base_config = lu.get_path('hal_vslam', 'config/nvblox_base.yaml')
    dynamics_config = lu.get_path('hal_vslam',
                                  'config/nvblox_dynamics.yaml')
    zed_config = lu.get_path('hal_vslam',
                             'config/nvblox_zed.yaml')

    if mode is NvbloxMode.static:
        mode_config = {}
    elif mode is NvbloxMode.dynamic:
        mode_config = dynamics_config
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')

    # ZED2 Camera

    remappings = get_remappings(mode, args.camera_name)
    camera_config = zed_config

    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(camera_config)

    # Param overrides

    parameters.append({'num_cameras': 1,
                       'use_lidar': False,
                       'print_rates_to_console': True,
                       # robot frame in the new stack (configs still say 'rover_2d')
                       'pose_frame': args.robot_frame,
                       'map_clearing_frame_id': args.robot_frame,
                       'esdf_slice_bounds_visualization_attachment_frame_id': args.robot_frame,
                       'workspace_height_bounds_visualization_attachment_frame_id': args.robot_frame,
                       'ground_plane_visualization_attachment_frame_id': args.robot_frame})

    # Add the nvblox node.
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions = []
    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info(
            ["Starting nvblox with the '",
             str(camera), "' camera in '",
             str(mode), "' mode."]))
    return actions



def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode', 'static')
    args.add_arg('camera', 'zed2')
    args.add_arg('camera_name', 'zed', description='Must match camera_name of zed.launch.py')
    args.add_arg('robot_frame', 'base_footprint', description='Robot frame for nvblox pose_frame / map clearing / visualization')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'True')
    args.add_arg('enable_mesh_to_pointcloud', 'True', description='Load hal_vslam mesh_to_pointcloud into the same container')

    args.add_opaque_function(add_nvblox)
    mesh_to_pointcloud = ComposableNode(
        package='hal_vslam',                          # package that contains the library
        plugin='hal_vslam::MeshToPointCloudComponent',  # fully-qualified C++ class
        name='mesh_to_pointcloud',
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    actions = args.get_launch_actions()
    actions.append(lu.load_composable_nodes(args.container_name, [mesh_to_pointcloud],
                                            condition=IfCondition(args.enable_mesh_to_pointcloud)))

    return LaunchDescription(actions)
