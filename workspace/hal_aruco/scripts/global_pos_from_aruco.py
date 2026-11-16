#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from isaac_ros_visual_slam_interfaces.srv import SetSlamPose
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from hal_interfaces.msg import ArucoPose
except ImportError:
    from hal_aruco.msg import ArucoPose


# Znane pozycje globalne znacznikow w ukladzie mapy: {id: [x, y, z]}
GLOBAL_POSITIONS = {
    0: [3.8, -8.0, 0.7],
    1: [3.3, -2.4, 0.6],
    2: [2.2, -22.6, 1.2],
}


class GlobalPositionFromAruco(Node):
    """
    Wezel ROS 2 obliczajacy pozycje obserwatora/lazika na podstawie
    detekcji znacznikow ArUco i ich znanych pozycji globalnych.
    """

    def __init__(self):
        super().__init__('global_pos_from_aruco')

        self.declare_parameter('aruco_topic', '/hal_aruco/aruco_pose')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'zed_left_camera_optical_frame')
        self.declare_parameter('slam_service_name', 'visual_slam/set_slam_pose')
        self.declare_parameter('enable_set_slam_pose', True)
        self.declare_parameter('slam_update_period_sec', 1.0)

        aruco_topic = self.get_parameter('aruco_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        slam_service_name = self.get_parameter('slam_service_name').get_parameter_value().string_value
        self.enable_set_slam_pose = (
            self.get_parameter('enable_set_slam_pose').get_parameter_value().bool_value
        )
        self.slam_update_period_sec = (
            self.get_parameter('slam_update_period_sec').get_parameter_value().double_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.aruco_subscription = self.create_subscription(
            ArucoPose,
            aruco_topic,
            self.aruco_pose_callback,
            10,
        )

        self.aruco_pose_publisher = self.create_publisher(
            PoseStamped,
            '/aruco_pose_rviz',
            10,
        )

        self.position_publisher = self.create_publisher(
            PointStamped,
            '/aruco_rover_position',
            10,
        )

        self.rover_orientation = None
        self.cli = self.create_client(SetSlamPose, slam_service_name)
        self.last_time = self.get_clock().now()

        self.get_logger().info('Global Position from ArUco node started')
        self.get_logger().info(f'Subscribing to: {aruco_topic}')
        self.get_logger().info(f'TF lookup: {self.target_frame} <- {self.source_frame}')
        self.get_logger().info(f'Available marker global positions: {list(GLOBAL_POSITIONS.keys())}')

    def quat_to_rot_matrix(self, q):
        x, y, z, w = q
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ])
        return R

    def calc_observer_position(self, global_marker_pos, observer_quat, marker_pos_rel):
        R_obs = self.quat_to_rot_matrix(observer_quat)

        camera_to_map_matrix = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ])
        p_rel_camera = np.array(marker_pos_rel).reshape((3, 1))
        p_rel = camera_to_map_matrix @ p_rel_camera

        p_global = np.array(global_marker_pos).reshape((3, 1))

        p_obs = p_global - R_obs @ p_rel
        return p_obs.flatten()

    def aruco_pose_callback(self, msg):
        self.get_logger().info(f'Pose for rviz: {msg.pose_stamped}')
        self.aruco_pose_publisher.publish(msg.pose_stamped)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, Time()
            )
        except TransformException as e:
            self.get_logger().warning(f'TF lookup failed: {str(e)}')
            return

        self.rover_orientation = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]

        marker_id = msg.id

        if marker_id not in GLOBAL_POSITIONS:
            self.get_logger().warning(f'No global position data for marker ID {marker_id}')
            return

        marker_global_pos = GLOBAL_POSITIONS[marker_id]

        rover_to_marker_local = [
            msg.pose_stamped.pose.position.x,
            msg.pose_stamped.pose.position.y,
            msg.pose_stamped.pose.position.z,
        ]

        rover_pos = self.calc_observer_position(
            global_marker_pos=marker_global_pos,
            observer_quat=self.rover_orientation,
            marker_pos_rel=rover_to_marker_local,
        )

        self.publish_rover_position(rover_pos, msg.pose_stamped.header.stamp)

        current_time = self.get_clock().now()
        delta = current_time - self.last_time
        if self.enable_set_slam_pose and (delta.nanoseconds / 1e9) > self.slam_update_period_sec:
            self.last_time = current_time
            self.set_slam_position(rover_pos)

        self.get_logger().info(
            f'Marker {marker_id}: Global pos {marker_global_pos}, '
            f'Rover-to-marker (local) {rover_to_marker_local}, '
            f'Rover global pos {rover_pos}'
        )

    def publish_rover_position(self, position, timestamp):
        msg = PointStamped()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = timestamp

        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = float(position[2])

        self.position_publisher.publish(msg)

    def set_slam_position(self, position):
        if self.rover_orientation is None:
            return
        if not self.cli.service_is_ready():
            self.get_logger().warning('SetSlamPose service not ready, skipping call')
            return

        req = SetSlamPose.Request()
        req.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        req.pose.orientation.x = float(self.rover_orientation[0])
        req.pose.orientation.y = float(self.rover_orientation[1])
        req.pose.orientation.z = float(self.rover_orientation[2])
        req.pose.orientation.w = float(self.rover_orientation[3])

        self.get_logger().info('AR to slam call')
        future = self.cli.call_async(req)
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            _ = future.result()
            self.get_logger().info('AR set the position')
        except Exception as e:
            self.get_logger().warning(f'SetSlamPose call failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    global_pos_node = GlobalPositionFromAruco()

    try:
        rclpy.spin(global_pos_node)
    except KeyboardInterrupt:
        global_pos_node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        global_pos_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
