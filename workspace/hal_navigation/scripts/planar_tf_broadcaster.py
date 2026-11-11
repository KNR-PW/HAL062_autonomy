#!/usr/bin/env python3
"""
Planar (2D) TF frames for Nav2: map_2d, odom_2d, rover_2d.

planar_tf_broadcaster.py (ported unchanged from hal_main, lint-only edits)

Produces:
 - map_2d    (child of map)   : map shifted vertically to match rover footprint z
                                (roll/pitch eliminated)
 - odom_2d   (child of map)   : odom projected onto the rover footprint plane, yaw only
 - rover_2d  (child of map)   : robot base (footprint) with roll/pitch removed (yaw only)

Parameters (ROS2):
 - parent_frame        (string) default "map"
 - source_frame        (string) default "odom"       # frame to read odom pose from (map->odom)
 - robot_frame         (string) default "base_link"  # frame where footprint z is read
                                                     # (map->base_link)
 - map_2d_frame        (string) default "map_2d"
 - odom_2d_frame       (string) default "odom_2d"
 - rover_2d_frame      (string) default "rover_2d"
 - rate                (double) default 30.0
"""
import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import tf2_ros
from tf2_ros import StaticTransformBroadcaster


def euler_from_quaternion(x, y, z, w):
    rot = Rotation.from_quat([x, y, z, w])
    return rot.as_euler('xyz', degrees=False)  # roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
    return rot.as_quat()  # [x, y, z, w]


class PlanarTfBroadcaster(Node):

    def __init__(self):
        super().__init__('planar_tf_broadcaster')

        # params
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('source_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('map_2d_frame', 'map_2d')
        self.declare_parameter('odom_2d_frame', 'odom_2d')
        self.declare_parameter('rover_2d_frame', 'rover_2d')
        self.declare_parameter('rate', 60.0)

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.map_2d_frame = self.get_parameter('map_2d_frame').get_parameter_value().string_value
        self.odom_2d_frame = self.get_parameter('odom_2d_frame').get_parameter_value().string_value
        self.rover_2d_frame = \
            self.get_parameter('rover_2d_frame').get_parameter_value().string_value
        self.rate = float(self.get_parameter('rate').get_parameter_value().double_value)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Dynamic broadcaster (odom_2d, rover_2d)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Static broadcaster (map -> map_2d)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.last_footprint_z = None

        # timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(
            f'Planar TF broadcaster running: parent="{self.parent_frame}", '
            f'source="{self.source_frame}", robot="{self.robot_frame}", '
            f'map_2d="{self.map_2d_frame}", odom_2d="{self.odom_2d_frame}", '
            f'rover_2d="{self.rover_2d_frame}", '
            f'rate={self.rate}Hz'
        )

    def timer_callback(self):
        # We'll attempt to read two transforms:
        #  1) map -> source_frame  (e.g. map -> odom)
        #  2) map -> robot_frame   (e.g. map -> base_link)
        #     <-- used to extract footprint z and robot pose
        try:
            # use a zero time to request latest available transform
            now = rclpy.time.Time()
            trans_source = self.tf_buffer.lookup_transform(
                self.parent_frame, self.source_frame, now
            )
            trans_robot = self.tf_buffer.lookup_transform(
                self.parent_frame, self.robot_frame, now
            )
        except Exception as e:
            # Not fatal — TF might not be available yet; skip this cycle.
            self.get_logger().debug(f'lookup_transform failed: {e}')
            return

        # extract robot footprint z (in parent_frame coordinates)
        footprint_z = trans_robot.transform.translation.z

        # -------------------------
        # 1) publish map -> map_2d (STATIC TF)
        #    This transform is static in TF time, but we re-send it if footprint_z changes.
        # -------------------------

        if self.last_footprint_z is None or not math.isclose(
            footprint_z, self.last_footprint_z, rel_tol=1e-4
        ):
            t_map2d = TransformStamped()
            t_map2d.header.stamp = rclpy.time.Time().to_msg()  # ignored for static TF
            t_map2d.header.frame_id = self.parent_frame
            t_map2d.child_frame_id = self.map_2d_frame

            t_map2d.transform.translation.x = 0.0
            t_map2d.transform.translation.y = 0.0
            t_map2d.transform.translation.z = footprint_z

            q_map2d = quaternion_from_euler(0.0, 0.0, 0.0)
            t_map2d.transform.rotation.x = float(q_map2d[0])
            t_map2d.transform.rotation.y = float(q_map2d[1])
            t_map2d.transform.rotation.z = float(q_map2d[2])
            t_map2d.transform.rotation.w = float(q_map2d[3])

            self.static_tf_broadcaster.sendTransform(t_map2d)
            self.last_footprint_z = footprint_z

        # -------------------------
        # 2) publish map -> odom_2d
        #    Project the map->odom transform to the footprint plane and drop roll/pitch.
        # -------------------------
        tx_o = trans_source.transform.translation.x
        ty_o = trans_source.transform.translation.y
        # tz_o = trans_source.transform.translation.z  # original tz (ignored)
        q_o = trans_source.transform.rotation
        (roll_o, pitch_o, yaw_o) = euler_from_quaternion(q_o.x, q_o.y, q_o.z, q_o.w)

        t_odom2d = TransformStamped()
        t_odom2d.header.stamp = trans_source.header.stamp
        t_odom2d.header.frame_id = self.parent_frame
        t_odom2d.child_frame_id = self.odom_2d_frame

        t_odom2d.transform.translation.x = tx_o
        t_odom2d.transform.translation.y = ty_o
        t_odom2d.transform.translation.z = footprint_z

        q_odom2d = quaternion_from_euler(0.0, 0.0, yaw_o)  # yaw preserved, roll/pitch zeroed
        t_odom2d.transform.rotation.x = float(q_odom2d[0])
        t_odom2d.transform.rotation.y = float(q_odom2d[1])
        t_odom2d.transform.rotation.z = float(q_odom2d[2])
        t_odom2d.transform.rotation.w = float(q_odom2d[3])

        # -------------------------
        # 3) publish map -> rover_2d
        #    Project the map->robot (base_link) transform to the footprint plane and drop
        #    roll/pitch.
        # -------------------------
        tx_r = trans_robot.transform.translation.x
        ty_r = trans_robot.transform.translation.y
        # tz_r = trans_robot.transform.translation.z  # already used as footprint_z
        q_r = trans_robot.transform.rotation
        (roll_r, pitch_r, yaw_r) = euler_from_quaternion(q_r.x, q_r.y, q_r.z, q_r.w)

        t_rover2d = TransformStamped()
        t_rover2d.header.stamp = trans_robot.header.stamp
        t_rover2d.header.frame_id = self.parent_frame
        t_rover2d.child_frame_id = self.rover_2d_frame

        t_rover2d.transform.translation.x = tx_r
        t_rover2d.transform.translation.y = ty_r
        t_rover2d.transform.translation.z = footprint_z

        q_rover2d = quaternion_from_euler(0.0, 0.0, yaw_r)
        t_rover2d.transform.rotation.x = float(q_rover2d[0])
        t_rover2d.transform.rotation.y = float(q_rover2d[1])
        t_rover2d.transform.rotation.z = float(q_rover2d[2])
        t_rover2d.transform.rotation.w = float(q_rover2d[3])

        # send transforms
        try:
            # self.tf_broadcaster.sendTransform(t_map2d)
            self.tf_broadcaster.sendTransform(t_odom2d)
            self.tf_broadcaster.sendTransform(t_rover2d)
        except Exception as e:
            self.get_logger().warning(f'Failed sending planar transforms: {e}')
            return


def main(args=None):
    rclpy.init(args=args)
    node = PlanarTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
