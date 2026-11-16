#!/usr/bin/env python3
# Test fixture: stands in for the ZED point cloud. Publishes a PointCloud2 in the LEFT OPTICAL
#   frame
# (so the real hal_description TF chain is exercised) containing flat ground in front of the rover
#   and
# one box obstacle, both defined in base_footprint coordinates and transformed with the live TF.
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FakeCloud(Node):

    def __init__(self):
        super().__init__('fake_cloud')
        self.declare_parameter('topic', '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('camera_frame', 'zed_left_camera_frame_optical')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('box_x', 4.0)
        self.declare_parameter('box_size', 0.5)
        self.declare_parameter('rate', 10.0)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.points_world = self.build_points()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('topic').value, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.publish)

    def build_points(self):
        # flat ground around the start, 10 cm grid, z = 0 (ground plane of the world frame)
        xs, ys = np.meshgrid(np.arange(-4.0, 10.0, 0.1), np.arange(-5.0, 5.0, 0.1))
        ground = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1)
        # box: front face + top, 0..box_size high, centred on y=0
        bx = self.get_parameter('box_x').value
        s = self.get_parameter('box_size').value
        fy, fz = np.meshgrid(np.arange(-s / 2, s / 2, 0.05), np.arange(0.0, s, 0.05))
        front = np.stack([np.full(fy.size, bx), fy.ravel(), fz.ravel()], axis=1)
        tx, ty = np.meshgrid(np.arange(bx, bx + s, 0.05), np.arange(-s / 2, s / 2, 0.05))
        top = np.stack([tx.ravel(), ty.ravel(), np.full(tx.size, s)], axis=1)
        return np.vstack([ground, front, top]).astype(np.float32)

    def publish(self):
        try:
            tf = self.buffer.lookup_transform(self.camera_frame, self.world_frame, Time(),
                                              Duration(seconds=0.2))
        except Exception as e:  # noqa: B902 - TF not up yet
            self.get_logger().info(f'waiting for TF: {e}', throttle_duration_sec=2.0)
            return
        q = tf.transform.rotation
        t = tf.transform.translation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        pts = self.points_world @ rot.T + np.array([t.x, t.y, t.z], dtype=np.float32)
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.camera_frame)
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
        self.pub.publish(point_cloud2.create_cloud(header, fields, pts.tolist()))


def main():
    rclpy.init()
    node = FakeCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
