#!/usr/bin/env python3
# Test fixture: stands in for cuVSLAM. Publishes TF map -> odom (identity) -> base_footprint and
# nav_msgs/Odometry on /visual_slam/tracking/odometry, integrating geometry_msgs/Twist from
#   /cmd_vel
# so Nav2 sees the "robot" move. Not installed; used by test_navigation_launch.py only.
import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeLocalization(Node):

    def __init__(self):
        super().__init__('fake_localization')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('odom_topic', '/visual_slam/tracking/odometry')
        self.x = self.y = self.yaw = 0.0
        self.cmd = Twist()
        self.last_cmd = self.get_clock().now()
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        rate = self.get_parameter('rate').value
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)

    def cmd_cb(self, msg):
        self.cmd = msg
        self.last_cmd = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds > 0.5e9:
            self.cmd = Twist()
        v, w = self.cmd.linear.x, self.cmd.angular.z
        self.yaw += w * self.dt
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        stamp = now.to_msg()
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)

        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = stamp
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.rotation.w = 1.0

        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = stamp
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_footprint'
        t_odom_base.transform.translation.x = self.x
        t_odom_base.transform.translation.y = self.y
        t_odom_base.transform.rotation.z = qz
        t_odom_base.transform.rotation.w = qw
        self.tf.sendTransform([t_map_odom, t_odom_base])

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = FakeLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
