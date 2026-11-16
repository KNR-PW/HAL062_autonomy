# Launch test (T1): description.launch.py -> robot_state_publisher publishes the full static
# TF chain.
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hal_description'), 'launch', 'description.launch.py')),
        launch_arguments={'simulation': 'false', 'camera_name': 'zed',
                          'camera_model': 'zed2i'}.items())
    return launch.LaunchDescription([
        description_launch,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestDescription(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_hal_description')
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.node, spin_thread=False)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_tf(self, target, source, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.buffer.can_transform(target, source, Time(), Duration(seconds=0)):
                return self.buffer.lookup_transform(target, source, Time())
        self.fail(f'TF {target} -> {source} not available after {timeout}s')

    def test_tf_chain(self):
        # rover chain
        self._wait_tf('base_footprint', 'wheel_link_RF')
        self._wait_tf('base_footprint', 'wheel_link_LR')
        # camera chain through the ZED macro
        tf = self._wait_tf('base_footprint', 'zed_camera_link')
        self.assertAlmostEqual(tf.transform.translation.x, 0.342, places=3)
        self.assertAlmostEqual(tf.transform.translation.z, 0.3564 + 0.1, places=3)
        self._wait_tf('base_footprint', 'zed_left_camera_frame_optical')
        self._wait_tf('base_footprint', 'zed_right_camera_frame_optical')
        # no legacy frames
        self.assertFalse(self.buffer.can_transform('base_footprint', 'camera_frame', Time(),
                                                   Duration(seconds=0)))

    def test_robot_description_latched(self):
        received = []
        qos = rclpy.qos.QoSProfile(depth=1,
                                   durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        sub = self.node.create_subscription(String, '/robot_description', received.append, qos)
        end = time.time() + 10.0
        while time.time() < end and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        self.assertTrue(received, '/robot_description not received (transient_local)')
        self.assertIn('<robot name="hal-062"', received[0].data)
        self.assertIn('zed_camera_link', received[0].data)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
