# Launch test (T3.2/T3.3 in docs/NAV2_DESCRIPTION_PORT_PLAN.md): the full Nav2 stack from
# navigation.launch.py + hal_description, with fake localization (stands in for cuVSLAM) and a fake
# ZED point cloud. No hardware. Needs ros-lyrical-navigation2 in the image.
import math
import os
import sys
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
import launch
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Path
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

TEST_DIR = os.path.dirname(__file__)
LIFECYCLE_NODES = ['map_server', 'controller_server', 'smoother_server', 'planner_server',
                   'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']
BOX_X = 4.0
GOAL_X = 2.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hal_description'), 'launch', 'description.launch.py')))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hal_navigation'), 'launch', 'navigation.launch.py')),
        launch_arguments={'location': 'blank', 'rviz': 'false'}.items())
    fake_localization = ExecuteProcess(
        cmd=[sys.executable, os.path.join(TEST_DIR, 'fake_localization.py')], output='screen')
    fake_cloud = ExecuteProcess(
        cmd=[sys.executable, os.path.join(TEST_DIR, 'fake_cloud.py'),
             '--ros-args', '-p', f'box_x:={BOX_X}'], output='screen')
    return launch.LaunchDescription([
        description,
        fake_localization,
        fake_cloud,
        navigation,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestNavigation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_hal_navigation')
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.node, spin_thread=False)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ---------- helpers ----------
    def spin_until(self, predicate, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(f'timeout ({timeout}s) waiting for {what}')

    def call(self, srv_type, name, request, timeout=5.0):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout), f'service {name} not available')
        future = client.call_async(request)
        self.spin_until(future.done, timeout, f'{name} response')
        self.node.destroy_client(client)
        return future.result()

    def latest(self, msg_type, topic, timeout, qos=None, predicate=lambda m: True):
        got = []
        sub = self.node.create_subscription(msg_type, topic, got.append, qos or 10)
        try:
            self.spin_until(lambda: any(predicate(m) for m in got), timeout, f'{topic}')
        finally:
            self.node.destroy_subscription(sub)
        return [m for m in got if predicate(m)][-1]

    # ---------- tests (run in alphabetical order) ----------
    def test_01_lifecycle_active(self):
        # lifecycle manager reports active, every node is in state 'active'
        resp = self.call(Trigger, '/lifecycle_manager_navigation/is_active', Trigger.Request(),
                         timeout=90.0)
        self.assertTrue(resp.success, 'lifecycle_manager_navigation not active')
        for name in LIFECYCLE_NODES:
            state = self.call(GetState, f'/{name}/get_state', GetState.Request())
            self.assertEqual(state.current_state.label, 'active', name)

    def test_02_planar_frames(self):
        for frame in ('map_2d', 'odom_2d', 'rover_2d'):
            self.spin_until(lambda: self.buffer.can_transform('map', frame, Time(),
                                                              Duration(seconds=0)),
                            20.0, f'TF map -> {frame}')
        tf = self.buffer.lookup_transform('map_2d', 'rover_2d', Time())
        self.assertAlmostEqual(tf.transform.translation.z, 0.0, places=3)

    def test_03_costmaps_publish_and_mark_obstacle(self):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # global: always_send_full_costmap is False -> OccupancyGrid only on size changes,
        # costmap_raw is published whenever someone subscribes
        raw = self.latest(Costmap, '/global_costmap/costmap_raw', 30.0, qos)
        self.assertEqual(raw.metadata.size_x, 1024)

        def box_is_lethal(grid):
            # count LETHAL (100) cells inside the box footprint [BOX_X, BOX_X+0.5] x [-0.25, 0.25]
            res = grid.info.resolution
            ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
            i0, i1 = int((BOX_X - ox) / res), int((BOX_X + 0.5 - ox) / res)
            j0, j1 = int((-0.25 - oy) / res), int((0.25 - oy) / res)
            if not (0 <= i0 and i1 < grid.info.width and 0 <= j0 and j1 < grid.info.height):
                return False
            lethal = sum(1 for j in range(j0, j1 + 1) for i in range(i0, i1 + 1)
                         if grid.data[j * grid.info.width + i] == 100)
            return lethal >= 20

        def ground_is_free(grid):
            res = grid.info.resolution
            ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
            i = int((1.5 - ox) / res)
            j = int((0.0 - oy) / res)
            return grid.data[j * grid.info.width + i] == 0

        grid = self.latest(OccupancyGrid, '/local_costmap/costmap', 40.0, qos,
                           predicate=lambda g: box_is_lethal(g))
        self.assertTrue(ground_is_free(grid), 'flat ground 1.5 m ahead should be free')

    def test_04_navigate_to_pose_moves_robot_with_twist(self):
        cmd = []
        twist_sub = self.node.create_subscription(Twist, '/cmd_vel', cmd.append, 10)
        plans = []
        plan_sub = self.node.create_subscription(Path, '/plan', plans.append, 10)

        client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.assertTrue(client.wait_for_server(20.0), 'navigate_to_pose server not available')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = GOAL_X
        goal.pose.pose.orientation.w = 1.0
        send = client.send_goal_async(goal)
        self.spin_until(send.done, 10.0, 'goal acceptance')
        handle = send.result()
        self.assertTrue(handle.accepted)

        self.spin_until(lambda: any(m.linear.x > 0.02 for m in cmd), 20.0, 'forward /cmd_vel')
        self.assertTrue(plans and len(plans[-1].poses) > 2, '/plan should be non-empty')

        result = handle.get_result_async()
        self.spin_until(result.done, 90.0, 'navigation result')
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)

        tf = self.buffer.lookup_transform('map', 'base_footprint', Time())
        dist = math.hypot(tf.transform.translation.x - GOAL_X, tf.transform.translation.y)
        self.assertLess(dist, 0.6, f'robot {dist:.2f} m from goal (xy_goal_tolerance 0.5)')
        self.node.destroy_subscription(twist_sub)
        self.node.destroy_subscription(plan_sub)
        client.destroy()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
