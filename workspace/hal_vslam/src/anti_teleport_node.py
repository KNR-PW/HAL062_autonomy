#!/usr/bin/env python3
import rclpy
import math
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Point, Quaternion
from hal_interfaces.msg import ArucoPose

from isaac_ros_visual_slam_interfaces.srv import SetSlamPose

import tf2_ros
from scipy.spatial.transform import Rotation
import numpy as np
import threading
from rclpy.clock import Clock


class AntiTeleportNode(LifecycleNode):
    """
    Watchdog pilnujacy, zeby VSLAM nie 'teleportowal' rovera.

    Wezel jest lifecycle-owy, bo automata (hal_automata) musi go uciszyc na czas
    sekwencji ResetSLAM: reset VSLAM-u + set_slam_pose to z definicji skok pozy,
    ktory watchdog uznalby za teleportacje i natychmiast probowalby 'naprawic'.

        unconfigured -> configure -> inactive -> activate -> active
        active -> deactivate -> inactive   (watchdog spi, korekty wylaczone)

    Historia pozycji/orientacji jest czyszczona przy KAZDEJ aktywacji, wiec po
    powrocie do 'active' wezel liczy predkosc od nowej pozy, a nie od tej sprzed resetu.
    """

    def __init__(self):
        super().__init__('anti_teleport_node')

        # ============================================================
        # Parameters
        # ============================================================

        # Parametry Anti-Teleport (Pozycja)
        self.declare_parameter('max_physical_speed', 1.0)       # meters/second
        self.declare_parameter('aruco_timeout', 2.0)            # seconds

        # Parametry Orientacji (IMU Watchdog)
        self.declare_parameter('orientation_threshold', 0.35)   # radians
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('init_position', [0.0, 0.0, 0.0, 0.0]) # [x, y, z, yaw] -> z tego brane jest w sumie yaw tylko. Niestety nie rozumiem po co jest brane to yaw.

        self.declare_parameter('roll_error_threshold', 0.1396)    # rad, ok. 8 deg
        self.declare_parameter('pitch_error_threshold', 0.1396)   # rad, ok. 8 deg
        self.declare_parameter('yaw_error_threshold', 0.35)       # rad, ok. 20 deg

        self.declare_parameter('max_vslam_roll_step', 0.20)       # rad, ok. 11.5 deg/cykl
        self.declare_parameter('max_vslam_pitch_step', 0.20)      # rad, ok. 11.5 deg/cykl
        self.declare_parameter('max_vslam_yaw_step', 0.35)        # rad, ok. 20 deg/cykl

        self.declare_parameter('magnetic_declination', 0.111)     # rad, do korekcji magnetometru

        # ============================================================
        # Uchwyty tworzone dopiero w on_configure()
        # ============================================================
        self.imu_subscriber = None
        self.mag_subscriber = None
        self.aruco_subscriber = None
        self.odom_client = None
        self.timer = None
        self.tf_buffer = None
        self.tf_listener = None

        # ============================================================
        # State
        # ============================================================
        self.active = False
        self.lock = threading.Lock()
        self.clock = Clock()

        self.latest_imu = None
        self.latest_imu_time = None

        # Kalibracja magnetometru - celowo NIE jest kasowana przy deactivate/activate,
        # reset SLAM-u nie zmienia relacji miedzy magnetometrem a frame'em map.
        self.mag_initial_yaw = None
        self.yaw_offset = 0.0
        self.offset_quat = self.yaw_to_quaternion(0.0)

        # Stan Anti-Teleport (ustawiany w resetWatchdogState)
        self.last_aruco_time = self.get_clock().now()
        self.last_good_position = None
        self.last_good_time = None
        self.correction_in_progress = False
        self.orientation_correction = False
        self.prev_vslam_rpy = None
        self.last_teleportation_time = self.get_clock().now()
        self.last_teleportation_threshold = 3.0

        self.get_logger().info('anti_teleport_node created (unconfigured)')

    # ============================================================
    # Lifecycle transitions
    # ============================================================
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring anti_teleport_node...')

        self.loadParameters()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.imu_callback,
            10
        )

        self.mag_subscriber = self.create_subscription(
            MagneticField,
            '/zed/zed_node/imu/mag',
            self.mag_callback,
            10
        )

        # Nowa subskrypcja na tagi AR
        self.aruco_subscriber = self.create_subscription(
            ArucoPose,
            '/hal_aruco/aruco_pose',
            self.aruco_callback,
            10
        )

        self.odom_client = self.create_client(
            SetSlamPose,
            '/visual_slam/set_slam_pose'
        )

        self.resetWatchdogState()

        # Timer zyje przez caly czas, ale chodzi tylko w stanie 'active'.
        self.timer = self.create_timer(0.1, self.watchdog)
        self.timer.cancel()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating anti_teleport_node - watchdog ON')

        self.resetWatchdogState()
        self.active = True
        self.timer.reset()

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating anti_teleport_node - watchdog OFF, korekty VSLAM wstrzymane')

        self.active = False
        self.timer.cancel()

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up anti_teleport_node...')
        self.destroyRuntimeEntities()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down anti_teleport_node...')
        self.destroyRuntimeEntities()
        return TransitionCallbackReturn.SUCCESS

    def loadParameters(self):
        self.max_physical_speed = self.get_parameter('max_physical_speed').value
        self.aruco_timeout = self.get_parameter('aruco_timeout').value

        self.orientation_threshold = self.get_parameter('orientation_threshold').value
        self.roll_error_threshold = self.get_parameter('roll_error_threshold').value
        self.pitch_error_threshold = self.get_parameter('pitch_error_threshold').value
        self.yaw_error_threshold = self.get_parameter('yaw_error_threshold').value

        self.max_vslam_roll_step = self.get_parameter('max_vslam_roll_step').value
        self.max_vslam_pitch_step = self.get_parameter('max_vslam_pitch_step').value
        self.max_vslam_yaw_step = self.get_parameter('max_vslam_yaw_step').value

        pos = self.get_parameter('init_position').value
        self.init_position = (float(pos[0]), float(pos[1]), float(pos[2]), float(pos[3]))

        self.parent_frame = self.get_parameter('parent_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        if self.mag_initial_yaw is None:
            self.yaw_offset = self.init_position[3]
            self.offset_quat = self.yaw_to_quaternion(self.yaw_offset)

    def resetWatchdogState(self):
        """Czysci historie watchdoga - wolane przy configure i przy kazdej aktywacji."""
        with self.lock:
            self.last_aruco_time = self.get_clock().now()
            self.last_good_position = None
            self.last_good_time = None
            self.correction_in_progress = False
            self.orientation_correction = False
            self.prev_vslam_rpy = None
            # Odsuwa w czasie sprawdzanie orientacji, zeby TF zdazyl sie ustabilizowac.
            self.last_teleportation_time = self.get_clock().now()

    def destroyRuntimeEntities(self):
        self.active = False

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        for subscriber in (self.imu_subscriber, self.mag_subscriber, self.aruco_subscriber):
            if subscriber is not None:
                self.destroy_subscription(subscriber)
        self.imu_subscriber = None
        self.mag_subscriber = None
        self.aruco_subscriber = None

        if self.odom_client is not None:
            self.destroy_client(self.odom_client)
            self.odom_client = None

        self.tf_listener = None
        self.tf_buffer = None

    # ============================================================
    # Callbacks
    # ============================================================
    def aruco_callback(self, msg: ArucoPose):
        self.last_aruco_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        self.latest_imu_time = self.get_clock().now()

    def mag_callback(self, msg: MagneticField):
        with self.lock:
            if self.latest_imu is None:
                return

            _, _, zed_yaw_raw = self.quat_to_euler(self.latest_imu.orientation)
            mag_heading = math.atan2(msg.magnetic_field.x, msg.magnetic_field.y)

            mag_yaw_in_map = self.angle_diff(
                mag_heading + self.init_position[3] - self.get_parameter('magnetic_declination').value,
                0.0
            )

            target_yaw_offset = self.angle_diff(mag_yaw_in_map, zed_yaw_raw)

            if self.mag_initial_yaw is None:
                self.mag_initial_yaw = mag_heading
                self.yaw_offset = target_yaw_offset

            self.offset_quat = self.yaw_to_quaternion(self.yaw_offset)

    # ============================================================
    # Quaternion / angle helpers
    # ============================================================
    def yaw_to_quaternion(self, yaw: float):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def angle_diff(self, a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def quat_norm(self, q):
        if hasattr(q, 'w'):
            q_arr = np.array([q.x, q.y, q.z, q.w], dtype=float)
        else:
            q_arr = np.asarray(q, dtype=float)

        norm = np.linalg.norm(q_arr)
        if norm < 1e-9:
            return np.array([0.0, 0.0, 0.0, 1.0])
        return q_arr / norm

    def quat_angle_error(self, q1, q2):
        q1_norm = self.quat_norm(q1)
        q2_norm = self.quat_norm(q2)
        dot_product = np.dot(q1_norm, q2_norm)
        dot_product = np.clip(dot_product, -1.0, 1.0)
        return 2.0 * math.acos(abs(dot_product))

    def quat_multiply(self, q1, q2):
        r1 = Rotation.from_quat(self.quat_norm(q1))
        r2 = Rotation.from_quat(self.quat_norm(q2))
        return self.quat_norm((r1 * r2).as_quat())

    def quat_to_euler(self, q):
        if hasattr(q, 'w'):
            x, y, z, w = q.x, q.y, q.z, q.w
        else:
            q_norm = self.quat_norm(q)
            x, y, z, w = q_norm[0], q_norm[1], q_norm[2], q_norm[3]

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # ============================================================
    # VSLAM orientation step detection
    # ============================================================
    def check_vslam_orientation_step(self, vslam_q):
        vslam_roll, vslam_pitch, vslam_yaw = self.quat_to_euler(vslam_q)

        if self.prev_vslam_rpy is None:
            self.prev_vslam_rpy = (vslam_roll, vslam_pitch, vslam_yaw)
            return False, 0.0, 0.0, 0.0

        prev_roll, prev_pitch, prev_yaw = self.prev_vslam_rpy

        roll_step = self.angle_diff(vslam_roll, prev_roll)
        pitch_step = self.angle_diff(vslam_pitch, prev_pitch)
        yaw_step = self.angle_diff(vslam_yaw, prev_yaw)

        self.prev_vslam_rpy = (vslam_roll, vslam_pitch, vslam_yaw)

        vslam_step_problem = (
            abs(roll_step) > self.max_vslam_roll_step or
            abs(pitch_step) > self.max_vslam_pitch_step or
            abs(yaw_step) > self.max_vslam_yaw_step
        )

        return vslam_step_problem, roll_step, pitch_step, yaw_step

    # ============================================================
    # TF helper
    # ============================================================
    def get_robot_pose_in_parent(self):
        try:
            now = rclpy.time.Time()
            return self.tf_buffer.lookup_transform(self.parent_frame, self.robot_frame, now)
        except Exception as e:
            self.get_logger().debug(f'lookup_transform failed: {e}')
            return None

    # ============================================================
    # Main watchdog
    # ============================================================
    def watchdog(self):
        # lifecyce node active check
        if not self.active:
            return

        now = self.get_clock().now()
        # 1. Pobieranie pozycji VSLAM z TF
        trans_robot = self.get_robot_pose_in_parent()
        if trans_robot is None:
            return

        vslam_x = trans_robot.transform.translation.x
        vslam_y = trans_robot.transform.translation.y
        vslam_z = trans_robot.transform.translation.z
        vslam_q = trans_robot.transform.rotation

        # 2. Weryfikacja Pozycji
        position_problem = False
        current_speed = 0.0

        if self.last_good_position is None:
            self.last_good_position = trans_robot.transform.translation
            self.last_good_time = now
        else:
            dt = (now - self.last_good_time).nanoseconds * 1e-9
            if dt > 0:
                dx = vslam_x - self.last_good_position.x
                dy = vslam_y - self.last_good_position.y
                dz = vslam_z - self.last_good_position.z

                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                current_speed = distance / dt

                time_since_aruco = (now - self.last_aruco_time).nanoseconds * 1e-9
                aruco_visible = (time_since_aruco <= self.aruco_timeout)

                if current_speed > self.max_physical_speed and not aruco_visible:
                    position_problem = True
                    self.get_logger().warn(f"TELEPORTACJA WYKRYTA! Prędkość: {current_speed:.2f} m/s.")
                else:
                    # Wszystko OK, aktualizujemy historię
                    self.last_good_position = trans_robot.transform.translation
                    self.last_good_time = now

        if (self.get_clock().now() - self.last_teleportation_time).nanoseconds * 1e-9 > self.last_teleportation_threshold:

            # 3. Weryfikacja Orientacji
            orientation_problem = False
            if self.latest_imu is None:
                self.get_logger().warn("No IMU data yet, skipping orientation check.")
            else:
                zed_q_raw = self.latest_imu.orientation
                zed_q_arr = self.quat_multiply(self.offset_quat, zed_q_raw)
                zed_q = Quaternion(x=float(zed_q_arr[0]), y=float(zed_q_arr[1]), z=float(zed_q_arr[2]), w=float(zed_q_arr[3]))

                orientation_error = self.quat_angle_error(zed_q, vslam_q)

                zed_roll, zed_pitch, zed_yaw = self.quat_to_euler(zed_q)
                vslam_roll, vslam_pitch, vslam_yaw = self.quat_to_euler(vslam_q)

                roll_error = self.angle_diff(zed_roll, vslam_roll)
                pitch_error = self.angle_diff(zed_pitch, vslam_pitch)
                yaw_error = self.angle_diff(zed_yaw, vslam_yaw)

                rpy_error_problem = (
                    abs(roll_error) > self.roll_error_threshold or
                    abs(pitch_error) > self.pitch_error_threshold or
                    abs(yaw_error) > self.yaw_error_threshold
                )

                vslam_step_problem, roll_step, pitch_step, yaw_step = self.check_vslam_orientation_step(vslam_q)

                orientation_problem = (orientation_error > self.orientation_threshold or rpy_error_problem or vslam_step_problem)

            # 4. Decyzja o Korekcie
            if not position_problem and not orientation_problem:
                return

            if self.correction_in_progress:
                self.get_logger().warn("Correction already in progress!")
                return

            if not self.odom_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("SetSlamPose service not available!")
                return

            req = SetSlamPose.Request()
            self.last_teleportation_time = self.get_clock().now()

            # Uzupełnianie Pozycji
            if position_problem:
                req.pose.position.x = float(self.last_good_position.x)
                req.pose.position.y = float(self.last_good_position.y)
                req.pose.position.z = float(self.last_good_position.z)
            else:
                req.pose.position.x = float(vslam_x)
                req.pose.position.y = float(vslam_y)
                req.pose.position.z = float(vslam_z)

            # Uzupełnianie Orientacji
            if orientation_problem and not self.orientation_correction:
                req.pose.orientation = zed_q
                self.orientation_correction = True
            else:
                req.pose.orientation = vslam_q
                self.orientation_correction = False

            self.correction_in_progress = True
            self.get_logger().info(f"Triggering VSLAM correction: {req.pose.position}, {req.pose.orientation}")
            future = self.odom_client.call_async(req)
            future.add_done_callback(self.correction_done_callback)

    # ============================================================
    # Correction callback
    # ============================================================
    def correction_done_callback(self, future):
        self.correction_in_progress = False

        # Reset historii aby własna korekta nie wywołała kaskady błędów na starcie
        self.prev_vslam_rpy = None
        self.last_good_position = None  # Wymusi ponowną inicjalizację pozycji po skoku

        try:
            response = future.result()
            if response.success:
                self.get_logger().info("VSLAM correction successful.")
            else:
                self.get_logger().error("VSLAM correction failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AntiTeleportNode()

    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
