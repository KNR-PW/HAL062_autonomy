#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from hal_interfaces.srv import Communication


# TODO precise measurements
WHEEL_RAD = 0.1575
ROVER_WIDTH = 0.91


class MessageSender(Node):
    def __init__(self):
        super().__init__('message_sender')
        self.init_light()
        self.init_engine()
        # rclpy.spin_once(self, timeout_sec=2)
        self.init_light()
        self.init_engine()
        self.create_subscription(Twist, '/cmd_vel', self.spd_callback, 10)

    def init_light(self):
        self.get_logger().info('Initializing light...')
        self.call_service(Communication, '/comm_server', 0, 50, [1 & 0xff, 0 & 0xff, 0 & 0xff, 1 & 0xff])

    def init_engine(self):
        self.get_logger().info('Initializing engine...')
        self.call_service(Communication, '/comm_server', 0, 22, [1 & 0xff, 1 & 0xff, 1 & 0xff])

    def spd_callback(self, msg):
        spd_lin = msg.linear.x  # m/s
        spd_ang = msg.angular.z  # rad/s
        spd_l = (spd_lin - spd_ang * ROVER_WIDTH / 2 * 2) / WHEEL_RAD  # rad/s
        spd_r = (spd_lin + spd_ang * ROVER_WIDTH / 2 * 2) / WHEEL_RAD  # rad/s
        spd_mb_unit_l = round(spd_l / (2 * math.pi) * 100)
        spd_mb_unit_r = round(spd_r / (2 * math.pi) * 100)

        if abs(spd_mb_unit_l) <= 1 and abs(spd_mb_unit_r) > 1:
            spd_mb_unit_l = -2
        if abs(spd_mb_unit_r) <= 1 and abs(spd_mb_unit_l) > 1:
            spd_mb_unit_r = -2

        self.get_logger().info(f'Sending speed L: {spd_mb_unit_l}, R: {spd_mb_unit_r}')
        self.call_service(Communication, '/comm_server', 0, 20, [spd_mb_unit_l & 0xff, spd_mb_unit_r & 0xff])

    def call_service(self, srv_type, service_name, comm_mode, req_id, data):
        client = self.create_client(srv_type, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'Service {service_name} not available, waiting...')

        request = srv_type.Request()
        request.comm_mode = comm_mode
        request.id = req_id
        request.data = data

        future = client.call_async(request) # (memory leak risk)

        # NIE DOSTAJEMY RESPONSE OD RASPBERRY, dlatego nie czekamy
        # czekanie powoowało blokowanie na tym oc jest zakomentowane poniżej
        # rclpy.spin_until_future_complete(self, future)
        # if future.result():
        #     self.get_logger().info(f'Service response: {future.result()}')
        # else:
        #     self.get_logger().error(f'Service call failed: {future.exception()}')


def main(args=None):
    rclpy.init(args=args)
    node = MessageSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
