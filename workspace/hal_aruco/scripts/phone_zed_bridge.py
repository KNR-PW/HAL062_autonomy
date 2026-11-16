#!/usr/bin/env python3
"""
Pomocniczy wezel testowy dla hal_aruco.
Odbiera strumien wideo z telefonu (IP Webcam) i publikuje go jako obraz bgra8 (1280x720)
na topiku kamery ZED 2i (/zed/zed_node/left/color/rect/image).

Uzycie:
  python3 phone_zed_bridge.py http://<IP_TELEFONU>:8080/video
"""
import sys
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class PhoneZedBridge(Node):
    def __init__(self, stream_url):
        super().__init__('phone_zed_bridge')
        self.pub = self.create_publisher(
            Image, '/zed/zed_node/left/color/rect/image', qos_profile_sensor_data)
        self.cap = cv2.VideoCapture(stream_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error(f'Nie mozna otworzyc strumienia: {stream_url}')
            sys.exit(1)

        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True

        # Osobny watek czytajacy klatki na biezaco (zapobiega buforowaniu i zawieszaniu)
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(f'Polaczono z kamera: {stream_url}')
        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)

    def _reader_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame

    def timer_cb(self):
        with self.lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame
            self.latest_frame = None

        frame = cv2.resize(frame, (1280, 720))
        bgra = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'zed_left_camera_optical_frame'
        msg.height = 720
        msg.width = 1280
        msg.encoding = 'bgra8'
        msg.is_bigendian = 0
        msg.step = 1280 * 4
        msg.data = bgra.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    url = sys.argv[1] if len(sys.argv) > 1 else 'http://10.55.66.15:8080/video'
    node = PhoneZedBridge(url)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
