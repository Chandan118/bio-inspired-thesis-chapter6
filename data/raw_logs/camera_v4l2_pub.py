#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraV4L2Pub(Node):
    def __init__(self):
        super().__init__('camera_v4l2_pub')
        self.pub = self.create_publisher(Image, '/rgb/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open /dev/video0 with V4L2')
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraV4L2Pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
