#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuAlias(Node):
    def __init__(self):
        super().__init__('imu_alias_node')
        self.pub = self.create_publisher(Imu, '/imu/data', 50)
        self.create_subscription(Imu, '/data', self.cb, 50)

    def cb(self, msg: Imu):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuAlias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
