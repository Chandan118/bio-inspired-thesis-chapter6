import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_relay')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_nav')

        self._input_topic = (
            self.get_parameter('input_topic').get_parameter_value().string_value
        )
        self._output_topic = (
            self.get_parameter('output_topic').get_parameter_value().string_value
        )

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._sub = self.create_subscription(Twist, self._input_topic, self._cb, 10)

        self.get_logger().info(
            f'Relaying Twist: {self._input_topic} -> {self._output_topic}'
        )

    def _cb(self, msg: Twist) -> None:
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

