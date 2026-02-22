#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumPublisher(Node):
    def __init__(self):
        super().__init__("num_publisher")
        self.get_logger().info(f"num_publisher Started")

        self.fixed_num = 5
        self.hz = 10

        self.publisher_ = self.create_publisher(Int64, "/number", 10)

        self.create_timer(1 / self.hz, self.publish_num_cb)

    def publish_num_cb(self):
        msg = Int64()
        msg.data = self.fixed_num
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
