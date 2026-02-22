#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumCounter(Node):
    def __init__(self):
        super().__init__("num_counter")
        self.get_logger().info("num_counter Started")

        self.count = 0

        self.subscriber = self.create_subscription(
            Int64, "number", self.num_received_cb, 10
        )
        self.publisher_ = self.create_publisher(Int64, "number_counter", 10)

    def num_received_cb(self, msg: Int64):
        self.count += msg.data

        msg = Int64()
        msg.data = self.count
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumCounter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
