#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Smartphone(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.get_logger().info(f"smartphone Started")

        self.subscriber_ = self.create_subscription(String, "robot_news", self.cb_robot_news, 10)
        
    def cb_robot_news(self, msg: String):
        self.get_logger().info(f"Received data: {msg.data}")
        


def main(args=None):
    rclpy.init(args=args)
    node = Smartphone()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()