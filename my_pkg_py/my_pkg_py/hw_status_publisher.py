#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interface.msg import HardwareStatus


class HWStatusPublisher(Node):
    def __init__(self):
        super().__init__("hw_status_publisher")
        self.get_logger().info(f"hw_status_publisher Started")

        self.hz = 0.5

        self.publisher_ = self.create_publisher(HardwareStatus, "hw_status", 10)
        self.create_timer(1 / self.hz, self.cb_publish_hw_status)

    def cb_publish_hw_status(self):
        data = HardwareStatus()
        data.temprature = 50.0
        data.is_motors_ready = False
        data.debug_msg = "Nothing interesting..."

        self.publisher_.publish(data)


def main(args=None):
    rclpy.init(args=args)
    node = HWStatusPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
