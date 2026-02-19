#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        
        self.count = 0
        
        self.get_logger().info("py_test initiated")
        self.create_timer(1/2, self.timer_cb)
        
    def timer_cb(self):
        self.get_logger().info(f"Counter: {self.count}")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()