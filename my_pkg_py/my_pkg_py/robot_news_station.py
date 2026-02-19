#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.get_logger().info("Robot News Station Started")
        
        self.robot_name = "X001"
        self.hz = 2
        
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(1/self.hz, self.publish_news)
        
    def publish_news(self):
        msg = String()
        msg.data = f"Hello, this is {self.robot_name} from the robot news station"
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()