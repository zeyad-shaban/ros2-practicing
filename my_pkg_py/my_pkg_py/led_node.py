#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from my_robot_interface.srv import LedState

class LedNode(Node):
    def __init__(self):
        super().__init__("led_node")
        self.get_logger().info(f"led_node Started")
        
        self.status_hz = 8
        self.led_status = "000"

        self.status_publisher = self.create_publisher(String, 'led_status', 10)
        self.create_timer(1/self.status_hz, self.cb_publish_status)
        
        self.create_service(LedState, "/set_led", self.handle_set_led_state)
        
    def handle_set_led_state(self, req: LedState.Request, res: LedState.Response):
        if req.led_num > len(self.led_status) or req.led_num <= 0:
            self.get_logger().error(f"handling set led state failed, led {req.led_num} out of range")
            res.success = False
            return res
        
        int_val =  "1" if req.state else "0"
        idx = req.led_num - 1
        self.led_status = self.led_status[:idx] + int_val + self.led_status[idx+1:]
        
        res.success = True
        return res
        
        
    def cb_publish_status(self):
        msg = String()
        msg.data =self.led_status
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()