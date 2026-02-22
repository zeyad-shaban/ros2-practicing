#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interface.msg import LedStateArr
from my_robot_interface.srv import SetLedState

class LedNode(Node):
    def __init__(self):
        super().__init__("led_node")
        self.get_logger().info(f"led_node Started")
        
        self.status_hz = 8
        self.led_state = [0, 0, 0]

        self.status_publisher = self.create_publisher(LedStateArr, 'led_status', 10)
        self.create_timer(1/self.status_hz, self.cb_publish_status)
        
        self.create_service(SetLedState, "/set_led", self.handle_set_led_state)
        
    def handle_set_led_state(self, req: SetLedState.Request, res: SetLedState.Response):
        if req.led_num > len(self.led_state) or req.led_num <= 0:
            self.get_logger().error(f"handling set led state failed, led {req.led_num} out of range")
            res.success = False
            return res
        
        self.led_state[req.led_num- 1] = 1 if req.state else 0
        
        res.success = True
        return res
        
        
    def cb_publish_status(self):
        msg = LedStateArr()
        msg.led_state =self.led_state
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()