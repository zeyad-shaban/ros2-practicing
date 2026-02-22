#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32
from my_robot_interface.srv import LedState
from enum import Enum

class BatteryStates(Enum):
    FULL= 1
    EMPTY= 0

class BatterStatusServer(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.get_logger().info(f"battery_node Started")
        
        self.status_hz = 10
        self.battery_state = BatteryStates.FULL
        
        self.charging_time_secs = 6
        self.discharge_time_secs = 4

        self.status_publisher = self.create_publisher(Int32, "battery_status", 10)
        self.create_timer(1/self.status_hz, self.cb_status_publish)
        
        self.led_service_client = self.create_client(LedState, "/set_led")
        
        # charge/discharge logic
        self.discharge_timer = self.create_timer(self.discharge_time_secs, self.cb_discharged)
        self.discharge_timer.cancel()
        
        self.charging_timer = self.create_timer(self.charging_time_secs, self.cb_charged)
        self.charging_timer.cancel()
        
        self.cb_charged()
        
    
    def cb_status_publish(self):
        msg = Int32()
        msg.data = self.battery_state.value
        self.status_publisher.publish(msg)
    
    def cb_charged(self):
        self.battery_state = BatteryStates.FULL
        self.charging_timer.cancel()
        self.discharge_timer.reset()
        
        req = LedState.Request()
        req.led_num = 3
        req.state = False
        
        fut = self.led_service_client.call_async(req)
        fut.add_done_callback(self.cb_led_service)
        
    def cb_discharged(self):
        self.battery_state = BatteryStates.EMPTY
        self.discharge_timer.cancel()
        self.charging_timer.reset()
        
        req = LedState.Request()
        req.led_num = 3
        req.state = True
        
        fut = self.led_service_client.call_async(req)
        fut.add_done_callback(self.cb_led_service)
        
    def cb_led_service(self, future):
        res: LedState.Response = future.result() # type: ignore
        self.get_logger().info(f"response success: {res.success}")


def main(args=None):
    rclpy.init(args=args)
    node = BatterStatusServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()