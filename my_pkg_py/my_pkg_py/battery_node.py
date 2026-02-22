#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32
from my_robot_interface.srv import SetLedState
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
        self.last_state_change = self.get_clock().now()
        
        self.charging_time_secs = 6
        self.discharge_time_secs = 4

        self.status_publisher = self.create_publisher(Int32, "battery_status", 10)
        self.create_timer(1/self.status_hz, self.cb_status_publish)
        
        self.led_service_client = self.create_client(SetLedState, "/set_led")
        
        # charge/discharge logic
        self.create_timer(1, self.cb_timer_tick)
        
    
    def cb_status_publish(self):
        msg = Int32()
        msg.data = self.battery_state.value
        self.status_publisher.publish(msg)
    
    def cb_timer_tick(self):
        elapsed_time = (self.get_clock().now() - self.last_state_change).nanoseconds / 1e9 # typ: ignore
        
        should_switch_to_empty = self.battery_state == BatteryStates.FULL and elapsed_time >= self.discharge_time_secs
        should_switch_to_full = self.battery_state == BatteryStates.EMPTY and elapsed_time >= self.charging_time_secs
        
        if not (should_switch_to_empty or should_switch_to_full):
            return
        
        self.last_state_change = self.get_clock().now()
        req = SetLedState.Request()
        req.led_num = 3
        
        if should_switch_to_empty:
            self.battery_state = BatteryStates.EMPTY
            req.state = True
        
        elif should_switch_to_full:
            self.battery_state = BatteryStates.FULL
            req.state = False
            
        while not self.led_service_client.wait_for_service(1):
            self.get_logger().warning("led_service not ready...")
            
        fut = self.led_service_client.call_async(req)
        fut.add_done_callback(self.cb_led_service)
            
    def cb_led_service(self, future):
        res: SetLedState.Response = future.result() # type: ignore
        self.get_logger().info(f"response success: {res.success}")


def main(args=None):
    rclpy.init(args=args)
    node = BatterStatusServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()