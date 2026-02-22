#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.get_logger().info(f"add_two_ints_client Started")

        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        self.call_add_two_ints(2, 5)

    def call_add_two_ints(self, a: int, b: int):
        while not self.client_.wait_for_service(1):
            self.get_logger().warning(f"waiting for service server add_two_ints...")

        req = AddTwoInts.Request(a=a, b=b)
        future = self.client_.call_async(req)
        future.add_done_callback(self.cb_call_add_two_ints)

    def cb_call_add_two_ints(self, future):
        res: AddTwoInts.Response = future.result()
        self.get_logger().info(f"Sum: {res.sum}, and this is with the clean way yay :D")


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
