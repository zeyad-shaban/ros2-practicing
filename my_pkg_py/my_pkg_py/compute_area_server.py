#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interface.srv import ComputeArea


class ComputeAreaServer(Node):
    def __init__(self):
        super().__init__("compute_area")
        self.get_logger().info(f"compute_area Started")

        self.service_ = self.create_service(
            ComputeArea, "compute_area", self.compute_area_cb
        )

    def compute_area_cb(self, req: ComputeArea.Request, res: ComputeArea.Response):
        res.area = req.length * req.width
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ComputeAreaServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
