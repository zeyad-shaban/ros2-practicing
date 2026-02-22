import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)

    node = Node("add_two_ints_client_no_oop")
    node.get_logger().info("Started add_two_ints_client_no_oop")
    add_two_ints_client = node.create_client(AddTwoInts, "add_two_ints")

    while not add_two_ints_client.wait_for_service(1):
        node.get_logger().warning("Waiting for add_two_ints service server")

    req = AddTwoInts.Request(a=8, b=10)
    future = add_two_ints_client.call_async(req)

    rclpy.spin_until_future_complete(node, future)

    res: AddTwoInts.Response = future.result()  # type: ignore
    node.get_logger().info(f"sum: {res.sum}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
