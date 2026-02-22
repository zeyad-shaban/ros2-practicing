from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package="my_pkg_py",
        executable="num_publisher",
        name="num_publisher1",
        namespace="publisher1",
    ))
    
    ld.add_action(Node(
        package="my_pkg_py",
        executable="num_publisher",
        name="num_publisher2",
        namespace="publisher2",
    ))

    return ld