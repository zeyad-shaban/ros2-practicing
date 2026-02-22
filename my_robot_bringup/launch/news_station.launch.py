from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # params_file = os.path.join(
    #     get_package_share_directory('my_pkg_py'),
    #     'config',
    #     'params.yaml'
    # )

    ld = LaunchDescription()
    
    names = ["giskard", "bb8", "daneel", "jander", "c3po", "x001"]
    
    for name in names:
        ld.add_action(
            Node(
                package='my_pkg_py',
                executable='robot_news_station',
                name=f'robot_news_station_{name}',
                parameters=[
                    {"robot_name": name},
                ],
            ),
        )
    
    ld.add_action(
        Node(
            package="my_pkg_py",
            executable="smartphone",
        )
    )
    
    return ld