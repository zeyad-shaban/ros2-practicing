import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    # Turtlesim
    ld.add_action(
        Node(
            package='turtlesim',
            executable='turtlesim_node',
        ),
    )
    
    # Turtle Controller
    ld.add_action(
        Node(
            package='turtle_eater',
            executable='turtle_controller',
            parameters=[
                {'Kp_v': 2.0},
                {'Kp_w': 2.0},
            ]
        ),
    )
    
    # Turtle Spawner
    ld.add_action(
        Node(
            package='turtle_eater',
            executable='world_manager',
        )
    )
    return ld