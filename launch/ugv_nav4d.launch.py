from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ugv_nav4d_ros2'),
        'config',
        'params.yaml'
        )

    return LaunchDescription([
        Node(
            package="ugv_nav4d_ros2",
            executable="ugv_nav4d_node",
            name="ugv_nav4d_node",
            output="screen",
            parameters=[config]
        )
    ])