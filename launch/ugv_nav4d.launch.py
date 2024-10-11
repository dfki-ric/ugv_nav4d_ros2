from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ugv_nav4d_ros2'),
        'config',
        'params.yaml'
        )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/ugv_nav4d_ros2/pointcloud',
            description='Topic name of the pointcloud used to generate MLS map'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'goal_topic',
            default_value='/ugv_nav4d_ros2/goal_pose',
            description='Topic name of the goal pose'
        )
    )
    declared_arguments.append(
        Node(
        package="ugv_nav4d_ros2",
        executable="ugv_nav4d_ros2_node",
        name="ugv_nav4d_ros2",
        output="screen",
        remappings=[
                ("/ugv_nav4d_ros2/pointcloud", LaunchConfiguration("pointcloud_topic")),
                ("/ugv_nav4d_ros2/goal_pose", LaunchConfiguration("goal_topic")),
            ],
        parameters=[config],
        )
    )

    return LaunchDescription(declared_arguments)