from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

import os

def generate_launch_description():

    default_param_file = os.path.join(
        get_package_share_directory('ugv_nav4d_ros2'),
        'config',
        'params.yaml'
        )

    declared_arguments = []

    declared_arguments.append(
	DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        )
    )

    declared_arguments.append(
        SetParameter(
            name="use_sim_time",
            value=LaunchConfiguration("use_sim_time"),
        )
    )
    
    declared_arguments.append(
	DeclareLaunchArgument(
            'main_param_file',
            default_value=default_param_file,
            description='Full path to main parameter file to load')	
    )
    
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
        DeclareLaunchArgument(
            'pose_samples_topic',
            default_value='/ugv_nav4d_ros2/start_pose',
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
                ("/ugv_nav4d_ros2/start_pose", LaunchConfiguration("pose_samples_topic")),
            ],
        parameters=[LaunchConfiguration("main_param_file")],
        )
    )
    declared_arguments.append(
        Node(
        package="ugv_nav4d_ros2",
        executable="visualize_path.py",
        name="ugv_nav4d_path_visualization",
        output="screen",
        remappings=[],
        parameters=[],
        )
    )

    return LaunchDescription(declared_arguments)
