import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[{'axis_linear.x': 3},
                        {'axis_angular.yaw': 0},
                        {'scale_linear.x': 2.0},
                        {'scale_angular.yaw':3.0}]),

        Node(
            package='differential-drive',
            executable='differential-drive',
            name='differential_drive',
            output='screen'),
    ])
