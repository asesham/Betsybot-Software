# your_robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                '/home/betsybot/Betsybot-Software/src/master_launch/config/ekf.yaml' # Replace with the actual path
            ]
        )
    ])
