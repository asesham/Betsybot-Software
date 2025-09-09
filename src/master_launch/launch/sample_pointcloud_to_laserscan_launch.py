import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        #Node(
        #    package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
        #    remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
        #    parameters=[{'cloud_frame_id': 'cloud', 'cloud_extent': 2.0, 'cloud_size': 500}],
        #    name='cloud_publisher'
        #),
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    name='static_transform_publisher',
        #    arguments=['0', '0.0298', '0.0124', '0', '0', '0', '1', 'base_link', 'hesai_lidar']
        #),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/lidar_points'),
                        ('scan', '/scan')],
            parameters=[
            #{
            #    'qos_overrides': {
            #        '/lidar_points': {
            #            'subscriber': {
            #                'reliability': 'reliable'
            #            }
            #        }
            #    }
            #},#os.path.join("/home/betsybot/Betsybot-Software/src/master_launch/config", "pointcloud_to_laserscan_qos.yaml"),
            {
                'target_frame': 'hesai_lidar',
                #'queue_size': 100,
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159, #6.283185,  # M_PI/2
                'angle_increment': 0.01745329251,  # M_PI/360.0
                'time_increment': 0.0,  # M_PI/360.0
                'scan_time': 0.0,
                'range_min': 0.12,
                'range_max': 120.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
