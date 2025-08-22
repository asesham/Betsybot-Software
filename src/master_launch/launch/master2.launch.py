from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespace = 'Betsybot_Software'
    
    launch_args = [
      # DeclareLaunchArgument('apriltag', default_value='true', description='Launch apriltag node'),
      # DeclareLaunchArgument('apriltag_ros_core', default_value='true', description='Launch apriltag_ros_core node'),
      # DeclareLaunchArgument('color_detector', default_value='true', description='Launch color detector node'),
      # DeclareLaunchArgument('cpp_parameters', default_value='true', description='Launch cpp_parameters node'),
      # DeclareLaunchArgument('differential_drive', default_value='true', description='Launch differential drive node'),
      # DeclareLaunchArgument('realsense_ros', default_value='true', description='Launch realsense_ros node'),
      # DeclareLaunchArgument('ros2_tut', default_value='true', description='Launch ros2_tut node'),
      # DeclareLaunchArgument('sensor_fusion', default_value='true', description='Launch sensor fusion node'),
    ]
    container = ComposableNodeContainer(
        name='camera_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[], # Nodes will be loaded later
        output='screen',
        emulate_tty=True,
    )
    
    realsense_launch_file = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_CNlaunch.py'
    ])
    
    color_detector_launch_file = PathJoinSubstitution([
        FindPackageShare('color-detector'),
        'launch',
        'cd_CNlaunch.py'
    ])

    apriltag_launch_file = PathJoinSubstitution([
        FindPackageShare('apriltag_ros'),
        'launch',
        'apriltag_CNlaunch.py'
    ])
    
    return LaunchDescription(launch_args + [container,
            IncludeLaunchDescription(PythonLaunchDescriptionSource(realsense_launch_file)),
            #IncludeLaunchDescription(PythonLaunchDescriptionSource(color_detector_launch_file)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(apriltag_launch_file)),
            ])

"""
    # Launch arguments for conditional node launching
    launch_args = [
      # DeclareLaunchArgument('apriltag', default_value='true', description='Launch apriltag node'),
        DeclareLaunchArgument('apriltag_ros_core', default_value='true', description='Launch apriltag_ros_core node'),
        DeclareLaunchArgument('color_detector', default_value='true', description='Launch color detector node'),
        DeclareLaunchArgument('cpp_parameters', default_value='true', description='Launch cpp_parameters node'),
        DeclareLaunchArgument('differential_drive', default_value='true', description='Launch differential drive node'),
        DeclareLaunchArgument('realsense_ros', default_value='true', description='Launch realsense_ros node'),
        DeclareLaunchArgument('ros2_tut', default_value='true', description='Launch ros2_tut node'),
        DeclareLaunchArgument('sensor_fusion', default_value='true', description='Launch sensor fusion node'),
    ]

    # Group nodes under a namespace
    nodes_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
           # Node(
            #    package='apriltag',
             #   executable='apriltag',
              #  name='apriltag_node',
               # output='screen',
                #condition=IfCondition(LaunchConfiguration('apriltag'))
           # ),
            Node(
                package='apriltag_ros',
                executable='apriltag_ros_continuous_detector_node',
                name='apriltag_ros_core_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('apriltag_ros_core'))
            ),
            Node(
                package='color-detector',
                executable='color-detector',
                name='color_detector_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('color_detector'))
            ),
            Node(
                package='cpp_parameters',
                executable='minimal_param_node',
                name='cpp_parameters_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('cpp_parameters'))
            ),
            Node(
                package='differential-drive',
                executable='differential-drive',
                name='diff_drive_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('differential_drive'))
            ),
            Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='realsense_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('realsense_ros'))
            ),
            Node(
                package='ros2_tut',
                executable='ros2_tut_node',
                name='ros2_tut_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('ros2_tut'))
            ),
            Node(
                package='sensor-fusion',
                executable='sensor_fusion',
                name='sensor_fusion_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('sensor_fusion'))
            ),
        ]
    )

    return LaunchDescription(launch_args + [nodes_group])
    
"""
