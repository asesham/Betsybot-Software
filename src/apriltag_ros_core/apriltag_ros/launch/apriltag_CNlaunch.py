import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare launch arguments
    # launch_prefix: Used for remote debugging, e.g., "gdbserver localhost:10000"
    launch_prefix_arg = DeclareLaunchArgument(
        'launch_prefix',
        default_value='',
        description='Prefix to launch command (e.g., "gdbserver localhost:10000" for debugging)'
    )

    # camera_name: Base name for the camera topics
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='/camera/camera/color',
        description='Base name for camera topics (e.g., /camera/camera/color)'
    )

    # image_topic: Suffix for the image topic
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='image_raw',
        description='Suffix for the image topic (e.g., image_raw)'
    )

    # queue_size: Size of the message queue
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='1',
        description='Size of the message queue for topic subscriptions'
    )

    # Get the share directory for the apriltag_ros package
    apriltag_ros_share_dir = get_package_share_directory('apriltag_ros')
    
    apriltag_detector_node = LoadComposableNodes(
        target_container='/Betsybot_Software/camera_container',
        composable_node_descriptions=[
            ComposableNode(
                package='apriltag_ros',
                plugin='apriltag_ros::ContinuousDetector',
                name='apriltag_ros_continuous_node',
                namespace='',
                remappings=[
                    ('image_rect', [LaunchConfiguration('camera_name'), '/', LaunchConfiguration('image_topic')]),
                    ('camera_info', [LaunchConfiguration('camera_name'), '/camera_info']),
                ],
                parameters=[
                    {'publish_tag_detections_image': True},
                    {'queue_size': LaunchConfiguration('queue_size')},
                    PathJoinSubstitution([apriltag_ros_share_dir, 'config', 'tags.yaml']),
                    PathJoinSubstitution([apriltag_ros_share_dir, 'config', 'settings.yaml']),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ]
    )
    """
    # Define the standard Node for the AprilTag detector
    apriltag_detector_node = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_node', # Assuming this is the executable for the continuous detector
        name='apriltag_ros_continuous_node',
        namespace='',
        # Apply launch_prefix if specified
        prefix=LaunchConfiguration('launch_prefix'),
        output='screen', # Display node output to the screen
        # Remap topics as defined in the XML
        remappings=[
            ('image_rect', [LaunchConfiguration('camera_name'), '/', LaunchConfiguration('image_topic')]),
            ('camera_info', [LaunchConfiguration('camera_name'), '/camera_info']),
        ],
        # Set parameters
        parameters=[
            {'publish_tag_detections_image': True},  # Corresponds to <param name="publish_tag_detections_image" type="bool" value="true" />
            {'queue_size': LaunchConfiguration('queue_size')},  # Corresponds to <param name="queue_size" type="int" value="$(var queue_size)" />
            # Load parameters from YAML files
            PathJoinSubstitution([apriltag_ros_share_dir, 'config', 'tags.yaml']),
            PathJoinSubstitution([apriltag_ros_share_dir, 'config', 'settings.yaml']),
        ]
    )
    """

    # Return the LaunchDescription with all defined actions
    return LaunchDescription([
        launch_prefix_arg,
        camera_name_arg,
        image_topic_arg,
        queue_size_arg,
        apriltag_detector_node # Now directly launching the node
    ])


'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node # Changed from ComposableNodeContainer and ComposableNode



def generate_launch_description():
    # Declare launch arguments
    # launch_prefix: Used for remote debugging, e.g., "gdbserver localhost:10000"
    launch_prefix_arg = DeclareLaunchArgument(
        'launch_prefix',
        default_value='',
        description='Prefix to launch command (e.g., "gdbserver localhost:10000" for debugging)'
    )

    # camera_name: Base name for the camera topics
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='/camera/camera/color',
        description='Base name for camera topics (e.g., /camera/camera/color)'
    )

    # image_topic: Suffix for the image topic
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='image_raw',
        description='Suffix for the image topic (e.g., image_raw)'
    )

    # queue_size: Size of the message queue
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='1',
        description='Size of the message queue for topic subscriptions'
    )

    # Get the share directory for the apriltag_ros package
    apriltag_ros_share_dir = get_package_share_directory('apriltag_ros')
    
    # Define the standard Node for the AprilTag detector
    apriltag_detector_node = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_node', # Assuming this is the executable for the continuous detector
        name='apriltag_ros_continuous_node',
        namespace='',
        # Apply launch_prefix if specified
        prefix=LaunchConfiguration('launch_prefix'),
        output='screen', # Display node output to the screen
        # Remap topics as defined in the XML
        remappings=[
            ('image_rect', [LaunchConfiguration('camera_name'), '/', LaunchConfiguration('image_topic')]),
            ('camera_info', [LaunchConfiguration('camera_name'), '/camera_info']),
        ],
        # Set parameters
        parameters=[
            {'publish_tag_detections_image': True},  # Corresponds to <param name="publish_tag_detections_image" type="bool" value="true" />
            {'queue_size': LaunchConfiguration('queue_size')},  # Corresponds to <param name="queue_size" type="int" value="$(var queue_size)" />
            # Load parameters from YAML files
            PathJoinSubstitution([apriltag_ros_share_dir, 'config', 'tags.yaml']),
            PathJoinSubstitution([apriltag_ros_share_dir, 'config', 'settings.yaml']),
        ]
    )

    # Define the ComposableNode for the AprilTag detector
    apriltag_detector_node = ComposableNode(
        package='apriltag_ros',
        plugin='apriltag_ros::ContinuousDetector',
        name='apriltag_ros_continuous_node',
        namespace='',
        # Remap topics as defined in the XML
        remappings=[
            ('image_rect', [LaunchConfiguration('camera_name'), '/', LaunchConfiguration('image_topic')]),
            ('camera_info', [LaunchConfiguration('camera_name'), '/camera_info']),
        ],
        # Set parameters
        parameters=[
            {'publish_tag_detections_image': True},  # Corresponds to <param name="publish_tag_detections_image" type="bool" value="true" />
            {'queue_size': LaunchConfiguration('queue_size')},  # Corresponds to <param name="queue_size" type="int" value="$(var queue_size)" />
            # Load parameters from YAML files
            PathJoinSubstitution([apriltag_ros_share_dir, 'config', 'tags.yaml']),
            PathJoinSubstitution([apriltag_ros_share_dir, 'config', 'settings.yaml']),
        ]
    )

    # Define the ComposableNodeContainer
    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='apriltag_ros_continuous_detector_container',
        namespace='',
        composable_node_descriptions=[
            apriltag_detector_node,
        ],
        output='screen',
        # Apply launch_prefix if specified
        prefix=LaunchConfiguration('launch_prefix'),
    )

    # Return the LaunchDescription with all defined actions
    return LaunchDescription([
        launch_prefix_arg,
        camera_name_arg,
        image_topic_arg,
        queue_size_arg,
        apriltag_detector_node,
    ])
'''
