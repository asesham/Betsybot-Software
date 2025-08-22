from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

# ..

def generate_launch_description():
    container = LoadComposableNodes(
        target_container='/Betsybot_Software/camera_container',
        composable_node_descriptions=[
            ComposableNode(
                package='color-detector',
                plugin='ColorDetector',
                name='color_detector',
                # ..
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ]
    )
    
    return LaunchDescription([container])
