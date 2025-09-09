# launch/map_odom_broadcaster.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="map_odom",
            executable="map_odom_broadcaster_node",  # ensure this matches your CMake target
            name="map_odom_broadcaster",
            output="screen",
            parameters=[{
                # Base pose in map (from localization):
                "global_odom_topic": "/state_estimation",
                # Base pose in odom (from wheel/VIO/LIO odom):
                "local_odom_topic": "/laser_odometry",

                # Frames
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame_out": "base_footprint",   # TF child for odom→base

                # Behavior
                "publish_rate_hz": 50.0,
                "use_msg_frame_ids": True,            # set False to lock frames above
                "identity_if_missing": False,

                # Odom→base shaping
                "project_to_2d": True,                # zero roll/pitch
                "zero_z": True,                       # flatten z (plus offset)
                "base_z_offset": 0.0
            }],
            
            # If your topics have different names, uncomment and edit:
            # remappings=[
            #     ("/state_estimation", "/your_global_pose_topic"),
            #     ("/laser_odometry",  "/your_local_odom_topic"),
            # ],
        )
    ])
