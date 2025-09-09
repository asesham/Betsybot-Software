# master_all_in_one_simple.launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package share dirs
    master_dir   = get_package_share_directory('master_launch')
    betsybot_dir = get_package_share_directory('betsybot')
    hesai_dir    = get_package_share_directory('hesai_ros_driver')
    rs_dir       = get_package_share_directory('realsense2_camera')
    slam_dir     = get_package_share_directory('super_odometry')
    gridMap_dir  = get_package_share_directory('pointcloud_to_grid')
    mapOdom_dir  = get_package_share_directory('map_odom')

    launch_dir   = os.path.join(master_dir, 'launch')

    # Includes (no args)
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_dir, 'launch', 'rs_launch.py'))
    )

    hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(hesai_dir, 'launch', 'start.py'))
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_dir, 'launch', 'vlp_16.launch.py'))
    )

    map_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mapOdom_dir, 'launch', 'map_odom_broadcaster.launch.py'))
    )

    gridMap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gridMap_dir, 'launch', 'demo.launch.py'))
    )

    betsy_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(betsybot_dir, 'launch', 'display.launch.py'))
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'Joystick_drive.launch.py'))
    )

    # Sequence with timers (replaces time.sleep)
    ld = LaunchDescription()
    ld.add_action(TimerAction(period=0.0, actions=[rs_launch]))
    ld.add_action(TimerAction(period=3.0, actions=[hesai_launch]))
    ld.add_action(TimerAction(period=4.0, actions=[slam_launch]))
    ld.add_action(TimerAction(period=5.0, actions=[map_odom_launch]))
    ld.add_action(TimerAction(period=6.0, actions=[gridMap_launch]))
    ld.add_action(TimerAction(period=6.5, actions=[betsy_robot_cmd]))
    ld.add_action(TimerAction(period=7.0, actions=[joy_launch]))

    return ld
