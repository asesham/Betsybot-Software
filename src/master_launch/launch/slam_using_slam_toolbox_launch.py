# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    master_dir = get_package_share_directory('master_launch')
    betsybot_dir = get_package_share_directory('betsybot')
    superodom_dir = get_package_share_directory('super_odometry')
    hesai_dir = get_package_share_directory('hesai_ros_driver')
    rs_dir = get_package_share_directory('realsense2_camera')
    slam_dir = get_package_share_directory('slam_toolbox')
    launch_dir = os.path.join(master_dir, 'launch')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    #diff_drive_node = Node(
    #    package='differential-drive',
    #    executable='differential-drive',
    #    name='differential_drive',
    #    output='screen')
     
    hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hesai_dir, 'launch', 'start.py')))
         
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'Joystick_drive.launch.py')))
                  
    pc_to_ls_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'sample_pointcloud_to_laserscan_launch.py')))
            
    points_qos_relay_node = Node(
        package='points_qos_relay',
        executable='points_qos_relay',
        name='points_qos_relay',
        output='screen',
        parameters=[
        {
        	'input_topic':'/lidar_points',
        	'output_topic':'/lidar_points_reliable'
        }]) 
                      
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(betsybot_dir, 'configs', 'nav2_params.yaml')]
        )
        
    betsy_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(betsybot_dir, 'launch', 'display.launch.py')))
            
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_dir, 'launch', 'rs_launch.py')))
            
    ekf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ekf_localization_launch.py')))            
            
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, "launch", 'online_async_launch.py')),
             launch_arguments={"use_sim_time": "false"}.items(),
             )
            
    super_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(superodom_dir, 'launch', 'vlp_16.launch.py')))
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    #ld.add_action(super_odom_launch)
    ld.add_action(betsy_robot_cmd)
    ld.add_action(hesai_launch)
    ld.add_action(rs_launch)
    ld.add_action(joy_launch)
    #ld.add_action(points_qos_relay_node)
    ld.add_action(pc_to_ls_launch)
    ld.add_action(ekf_localization_launch)
    #ld.add_action(amcl_node)
    ld.add_action(slam_launch)

    return ld
