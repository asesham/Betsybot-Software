
"""Launch apriltag_ros node."""
import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


configurable_parameters = [{'name': 'apriltag_ros',         'default': 'apriltag_ros', 'description': 'apriltag name'},
                           {'name': 'camera_name',          'default': '/camera_rect', 'description': 'camera unique name'},
                           {'name': 'image_topic',          'default': 'image_rect', 'description': 'Rectified Image'},
                           {'name': 'queue_size',           'default': "1", 'description': 'Queue Size'},
                           {'name': 'apriltag_namespace',   'default': "apriltag", 'description': 'namespace for apriltag'},
                           {'name': 'output',               'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'log_level',            'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'publish_tag_detections_image',            'default': 'true', 'description': ''},
                           ]
                           
def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    _output = LaunchConfiguration('output' + param_name_suffix)
    if(os.getenv('ROS_DISTRO') == 'foxy'):
        # Foxy doesn't support output as substitution object (LaunchConfiguration object)
        # but supports it as string, so we fetch the string from this substitution object
        # see related PR that was merged for humble, iron, rolling: https://github.com/ros2/launch/pull/577
        _output = context.perform_substitution(_output)

    return [
        launch_ros.actions.Node(
            package='apriltag_ros',
            namespace=LaunchConfiguration('apriltag_namespace' + param_name_suffix),
            name=LaunchConfiguration('apriltag_ros' + param_name_suffix),
            executable='apriltag_ros_node',
            parameters=[params, params_from_file],
            output=_output,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
            remappings=[
            	('/image_rect', LaunchConfiguration('camera_name') + '/' + LaunchConfiguration('image_rect')),
            	('/camera_info', 'LaunchConfiguration('camera_name') + '/camera_info'),
            ]
            )
    ]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup, kwargs = {'params' : set_configurable_parameters(configurable_parameters)})
    ])
    
