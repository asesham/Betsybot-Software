import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    config_path = get_share_file(
        package_name="super_odometry",
        file_name="config/vlp_16.yaml"
    )
    calib_path = get_share_file(
        package_name="super_odometry",
        file_name="config/velodyne/vlp_16_calibration.yaml"
    )

    # ---- Launch args ----
    config_path_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_path,
        description="Path to config file for super_odometry"
    )
    calib_path_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=calib_path,
        description="Path to LiDAR calibration YAML"
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="integrated_to_init"
    )
    world_frame_arg = DeclareLaunchArgument("world_frame", default_value="map")
    world_frame_rot_arg = DeclareLaunchArgument("world_frame_rot", default_value="map_rot")
    sensor_frame_arg = DeclareLaunchArgument("sensor_frame", default_value="sensor")
    sensor_frame_rot_arg = DeclareLaunchArgument("sensor_frame_rot", default_value="sensor_rot")

    # NEW: toggle GPU paths in your nodes
    use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="true",
        description="If true, nodes prefer CUDA code paths when compiled with USE_CUDA"
    )
    gpu_backend_arg = DeclareLaunchArgument(
        "gpu_backend",
        default_value="cuda",  # keep 'cuda' for custom kernels; (tensorrt for NN nodes, not used here)
        description="GPU backend selector passed to nodes"
    )

    # ---- Environment for CUDA on Jetson ----
    # These are safe on host; if a var isn't used, it's simply ignored.
    env_cuda_visible = SetEnvironmentVariable("CUDA_VISIBLE_DEVICES", "0")
    env_driver_caps   = SetEnvironmentVariable("NVIDIA_DRIVER_CAPABILITIES", "all")
    # Optional caching helps first-run kernel JITs (harmless if unused):
    env_cuda_cache    = SetEnvironmentVariable("CUDA_CACHE_DISABLE", "0")
    env_cuda_cache_sz = SetEnvironmentVariable("CUDA_CACHE_MAXSIZE", "2147483647")  # 2GB
    # Optional: improve concurrent kernel launches
    env_cuda_conns    = SetEnvironmentVariable("CUDA_DEVICE_MAX_CONNECTIONS", "32")

    # ---- Nodes ----
    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "calibration_file": LaunchConfiguration("calibration_file"),
                # NEW: GPU toggles (your C++ should read these)
                "use_gpu": LaunchConfiguration("use_gpu"),
                "gpu_backend": LaunchConfiguration("gpu_backend")
            },
        ],
    )

    laser_mapping_node = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "calibration_file": LaunchConfiguration("calibration_file"),
                "map_dir": "/home/betsybot/Documents/slam_map/basement_loc_pcl.pcd",
                # NEW
                "use_gpu": LaunchConfiguration("use_gpu"),
                "gpu_backend": LaunchConfiguration("gpu_backend")
            },
        ],
        remappings=[("laser_odom_to_init", LaunchConfiguration("odom_topic"))],
    )

    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "calibration_file": LaunchConfiguration("calibration_file"),
                # IMU preintegration is mostly CPU/GTSAM; still pass for consistency
                "use_gpu": LaunchConfiguration("use_gpu"),
                "gpu_backend": LaunchConfiguration("gpu_backend")
            },
        ],
    )

    return LaunchDescription([
        # time source
        launch_ros.actions.SetParameter(name="use_sim_time", value="false"),

        # args
        config_path_arg,
        calib_path_arg,
        odom_topic_arg,
        world_frame_arg,
        world_frame_rot_arg,
        sensor_frame_arg,
        sensor_frame_rot_arg,
        use_gpu_arg,
        gpu_backend_arg,

        # env
        env_cuda_visible,
        env_driver_caps,
        env_cuda_cache,
        env_cuda_cache_sz,
        env_cuda_conns,

        # nodes
        feature_extraction_node,
        laser_mapping_node,
        imu_preintegration_node,
    ])
