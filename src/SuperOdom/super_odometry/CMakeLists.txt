cmake_minimum_required(VERSION 3.18)  # CUDA friendly on Jetson
project(super_odometry)

# =========================
# Build type & defaults
# =========================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# C/C++ standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Safer flags
add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)

# Optional CUDA
option(USE_CUDA "Enable CUDA acceleration for selected kernels" OFF)

# =========================
# Ament / ROS 2 packages
# =========================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(super_odometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)


# =========================
# CUDA (optional)
# =========================
if(USE_CUDA)
  find_package(CUDAToolkit REQUIRED)
  enable_language(CUDA)
  message(STATUS "CUDA enabled")
  # Orin Nano (Ampere) = compute capability 8.7
  set(CMAKE_CUDA_ARCHITECTURES 87)
  # Use C++14/17 compatible CUDA
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
endif()

# =========================
# Include / Link dirs
# =========================
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${TBB_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

set(GTSAM_LIBRARIES gtsam)

# =========================
# SuperOdom Library  (single definition)
# =========================
add_library(SuperOdomLib SHARED
  src/FeatureExtraction/featureExtraction.cpp
  src/ImuPreintegration/imuPreintegration.cpp
  src/LaserMapping/laserMapping.cpp
  src/LaserMapping/lidarOptimization.cpp
  src/LaserMapping/LocalMap.cpp
  src/LidarProcess/LidarSlam.cpp
  src/LidarProcess/pose_local_parameterization.cpp
  src/LidarProcess/SE3AbsolutatePoseFactor.cpp
  src/parameter/parameter.cpp
  src/utils/superodom_utils.cpp
  $<$<BOOL:${USE_CUDA}>:src/gpu/kernels.cu> 
)

if(USE_CUDA)
  target_compile_definitions(SuperOdomLib PUBLIC USE_CUDA=1)
endif()

target_include_directories(SuperOdomLib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/SuperOdomLib>
  $<INSTALL_INTERFACE:include/SuperOdomLib>
)

ament_target_dependencies(SuperOdomLib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  super_odometry_msgs
  visualization_msgs
  pcl_conversions
)

# Use keyword style consistently
target_link_libraries(SuperOdomLib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  ${GTSAM_LIBRARIES}
  $<$<BOOL:${USE_CUDA}>:CUDA::cudart>
)

# =========================
# Executables
# =========================
add_executable(imu_preintegration_node src/imuPreintegration_node.cpp)
ament_target_dependencies(imu_preintegration_node rclcpp nav_msgs sensor_msgs tf2_ros)
target_link_libraries(imu_preintegration_node
  ${GTSAM_LIBRARIES}
  SuperOdomLib
)

add_executable(feature_extraction_node src/featureExtraction_node.cpp)
ament_target_dependencies(feature_extraction_node rclcpp nav_msgs sensor_msgs super_odometry_msgs)
target_link_libraries(feature_extraction_node
  ${PCL_LIBRARIES}
  ${TBB_LIBRARIES}
  SuperOdomLib
)

add_executable(laser_mapping_node src/laserMapping_node.cpp)
ament_target_dependencies(laser_mapping_node rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)
target_link_libraries(laser_mapping_node
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  SuperOdomLib
)

add_executable(odom_tf_publisher src/tf_broadcaster_node.cpp)
ament_target_dependencies(odom_tf_publisher rclcpp nav_msgs geometry_msgs tf2_ros)


# =========================
# Install / Export
# =========================
ament_export_targets(export_super_odometry HAS_LIBRARY_TARGET)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS SuperOdomLib
  EXPORT export_super_odometry
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(SuperOdomLib)

install(TARGETS
  imu_preintegration_node
  feature_extraction_node
  laser_mapping_node
  odom_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
