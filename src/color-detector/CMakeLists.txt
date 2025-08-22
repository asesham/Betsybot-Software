cmake_minimum_required(VERSION 3.8)
project(color-detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(color-detector src/color-detector.cpp)
add_library(color_detector_component SHARED src/color-detector.cpp)

target_include_directories(color_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(color_detector_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

set(dependencies
  rclcpp
  OpenCV
  cv_bridge
  sensor_msgs
  image_transport
  realsense2_camera_msgs
  rclcpp_components
)
ament_target_dependencies(color_detector_component ${dependencies})

ament_export_targets(export_color_detector_component)
install(TARGETS color_detector_component
        EXPORT export_color_detector_component
        DESTINATION lib)
        
# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rclcpp_components_register_node(
    color_detector_component
    PLUGIN "ColorDetector"
    EXECUTABLE color-detector
)

ament_export_dependencies(${dependencies})

ament_package()
