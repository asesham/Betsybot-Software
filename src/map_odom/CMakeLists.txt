cmake_minimum_required(VERSION 3.8)
project(map_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(map_odom_broadcaster_node src/map_odom_broadcaster.cpp)
ament_target_dependencies(map_odom_broadcaster_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros)

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})

install(TARGETS map_odom_broadcaster_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
