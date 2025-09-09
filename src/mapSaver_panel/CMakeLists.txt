cmake_minimum_required(VERSION 3.8)
project(mapSaver_panel)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Helpful warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Qt (RViz on Humble uses Qt5)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# ROS / RViz / msgs
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS imgcodecs core)

# Qt (RViz on Humble uses Qt5)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

add_library(map_save_panel SHARED
  src/map_save_panel.cpp
  include/mapSaver_panel/map_save_panel.hpp   # <-- add the header with Q_OBJECT here
)


target_include_directories(map_save_panel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(map_save_panel
  rclcpp
  pluginlib
  rviz_common
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(map_save_panel
  Qt5::Widgets
  Qt5::Core
  ${OpenCV_LIBS}
)


# This is required by rviz plugins
target_compile_definitions(map_save_panel
  PRIVATE "RVIZ_DEFAULT_PLUGIN_BUILDING_LIBRARY"
)

# Install the plugin library and headers
install(TARGETS map_save_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install plugin XML (so pluginlib can discover it)
# Make sure you have resource/map_save_panel_plugin.xml in your package
install(FILES resource/map_save_panel_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rviz config
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Export the plugin description to RViz
pluginlib_export_plugin_description_file(rviz_common resource/map_save_panel_plugin.xml)


ament_package()
