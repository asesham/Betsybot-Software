cmake_minimum_required(VERSION 3.8)
project(differential-drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(phoenix6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

MESSAGE(STATUS "${phoenix6_INCLUDE_DIRS} ${phoenix6_LIBRARIES}" )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(differential-drive src/diff-drive-main.cpp)
target_link_libraries(${PROJECT_NAME} /usr/lib/phoenix6/libCTRE_Phoenix6.so /usr/lib/phoenix6/libCTRE_PhoenixTools.so)
#target_link_libraries(${PROJECT_NAME} phoenix6)
target_include_directories(differential-drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(differential-drive PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

set(dependencies
  rclcpp
  phoenix6
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)
ament_target_dependencies(differential-drive ${dependencies})

install(TARGETS differential-drive
  DESTINATION lib/${PROJECT_NAME})
  
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

ament_export_dependencies(${dependencies})
ament_package()
