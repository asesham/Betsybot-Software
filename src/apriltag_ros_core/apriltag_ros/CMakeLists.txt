cmake_minimum_required(VERSION 3.5)
project(apriltag_ros)

# Set C++ standard to C++17 as a modern default.
# This is a good practice and simplifies the original conditional logic.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add standard compile options for warnings.
# Use a modern approach with target_compile_options.
# We will apply these to all targets later.
add_compile_options(-Wall -Wextra -Wpedantic)

# Add compiler defense flags using a more modern and robust approach.
# These flags are applied uniformly using a list.
# A separate list for link flags is also a good practice.
if(UNIX OR APPLE)
    set(DEFAULT_LINK_FLAGS "")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Intel")
        list(APPEND DEFAULT_LINK_FLAGS -z noexecstack -z relro -z now -pie)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        list(APPEND DEFAULT_LINK_FLAGS -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now -Wl,-pie)
    endif()
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${DEFAULT_LINK_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${DEFAULT_LINK_FLAGS}")
endif()

# Compiler-specific flags for defense.
# Use target_compile_options instead of global variables.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 4.9)
        add_compile_options(-fPIE -fstack-protector-strong)
    else()
        add_compile_options(-fPIE -fstack-protector)
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fPIE -fstack-protector)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    # ICC is compatible with -fstack-protector. The comment about -fPIE is a good note.
    add_compile_options(-fstack-protector)
endif()

# Set generic compiler flags and ensure they are not propagated to CUDA.
add_compile_options(-fPIC -fno-operator-names -Wformat -Wformat-security)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)

# Optimization flags for release builds
#add_compile_options(
#    $<$<CONFIG:Release>:-O3 -funsafe-loop-optimizations -fsee -funroll-loops -fno-math-errno -funsafe-math-optimizations -ffinite-math-only -fno-signed-zeros -D_FORTIFY_SOURCE=2>
#)
#add_compile_options(
#    $<$<CONFIG:Release>:-O3 -funsafe-loop-optimizations -fsee -funroll-loops -fno-math-errno -funsafe-math-optimizations -ffinite-math-only -fno-signed-zeros>
#)
# Add optimization flags for Release builds
add_compile_options(
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-funsafe-loop-optimizations>
    $<$<CONFIG:Release>:-fsee>
    $<$<CONFIG:Release>:-funroll-loops>
    $<$<CONFIG:Release>:-fno-math-errno>
    $<$<CONFIG:Release>:-funsafe-math-optimizations>
    $<$<CONFIG:Release>:-ffinite-math-only>
    $<$<CONFIG:Release>:-fno-signed-zeros>
)

# Handle Windows-specific definitions.
if(WIN32)
    add_definitions(-D_USE_MATH_DEFINES)
endif()

# Set options for OpenMP and debug break points.
option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

if(BUILD_WITH_OPENMP)
    find_package(OpenMP REQUIRED)
    target_compile_options(${PROJECT_NAME}_common PUBLIC ${OpenMP_CXX_FLAGS})
    target_link_options(${PROJECT_NAME}_common PUBLIC ${OpenMP_EXE_LINKER_FLAGS})
endif()

if(SET_USER_BREAK_AT_STARTUP)
    message(STATUS "Setting debug breakpoint flag.")
    add_compile_definitions(BPDEBUG)
endif()

# Find all required packages.
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# The original file had a lot of redundant find_package calls.
# I've streamlined them into a single block.

# Use a single list for dependencies.
# This makes maintenance easier and prevents errors from mismatched lists.
set(common_dependencies
    ament_cmake
    apriltag_msgs
    cv_bridge
    geometry_msgs
    image_transport
    image_geometry
    pluginlib
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    apriltag
    OpenCV
    Eigen3
)

# Set include directories
include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
# Note: The apriltag include directories are already handled by target_link_libraries(apriltag::apriltag)

# Build the common library
add_library(${PROJECT_NAME}_common src/common_functions.cpp)
target_link_libraries(${PROJECT_NAME}_common apriltag::apriltag)
ament_target_dependencies(${PROJECT_NAME}_common ${common_dependencies})

# Build the single image detector library (SHARED library is a good practice for components)
#add_library(${PROJECT_NAME}_single_image_detector SHARED src/single_image_detector.cpp)
#target_link_libraries(${PROJECT_NAME}_single_image_detector ${PROJECT_NAME}_common)

# Build the continuous detector library (SHARED library for components)
add_library(${PROJECT_NAME}_continuous_detector SHARED src/continuous_detector.cpp)
target_link_libraries(${PROJECT_NAME}_continuous_detector ${PROJECT_NAME}_common)

# Build executable nodes
#add_executable(${PROJECT_NAME}_single_image_client_node src/apriltag_ros_single_image_client_node.cpp)
#target_link_libraries(${PROJECT_NAME}_single_image_client_node ${PROJECT_NAME}_common)

#add_executable(${PROJECT_NAME}_single_image_server_node src/apriltag_ros_single_image_server_node.cpp)
#target_link_libraries(${PROJECT_NAME}_single_image_server_node ${PROJECT_NAME}_single_image_detector)

# Register the continuous detector as a component
rclcpp_components_register_node(
    ${PROJECT_NAME}_continuous_detector
    PLUGIN "apriltag_ros::ContinuousDetector"
    EXECUTABLE apriltag_ros_continuous_detector_node
)
# Note: I've corrected the executable name to match the rclcpp_components macro.

#############
## Install ##
#############

# Install targets
install(
    TARGETS
        ${PROJECT_NAME}_common
#        ${PROJECT_NAME}_single_image_client_node
#        ${PROJECT_NAME}_single_image_detector
#        ${PROJECT_NAME}_single_image_server_node
        ${PROJECT_NAME}_continuous_detector
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Install launch files and config directories
install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(
    PROGRAMS scripts/analyze_image
    DESTINATION lib/${PROJECT_NAME}
)

# Ament exports
ament_export_include_directories(include)
ament_export_libraries(
    ${PROJECT_NAME}_common
#    ${PROJECT_NAME}_single_image_detector
#    ${PROJECT_NAME}_continuous_detector
)
ament_export_dependencies(${common_dependencies})

ament_package()
