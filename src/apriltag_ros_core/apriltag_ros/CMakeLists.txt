cmake_minimum_required(VERSION 3.5)
project(apriltag_ros)

#set(CMAKE_CXX_STANDARD 17)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

# Compiler Defense Flags
if(UNIX OR APPLE)
  # Linker flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security -Wall")
  # Dot not forward c++ flag to GPU beucause it is not supported
  set( CUDA_PROPAGATE_HOST_FLAGS OFF )
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_FORTIFY_SOURCE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
endif()

if(WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)
if (${uppercase_CMAKE_BUILD_TYPE} STREQUAL "RELEASE")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
    message(STATUS "Create Debug Build.")
endif()

if(BUILD_WITH_OPENMP)
    find_package(OpenMP)
    if(NOT OpenMP_FOUND)
        message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
endif()

if(SET_USER_BREAK_AT_STARTUP)
    message("GOT FLAG IN CmakeLists.txt")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBPDEBUG")
endif()



#find_package(catkin REQUIRED COMPONENTS
#  cmake_modules
#  cv_bridge
#  geometry_msgs
#  image_geometry
#  image_transport
#  message_generation
#  nodelet
#  pluginlib
#  roscpp
#  sensor_msgs
#  std_msgs
#  tf
#)


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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag REQUIRED)
find_package(apriltag_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# We default to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Coverage" "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

add_compile_options("-O3" "-funsafe-loop-optimizations" "-fsee" "-funroll-loops" "-fno-math-errno" "-funsafe-math-optimizations" "-ffinite-math-only" "-fno-signed-zeros")

# Extract the include directories and libraries from apriltag::apriltag as catkin_package() does not support modern cmake.
get_target_property(apriltag_INCLUDE_DIRS apriltag::apriltag INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(apriltag_LIBRARIES apriltag::apriltag INTERFACE_LINK_LIBRARIES)

include_directories(include)

include_directories(${OpenCV_INCLUDE_DIRS})  # add OpenCV includes to the included dirs
include_directories(${roscpp_INCLUDE_DIRS})  # add OpenCV includes to the included dirs
include_directories(${apriltag_INCLUDE_DIRS})  # add OpenCV includes to the included dirs

set(SOURCES
#    src/apriltag_ros_continuous_node.cpp
#    src/apriltag_ros_single_image_client_node.cpp
#    src/apriltag_ros_single_image_server_node.cpp
    src/common_functions.cpp
    src/continuous_detector.cpp
#    src/single_image_detector.cpp
  )
  
set(INCLUDES
    include/apriltag_ros/common_functions.h
    include/apriltag_ros/continuous_detector.h
#    include/apriltag_ros/single_image_detector.h
)

set(dependencies
    OpenCV
    apriltag_msgs
    cv_bridge
    image_transport
    tf2_ros
    tf2_geometry_msgs
    image_geometry
    rclcpp_components
    std_srvs
    )
    

set(ament_dependencies
  cv_bridge
  geometry_msgs
  image_transport
  rclcpp
  rclcpp_components
  pluginlib
  sensor_msgs
  std_msgs
  std_srvs
  apriltag
  apriltag_msgs
  OpenCV
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
)
set (link_dependencies
  ${PROJECT_NAME}_common
  #${PROJECT_NAME}_continuous_detector
  ${PROJECT_NAME}_single_image_detector
)

add_library(${PROJECT_NAME}_common src/common_functions.cpp)
target_link_libraries(${PROJECT_NAME}_common apriltag)
ament_target_dependencies(${PROJECT_NAME}_common ${dependencies})

add_library(${PROJECT_NAME}_single_image_detector src/single_image_detector.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_detector ${PROJECT_NAME}_common)

add_library(${PROJECT_NAME}_continuous_node SHARED src/continuous_detector.cpp)
target_link_libraries(${PROJECT_NAME}_continuous_node ${PROJECT_NAME}_common)
ament_target_dependencies(${PROJECT_NAME}_continuous_node ${common_dependencies})

add_executable(${PROJECT_NAME}_single_image_client_node src/${PROJECT_NAME}_single_image_client_node.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_client_node ${PROJECT_NAME}_common)
ament_target_dependencies(${PROJECT_NAME}_single_image_client_node ${common_dependencies})

#add_executable(${PROJECT_NAME}_continuous_node src/${PROJECT_NAME}_continuous_node.cpp)
#target_link_libraries(${PROJECT_NAME}_continuous_node ${PROJECT_NAME}_continuous_detector_component)

add_executable(${PROJECT_NAME}_single_image_server_node src/${PROJECT_NAME}_single_image_server_node.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_server_node ${PROJECT_NAME}_single_image_detector)
ament_target_dependencies(${PROJECT_NAME}_single_image_server_node ${common_dependencies})



#add_library(${PROJECT_NAME} SHARED
#    ${INCLUDES}
#    ${SOURCES}
#)

#ament_target_dependencies(${PROJECT_NAME}
#  ${ament_dependencies}
#)

#target_link_libraries(${PROJECT_NAME} ${link_dependencies})

rclcpp_components_register_node(${PROJECT_NAME}_continuous_node
  PLUGIN "apriltag_ros::ContinuousDetector"
  EXECUTABLE apriltag_ros_continuous_detector_node
)


#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_common
  ${PROJECT_NAME}_single_image_client_node
  ${PROJECT_NAME}_single_image_detector
  ${PROJECT_NAME}_single_image_server_node
  ${PROJECT_NAME}_continuous_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Install binaries
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY 
    config launch
    DESTINATION share/${PROJECT_NAME}
    )


install(PROGRAMS scripts/analyze_image DESTINATION bin)

# Ament exports
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_common)
ament_export_libraries(${PROJECT_NAME}_single_image_client_node)
ament_export_libraries(${PROJECT_NAME}_single_image_detector)
ament_export_libraries(${PROJECT_NAME}_single_image_server_node)
ament_export_libraries(${PROJECT_NAME}_continuous_node)
ament_export_dependencies(${dependencies})

ament_package()

