# This fork supports only ROS 2 Jazzy and ROS 2 Lyrical.
cmake_minimum_required(VERSION 3.14)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

include(cmake/version.cmake)
project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES C CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

if(NOT DEFINED DISTRO_ROS OR DISTRO_ROS STREQUAL "")
  set(DISTRO_ROS "$ENV{ROS_DISTRO}")
endif()
string(TOLOWER "${DISTRO_ROS}" DISTRO_ROS)
if(NOT DISTRO_ROS MATCHES "^(jazzy|lyrical)$")
  message(FATAL_ERROR "Unsupported ROS 2 distro '${DISTRO_ROS}'. Supported distros: jazzy, lyrical.")
endif()
message(STATUS "Building for ROS 2 ${DISTRO_ROS}")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if(APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif()

set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)
find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES "livox_lidar_api.h" "livox_lidar_def.h"
  REQUIRED
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  ${APR_INCLUDE_DIRS}
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  3rdparty
  src
)

target_link_libraries(${PROJECT_NAME}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  "${cpp_typesupport_target}"
  ${PPT_LIBRARY}
  ${APR_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_library(livox_pointcloud_throttle_component SHARED
  src/pointcloud_throttle_node.cpp
)
target_include_directories(livox_pointcloud_throttle_component PUBLIC src)
rclcpp_components_register_nodes(livox_pointcloud_throttle_component
  "livox_ros::PointCloudThrottleNode"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch_ROS2
)
