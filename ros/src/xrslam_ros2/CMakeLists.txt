cmake_minimum_required(VERSION 3.16)
project(xrslam_ros2 VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(NOT XRSLAM_SOURCE_ROOT AND DEFINED ENV{XRSLAM_SOURCE_ROOT})
  set(XRSLAM_SOURCE_ROOT "$ENV{XRSLAM_SOURCE_ROOT}")
endif()
if(NOT XRSLAM_SOURCE_ROOT)
  message(FATAL_ERROR "Set -DXRSLAM_SOURCE_ROOT=/home/kevin/Code/xrslam (already built)")
endif()
if(NOT EXISTS "${XRSLAM_SOURCE_ROOT}/xrslam-interface/include/XRSLAM.h")
  message(FATAL_ERROR "XRSLAM.h not found under XRSLAM_SOURCE_ROOT")
endif()
find_library(XRSLAM_SHARED_LIBRARY NAMES xrslam
  PATHS "${XRSLAM_SOURCE_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

add_executable(xrslam_ros2_node src/xrslam_ros2_node.cpp)
target_compile_features(xrslam_ros2_node PRIVATE cxx_std_17)
target_include_directories(xrslam_ros2_node PRIVATE
  include "${XRSLAM_SOURCE_ROOT}/xrslam-interface/include")
ament_target_dependencies(xrslam_ros2_node
  rclcpp sensor_msgs geometry_msgs nav_msgs diagnostic_msgs tf2 tf2_ros)
target_link_libraries(xrslam_ros2_node "${XRSLAM_SHARED_LIBRARY}")
# Local developer install; upstream XRSLAM is not yet an ament library.
get_filename_component(XRSLAM_LIB_DIR "${XRSLAM_SHARED_LIBRARY}" DIRECTORY)
set_target_properties(xrslam_ros2_node PROPERTIES
  BUILD_RPATH "${XRSLAM_LIB_DIR}"
  INSTALL_RPATH "${XRSLAM_LIB_DIR}")
install(TARGETS xrslam_ros2_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
