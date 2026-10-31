cmake_minimum_required(VERSION 3.16)
project(leroy_localization LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# Build the upstream XRSLAM superbuild separately. The ROS adapter is not
# included from its top-level CMakeLists.txt and does not use catkin.
set(XRSLAM_ROOT "/opt/xrslam" CACHE PATH "Standalone XRSLAM source root")
set(XRSLAM_LIBRARY "" CACHE FILEPATH "Full path to separately built libxrslam.so")
if(NOT XRSLAM_LIBRARY)
  find_library(XRSLAM_LIBRARY NAMES xrslam
    HINTS "${XRSLAM_ROOT}/lib" "${XRSLAM_ROOT}/build-eigenplaces-cpu/xrslam-interface"
    REQUIRED)
endif()
if(NOT EXISTS "${XRSLAM_LIBRARY}")
  message(FATAL_ERROR "Build XRSLAM first and set -DXRSLAM_LIBRARY=/absolute/path/libxrslam.so")
endif()
if(NOT EXISTS "${XRSLAM_ROOT}/xrslam-interface/include/XRSLAM.h")
  message(FATAL_ERROR "XRSLAM_ROOT must point to the XRSLAM fork")
endif()

add_executable(leroy_xrslam_node src/leroy_xrslam_node.cpp)
target_include_directories(leroy_xrslam_node PRIVATE
  "${XRSLAM_ROOT}/xrslam-interface/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(leroy_xrslam_node "${XRSLAM_LIBRARY}" yaml-cpp)
ament_target_dependencies(leroy_xrslam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
target_compile_options(leroy_xrslam_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS leroy_xrslam_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/integration/test_xrslam_startup_contract.py")
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_xrslam_startup_contract
      "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/integration/test_xrslam_startup_contract.py")
  endif()
  foreach(name test_startup_timing test_sensor_modes)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/unit/${name}.cpp")
      add_executable(${name} "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/unit/${name}.cpp")
      target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
      add_test(NAME ${name} COMMAND ${name})
    endif()
  endforeach()
endif()

ament_package()
