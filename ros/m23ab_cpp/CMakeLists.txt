cmake_minimum_required(VERSION 3.8)
project(leroy_m23ab_direct_rectifier LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_proc REQUIRED)

# Link the installed, unchanged Jazzy rectifier shared library directly.
# This prevents the executable from using the generated component-factory
# main; it does NOT remove transitive plugin/loader dependencies.
find_library(IMAGE_PROC_RECTIFY_LIBRARY
  NAMES rectify
  PATHS /opt/ros/jazzy/lib
  NO_DEFAULT_PATH
  REQUIRED)

add_executable(rectify_node src/direct_rectify.cpp)
target_compile_features(rectify_node PRIVATE cxx_std_17)
target_compile_options(rectify_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(rectify_node rclcpp image_proc)
target_link_libraries(rectify_node PRIVATE ${IMAGE_PROC_RECTIFY_LIBRARY})

install(TARGETS rectify_node DESTINATION lib/${PROJECT_NAME})
ament_package()
