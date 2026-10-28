cmake_minimum_required(VERSION 3.8)
project(m23bv_reliable_rectifier)

# Isolated no-device candidate; never replace the system image_proc package.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_proc REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tracetools_image_pipeline REQUIRED)
find_package(OpenCV REQUIRED)

add_library(m23bv_rectify_reliable_component SHARED src/rectify.cpp)
target_compile_definitions(m23bv_rectify_reliable_component PRIVATE COMPOSITION_BUILDING_DLL)
ament_target_dependencies(m23bv_rectify_reliable_component
  rclcpp rclcpp_components image_proc image_transport image_geometry cv_bridge
  sensor_msgs tracetools_image_pipeline)
target_link_libraries(m23bv_rectify_reliable_component ${OpenCV_LIBRARIES})
rclcpp_components_register_node(m23bv_rectify_reliable_component
  PLUGIN "image_proc::RectifyNode"
  EXECUTABLE m23bv_rectify_reliable_node)
install(TARGETS m23bv_rectify_reliable_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_package()
