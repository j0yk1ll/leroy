cmake_minimum_required(VERSION 3.8)
project(leroy_m23ac_loaded_rectify_lifetime LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# Both A/B arms build and run the same component manager executable.
# The ONLY in-process experimental difference is RTLD_NODELETE in an
# RTLD_NOLOAD promotion after the normal class_loader factory is created.
add_executable(component_container src/loaded_rectify_container.cpp)
target_compile_features(component_container PRIVATE cxx_std_17)
target_compile_options(component_container PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(component_container rclcpp rclcpp_components)
# Plain signature is required because ament_target_dependencies uses plain.
target_link_libraries(component_container dl)

install(TARGETS component_container DESTINATION lib/${PROJECT_NAME})
ament_package()
