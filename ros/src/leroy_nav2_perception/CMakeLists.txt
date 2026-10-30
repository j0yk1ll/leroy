cmake_minimum_required(VERSION 3.8)
project(leroy_nav2_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

ament_python_install_package(${PROJECT_NAME})

foreach(command depth_obstacles live_pipeline_probe live_costmap_probe)
  install(PROGRAMS scripts/${command} DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(costmap_fixture src/costmap_fixture.cpp)
target_compile_features(costmap_fixture PUBLIC cxx_std_17)
ament_target_dependencies(costmap_fixture
  rclcpp rclcpp_lifecycle nav2_costmap_2d nav2_util)
install(TARGETS costmap_fixture DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(perception_python_tests test)
  endif()
endif()

ament_package()
