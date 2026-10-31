cmake_minimum_required(VERSION 3.8)
project(leroy_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

foreach(command depth_obstacles depthart_node)
  install(PROGRAMS scripts/${command} DESTINATION lib/${PROJECT_NAME})
endforeach()


install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
