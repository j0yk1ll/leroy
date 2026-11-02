cmake_minimum_required(VERSION 3.8)
project(leroy_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(leroy_safety REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/readiness_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
