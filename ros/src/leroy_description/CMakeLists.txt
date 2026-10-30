cmake_minimum_required(VERSION 3.8)
project(leroy_description)

find_package(ament_cmake REQUIRED)

install(
  FILES launch/description.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
