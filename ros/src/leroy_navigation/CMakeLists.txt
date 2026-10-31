cmake_minimum_required(VERSION 3.8)
project(leroy_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dry_run_contract ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/integration/test_navigation_contract.py)
endif()

ament_package()
