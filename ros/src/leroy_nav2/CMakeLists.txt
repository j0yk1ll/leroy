cmake_minimum_required(VERSION 3.8)
project(leroy_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dry_run_contract test/test_dry_run_contract.py)
endif()

ament_package()
