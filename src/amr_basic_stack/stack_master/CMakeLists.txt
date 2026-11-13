cmake_minimum_required(VERSION 3.8)
project(stack_master)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(autonomy_launch test/test_autonomy_semantic_launch.py TIMEOUT 60)
endif()

install(
  DIRECTORY launch config docs rviz scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
