cmake_minimum_required(VERSION 3.5)

project(saye_bringup)

find_package(ament_cmake REQUIRED)
find_package(saye_description REQUIRED)

install(
  DIRECTORY
    launch
    maps
    rviz
    config
  DESTINATION share/${PROJECT_NAME}/
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
