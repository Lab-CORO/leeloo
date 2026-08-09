cmake_minimum_required(VERSION 3.8)
project(leeloo_rviz)

find_package(ament_cmake REQUIRED)

# Launch-only package: no compiled targets, just install the assets.
install(DIRECTORY
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
