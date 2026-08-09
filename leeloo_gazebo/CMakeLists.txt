cmake_minimum_required(VERSION 3.8)
project(leeloo_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/dsr_sim_bridge.py
  ${PROJECT_NAME}/base_sim_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# `ros2 run leeloo_gazebo dsr_sim_bridge` (no .py) -- matches how the rest of the
# workspace's nodes are invoked from launch files.
install(CODE "
  execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    dsr_sim_bridge.py
    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/dsr_sim_bridge)
  execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    base_sim_bridge.py
    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/base_sim_bridge)
")

install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
