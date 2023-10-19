cmake_minimum_required(VERSION 3.8)
project(launch_remote_ssh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
## DEPENDENCIES ##
##################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

###########
## BUILD ##
###########

# Remote process handler
add_executable(remote_process_handler
  src/remote_process_handler.cpp
)
ament_target_dependencies(remote_process_handler
  rclcpp
)

#############
## INSTALL ##
#############

# Remote process handler
install(TARGETS
  remote_process_handler
  DESTINATION lib/${PROJECT_NAME}
)

# Launch utilities
ament_python_install_package(launch_remote_ssh
  PACKAGE_DIR launch_remote_ssh
  SETUP_CFG setup.cfg
)

# Copy utility ROS executables
install(PROGRAMS
  src/copy_install_space.py
  src/stop_all_remote_processes
  DESTINATION lib/${PROJECT_NAME}
)

# Copy utility scripts
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

# Install custom cmake
install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # TODO(anyone) lint testing
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  find_package(rclpy REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(catch_ros2 REQUIRED)
  include(CTest)

  # Node for testing remote launch capabilities
  install(PROGRAMS
    test/param_node.py
    test/service_node.py
    DESTINATION lib/${PROJECT_NAME}
  )
  # Launch files for testing remote launch capabilities
  install(FILES
    test/launch/test_launch_entry_point.launch.py
    test/launch/test_launch_entry_point.launch.xml
    test/launch/test_launch_entry_point.launch.yaml
    test/launch/test_launch_remotely_launched.launch.py
    test/launch/test_launch_remotely_launched.launch.xml
    test/launch/test_launch_remotely_launched.launch.yaml
    test/node/test_node_entry_point.launch.py
    test/node/test_node_entry_point.launch.xml
    test/node/test_node_entry_point.launch.yaml
    DESTINATION share/${PROJECT_NAME}/launch
  )
  install(FILES
    test/node/test_node_params1.yaml
    test/node/test_node_params2.yaml
    DESTINATION share/${PROJECT_NAME}/test
  )
  # catch_ros2 tester nodes
  add_executable(test_launch_tester_node
    test/launch/test_launch_tester_node.cpp
  )
  target_link_libraries(test_launch_tester_node
    catch_ros2::catch_ros2_with_node_main
  )
  install(TARGETS
    test_launch_tester_node
    DESTINATION lib/${PROJECT_NAME}
  )
  add_executable(test_node_tester_node
    test/node/test_node_tester_node.cpp
  )
  ament_target_dependencies(test_node_tester_node
    std_srvs
  )
  target_link_libraries(test_node_tester_node
    catch_ros2::catch_ros2_with_node_main
  )
  install(TARGETS
    test_node_tester_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # Set directories and include CMake function for use in tests
  set(launch_remote_ssh_DIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_flexible_launch_xmls.cmake)

  launch_remote_ssh_generate_flexible_launch_xmls(
    DESTINATION share/${PROJECT_NAME}/launch
    PACKAGE ${PROJECT_NAME}
    DIRECTORIES
    test/flexible_launch_xmls/to_generate
    FILES
    test/flexible_launch_xmls/test_generate_flexible3.launch.core
    test/flexible_launch_xmls/test_generate_flexible4.launch.core.xml
  )
endif()

ament_export_dependencies(
  rclcpp
)

ament_package(CONFIG_EXTRAS
  cmake/generate_flexible_launch_xmls.cmake
)
