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
ament_python_install_package(launch_remote_ssh PACKAGE_DIR launch_remote_ssh)

# Copy install space utility script
install(PROGRAMS
  scripts/copy_install_space.py
  DESTINATION lib/${PROJECT_NAME}
)

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
    test/launch/test_launch_remotely_launched.launch.py
    test/node/test_node_entry_point.launch.py
    DESTINATION share/${PROJECT_NAME}/launch
  )
  install(FILES
    test/node/test_node_params1.yaml
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
endif()

ament_export_dependencies(
  rclcpp
)

ament_package()
