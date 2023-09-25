cmake_minimum_required(VERSION 3.8)
project(launch_remote_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
## DEPENDENCIES ##
##################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

###########
## BUILD ##
###########

# Launch manager
add_executable(launch_manager
  src/launch_manager.cpp
)
ament_target_dependencies(launch_manager
  rclcpp
)

#############
## INSTALL ##
#############

# Launch manager
install(TARGETS
  launch_manager
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
endif()

ament_package()
