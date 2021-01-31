cmake_minimum_required(VERSION 3.5)
project(my_rotate_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
### gazebo ros2 control client command
find_package(rclcpp_action REQUIRED)

## COMPILE

### ros2 control hardware plugin
add_library(
  ${PROJECT_NAME}
  SHARED
  src/my_rotate_bot_hardware.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)
### gazebo ros2 control client command
add_executable(gazebo_client_control_message src/gazebo_client_control_message.cpp)
ament_target_dependencies(
  gazebo_client_control_message
  rclcpp
  rclcpp_action
  control_msgs
)

pluginlib_export_plugin_description_file(hardware_interface my_rotate_bot_hardware_plugin.xml)

## INSTALL

### launch files
install(DIRECTORY
  launch
  controllers
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

### ros2 control hardware plugin
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

### gazebo ros2 control client command
install(
  TARGETS gazebo_client_control_message
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
