cmake_minimum_required(VERSION 3.8)
project(manymove_signals)

# Default to C++17 if not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(manymove_msgs REQUIRED)

# Include directories (if you have local headers)
include_directories(include)

# Add executable for the action server node
add_executable(signals_node src/signals_node.cpp)

# Specify libraries and dependencies for signals_node
ament_target_dependencies(signals_node
  rclcpp
  rclcpp_action
  action_msgs
  xarm_msgs
  manymove_msgs
)

# Install the executable
install(TARGETS
  signals_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies for downstream users
ament_export_dependencies(
  rclcpp
  rclcpp_action
  action_msgs
  xarm_msgs
  manymove_msgs
)

ament_package()
