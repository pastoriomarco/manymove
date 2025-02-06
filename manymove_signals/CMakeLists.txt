cmake_minimum_required(VERSION 3.8)
project(manymove_signals)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(manymove_planner REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate actions with both C and C++ type support
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SetOutput.action"
  "action/GetInput.action"
  "action/CheckRobotState.action"
  "action/ResetRobotState.action"
  DEPENDENCIES xarm_msgs action_msgs
)

# Include directories
include_directories(include)

# Declare ament dependencies for export
ament_export_dependencies(rosidl_default_runtime)

# Add executable for the action server node
add_executable(signals_node src/signals_node.cpp)

# Specify libraries against which to link
ament_target_dependencies(signals_node
  rclcpp
  rclcpp_action
  action_msgs
  xarm_msgs
  manymove_planner
)

# Get the typesupport target for linking
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Get the typesupport target for C
rosidl_get_typesupport_target(c_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_c")

# Link C and C++ typesupport libraries
target_link_libraries(signals_node ${cpp_typesupport_target} ${c_typesupport_target})


# Link generated action libraries
target_link_libraries(signals_node ${cpp_typesupport_target})

# Install executable
install(TARGETS
  signals_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install action files
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_action
  action_msgs
  xarm_msgs
  manymove_planner
)

ament_package()
