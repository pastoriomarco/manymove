cmake_minimum_required(VERSION 3.8)
project(manymove_planner)

# Set compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(manymove_msgs REQUIRED)

# Include public headers
include_directories(include)

##########################
# Build Library Targets
##########################

# Build the MoveGroupPlanner Library
add_library(move_group_planner_lib
  src/move_group_planner.cpp
)
ament_target_dependencies(move_group_planner_lib
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  control_msgs
  manymove_msgs
)

# Build the MoveItCppPlanner Library
add_library(moveit_cpp_planner_lib
  src/moveit_cpp_planner.cpp
)
ament_target_dependencies(moveit_cpp_planner_lib
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  control_msgs
  manymove_msgs
)

##########################
# Build Executable Targets (not exported)
##########################
add_executable(action_server_node
  src/action_server_node.cpp
  src/action_server.cpp
)
target_link_libraries(action_server_node
  move_group_planner_lib
  moveit_cpp_planner_lib
)
ament_target_dependencies(action_server_node
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  action_msgs
  controller_manager_msgs
  manymove_msgs
)

add_executable(moveitcpp_action_server_node
  src/moveitcpp_action_server_node.cpp
  src/action_server.cpp
)
target_link_libraries(moveitcpp_action_server_node
  move_group_planner_lib
  moveit_cpp_planner_lib
)
ament_target_dependencies(moveitcpp_action_server_node
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  action_msgs
  controller_manager_msgs
  manymove_msgs
)

add_executable(standalone_micpp_demo
  src/standalone_micpp_demo.cpp
)
ament_target_dependencies(standalone_micpp_demo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
  manymove_msgs
)

##########################
# Installation
##########################
# Install only the library targets for export.
install(TARGETS
  move_group_planner_lib
  moveit_cpp_planner_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the executables without exporting them.
install(TARGETS
  action_server_node
  moveitcpp_action_server_node
  standalone_micpp_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install public headers and launch/config directories
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

##########################
# Export Targets and Dependencies
##########################
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Create alias targets for easier linking:
add_library(manymove_planner::move_group_planner_lib ALIAS move_group_planner_lib)
add_library(manymove_planner::moveit_cpp_planner_lib ALIAS moveit_cpp_planner_lib)

ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  action_msgs
  rclcpp_action
  manymove_msgs
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Skip checks that are not yet enforced in this package
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
