cmake_minimum_required(VERSION 3.8)
project(manymove_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Generate messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MovementConfig.msg"
  "msg/MoveManipulatorGoal.msg"
  "action/PlanManipulator.action"
  "action/ExecuteTrajectory.action"
  "action/LoadTrajController.action"
  "action/UnloadTrajController.action"
  DEPENDENCIES geometry_msgs action_msgs moveit_msgs
)

# Get the typesupport target for linking
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

##########################
# MoveGroupPlanner Library
##########################
add_library(move_group_planner_lib
  src/move_group_planner.cpp
)

# Link dependencies for MoveGroupPlanner
ament_target_dependencies(move_group_planner_lib
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  control_msgs
  rosidl_default_runtime
)

# Link the generated typesupport library
target_link_libraries(move_group_planner_lib ${cpp_typesupport_target})

# Install MoveGroupPlanner library
install(TARGETS
  move_group_planner_lib
  DESTINATION lib
)

##########################
# MoveItCppPlanner Library
##########################
add_library(moveit_cpp_planner_lib
  src/moveit_cpp_planner.cpp
)

# Link dependencies for MoveItCppPlanner
ament_target_dependencies(moveit_cpp_planner_lib
  rclcpp
  moveit_core
  tf2_geometry_msgs
  geometry_msgs
  control_msgs
  rosidl_default_runtime
)

# Link the generated typesupport library
target_link_libraries(moveit_cpp_planner_lib ${cpp_typesupport_target})

# Install MoveItCppPlanner library
install(TARGETS
  moveit_cpp_planner_lib
  DESTINATION lib
)

###########################################
# Move Action Server (Single and Sequence)
###########################################
add_executable(action_server_node
  src/action_server_node.cpp
  src/action_server.cpp
)

# Link both planners to the action server
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
  rosidl_default_runtime
  controller_manager_msgs
)

install(TARGETS action_server_node
  DESTINATION lib/${PROJECT_NAME}
)

#########################
# Standalone MICPP Demo
#########################
add_executable(standalone_micpp_demo src/standalone_micpp_demo.cpp)
ament_target_dependencies(standalone_micpp_demo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
)
target_link_libraries(standalone_micpp_demo ${cpp_typesupport_target})

install(TARGETS standalone_micpp_demo
  DESTINATION lib/${PROJECT_NAME}
)

#########################
# Install launch & config
#########################
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

#########################
# Exporting
#########################
ament_export_libraries(move_group_planner_lib moveit_cpp_planner_lib)
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  action_msgs
  rclcpp_action
  rosidl_default_runtime
)
ament_export_include_directories(include)

ament_package()
