cmake_minimum_required(VERSION 3.8)
project(manymove_cpp_trees)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(manymove_planner REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(manymove_object_manager REQUIRED)
find_package(manymove_signals REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(manymove_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable for BT client
add_executable(bt_client
    src/bt_client.cpp
    src/action_nodes_planner.cpp
    src/action_nodes_signals.cpp
    src/action_nodes_objects.cpp
    src/action_nodes_logic.cpp
    src/hmi_service_node.cpp
    src/tree_helper.cpp
)

ament_target_dependencies(bt_client
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  manymove_planner
  manymove_object_manager
  manymove_signals
  moveit_msgs
  tf2
  std_srvs
  manymove_msgs
)

# Add executable for BT client for dual robot
add_executable(bt_client_dual
    src/bt_client_dual.cpp
    src/action_nodes_planner.cpp
    src/action_nodes_signals.cpp
    src/action_nodes_objects.cpp
    src/action_nodes_logic.cpp
    src/hmi_service_node.cpp
    src/tree_helper.cpp
)

ament_target_dependencies(bt_client_dual
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  manymove_planner
  manymove_object_manager
  manymove_signals
  moveit_msgs
  tf2
  std_srvs
  manymove_msgs
)

# Add executable for BT client for dual robot
add_executable(bt_client_app_dual
    src/bt_client_app_dual.cpp
    src/action_nodes_planner.cpp
    src/action_nodes_signals.cpp
    src/action_nodes_objects.cpp
    src/action_nodes_logic.cpp
    src/hmi_service_node.cpp
    src/tree_helper.cpp
)

ament_target_dependencies(bt_client_app_dual
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  manymove_planner
  manymove_object_manager
  manymove_signals
  moveit_msgs
  tf2
  std_srvs
  manymove_msgs
)

# Add executable for BT client
add_executable(bt_client_panda
    src/bt_client_panda.cpp
    src/action_nodes_planner.cpp
    src/action_nodes_signals.cpp
    src/action_nodes_objects.cpp
    src/action_nodes_logic.cpp
    src/hmi_service_node.cpp
    src/tree_helper.cpp
)

ament_target_dependencies(bt_client_panda
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  manymove_planner
  manymove_object_manager
  manymove_signals
  moveit_msgs
  tf2
  std_srvs
  manymove_msgs
)

# Install executable
install(TARGETS
  bt_client
  bt_client_dual
  bt_client_app_dual
  bt_client_panda
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (tree_helper.hpp, etc.)
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
