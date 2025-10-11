cmake_minimum_required(VERSION 3.8)
project(manymove_cpp_trees)

#--------------------------------------------------------------------
# Global settings
#--------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#--------------------------------------------------------------------
# Dependencies
#--------------------------------------------------------------------
find_package(ament_cmake              REQUIRED)
find_package(rclcpp                   REQUIRED)
find_package(rclcpp_action            REQUIRED)
find_package(behaviortree_cpp_v3      REQUIRED)
find_package(manymove_planner         REQUIRED)
find_package(moveit_msgs              REQUIRED)
find_package(geometry_msgs            REQUIRED)
find_package(manymove_object_manager  REQUIRED)
find_package(tf2                      REQUIRED)
find_package(tf2_ros                  REQUIRED)
find_package(tf2_geometry_msgs        REQUIRED)
find_package(std_srvs                 REQUIRED)
find_package(manymove_msgs            REQUIRED)
find_package(control_msgs             REQUIRED)
find_package(std_msgs                 REQUIRED)
find_package(topic_based_ros2_control REQUIRED)
find_package(simulation_interfaces    REQUIRED)
find_package(vision_msgs              REQUIRED)

#--------------------------------------------------------------------
# Library with all BT helper nodes
#--------------------------------------------------------------------
add_library(manymove_cpp_trees_lib
  src/action_nodes_planner.cpp
  src/action_nodes_objects.cpp
  src/action_nodes_signals.cpp
  src/action_nodes_logic.cpp
  src/action_nodes_gripper.cpp
  src/action_nodes_isaac.cpp
  src/hmi_service_node.cpp
  src/tree_helper.cpp
)

target_include_directories(manymove_cpp_trees_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(manymove_cpp_trees_lib
  rclcpp rclcpp_action behaviortree_cpp_v3
  manymove_planner manymove_object_manager
  moveit_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs manymove_msgs control_msgs
  std_msgs topic_based_ros2_control simulation_interfaces vision_msgs
)

#--------------------------------------------------------------------
# Helper macro to create BT-client executables
#--------------------------------------------------------------------
macro(bt_client_exec NAME)
  add_executable(${NAME} src/${NAME}.cpp)
  # *** plain signature – no PUBLIC / PRIVATE keywords ***
  target_link_libraries(${NAME} manymove_cpp_trees_lib)

  ament_target_dependencies(${NAME}
    rclcpp rclcpp_action behaviortree_cpp_v3
    manymove_planner manymove_object_manager
    moveit_msgs tf2 std_srvs manymove_msgs control_msgs
  std_msgs topic_based_ros2_control simulation_interfaces
  )
endmacro()

bt_client_exec(bt_client)
bt_client_exec(bt_client_mixed_pipelines)
bt_client_exec(bt_client_cumotion)
bt_client_exec(bt_client_isaac)
bt_client_exec(bt_client_dual)
bt_client_exec(bt_client_app_dual)
bt_client_exec(bt_client_panda)
bt_client_exec(bt_client_panda_cumotion)
bt_client_exec(bt_client_uf850)
bt_client_exec(bt_client_xarm7)
bt_client_exec(collision_test)
bt_client_exec(tutorial_01)
bt_client_exec(bt_client_foundationpose)

#--------------------------------------------------------------------
# Installation & export
#--------------------------------------------------------------------
install(TARGETS
  manymove_cpp_trees_lib
  EXPORT export_manymove_cpp_trees_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  bt_client
  bt_client_mixed_pipelines
  bt_client_cumotion
  bt_client_isaac
  bt_client_dual
  bt_client_app_dual
  bt_client_panda
  bt_client_panda_cumotion
  bt_client_uf850
  bt_client_xarm7
  collision_test
  tutorial_01
  bt_client_foundationpose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_targets(export_manymove_cpp_trees_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_action behaviortree_cpp_v3
  manymove_planner manymove_object_manager
  moveit_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs manymove_msgs control_msgs
  std_msgs topic_based_ros2_control simulation_interfaces vision_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Disable flake8 checks in this C++-only package
  set(ament_cmake_flake8_FOUND TRUE)
  # Disable checks that are not applicable for this package's current state
  # (e.g. copyright headers not present in all files).
  set(AMENT_UNCRUSTIFY_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../uncrustify.cfg)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
