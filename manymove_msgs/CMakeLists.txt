cmake_minimum_required(VERSION 3.8)
project(manymove_msgs)

# Set common compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Generate messages, actions, and (if any) services.
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages:
  "msg/MovementConfig.msg"
  "msg/MoveManipulatorGoal.msg"

  # Services:
  "srv/SetBlackboardValues.srv"

  # Actions:
  "action/AddCollisionObject.action"
  "action/RemoveCollisionObject.action"
  "action/CheckObjectExists.action"
  "action/AttachDetachObject.action"
  "action/GetObjectPose.action"

  "action/PlanManipulator.action"
  "action/MoveManipulator.action"
  "action/LoadTrajController.action"
  "action/UnloadTrajController.action"

  "action/SetOutput.action"
  "action/GetInput.action"
  "action/CheckRobotState.action"
  "action/ResetRobotState.action"

DEPENDENCIES
    geometry_msgs
    shape_msgs
    action_msgs
    xarm_msgs
    moveit_msgs
    std_msgs
    std_srvs
    control_msgs
    controller_manager_msgs
)

# Ensure generated headers are properly exported
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
  DESTINATION include/
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

ament_package()