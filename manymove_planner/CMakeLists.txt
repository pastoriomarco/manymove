cmake_minimum_required(VERSION 3.14.4)
project(manymove_planner)

# Set compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wthread-safety)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(manymove_msgs REQUIRED)
find_package(rcpputils REQUIRED)

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
  trajectory_msgs
  control_msgs
  manymove_msgs
  rcpputils
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
  trajectory_msgs
  control_msgs
  manymove_msgs
  rcpputils
)

##########################
# Build Executable Targets (not exported)
##########################
add_library(manipulator_action_server_lib
  src/action_server.cpp
)
target_link_libraries(manipulator_action_server_lib
  move_group_planner_lib
  moveit_cpp_planner_lib
)
ament_target_dependencies(manipulator_action_server_lib
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  action_msgs
  controller_manager_msgs
  control_msgs
  sensor_msgs
  manymove_msgs
  rcpputils
)
target_include_directories(manipulator_action_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(action_server_node
  src/action_server_node.cpp
)
target_link_libraries(action_server_node
  manipulator_action_server_lib
)
ament_target_dependencies(action_server_node
  rclcpp
  rcpputils
)

add_executable(moveitcpp_action_server_node
  src/moveitcpp_action_server_node.cpp
)
target_link_libraries(moveitcpp_action_server_node
  manipulator_action_server_lib
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
  rcpputils
)

##########################
# Installation
##########################
# Install only the library targets for export.
install(TARGETS
  move_group_planner_lib
  moveit_cpp_planner_lib
  manipulator_action_server_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the executables without exporting them.
install(TARGETS
  action_server_node
  moveitcpp_action_server_node
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
add_library(manymove_planner::manipulator_action_server_lib ALIAS manipulator_action_server_lib)

ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  trajectory_msgs
  action_msgs
  rclcpp_action
  manymove_msgs
  rcpputils
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_manipulator_action_server
    test/test_action_server.cpp
  )
  target_link_libraries(test_manipulator_action_server
    manipulator_action_server_lib
  )
  ament_target_dependencies(test_manipulator_action_server
    rclcpp
    rclcpp_action
    action_msgs
    control_msgs
    controller_manager_msgs
    moveit_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    manymove_msgs
    rcpputils
  )

  find_package(ament_cmake_test QUIET)
  if(NOT ament_cmake_test_FOUND)
    message(WARNING "ament_cmake_test not found; skipping linters for ${PROJECT_NAME}.")
  else()
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import ament_cmake_test"
      RESULT_VARIABLE AMENT_CMAKE_TEST_IMPORT_RESULT
      OUTPUT_QUIET ERROR_QUIET)
    if(NOT AMENT_CMAKE_TEST_IMPORT_RESULT EQUAL 0)
      message(WARNING
        "Python module 'ament_cmake_test' not importable; skipping linters for ${PROJECT_NAME}.")
    else()
      find_package(ament_lint_auto REQUIRED)
      find_package(ament_cmake_uncrustify REQUIRED)
      set(ENV{AMENT_FLake8_ARGS} "--jobs=1")
      set(ENV{AMENT_XMILLINT_SKIP_URL} "1")
      ament_lint_auto_find_test_dependencies()
    endif()
  endif()
endif()

ament_package()
