cmake_minimum_required(VERSION 3.8)
project(manymove_object_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(manymove_msgs REQUIRED)

# Include directories (if you have local includes)
include_directories(include)

# Add executable for object_manager_node with main function
add_executable(object_manager_node src/object_manager_node.cpp)
ament_target_dependencies(object_manager_node
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
  geometric_shapes
  manymove_msgs
)

# Add executable for collision_spawner
add_executable(collision_spawner src/collision_spawner.cpp)
ament_target_dependencies(collision_spawner
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
  yaml-cpp
  manymove_msgs
)

# Simply link yaml-cpp (and any other libraries) if needed.
target_link_libraries(collision_spawner yaml-cpp)

# Install executables
install(TARGETS
  object_manager_node
  collision_spawner
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes/
        DESTINATION share/${PROJECT_NAME}/meshes)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
