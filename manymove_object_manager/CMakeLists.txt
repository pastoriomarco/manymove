cmake_minimum_required(VERSION 3.14.4)
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

# Core library for ObjectManagerNode logic (without main)
add_library(object_manager_node_lib STATIC src/object_manager_node.cpp)
target_include_directories(object_manager_node_lib PUBLIC include)
ament_target_dependencies(object_manager_node_lib
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

# Add executable for object_manager_node with main function
add_executable(object_manager_node src/object_manager_main.cpp)
target_link_libraries(object_manager_node object_manager_node_lib)

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
  find_package(ament_cmake_test QUIET)
  if(NOT ament_cmake_test_FOUND)
    message(WARNING "ament_cmake_test not found; skipping tests for ${PROJECT_NAME}.")
  else()
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import ament_cmake_test"
      RESULT_VARIABLE AMENT_CMAKE_TEST_IMPORT_RESULT
      OUTPUT_QUIET ERROR_QUIET)
    if(NOT AMENT_CMAKE_TEST_IMPORT_RESULT EQUAL 0)
      message(WARNING
        "Python module 'ament_cmake_test' not importable; skipping tests for ${PROJECT_NAME}.")
    else()
      find_package(ament_lint_auto REQUIRED)
      find_package(ament_cmake_uncrustify REQUIRED)
      ament_lint_auto_find_test_dependencies()

      find_package(ament_cmake_gtest REQUIRED)
      ament_add_gtest(test_object_manager_node
        test/test_object_manager_node.cpp
        test/fake_planning_scene_server.cpp
      )
      target_link_libraries(test_object_manager_node
        object_manager_node_lib
      )
      ament_target_dependencies(test_object_manager_node
        rclcpp
        rclcpp_action
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
    endif()
  endif()
endif()

ament_package()
