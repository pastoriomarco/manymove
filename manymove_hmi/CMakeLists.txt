cmake_minimum_required(VERSION 3.8)
project(manymove_hmi VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard and require it.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automatic handling of Qt meta-object code, UI files, and resource files.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find ament and required ROS2 packages.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Find Qt5 packages.
find_package(Qt5 REQUIRED COMPONENTS Widgets Network)

# Include the "include" directory (which contains your headers, e.g. include/manymove_hmi/hmi_gui.hpp)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# List your source files.
# Adding the header with Q_OBJECT explicitly helps automoc pick it up reliably.
set(SOURCES
  src/main.cpp
  src/hmi_gui.cpp
  src/ros2_worker.cpp
  include/manymove_hmi/hmi_gui.hpp
)

# Create the executable.
add_executable(${PROJECT_NAME}_executable ${SOURCES})

# Link against the ROS2 libraries.
ament_target_dependencies(${PROJECT_NAME}_executable
  rclcpp
  std_msgs
  std_srvs
)

# Link against the required Qt5 libraries.
target_link_libraries(${PROJECT_NAME}_executable
  Qt5::Widgets
  Qt5::Network
)

# Install the executable to the lib/manymove_hmi directory.
install(TARGETS ${PROJECT_NAME}_executable
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
