cmake_minimum_required(VERSION 3.14.4)
project(manymove_hmi VERSION 0.1.0 LANGUAGES CXX)

# ──────────────────────────────────────────────────────────────
#  Global / Qt
# ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ──────────────────────────────────────────────────────────────
#  ROS 2 & Qt deps
# ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(std_srvs      REQUIRED)
find_package(manymove_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ──────────────────────────────────────────────────────────────
# 1.  Core HMI library  (all Qt widgets)
# ──────────────────────────────────────────────────────────────
set(HMI_LIB_SOURCES
  src/hmi_gui.cpp
  src/ros2_worker.cpp
  src/app_module.cpp
  src/default_app_hmi.cpp          # <── NEW
  ## headers that contain Q_OBJECT
  include/manymove_hmi/hmi_gui.hpp
  include/manymove_hmi/ros2_worker.hpp
  include/manymove_hmi/app_module.hpp
  include/manymove_hmi/default_app_hmi.hpp
)

add_library(manymove_hmi_lib ${HMI_LIB_SOURCES})

target_link_libraries(manymove_hmi_lib Qt5::Core Qt5::Widgets Qt5::Network)
ament_target_dependencies(manymove_hmi_lib
  rclcpp std_msgs std_srvs manymove_msgs)

# ──────────────────────────────────────────────────────────────
# 2.  Minimal “empty-GUI” executable (still useful sometimes)
# ──────────────────────────────────────────────────────────────
add_executable(manymove_hmi_executable src/main.cpp)
target_link_libraries(manymove_hmi_executable
  manymove_hmi_lib Qt5::Core Qt5::Widgets Qt5::Network)
ament_target_dependencies(manymove_hmi_executable
  rclcpp std_msgs std_srvs manymove_msgs)

# ──────────────────────────────────────────────────────────────
# 3.  Default application (uses DefaultAppModule)
# ──────────────────────────────────────────────────────────────
add_executable(manymove_hmi_app_executable src/main_app.cpp)
target_link_libraries(manymove_hmi_app_executable
  manymove_hmi_lib Qt5::Core Qt5::Widgets Qt5::Network)
ament_target_dependencies(manymove_hmi_app_executable
  rclcpp std_msgs std_srvs manymove_msgs)

# ──────────────────────────────────────────────────────────────
# 4.  Install / Export
# ──────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)

install(TARGETS manymove_hmi_lib
        EXPORT  export_manymove_hmi_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(TARGETS
        manymove_hmi_executable
        manymove_hmi_app_executable
        DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_manymove_hmi_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs std_srvs manymove_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
