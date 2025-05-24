cmake_minimum_required(VERSION 3.8)
project(manymove_hmi VERSION 0.1.0 LANGUAGES CXX)

# ─────────────────────────────────────────────────────────────────────────────
#  Global compiler / Qt options
# ─────────────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ─────────────────────────────────────────────────────────────────────────────
#  ROS 2 & Qt dependencies
# ─────────────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(std_srvs      REQUIRED)
find_package(manymove_msgs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ─────────────────────────────────────────────────────────────────────────────
# 1.  Generic HMI library (all Qt classes live here)
# ─────────────────────────────────────────────────────────────────────────────
set(HMI_LIB_SOURCES
  # -------- implementation (.cpp) ----------
  src/hmi_gui.cpp
  src/ros2_worker.cpp
  src/app_module.cpp

  # -------- headers that contain Q_OBJECT ----------
  include/manymove_hmi/hmi_gui.hpp
  include/manymove_hmi/ros2_worker.hpp
  include/manymove_hmi/app_module.hpp
)

add_library(manymove_hmi_lib ${HMI_LIB_SOURCES})

target_link_libraries(manymove_hmi_lib
  Qt5::Core Qt5::Widgets Qt5::Network)

ament_target_dependencies(manymove_hmi_lib
  rclcpp std_msgs std_srvs manymove_msgs)

# ─────────────────────────────────────────────────────────────────────────────
# 2.  “Generic” HMI executable (just shows the GUI)
# ─────────────────────────────────────────────────────────────────────────────
add_executable(manymove_hmi_executable
  src/main.cpp)

target_link_libraries(manymove_hmi_executable
  manymove_hmi_lib
  Qt5::Core Qt5::Widgets Qt5::Network)

ament_target_dependencies(manymove_hmi_executable
  rclcpp std_msgs std_srvs manymove_msgs)

# ─────────────────────────────────────────────────────────────────────────────
# 3.  Customer-specific HMI executable (adds AppModule logic)
# ─────────────────────────────────────────────────────────────────────────────
add_executable(manymove_hmi_app_executable
  src/main_app.cpp)

target_link_libraries(manymove_hmi_app_executable
  manymove_hmi_lib
  Qt5::Core Qt5::Widgets Qt5::Network)

ament_target_dependencies(manymove_hmi_app_executable
  rclcpp std_msgs std_srvs manymove_msgs)

# ─────────────────────────────────────────────────────────────────────────────
# 4.  Installation & export  (⚠ the export part is important for other pkgs)
# ─────────────────────────────────────────────────────────────────────────────
# -- our headers so that “#include <manymove_hmi/…>” works elsewhere
install(DIRECTORY include/ DESTINATION include)

# -- the library itself
install(TARGETS
  manymove_hmi_lib
  EXPORT  export_manymove_hmi_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# -- the runnable binaries
install(TARGETS
  manymove_hmi_executable
  manymove_hmi_app_executable
  DESTINATION lib/${PROJECT_NAME})

# ---------------- NEW: make sure we actually export the headers -------------
ament_export_include_directories(include)
ament_export_targets(export_manymove_hmi_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs std_srvs manymove_msgs)

ament_package()
