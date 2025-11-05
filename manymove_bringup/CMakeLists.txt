cmake_minimum_required(VERSION 3.16)
project(manymove_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY
    config
    launch
    urdf
    srdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
  FILES_MATCHING PATTERN "*"
)

ament_package()
