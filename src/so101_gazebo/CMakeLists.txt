cmake_minimum_required(VERSION 3.8)
project(so101_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    assets
    config
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

