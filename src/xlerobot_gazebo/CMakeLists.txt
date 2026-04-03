cmake_minimum_required(VERSION 3.8)
project(xlerobot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_relay.py
    scripts/planar_joint_odometry.py
    scripts/twist_stamped_relay.py
    scripts/twist_to_planar_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
