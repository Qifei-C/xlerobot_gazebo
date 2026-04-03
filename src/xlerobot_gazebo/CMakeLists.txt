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
    scripts/base_twist_utils.py
    scripts/odom_relay.py
    scripts/planar_joint_odometry.py
    scripts/twist_stamped_relay.py
    scripts/twist_to_planar_cmd.py
    scripts/xlerobot_official_base_adapter.py
    scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
