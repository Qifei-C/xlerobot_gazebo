# Third-Party Assets

This repository contains adapted assets derived from the following upstream projects:

- `XLeRobot`
  - Local source used during integration:
    - `/home/qifei/vendor/XLeRobot`
  - Used for:
    - dual-arm mobile robot meshes
    - ManiSkill / MuJoCo geometry reference
    - mobile base layout reference

- `SO-ARM100 / SO101`
  - Local source used during integration:
    - `/home/qifei/vendor/SO-ARM100`
  - Used for:
    - `SO101` meshes
    - public arm URDF assets

Notes:

- This repository keeps only the assets and integration code needed for the Gazebo + ROS 2 + `ros2_control` workflows in this project.
- The full upstream repositories are not vendored here.
- If you need to refresh meshes or compare against upstream layouts, use the local vendor clones above or re-clone upstream separately.
