# XLeRobot Gazebo

<img width="1002" height="884" alt="nazebo_xlerobot" src="https://github.com/user-attachments/assets/7a75a148-d21e-43ea-9869-cfbd9b446b19" />


This repository provides a self-contained, containerized Gazebo simulation environment including:

- `Gazebo Harmonic` + `ROS 2 Jazzy`
- A minimal differential-drive robot `demo_bot_gazebo`
- `SO101` single-arm manipulator simulation package `so101_gazebo`
- `XLeRobot` dual-arm mobile robot simulation package `xlerobot_gazebo`
- `ros2_control` + `gz_ros2_control`

## Repository Structure

- `Dockerfile` / `compose.yml`: Container image and runtime entry points
- `ros_entrypoint_overlay.sh`: ROS 2 workspace entry point inside the container
- `scripts/`: Common launch scripts
- `src/demo_bot_gazebo`: Minimal example robot
- `src/so101_gazebo`: `SO101` Gazebo integration package
- `src/xlerobot_gazebo`: `XLeRobot` Gazebo main package
- `THIRD_PARTY.md`: Third-party asset attribution

Overview:

```text
xlerobot_gazebo/
├── Dockerfile
├── compose.yml
├── scripts/
├── src/
│   ├── demo_bot_gazebo/
│   ├── so101_gazebo/
│   └── xlerobot_gazebo/
└── THIRD_PARTY.md
```

## Prerequisites

- Docker (with Docker Compose)
- GUI mode requires an X11 desktop environment and GPU (`/dev/dri`)

## Build

```bash
git clone git@github.com:Qifei-C/xlerobot_gazebo.git && cd xlerobot_gazebo
docker compose build
```

## Headless Mode

Suitable for SSH, terminal verification, and CI scenarios:

```bash
# demo_bot
./scripts/run_headless.sh

# SO101
./scripts/run_so101_headless.sh

# XLeRobot (omni backend, default)
./scripts/run_xlerobot_headless.sh

# XLeRobot (planar backend)
./scripts/run_xlerobot_headless.sh planar
```

## GUI Mode

Run on a local desktop environment (requires `$DISPLAY` to be set):

```bash
# demo_bot
./scripts/run_gui.sh

# SO101
./scripts/run_so101_gui.sh

# XLeRobot (omni backend, default)
./scripts/run_xlerobot_gui.sh

# XLeRobot (planar backend)
./scripts/run_xlerobot_gui.sh planar
```

If Gazebo GUI cannot connect to X11, run this once:

```bash
xhost +SI:localuser:$(id -un)
```

## Entering the Container

The ROS environment is automatically loaded on `docker exec` — no manual `source` needed:

```bash
# GUI container
docker exec -it xlerobot-gazebo-gui bash

# Headless container
docker exec -it xlerobot-gazebo-headless bash
```

## Keyboard Teleop (WASD)

Run inside the container:

```bash
ros2 run xlerobot_gazebo keyboard_teleop.py
```

| Key | Action |
|-----|--------|
| W / S | Forward / Backward |
| A / D | Strafe left / right |
| Q / E | Rotate left / right |
| Space | Stop all axes |
| Esc | Quit |

Keys are independent — press multiple for combined motion (e.g. W + A = forward-left diagonal).

Adjustable parameter:

```bash
ros2 run xlerobot_gazebo keyboard_teleop.py --ros-args \
  -p speed:=0.3
```

## Demo Bot

Check controllers:

```bash
docker exec -it demo-bot-gazebo-headless bash
ros2 control list_controllers
```

Send a forward velocity to `demo_bot`:

```bash
ros2 topic pub --once /diff_drive_base_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

## SO101

Check controllers:

```bash
docker exec -it so101-gazebo-headless bash
ros2 control list_controllers
```

Verified controllers:

- `joint_state_broadcaster`
- `arm_position_controller`

## XLeRobot

Check controllers:

```bash
docker exec -it xlerobot-gazebo-headless bash
ros2 control list_controllers
```

Verified controllers:

- `joint_state_broadcaster`
- `omni_base_controller`
- `left_arm_position_controller`
- `left_gripper_position_controller`
- `right_arm_position_controller`
- `right_gripper_position_controller`
- `head_position_controller`

### Topic Interface

Unified high-level base command interface:

```
/base/cmd_vel    (geometry_msgs/msg/TwistStamped)
```

Rate-limited debug topic:

```
/base/cmd_vel_limited
```

Odometry output:

```
/odom
```

Details:

- With `backend:=omni`: `/base/cmd_vel -> /omni_base_controller/cmd_vel -> wheel joints`
- With `backend:=planar`: `/base/cmd_vel -> /ideal_base_velocity_controller/commands -> root_x/root_y/yaw`
- With `backend:=omni`: odometry is forwarded from `omni_base_controller` to `/odom`
- With `backend:=planar`: odometry is generated from root joint states to `/odom`

### Manual Control Examples

Base:

```bash
# Forward
ros2 topic pub --once /base/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1}}}"

# Strafe
ros2 topic pub --once /base/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {y: 0.1}}}"

# Rotate
ros2 topic pub --once /base/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {angular: {z: 0.5}}}"
```

Arms (5 joints: Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll):

```bash
# Left arm
ros2 topic pub --once /left_arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 0.8, 0.0, 0.0]}"

# Right arm
ros2 topic pub --once /right_arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 0.8, 0.0, 0.0]}"
```

Gripper (1 joint):

```bash
# Open
ros2 topic pub --once /left_gripper_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5]}"

# Close
ros2 topic pub --once /left_gripper_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

Head (pan + tilt):

```bash
ros2 topic pub --once /head_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.3, -0.2]}"
```

### Base Structure

- `base_link`
- `wheel_0_joint` / `wheel_1_joint` / `wheel_2_joint`
- Three wheels equally spaced at 120° intervals; wheel axles point radially toward the base center, wheels drive tangentially
- Custom omni-wheel inverse kinematics with directional friction and WheelSlip plugin
- Three-wheel geometry center aligned with the official `MuJoCo` asset

The `omni` pipeline separates the high-level interface from the low-level plant:

- The high-level always outputs a body twist `u = [vx, vy, wz]`
- The Gazebo low-level maps the body twist to 3 wheel joints
- Wheel radius, base radius, and wheel mounting angles share a single parameter set between the controller and plant
- A body-level limiter enforces speed, acceleration, and wheel speed limits before sending to the wheel velocity controller

The `planar` backend is retained as a debugging fallback:

- Still consumes the same `/base/cmd_vel`
- Bypasses wheel dynamics, directly driving `root_x/root_y/yaw`
- Useful for planner regression, SLAM / perception / pipeline integration testing
- Uses a simplified envelope without wheel speed constraints

### Official Controller Adapter

The repository provides a thin adapter layer from the official XLeRobot action/observation dict to ROS:

- Input topic: `/xlerobot/action_dict`
- Output topic: `/xlerobot/observation_dict`
- Message type: `std_msgs/msg/String`
- Payload format: JSON object

This adapter preserves the official `XLeRobot` whole-body key names:

- Left arm: `left_arm_*.pos`
- Right arm: `right_arm_*.pos`
- Head: `head_motor_1.pos`, `head_motor_2.pos`
- Base: `x.vel`, `y.vel`, `theta.vel`

Base units follow the official 3-wheel base convention:

- `x.vel`: `m/s`
- `y.vel`: `m/s`
- `theta.vel`: `deg/s`

Position values support two representations:

- Default `official_use_degrees:=false`: aligns with the official `use_degrees=False` configuration; non-gripper joints are mapped to `[-100, 100]` based on their respective limits
- Launch with `official_use_degrees:=true`: non-gripper joints use degrees (`deg`), more suitable for manual debugging
- Gripper always uses the official `0..100` semantic

Examples:

```bash
ros2 topic pub --once /xlerobot/action_dict std_msgs/msg/String \
  '{"data":"{\"x.vel\": 0.15, \"y.vel\": 0.0, \"theta.vel\": 0.0}"}'
```

```bash
ros2 topic echo /xlerobot/observation_dict --once
```

Launch with degree mode:

```bash
ros2 launch xlerobot_gazebo sim.launch.py headless:=true backend:=omni official_use_degrees:=true
```

### Base Envelope

The `omni` pipeline currently uses a conservative real-hardware envelope:

- Max planar speed: `0.3 m/s`
- Max angular speed: `1.57 rad/s`
- Max planar acceleration: `0.6 m/s^2`
- Max angular acceleration: `3.14 rad/s^2`
- Max wheel speed: `9.0 rad/s`

Local parameter files:

- `src/xlerobot_gazebo/config/base_geometry.yaml`
- `src/xlerobot_gazebo/config/base_envelope_real.yaml`
- `src/xlerobot_gazebo/config/planner_test_envelope.yaml`

## Stopping

```bash
docker compose down
```

If containers were launched via standalone scripts, stop them directly:

```bash
docker rm -f so101-gazebo-headless
docker rm -f so101-gazebo-gui
docker rm -f xlerobot-gazebo-headless
docker rm -f xlerobot-gazebo-gui
```

## Local Development Notes

`compose.yml` uses `network_mode: host` and `ipc: host` to simplify Gazebo / ROS 2 networking and shared memory configuration for local development.

This compose configuration is designed for local development machines and is not intended as a general-purpose container template for multi-user or strong-isolation environments.
