# XLeRobot Gazebo

这个仓库提供一套独立的容器化 Gazebo 环境，内容包括：

- `Gazebo Harmonic` + `ROS 2 Jazzy`
- 一个最小差速机器人 `demo_bot_gazebo`
- `SO101` 单臂机械臂仿真包 `so101_gazebo`
- `XLeRobot` 双臂机器人仿真包 `xlerobot_gazebo`
- `ros2_control` + `gz_ros2_control`

## 目录

- `compose.yml`: GUI / headless 两个容器入口
- `Dockerfile`: 镜像构建
- `src/demo_bot_gazebo`: 示例机器人包
- `src/so101_gazebo`: 基于开源 `SO-ARM100/SO101` 资产的 Gazebo 包
- `src/xlerobot_gazebo`: 基于开源 `XLeRobot` 资产的 Gazebo 包
- `scripts/run_gui.sh`: 本地图形界面启动
- `scripts/run_headless.sh`: 无头模式启动
- `scripts/run_so101_headless.sh`: 无头启动 `SO101`
- `scripts/run_xlerobot_headless.sh`: 无头启动 `XLeRobot`

## 构建

```bash
cd /home/qifei/xlerobot_gazebo
docker compose build
```

## 无头模式

适合 SSH、终端验证和 CI 场景：

```bash
cd /home/qifei/xlerobot_gazebo
./scripts/run_headless.sh
```

另开一个终端检查 `demo_bot` 控制器：

```bash
docker exec -it demo-bot-gazebo-headless bash
ros2 control list_controllers
```

给 `demo_bot` 发一个前进速度：

```bash
docker exec -it demo-bot-gazebo-headless bash
ros2 topic pub --once /diff_drive_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

## GUI 模式

本机桌面环境下执行：

```bash
cd /home/qifei/xlerobot_gazebo
./scripts/run_gui.sh
```

如果 Gazebo GUI 无法连接 X11，先执行一次：

```bash
xhost +local:docker
```

## 停止

```bash
cd /home/qifei/xlerobot_gazebo
docker compose down
```

如果是通过单独脚本启动的 `SO101` 或 `XLeRobot`，直接停止对应容器：

```bash
docker rm -f so101-gazebo-headless
docker rm -f xlerobot-gazebo-headless
```

## SO101

无头启动：

```bash
cd /home/qifei/xlerobot_gazebo
./scripts/run_so101_headless.sh
```

检查控制器：

```bash
docker exec -it so101-gazebo-headless bash
source /opt/ros/jazzy/setup.bash
source /workspaces/ws/install/setup.bash
ros2 control list_controllers
```

当前已验证控制器：

- `joint_state_broadcaster`
- `arm_position_controller`

## XLeRobot

无头启动：

```bash
cd /home/qifei/xlerobot_gazebo
./scripts/run_xlerobot_headless.sh
```

切到理想平面后端：

```bash
cd /home/qifei/xlerobot_gazebo
./scripts/run_xlerobot_headless.sh planar
```

检查控制器：

```bash
docker exec -it xlerobot-gazebo-headless bash
source /opt/ros/jazzy/setup.bash
source /workspaces/ws/install/setup.bash
ros2 control list_controllers
```

当前已验证控制器：

- `joint_state_broadcaster`
- `omni_base_controller`
- `left_arm_position_controller`
- `right_arm_position_controller`
- `head_position_controller`

统一的上层底盘命令接口：

```bash
/base/cmd_vel
```

消息类型为：

```bash
geometry_msgs/msg/TwistStamped
```

统一的限幅后调试话题：

```bash
/base/cmd_vel_limited
```

其中：

- `backend:=omni` 时，`/base/cmd_vel -> /omni_base_controller/cmd_vel -> wheel joints`
- `backend:=planar` 时，`/base/cmd_vel -> /ideal_base_velocity_controller/commands -> root_x/root_y/yaw`

统一的里程计出口：

```bash
/odom
```

其中：

- `backend:=omni` 时由 `omni_base_controller` 的里程计转发到 `/odom`
- `backend:=planar` 时由根关节状态生成 `/odom`

当前底盘结构：

- `base_link`
- `wheel_0_joint`
- `wheel_1_joint`
- `wheel_2_joint`
- 控制器为官方 `omni_wheel_drive_controller`
- 三轮几何中心对齐官方 `MuJoCo` 资产

`omni` 主线的上层接口与下层 plant 分层：

- 上层始终只输出 body twist `u = [vx, vy, wz]`
- Gazebo 下层把 body twist 落到 3 个轮关节
- 轮半径、底盘半径、轮安装角在控制器和 plant 中共用一套参数
- body-level limiter 负责速度、加速度和轮速上限，再送给 `omni_wheel_drive_controller`

`planar` 后端保留为调试后门：

- 仍然吃同一个 `/base/cmd_vel`
- 不走轮式动力学，直接驱动 `root_x/root_y/yaw`
- 适合 planner 回归、SLAM / perception / pipeline 联调
- 使用简化版 envelope，不做轮速约束

当前已验证行为：

- `linear.x` 前进
- `linear.y` 横移
- `angular.z` 原地转动

### 底盘 envelope

`omni` 主线当前按保守实机 envelope 约束：

- 最大平面速度 `0.3 m/s`
- 最大角速度 `1.57 rad/s`
- 最大平面加速度 `0.6 m/s^2`
- 最大角加速度 `3.14 rad/s^2`
- 最大轮速 `9.0 rad/s`

这组值参考了官方 `xlerobot_2wheels` 软件里的快档速度与轮半径配置，并把三轮 omni 底盘做了保守收敛：

- [config_xlerobot_2wheels.py](/home/qifei/vendor/XLeRobot/software/src/robots/xlerobot_2wheels/config_xlerobot_2wheels.py)
- [xlerobot_2wheels.py](/home/qifei/vendor/XLeRobot/software/src/robots/xlerobot_2wheels/xlerobot_2wheels.py)

本地参数文件：

- [base_geometry.yaml](/home/qifei/xlerobot_gazebo/src/xlerobot_gazebo/config/base_geometry.yaml)
- [base_envelope_real.yaml](/home/qifei/xlerobot_gazebo/src/xlerobot_gazebo/config/base_envelope_real.yaml)
- [planner_test_envelope.yaml](/home/qifei/xlerobot_gazebo/src/xlerobot_gazebo/config/planner_test_envelope.yaml)
