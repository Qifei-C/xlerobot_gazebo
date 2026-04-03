# XLeRobot Gazebo

这个仓库提供一套独立的容器化 Gazebo 环境，内容包括：

- `Gazebo Harmonic` + `ROS 2 Jazzy`
- 一个最小差速机器人 `demo_bot_gazebo`
- `SO101` 单臂机械臂仿真包 `so101_gazebo`
- `XLeRobot` 双臂机器人仿真包 `xlerobot_gazebo`
- `ros2_control` + `gz_ros2_control`

## Repo 结构

- `Dockerfile` / `compose.yml`: 容器镜像与运行入口
- `ros_entrypoint_overlay.sh`: 容器内 ROS 2 工作空间入口
- `scripts/`: 常用启动脚本
- `src/demo_bot_gazebo`: 最小示例机器人
- `src/so101_gazebo`: `SO101` Gazebo 集成包
- `src/xlerobot_gazebo`: `XLeRobot` Gazebo 主包
- `THIRD_PARTY.md`: 第三方资产来源说明

精简视图：

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

## 前置条件

- Docker（含 Docker Compose）
- GUI 模式需要 X11 桌面环境和 GPU（`/dev/dri`）

## 构建

```bash
git clone <repo-url> && cd xlerobot_gazebo
docker compose build
```

## 无头模式

适合 SSH、终端验证和 CI 场景：

```bash
# demo_bot
./scripts/run_headless.sh

# SO101
./scripts/run_so101_headless.sh

# XLeRobot (omni 后端，默认)
./scripts/run_xlerobot_headless.sh

# XLeRobot (planar 后端)
./scripts/run_xlerobot_headless.sh planar
```

## GUI 模式

本机桌面环境下执行（需要 `$DISPLAY` 已设置）：

```bash
# demo_bot
./scripts/run_gui.sh

# SO101
./scripts/run_so101_gui.sh

# XLeRobot (omni 后端，默认)
./scripts/run_xlerobot_gui.sh

# XLeRobot (planar 后端)
./scripts/run_xlerobot_gui.sh planar
```

如果 Gazebo GUI 无法连接 X11，先执行一次：

```bash
xhost +SI:localuser:$(id -un)
```

## 进入容器

ROS 环境在 `docker exec` 时会自动加载，无需手动 `source`：

```bash
# GUI 容器
docker exec -it xlerobot-gazebo-gui bash

# 无头容器
docker exec -it xlerobot-gazebo-headless bash
```

## 键盘遥控 (WASD)

在容器内运行：

```bash
ros2 run xlerobot_gazebo keyboard_teleop.py
```

| 按键 | 动作 |
|------|------|
| W / S | 前进 / 后退 |
| A / D | 左移 / 右移 |
| Q / E | 左转 / 右转 |
| 空格 | 停止 |
| Esc | 退出 |

可调参数：

```bash
ros2 run xlerobot_gazebo keyboard_teleop.py --ros-args \
  -p linear_speed:=0.3 \
  -p angular_speed:=1.5
```

## Demo Bot

检查控制器：

```bash
docker exec -it demo-bot-gazebo-headless bash
ros2 control list_controllers
```

给 `demo_bot` 发一个前进速度：

```bash
ros2 topic pub --once /diff_drive_base_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

## SO101

检查控制器：

```bash
docker exec -it so101-gazebo-headless bash
ros2 control list_controllers
```

当前已验证控制器：

- `joint_state_broadcaster`
- `arm_position_controller`

## XLeRobot

检查控制器：

```bash
docker exec -it xlerobot-gazebo-headless bash
ros2 control list_controllers
```

当前已验证控制器：

- `joint_state_broadcaster`
- `omni_base_controller`
- `left_arm_position_controller`
- `left_gripper_position_controller`
- `right_arm_position_controller`
- `right_gripper_position_controller`
- `head_position_controller`

### 话题接口

统一的上层底盘命令接口：

```
/base/cmd_vel    (geometry_msgs/msg/TwistStamped)
```

统一的限幅后调试话题：

```
/base/cmd_vel_limited
```

统一的里程计出口：

```
/odom
```

其中：

- `backend:=omni` 时，`/base/cmd_vel -> /omni_base_controller/cmd_vel -> wheel joints`
- `backend:=planar` 时，`/base/cmd_vel -> /ideal_base_velocity_controller/commands -> root_x/root_y/yaw`
- `backend:=omni` 时由 `omni_base_controller` 的里程计转发到 `/odom`
- `backend:=planar` 时由根关节状态生成 `/odom`

### 手动控制示例

底盘：

```bash
# 前进
ros2 topic pub --once /base/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1}}}"

# 横移
ros2 topic pub --once /base/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {y: 0.1}}}"

# 旋转
ros2 topic pub --once /base/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {angular: {z: 0.5}}}"
```

手臂（5 关节：Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll）：

```bash
# 左臂
ros2 topic pub --once /left_arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 0.8, 0.0, 0.0]}"

# 右臂
ros2 topic pub --once /right_arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 0.8, 0.0, 0.0]}"
```

夹爪（1 关节）：

```bash
# 开
ros2 topic pub --once /left_gripper_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5]}"

# 关
ros2 topic pub --once /left_gripper_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

头部（pan + tilt）：

```bash
ros2 topic pub --once /head_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.3, -0.2]}"
```

### 底盘结构

- `base_link`
- `wheel_0_joint` / `wheel_1_joint` / `wheel_2_joint`
- 三轮 120° 等分排列，轮轴指向底盘中心（径向），轮子沿切向驱动
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

### 官方 controller 适配

仓库现在额外提供一层"官方 XLeRobot action/observation dict -> ROS"薄适配：

- 输入话题：`/xlerobot/action_dict`
- 输出话题：`/xlerobot/observation_dict`
- 消息类型：`std_msgs/msg/String`
- 载荷格式：JSON object

这个适配层保持官方 `XLeRobot` 的整机键名：

- 左臂：`left_arm_*.pos`
- 右臂：`right_arm_*.pos`
- 头部：`head_motor_1.pos`、`head_motor_2.pos`
- 底盘：`x.vel`、`y.vel`、`theta.vel`

底盘单位保持官方 3 轮底盘约定：

- `x.vel`: `m/s`
- `y.vel`: `m/s`
- `theta.vel`: `deg/s`

位置量支持两种表示：

- 默认 `official_use_degrees:=false`，对齐官方 `use_degrees=False` 配置，非夹爪关节按各自限位映射到 `[-100, 100]`
- 启动时传 `official_use_degrees:=true`，则非夹爪关节按角度 `deg` 收发，更适合人工调试
- 夹爪始终按官方 `0..100` 语义收发

示例：

```bash
ros2 topic pub --once /xlerobot/action_dict std_msgs/msg/String \
  '{"data":"{\"x.vel\": 0.15, \"y.vel\": 0.0, \"theta.vel\": 0.0}"}'
```

```bash
ros2 topic echo /xlerobot/observation_dict --once
```

用角度模式启动：

```bash
ros2 launch xlerobot_gazebo sim.launch.py headless:=true backend:=omni official_use_degrees:=true
```

### 底盘 envelope

`omni` 主线当前按保守实机 envelope 约束：

- 最大平面速度 `0.3 m/s`
- 最大角速度 `1.57 rad/s`
- 最大平面加速度 `0.6 m/s^2`
- 最大角加速度 `3.14 rad/s^2`
- 最大轮速 `9.0 rad/s`

本地参数文件：

- `src/xlerobot_gazebo/config/base_geometry.yaml`
- `src/xlerobot_gazebo/config/base_envelope_real.yaml`
- `src/xlerobot_gazebo/config/planner_test_envelope.yaml`

## 停止

```bash
docker compose down
```

如果是通过单独脚本启动的容器，直接停止对应容器：

```bash
docker rm -f so101-gazebo-headless
docker rm -f so101-gazebo-gui
docker rm -f xlerobot-gazebo-headless
docker rm -f xlerobot-gazebo-gui
```

## 本地运行说明

`compose.yml` 里保留了 `network_mode: host` 和 `ipc: host`，这是为了本机 Gazebo / ROS 2 联调时减少网络和共享内存层面的额外配置。

这套 compose 配置是面向本地开发机的便利配置，不适合作为多用户或强隔离环境的通用容器模板。
