#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY 未设置，无法启动 Gazebo GUI。"
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || true
fi

docker compose build
docker rm -f so101-gazebo-gui >/dev/null 2>&1 || true

docker run --rm \
  --name so101-gazebo-gui \
  --network host \
  --ipc host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-22}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri \
  demo-bot-gazebo:jazzy \
  ros2 launch so101_gazebo sim.launch.py headless:=false
