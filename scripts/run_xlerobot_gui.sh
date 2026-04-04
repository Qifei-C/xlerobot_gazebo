#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
backend="${1:-omni}"

if pgrep -af "ros2 launch xlerobot_gazebo sim.launch.py" >/dev/null \
  || pgrep -af "gz sim .*demo_world.sdf" >/dev/null; then
  echo "Detected an existing XLeRobot/Gazebo simulation process."
  echo "Stop the old sim before starting a new GUI run, otherwise Gazebo will keep old entities alive."
  echo "Matching processes:"
  pgrep -af "ros2 launch xlerobot_gazebo sim.launch.py|gz sim .*demo_world.sdf" || true
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set — cannot start Gazebo GUI."
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || true
fi

docker compose build
docker rm -f xlerobot-gazebo-gui >/dev/null 2>&1 || true

docker run --rm \
  --name xlerobot-gazebo-gui \
  --network host \
  --ipc host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri \
  demo-bot-gazebo:jazzy \
  ros2 launch xlerobot_gazebo sim.launch.py headless:=false backend:="${backend}"
