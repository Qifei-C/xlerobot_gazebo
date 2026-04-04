#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
backend="${1:-omni}"

if pgrep -af "ros2 launch xlerobot_gazebo sim.launch.py" >/dev/null \
  || pgrep -af "gz sim .*demo_world.sdf" >/dev/null; then
  echo "Detected an existing XLeRobot/Gazebo simulation process."
  echo "Stop the old sim before starting a new headless run, otherwise Gazebo will keep old entities alive."
  echo "Matching processes:"
  pgrep -af "ros2 launch xlerobot_gazebo sim.launch.py|gz sim .*demo_world.sdf" || true
  exit 1
fi

docker compose build
docker rm -f xlerobot-gazebo-headless >/dev/null 2>&1 || true

docker run --rm \
  --name xlerobot-gazebo-headless \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}" \
  demo-bot-gazebo:jazzy \
  ros2 launch xlerobot_gazebo sim.launch.py headless:=true backend:="${backend}"
