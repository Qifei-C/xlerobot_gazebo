#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
backend="${1:-omni}"

docker compose build
docker rm -f xlerobot-gazebo-headless >/dev/null 2>&1 || true

docker run --rm \
  --name xlerobot-gazebo-headless \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}" \
  demo-bot-gazebo:jazzy \
  ros2 launch xlerobot_gazebo sim.launch.py headless:=true backend:="${backend}"
