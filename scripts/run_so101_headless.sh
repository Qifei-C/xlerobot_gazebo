#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

docker compose build
docker rm -f so101-gazebo-headless >/dev/null 2>&1 || true

docker run --rm \
  --name so101-gazebo-headless \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-22}" \
  demo-bot-gazebo:jazzy \
  ros2 launch so101_gazebo sim.launch.py headless:=true
