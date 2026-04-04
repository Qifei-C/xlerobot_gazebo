#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set — cannot start Gazebo GUI."
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || true
fi

docker compose up --build gazebo
