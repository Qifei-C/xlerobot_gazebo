#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /workspaces/ws/install/setup.bash ]; then
  source /workspaces/ws/install/setup.bash
fi

exec "$@"
