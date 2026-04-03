#!/bin/bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash

if [ -f /workspaces/ws/install/setup.bash ]; then
  source /workspaces/ws/install/setup.bash
fi
set -u

exec "$@"
