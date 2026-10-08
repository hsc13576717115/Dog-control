#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=lib/workspace.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"
dog_source_runtime
exec ros2 launch custom_dog_control gazebo.launch.py use_rviz:=false "$@"
