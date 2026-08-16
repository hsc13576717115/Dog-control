#!/usr/bin/env bash
set -euo pipefail

failed=0
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../../.." && pwd)"
deps_workspace="${CUSTOM_DOG_CONTROL_DEPS_WS:-${workspace_dir}/../custom_dog_control_deps_ws}"

check_command() {
  if ! command -v "$1" >/dev/null; then
    echo "MISSING command: $1"
    failed=1
  fi
}

check_ros_package() {
  if ! ros2 pkg prefix "$1" >/dev/null 2>&1; then
    echo "MISSING ROS package: $1"
    failed=1
  fi
}

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ROS_DISTRO must be humble (current: ${ROS_DISTRO:-unset})"
  failed=1
fi

check_command colcon
check_command ros2
check_ros_package controller_interface
check_ros_package hardware_interface
check_ros_package controller_manager
check_ros_package joint_state_broadcaster
check_ros_package custom_dog_description
check_ros_package urdf
check_ros_package pinocchio
check_ros_package coal
if [[ "${CUSTOM_DOG_CONTROL_ALLOW_OCS2_SOURCE_BUILD:-0}" == "1" &&
      -d "${workspace_dir}/src/ocs2" ]]; then
  echo "INFO: OCS2 source checkout will be built in this workspace"
else
  check_ros_package ocs2_legged_robot
  check_ros_package ocs2_self_collision
  check_ros_package ocs2_sqp
fi

if [[ ! -d "$(dirname "${BASH_SOURCE[0]}")/../third_party/qpOASES" ]]; then
  echo "MISSING qpOASES submodule: git submodule update --init --recursive"
  failed=1
fi

if [[ "${CUSTOM_DOG_CONTROL_REAL_HARDWARE:-0}" == "1" ]]; then
  if [[ -z "${UNITREE_ACTUATOR_SDK_ROOT:-}" ]]; then
    echo "UNITREE_ACTUATOR_SDK_ROOT is required for real hardware"
    failed=1
  fi
else
  check_ros_package gazebo_ros2_control
fi

if (( failed )); then
  echo
  echo "Dependency preflight failed. For a simulation build, install the ROS packages with:"
  echo "  sudo apt update"
  echo "  sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers"
  echo "  sudo apt install ros-humble-gazebo-ros2-control ros-humble-urdf ros-humble-pinocchio ros-humble-coal"
  echo "If only OCS2 packages are missing, fetch and build them first:"
  echo "  ${workspace_dir}/src/custom_dog_control/scripts/fetch_ocs2.sh \"${deps_workspace}\""
  echo "  cd \"${deps_workspace}\""
  echo "  colcon build --symlink-install --packages-up-to ocs2_legged_robot ocs2_self_collision ocs2_sqp"
fi

exit "${failed}"
