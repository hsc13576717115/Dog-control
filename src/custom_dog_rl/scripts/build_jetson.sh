#!/usr/bin/env bash
# Build the same CPU inference path on x86_64 and Jetson ARM64, without OCS2.
# This script builds only; it never starts controller_manager or opens a port.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "$script_dir/../../.." && pwd)"
ros_setup="${CUSTOM_DOG_ROS_SETUP:-/opt/ros/humble/setup.bash}"
model_source="${CUSTOM_DOG_DESCRIPTION_DIR:-$workspace/../himloco_custom_dog/assets/custom_dog_description}"
sdk_source="${UNITREE_ACTUATOR_SDK_ROOT:-$workspace/src/unitree_guide/include/thirdParty/unitree_actuator_sdk-main}"
if [[ ! -f "$ros_setup" || ! -f "$model_source/package.xml" ]]; then
  echo 'Set CUSTOM_DOG_ROS_SETUP and CUSTOM_DOG_DESCRIPTION_DIR to existing ROS/model packages.' >&2
  exit 2
fi
if [[ -z "${ONNXRUNTIME_ROOT:-}" ]]; then
  echo 'Set ONNXRUNTIME_ROOT to the SDK printed by install_onnxruntime.sh.' >&2
  exit 2
fi
set +u
source "$ros_setup"
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cd "$workspace"
colcon --log-base log_rl build \
  --build-base build_rl --install-base install_rl --executor sequential \
  --base-paths "$model_source" src/custom_dog_control src/custom_dog_rl src/serial_ros2 src/fdilink_ahrs \
  --packages-select custom_dog_description serial fdilink_ahrs custom_dog_control custom_dog_rl \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCUSTOM_DOG_CONTROL_BUILD_NMPC=OFF \
    -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=ON \
    -DUNITREE_ACTUATOR_SDK_ROOT="$sdk_source" \
    -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT"
printf 'Build complete. In a fresh shell: source %q\n' "$workspace/install_rl/setup.bash"
