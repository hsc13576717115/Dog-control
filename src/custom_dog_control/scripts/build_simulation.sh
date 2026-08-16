#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace="$(cd "${package_dir}/../.." && pwd)"
stack_root="$(cd "${workspace}/../.." && pwd)"
model_workspace="${stack_root}/ros2"
deps_workspace="${CUSTOM_DOG_CONTROL_DEPS_WS:-${workspace}/../custom_dog_control_deps_ws}"

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
fi

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ERROR: ROS 2 Humble is not sourced (ROS_DISTRO=${ROS_DISTRO:-unset})."
  echo "Run: source /opt/ros/humble/setup.bash"
  exit 2
fi

if [[ ! -f "${model_workspace}/src/custom_dog_description/package.xml" ]]; then
  echo "ERROR: custom_dog_description source package was not found:"
  echo "  ${model_workspace}/src/custom_dog_description"
  exit 2
fi

echo "Updating custom_dog_description model underlay..."
colcon --log-base "${model_workspace}/log" build \
  --base-paths "${model_workspace}/src" \
  --build-base "${model_workspace}/build" \
  --install-base "${model_workspace}/install" \
  --packages-select custom_dog_description

set +u
# shellcheck disable=SC1090
source "${model_workspace}/install/setup.bash"
set -u
deps_setup=""
if [[ -f "${deps_workspace}/install/setup.bash" ]]; then
  deps_setup="${deps_workspace}/install/setup.bash"
elif [[ -f "${deps_workspace}/setup.bash" ]]; then
  deps_setup="${deps_workspace}/setup.bash"
fi
if [[ -n "${deps_setup}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${deps_setup}"
  set -u
fi
if [[ -d "${workspace}/src/ocs2" ]]; then
  export CUSTOM_DOG_CONTROL_ALLOW_OCS2_SOURCE_BUILD=1
fi
"${script_dir}/check_dependencies.sh"

multiarch="$(gcc -print-multiarch 2>/dev/null || true)"
ros_multiarch_lib="/opt/ros/humble/lib/${multiarch}"
if [[ -n "${multiarch}" && -d "${ros_multiarch_lib}" ]]; then
  export LIBRARY_PATH="${ros_multiarch_lib}:${LIBRARY_PATH:-}"
fi
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""

cd "${workspace}"
colcon build --symlink-install --packages-up-to custom_dog_control \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
               -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=OFF

# Only source a freshly successful build. set -e above prevents stale installs
# from being used after a compiler or linker failure.
set +u
# shellcheck disable=SC1091
source "${workspace}/install/setup.bash"
set -u
exec ros2 launch custom_dog_control gazebo.launch.py use_rviz:=false "$@"
