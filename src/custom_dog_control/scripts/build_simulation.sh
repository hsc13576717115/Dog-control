#!/usr/bin/env bash
set -euo pipefail

# shellcheck source=lib/workspace.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"
script_dir="$DOG_SCRIPTS_DIR"
workspace="$DOG_WORKSPACE"
model_workspace="$DOG_MODEL_WS"
model_source="$DOG_MODEL_SOURCE"

dog_source_ros

model_package="${model_workspace}/src/custom_dog_description"
if [[ -f "${model_source}/package.xml" ]]; then
  python3 "${script_dir}/prepare_simulation_model.py" "${model_source}" "${model_package}"
elif [[ -f "${model_package}/package.xml" ]]; then
  python3 "${script_dir}/prepare_simulation_model.py" "${model_package}" "${model_package}"
else
  echo "ERROR: Set CUSTOM_DOG_DESCRIPTION_DIR to the custom_dog_description source package."
  echo "Searched: ${model_source} and ${model_package}"
  exit 2
fi

echo "Updating custom_dog_description model underlay..."
colcon --log-base "${model_workspace}/log" build \
  --base-paths "${model_workspace}/src" \
  --build-base "${model_workspace}/build" \
  --install-base "${model_workspace}/install" \
  --packages-select custom_dog_description

dog_source "${model_workspace}/install/local_setup.bash"
if [[ -d "${DOG_DEPS_WS}" ]]; then
  dog_source_dependencies
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
# Pinocchio/CppAD translation units are memory intensive on desktop systems.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL}}"

cd "${workspace}"
colcon build --symlink-install --packages-up-to custom_dog_control \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
               -DCUSTOM_DOG_CONTROL_CANONICAL_URDF="${model_package}/urdf/custom_dog.urdf" \
               -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=OFF

# Only source a freshly successful build. set -e above prevents stale installs
# from being used after a compiler or linker failure.
dog_source "${workspace}/install/local_setup.bash"
if [[ "${CUSTOM_DOG_BUILD_ONLY:-0}" == "1" ]]; then
  exit 0
fi
exec ros2 launch custom_dog_control gazebo.launch.py use_rviz:=false "$@"
