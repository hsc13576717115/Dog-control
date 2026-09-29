# Shared by source-workspace entry points; does not change caller shell options.
# All paths are derived from this file, so commands also work outside the repo.
DOG_SCRIPTS_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOG_WORKSPACE="$(cd -P "${DOG_SCRIPTS_DIR}/../../.." && pwd)"
DOG_MODEL_WS="${CUSTOM_DOG_MODEL_WS:-${DOG_WORKSPACE}/external/model_ws}"
DOG_MODEL_SOURCE="${CUSTOM_DOG_DESCRIPTION_DIR:-${DOG_WORKSPACE}/../himloco_custom_dog/assets/custom_dog_description}"
DOG_DEPS_WS="${CUSTOM_DOG_CONTROL_DEPS_WS:-${DOG_WORKSPACE}/external/ocs2_ws}"

# ROS/colcon setup scripts may reference unset variables. Restore nounset even
# on failure, and report missing underlays instead of loading a partial stack.
dog_source() {
  local setup_file="$1" had_nounset=0 status=0
  if [[ ! -f "$setup_file" ]]; then
    echo "ERROR: Missing environment: $setup_file" >&2
    return 2
  fi
  [[ $- == *u* ]] && had_nounset=1
  set +u
  # shellcheck disable=SC1090
  source "$setup_file" || status=$?
  if (( had_nounset )); then set -u; fi
  return "$status"
}

dog_source_ros() {
  dog_source /opt/ros/humble/setup.bash
}

dog_source_dependencies() {
  local prefix="$DOG_DEPS_WS"
  [[ -f "$prefix/install/local_setup.bash" ]] && prefix="$prefix/install"
  dog_source "$prefix/local_setup.bash"
}

dog_source_runtime() {
  dog_source_ros &&
    dog_source "$DOG_MODEL_WS/install/local_setup.bash" &&
    dog_source_dependencies &&
    dog_source "$DOG_WORKSPACE/install/local_setup.bash"
}
