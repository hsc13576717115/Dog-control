#!/usr/bin/env bash
set -euo pipefail

# shellcheck source=lib/workspace.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"
workspace="${1:-${DOG_DEPS_WS}}"
source_package_dir="${DOG_SCRIPTS_DIR}/.."
repos_file="${source_package_dir}/repos/ocs2_humble.repos"
patch_files=(
  "${source_package_dir}/patches/ocs2-humble-custom-dog.patch"
  "${source_package_dir}/patches/ocs2-coal-self-collision.patch"
)

if ! command -v vcs >/dev/null; then
  user_bin="$(python3 -m site --user-base 2>/dev/null || true)/bin"
  if [[ -x "${user_bin}/vcs" ]]; then
    export PATH="${user_bin}:${PATH}"
  fi
fi
command -v vcs >/dev/null || {
  echo "vcstool is required: sudo apt install python3-vcstool" >&2
  exit 1
}

mkdir -p "${workspace}/src"
vcs import "${workspace}/src" < "${repos_file}"

ocs2_dir="${workspace}/src/ocs2"
expected_revision=cc73189408d4931d4d9b467a5e280a56ad7ab492
actual_revision=$(git -C "${ocs2_dir}" rev-parse HEAD)
if [[ "${actual_revision}" != "${expected_revision}" ]]; then
  echo "Unexpected OCS2 revision: ${actual_revision}" >&2
  exit 1
fi

for patch_file in "${patch_files[@]}"; do
  if git -C "${ocs2_dir}" apply --check "${patch_file}"; then
    git -C "${ocs2_dir}" apply "${patch_file}"
  elif git -C "${ocs2_dir}" apply --reverse --check "${patch_file}"; then
    echo "OCS2 patch is already applied: $(basename "${patch_file}")"
  else
    echo "OCS2 patch does not apply cleanly: ${patch_file}" >&2
    exit 1
  fi
done

echo "Dependencies fetched into ${workspace}/src"
echo "Run rosdep, then build OCS2 and custom_dog_control with colcon."
