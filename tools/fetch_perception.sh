#!/usr/bin/env bash
# Reproduce only the two sensor packages selected from the user's repository.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source_url='https://github.com/hsc13576717115/ROBOCON_NBUT_R2.git'
revision='300d07241372180c5267a129e80ad9a3410e2997'
perception_ws="${QR_PERCEPTION_WS:-${repo_root}/external/perception_ws}"
if [[ -e "${perception_ws}" ]]; then
  echo "Refusing to overwrite existing workspace: ${perception_ws}" >&2
  exit 1
fi
scratch="$(mktemp -d)"
trap 'rm -rf -- "${scratch}"' EXIT
# QR_SOURCE_CACHE permits an offline export from an already fetched Git repository.
if [[ -n "${QR_SOURCE_CACHE:-}" ]]; then
  source_git="${QR_SOURCE_CACHE}"
else
  source_git="${scratch}/upstream"
  git clone --filter=blob:none --no-checkout --depth 1 "${source_url}" "${source_git}"
  git -C "${source_git}" fetch --filter=blob:none --depth 1 origin "${revision}"
fi
git -C "${source_git}" cat-file -e "${revision}^{commit}"
git -C "${source_git}" archive --format=tar --output="${scratch}/packages.tar" \
  "${revision}" src/livox_ros_driver2 src/FAST_LIO_ROS2
mkdir -p "${scratch}/workspace"
tar -xf "${scratch}/packages.tar" -C "${scratch}/workspace"
printf 'repository: %s\ncommit: %s\n' "${source_url}" "${revision}" > "${scratch}/workspace/SOURCE.yaml"
# Keep upstream LICENSE files and source unchanged. Do not run upstream build.sh,
# which deletes ../../build and ../../install in its current workspace.
mkdir -p "$(dirname -- "${perception_ws}")"
mv -T -- "${scratch}/workspace" "${perception_ws}"
echo "Sources ready (not built or launched): ${perception_ws}"
