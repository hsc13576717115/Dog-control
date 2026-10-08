#!/usr/bin/env bash
# Usage: source src/custom_dog_control/scripts/env.sh
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi
# shellcheck source=lib/workspace.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"
dog_source_runtime
