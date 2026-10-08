#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=lib/workspace.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"
dog_source_runtime
cd "$DOG_WORKSPACE"
exec python3 "$DOG_SCRIPTS_DIR/run_simulation_tests.py" "$@"
