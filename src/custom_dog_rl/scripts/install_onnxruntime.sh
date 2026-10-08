#!/usr/bin/env bash
# Install a pinned CPU C++ SDK into a user-owned directory; never changes CUDA.
set -euo pipefail
version=1.23.2
case "$(uname -m)" in
  x86_64) arch=x64; digest=1fa4dcaef22f6f7d5cd81b28c2800414350c10116f5fdd46a2160082551c5f9b ;;
  aarch64|arm64) arch=aarch64; digest=7c63c73560ed76b1fac6cff8204ffe34fe180e70d6582b5332ec094810241e5c ;;
  *) echo 'Supported architectures: x86_64 and aarch64' >&2; exit 2 ;;
esac
destination="${1:-${XDG_CACHE_HOME:-$HOME/.cache}/customdog-onnxruntime}"
mkdir -p "$destination"
destination="$(cd "$destination" && pwd)"
archive="onnxruntime-linux-${arch}-${version}.tgz"
temporary="$(mktemp -d)"
trap 'rm -rf "$temporary"' EXIT
curl --fail --location --retry 3 \
  "https://github.com/microsoft/onnxruntime/releases/download/v${version}/${archive}" \
  --output "$temporary/$archive"
printf '%s  %s\n' "$digest" "$temporary/$archive" | sha256sum --check - >&2
tar -xzf "$temporary/$archive" -C "$destination"
printf '%s\n' "$destination/onnxruntime-linux-${arch}-${version}"
