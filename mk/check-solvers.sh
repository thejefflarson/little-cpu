#!/bin/sh
# Refuses by name rather than letting sby fail on a missing one opaquely.
set -eu

if [ "$#" -eq 0 ]; then
  echo "usage: mk/check-solvers.sh <tool>..." >&2
  exit 2
fi

missing=0
for tool in "$@"; do
  if ! command -v "$tool" > /dev/null 2>&1; then
    echo "*** $tool is not on PATH. It ships in the OSS CAD Suite, not Homebrew:" >&2
    echo "*** \`make cad-setup\` fetches the pinned release (mk/oss-cad-pin.mk) and" >&2
    echo "*** mk/toolchain.mk puts its bin/ first on PATH once it exists." >&2
    missing=1
  fi
done
exit "$missing"
