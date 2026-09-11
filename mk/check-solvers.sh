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
    echo "*** fetch a release from https://github.com/YosysHQ/oss-cad-suite-build" >&2
    echo "*** and unpack it into \${XDG_CACHE_HOME:-\$HOME/.cache}/little-cpu/oss-cad-suite" >&2
    echo "*** (mk/toolchain.mk puts that bin/ first on PATH once it exists)." >&2
    missing=1
  fi
done
exit "$missing"
