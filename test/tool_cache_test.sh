#!/bin/bash
# Asserts that the downloaded tools land somewhere every checkout can reach, and that the
# two files which independently compute that location still agree.
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: tool_cache_test.sh <sail-dir> <svlint-dir> <sail-download-dir>" >&2
  exit 1
fi

MAKE_SAIL_DIR=$1
MAKE_SVLINT_DIR=$2
MAKE_SAIL_DOWNLOAD_DIR=$3
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)

if ! command -v python3 >/dev/null 2>&1; then
  echo "error: python3 is not on PATH, so test/cosim.py's idea of where the" >&2
  echo "Sail install lives cannot be read. Without it there is no comparison." >&2
  exit 1
fi

# Imported rather than re-derived here.
py_sail_dir=$(cd "$HERE" && python3 -B -c 'import cosim; print(cosim.SAIL_DIR)')

if [ -z "$py_sail_dir" ]; then
  echo "error: test/cosim.py named no Sail install directory." >&2
  exit 1
fi

rc=0

if [ "$MAKE_SAIL_DIR" != "$py_sail_dir" ]; then
  echo "error: the Makefile and test/cosim.py do not agree on where the Sail" >&2
  echo "install lives:" >&2
  echo "  Makefile     : $MAKE_SAIL_DIR" >&2
  echo "  test/cosim.py: $py_sail_dir" >&2
  echo "\`make sail-setup\` writes the first and \`make cosim-suite\` reads the" >&2
  echo "second, so co-simulation would report a binary it just installed as" >&2
  echo "missing. Both derive from XDG_CACHE_HOME; change them together." >&2
  rc=1
fi

outside_checkout() {
  local who=$1 dir=$2
  case "$dir" in
    /*) ;;
    *)
      echo "error: $who names a relative tool install directory '$dir'." >&2
      echo "It has to be absolute; a relative one moves with the caller's" >&2
      echo "working directory and cannot be checked against the checkout." >&2
      rc=1
      return
      ;;
  esac
  case "$dir/" in
    "$REPO"/*)
      echo "error: $who installs tools inside the checkout:" >&2
      echo "  $dir" >&2
      echo "  checkout: $REPO" >&2
      echo "Downloaded tools are gitignored, and a git worktree is given" >&2
      echo "tracked files only, so an install here is invisible from every" >&2
      echo "worktree. Put it under the shared cache instead." >&2
      rc=1
      ;;
  esac
}

outside_checkout Makefile "$MAKE_SAIL_DIR"
outside_checkout test/cosim.py "$py_sail_dir"
outside_checkout Makefile "$MAKE_SVLINT_DIR"
outside_checkout Makefile "$MAKE_SAIL_DOWNLOAD_DIR"

if [ "$rc" -ne 0 ]; then
  exit 1
fi

echo "Tool installs are outside the checkout and agreed on: $py_sail_dir"
