#!/bin/bash
# Asserts that the downloaded tools land somewhere every checkout can reach,
# and that the two files which independently compute that location still agree.
#
# Usage: tool_cache_test.sh <sail-dir> <svlint-dir> <sail-download-dir>
#        # the Makefile's values
#
# WHY THIS EXISTS. `make sail-setup` used to unpack into the gitignored
# `tools/sail`. A git worktree is given tracked files only, so the install was
# present in the main checkout and in no worktree -- and the message a worktree
# got, "run 'make sail-setup'", never said the binary already existed one
# directory up. Six of six live worktrees lacked it, and a recorded measurement
# shipped with co-simulation unrun for that reason alone. Co-simulation
# is the only oracle here that reads the core's real register array, so that is
# the instrument every engineer working in a worktree could not run.
#
# The fix moves the installs out of the checkout, which creates the one thing
# that can silently regress: the Makefile computes the path for `sail-setup` to
# write, test/cosim.py computes it again for the runner to read, and nothing
# makes them the same string. If they drift, `make cosim-suite` looks for a
# binary `make sail-setup` did not write and reports it as missing -- the exact
# failure this replaced. The comparison below is that check.
#
# Hermetic: python3 and a shell, no toolchain, no Sail, no yosys, so it runs
# inside `make test` anywhere that already ran.
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

# Imported rather than re-derived here. A second copy of the rule would agree
# with the Makefile while cosim.py disagreed with both, which is the failure
# this is written to catch. `-B` keeps the import from leaving a __pycache__
# behind in a tracked directory.
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

# outside_checkout <who> <dir>
outside_checkout() {
  local who=$1 dir=$2
  # An absolute path is what makes the prefix test mean anything: a relative
  # one is read against whatever directory make happened to be run from.
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
# The kept release tarball is a downloaded tool too, and CI caches that
# directory by name. Inside the checkout it would be gitignored, invisible from
# every worktree, and swept up by whatever cleans the tree between jobs.
outside_checkout Makefile "$MAKE_SAIL_DOWNLOAD_DIR"

if [ "$rc" -ne 0 ]; then
  exit 1
fi

echo "Tool installs are outside the checkout and agreed on: $py_sail_dir"
