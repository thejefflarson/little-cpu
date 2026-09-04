#!/bin/bash
# Asserts that no workflow file enumerates more than one `rtl/*.v` path on a
# single line.
#
# Usage: workflow_rtl_list_test.sh [repo-root]     # defaults to this script's parent
#
# WHY THIS EXISTS. Both sim legs build from ONE list, the Makefile's
# SIM_RTL_SRCS, and CI reaches it only through `make elaborate-strict` --
# never by naming the RTL files itself. A second, hand-written copy of that
# list is exactly how this repo's CI job used to miss two files when they
# landed and spent a run elaborating a testbench whose memories were not
# there; nothing caught it because a comment describing the rule cannot go
# red. Two or more distinct `rtl/*.v` paths on one line is the shape that
# defect took and the shape a workflow step invoking a tool directly --
# `iverilog rtl/a.v rtl/b.v ...` -- would still take if it came back. A single
# `rtl/*.v` mention, prose pointing at one file, is not that shape and is left
# alone.
#
# Hermetic: grep only. No toolchain, no simulator, no yosys, so this runs
# inside `make test` anywhere.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

WORKFLOW_DIR="$REPO/.github/workflows"
if [ ! -d "$WORKFLOW_DIR" ]; then
  echo "error: '$WORKFLOW_DIR' does not exist, so there is nothing to scan." >&2
  exit 1
fi

shopt -s nullglob
files=("$WORKFLOW_DIR"/*.yml "$WORKFLOW_DIR"/*.yaml)
shopt -u nullglob
if [ "${#files[@]}" -eq 0 ]; then
  echo "error: no *.yml or *.yaml files under '$WORKFLOW_DIR' -- either CI moved" >&2
  echo "and this check needs to move with it, or the workflow directory is empty" >&2
  echo "and this check is asserting a property of nothing." >&2
  exit 1
fi

rc=0
checked=0

for f in "${files[@]}"; do
  checked=$((checked + 1))
  # One pass over the file, not one grep per matching line. -o prints each
  # match on its own line as `N:path`; sort -u collapses a path repeated on
  # the same line to one entry, and the entries that share a line number stay
  # adjacent under that sort regardless of overall line order, which is what
  # lets uniq -c count them correctly with no second sort.
  while IFS=: read -r lineno n; do
    [ -n "$lineno" ] || continue
    if [ "$n" -ge 2 ]; then
      rc=1
      echo "error: $f:$lineno names $n distinct rtl/*.v paths on one line -- a" >&2
      echo "second copy of SIM_RTL_SRCS, the shape that once made CI elaborate a" >&2
      echo "testbench two files short of its memories. Call 'make elaborate-strict'" >&2
      echo "(or another Makefile target) instead of naming RTL files here:" >&2
      sed -n "${lineno}p" "$f" | sed -e 's|^|    |' >&2
    fi
  done < <(grep -n -o -E 'rtl/[A-Za-z0-9_]+\.v' "$f" | sort -u \
           | cut -d: -f1 | uniq -c | awk '{print $2":"$1}')
done

if [ "$rc" -ne 0 ]; then
  exit 1
fi

echo "no workflow file enumerates more than one rtl/*.v path per line, across $checked file(s)"
