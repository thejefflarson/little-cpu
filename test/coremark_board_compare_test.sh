#!/bin/bash
# Green direction of coremark_board_compare.sh; the red ones are forced in test/probe_gates.sh.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
COMPARE="$HERE/bench/coremark_board_compare.sh"

tmp=$(mktemp -d "${TMPDIR:-/tmp}/coremark-compare.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

report() {  # $1 = cycles
  printf 'Cycles         : %s\nIterations     : 800\nSelf-check     : PASS\n' "$1"
}

{ echo "CoreMark 1.0 : 1.776 / GCC 15.2.0"; report 450000123; } > "$tmp/sim.log"
{ echo "elf: ignored"; report 450000123; } > "$tmp/exact.log"
{ printf 'ycles         : 1\n'; report 450000123; report 450000123; } > "$tmp/midblock.log"

for board in exact midblock; do
  if ! "$COMPARE" "$tmp/sim.log" "$tmp/$board.log" > "$tmp/out" 2>&1; then
    cat "$tmp/out" >&2
    echo "error: matching reports ($board capture) were refused." >&2
    exit 1
  fi
done
echo "matching reports accepted, including a mid-report capture"
