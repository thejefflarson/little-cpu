#!/bin/bash
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
MK=${1:-"$HERE/../../Makefile"}

if [ ! -f "$MK" ]; then
  echo "error: '$MK' does not exist, so there is no Makefile to check." >&2
  exit 1
fi

uses=$(grep -c '\$(VEXRISCV_V)' "$MK" || true)
if [ "$uses" -eq 0 ]; then
  echo "error: $MK names \$(VEXRISCV_V) nowhere. Either this harness reads no" >&2
  echo "VexRiscv at all, or it reaches one by some spelling this scan cannot" >&2
  echo "see -- and in both cases the absence test below proves nothing." >&2
  exit 1
fi

bad=$(grep -n 'VexRiscv\.v' "$MK" | grep -vE '^[0-9]+:[[:space:]]*#' \
  | grep -v '\$(VEXRISCV_V)' || true)
if [ -n "$bad" ]; then
  echo "error: a soc/compare/ recipe in $MK names a VexRiscv.v path other" >&2
  echo "than \$(VEXRISCV_V):" >&2
  echo "$bad" >&2
  exit 1
fi

# bench_vexriscv_lrsc.v is bench_vexriscv.v with the core's module name changed, and nothing
# else: a second adapter that quietly differed would make the LR/SC build's cycles a
# measurement of a different machine.
BENCH_DIR=$(dirname "$MK")/soc/compare
if ! diff <(sed -e 's/bench_vexriscv_lrsc/bench_vexriscv/' -e 's/VexRiscvLrsc/VexRiscv/' \
              "$BENCH_DIR/bench_vexriscv_lrsc.v") \
          "$BENCH_DIR/bench_vexriscv.v" > /dev/null; then
  echo "error: $BENCH_DIR/bench_vexriscv_lrsc.v differs from bench_vexriscv.v in more" >&2
  echo "than the core's module name, so the two VexRiscv builds are not in one harness." >&2
  exit 1
fi

echo "$MK: $uses references to \$(VEXRISCV_V), and no other VexRiscv.v path"
