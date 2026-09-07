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

echo "$MK: $uses references to \$(VEXRISCV_V), and no other VexRiscv.v path"
