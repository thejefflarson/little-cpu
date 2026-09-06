#!/bin/bash
# Every soc/compare/ recipe reads VexRiscv through $(VEXRISCV_V), the vendored
# core soc/compare/vexriscv_pin.mk digest-pins. riscv-formal's own clone under
# $(RISCV_FORMAL_DIR) carries a DIFFERENT VexRiscv build -- FormalSimple, with
# no CsrPlugin -- so a recipe naming that path instead still elaborates, still
# simulates, and silently measures the wrong core: `iverilog` does not care
# which VexRiscv.v it is handed, so nothing else here would notice.
#
# Usage: vexriscv_path_test.sh [Makefile path]     # defaults to the repo's own
#
# Hermetic: grep only. No toolchain, no simulator, no yosys.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
MK=${1:-"$HERE/../../Makefile"}

if [ ! -f "$MK" ]; then
  echo "error: '$MK' does not exist, so there is no Makefile to check." >&2
  exit 1
fi

# THE POSITIVE ASSERTION FIRST. The scan below is an absence test, and an
# absence test alone passes a Makefile with no VexRiscv in it at all -- zero
# matches for anything reads exactly like zero matches for the forbidden path.
# So require the vendored variable to be named at least once before believing
# anything the absence test says.
uses=$(grep -c '\$(VEXRISCV_V)' "$MK" || true)
if [ "$uses" -eq 0 ]; then
  echo "error: $MK names \$(VEXRISCV_V) nowhere. Either this harness reads no" >&2
  echo "VexRiscv at all, or it reaches one by some spelling this scan cannot" >&2
  echo "see -- and in both cases the absence test below proves nothing." >&2
  exit 1
fi

# Any non-comment line naming a literal VexRiscv.v path is suspect; comment
# lines are prose, not a make recipe, and $(VEXRISCV_V) is a make variable
# reference that never matches the literal filename, so a line using it
# correctly never appears in this list.
bad=$(grep -n 'VexRiscv\.v' "$MK" | grep -vE '^[0-9]+:[[:space:]]*#' \
  | grep -v '\$(VEXRISCV_V)' || true)
if [ -n "$bad" ]; then
  echo "error: a soc/compare/ recipe in $MK names a VexRiscv.v path other" >&2
  echo "than \$(VEXRISCV_V):" >&2
  echo "$bad" >&2
  exit 1
fi

echo "$MK: $uses references to \$(VEXRISCV_V), and no other VexRiscv.v path"
