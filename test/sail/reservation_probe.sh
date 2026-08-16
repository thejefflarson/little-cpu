#!/bin/bash
# Builds test/sail/reservation_probe.S and runs it under the Sail model ALONE,
# then decodes the one number the run prints into a labelled table. Invoked by
# `make sail-reservation-probe`. The finding it produced is docs/adr/0102.
#
# NO CORE IS INVOLVED, on purpose. The question is what the REFERENCE MODEL does
# to an LR reservation at a trap and at an mret, and a comparison against a core
# that does not decode lr.w/sc.w yet could not answer it. This is the only
# executable in this repo that asks the model a question instead of grading it
# against something.
#
# IT BUILDS AT THE SUITE'S ISA STRING, which it did not always: this file
# carried the only `a` in the tree while the core decoded no atomic. The whole
# set is graded by test/march_test.sh now, this is one of the sites it grades,
# and the two spellings that must NOT move with it are named there.
#
# The probe is graded, not merely printed: bit 0 is a control case whose SC
# nothing could have invalidated, and a run where it failed exits nonzero
# instead of reporting five meaningless answers.
#
# Usage: reservation_probe.sh <sail-binary>   # the Makefile's SAIL_SIM_BIN
#
# The binary is an ARGUMENT and is deliberately not computed here. The Makefile
# and test/cosim.py already derive that cache path independently, and
# test/tool_cache_test.sh exists because those two can drift; a third copy would
# be one this repo has no check for.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: reservation_probe.sh <sail-binary>" >&2
  exit 1
fi

SAIL_BIN=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
CONFIG="$HERE/rv32imac_zicsr.json"
PROBE="$HERE/reservation_probe.S"

if [ ! -x "$SAIL_BIN" ]; then
  echo "error: no sail_riscv_sim at $SAIL_BIN. Run 'make sail-setup'." >&2
  exit 1
fi

CC=""
for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do
  if command -v "$candidate" > /dev/null 2>&1; then CC=$candidate; break; fi
done
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found. Run 'make setup'." >&2
  exit 1
fi

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-rsrv.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

"$CC" -march=rv32imac_zicsr_zifencei -mabi=ilp32 -nostdlib \
  -I "$REPO/test/asm" -T "$REPO/test/asm/sections.lds" \
  "$PROBE" -o "$tmp/probe.elf"

# Built once and both printed and run, so the command an ADR quotes out of this
# output is the command that produced the number beneath it.
run=("$SAIL_BIN" --config "$CONFIG" --inst-limit 5000 "$tmp/probe.elf")

echo "sail:   $SAIL_BIN"
echo "config: $CONFIG"
echo "isa:    $("$SAIL_BIN" --config "$CONFIG" --print-isa-string)"
echo "command: ${run[*]}"
echo

# Lifted for this call alone: the model exits nonzero on an HTIF FAILURE, and
# the probe reports its answer AS one.
set +e
out=$("${run[@]}" 2>&1)
set -e
printf '%s\n' "$out"
echo

# At least one digit, deliberately: the model also prints prose under FAILURE:
# ("possible trap loop detected"), and a pattern that admitted the empty match
# would turn a run that never executed an lr.w into a mask of zero.
mask=$(printf '%s\n' "$out" | sed -n 's/^FAILURE:[[:space:]]*\([0-9][0-9]*\)\([[:space:]].*\)\{0,1\}$/\1/p' | head -1)
if [ -z "$mask" ]; then
  echo "error: the model printed no HTIF verdict; nothing was observed." >&2
  echo "A run that never reaches the probe's tohost write has measured nothing." >&2
  exit 1
fi

# Bit 6 is the probe's unconditional marker. Without it the model would have
# read an all-cases-failed word as HTIF's pass encoding and printed SUCCESS.
if [ $(( (mask >> 6) & 1 )) -ne 1 ]; then
  echo "error: verdict $mask carries no marker bit; this is not the probe's word." >&2
  exit 1
fi

bit() { echo $(( (mask >> $1) & 1 )); }
say() { if [ "$(bit "$1")" -eq 1 ]; then echo "SUCCEEDED"; else echo "failed"; fi; }

printf '%-46s %s\n' "sc.w back to back with the lr.w (control)"      "$(say 0)"
printf '%-46s %s\n' "sc.w inside the trap handler, no mret"          "$(say 1)"
printf '%-46s %s\n' "sc.w after trap entry and mret"                 "$(say 2)"
printf '%-46s %s\n' "sc.w after an mret with no trap behind it"      "$(say 3)"
printf '%-46s %s\n' "sc.w after a same-hart sw to the reserved word" "$(say 4)"
printf '%-46s %s\n' "sc.w one word up from the reservation"          "$(say 5)"

if [ "$(bit 0)" -ne 1 ]; then
  echo >&2
  echo "error: the control case failed. An SC that nothing invalidated did not" >&2
  echo "succeed, so no other line above is evidence of anything." >&2
  exit 1
fi
