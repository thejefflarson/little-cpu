#!/bin/bash
# Builds test/sail/reservation_probe.S and runs it under the Sail model ALONE, then
# decodes the one number the run prints into a labelled table.
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

"$CC" -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -nostdlib \
  -I "$REPO/test/asm" -T "$REPO/test/asm/sections.lds" \
  "$PROBE" -o "$tmp/probe.elf"

run=("$SAIL_BIN" --config "$CONFIG" --inst-limit 5000 "$tmp/probe.elf")

isa=$("$SAIL_BIN" --config "$CONFIG" --print-isa-string)

echo "sail:   $SAIL_BIN"
echo "config: $CONFIG"
echo "isa:    $isa"
echo "command: ${run[*]}"
echo

set +e
out=$("${run[@]}" 2>&1)
set -e
printf '%s\n' "$out"
echo

mask=$(printf '%s\n' "$out" | sed -n 's/^FAILURE:[[:space:]]*\([0-9][0-9]*\)\([[:space:]].*\)\{0,1\}$/\1/p' | head -1)
if [ -z "$mask" ]; then
  echo "error: the model printed no HTIF verdict; nothing was observed." >&2
  echo "A run that never reaches the probe's tohost write has measured nothing." >&2
  exit 1
fi

# Bit 6 is the probe's unconditional marker.
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
