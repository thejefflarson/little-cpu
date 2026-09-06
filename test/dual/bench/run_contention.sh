#!/bin/bash
# Measures what the bus arbiter costs hart 0's Dhrystone: the same image, run
# twice on the DUAL harness -- once with hart 1 streaming real loads and
# stores, once with hart 1 held in reset -- and the delta between hart 0's own
# two `Cycles` figures.
#
# test/dual/bench/dhry_contend.S is the program: hart 0 is the unmodified
# Dhrystone port (test/bench/dhry_1.c, dhry_2.c, dhry_port.c), linked with
# test/bench/bench.lds exactly as `make dhrystone` links it, and hart 1 is a
# tight lw/sw stream over a buffer instead of test/crt0.S's read-only park
# loop. THE ISOLATED RUN IS NOT `make dhrystone`'s NUMBER: both runs here go
# through rtl/littledual.v's bus arbiter, which registers its grant one cycle
# before decode's transaction proceeds -- an overhead the single-hart
# `littlesoc` never pays, present or not present a second hart. Isolated and
# contended are two runs of the SAME dual design, and the delta between them
# is the arbiter's contention cost alone, not the whole dual tax.
#
# Usage: run_contention.sh <dual-sim-binary> <runs> <cycle-limit> <cflags>
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: run_contention.sh <dual-sim-binary> <runs> <cycle-limit> <cflags>" >&2
  exit 1
fi

SIM=$1
RUNS=$2
CYCLE_LIMIT=$3
CFLAGS=$4
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)

if [ ! -x "$SIM" ]; then
  echo "error: '$SIM' is not an executable runner; build it with 'make dual-sim'." >&2
  exit 1
fi

CC=""
for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do
  if command -v "$candidate" >/dev/null 2>&1; then CC=$candidate; break; fi
done
if [ -z "$CC" ]; then
  echo "error: no RISC-V cross compiler found; see 'make setup'." >&2
  exit 1
fi
OBJCOPY=${CC%gcc}objcopy
NM=${CC%gcc}nm

tmp=$(mktemp -d "${TMPDIR:-/tmp}/dual-contention.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

objects=()
for unit in dhry_1 dhry_2 dhry_port; do
  # shellcheck disable=SC2086
  $CC $CFLAGS -DDHRY_RUNS="$RUNS" -DDHRY_FLAGS="\"$CFLAGS\"" \
    -c "$REPO/test/bench/$unit.c" -o "$tmp/$unit.o"
  objects+=("$tmp/$unit.o")
done

elf="$tmp/contend.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$REPO/test/bench/bench.lds" -o "$elf" \
     "$HERE/dhry_contend.S" "$HERE/data_init.S" "${objects[@]}" 2> "$tmp/link.log"; then
  cat "$tmp/link.log" >&2
  echo "*** did not link -- one Dhrystone copy plus the contention loop did" >&2
  echo "*** not fit the SoC's real 8 KB/64 KB budget, which this measurement" >&2
  echo "*** shares with 'make dhrystone' because it is the same linker script." >&2
  exit 1
fi
if [ -s "$tmp/link.log" ]; then
  cat "$tmp/link.log" >&2
  echo "error: the link produced diagnostics; warnings are errors here." >&2
  exit 1
fi

$OBJCOPY -O verilog --verilog-data-width=4 -j .text -j .data "$elf" "$tmp/rom.hex"
$OBJCOPY -O verilog --verilog-data-width=4 -j .tohost "$elf" "$tmp/ram.hex"

console_addr=$($NM "$elf" | awk '$3 == "dhry_console" { print "0x" $1 }')
if [ -z "$console_addr" ]; then
  echo "error: dhry_console is not in the linked image, so hart 0's report" >&2
  echo "cannot be read back out of RAM. Nothing was run." >&2
  exit 1
fi

# The two runs share no state -- same image, same cycle limit, different
# `--hold-hart1` -- so they run as background jobs rather than back to back,
# each writing its own exit status beside its own log.
run_one() {  # $1 = label, $2 = extra flag (may be empty)
  local out="$tmp/$1.log"
  set +e
  # shellcheck disable=SC2086
  "$SIM" --rom "$tmp/rom.hex" --ram "$tmp/ram.hex" --cycles "$CYCLE_LIMIT" \
    --console "$console_addr" $2 > "$out" 2>&1
  echo "$?" > "$tmp/$1.status"
}

run_one contended "" &
run_one isolated "--hold-hart1" &
wait

contended_status=$(cat "$tmp/contended.status")
held_status=$(cat "$tmp/isolated.status")

# Exit 0 (PASS) is the contended run's expected outcome: both harts retire and
# hart 0's own verdict reaches `tohost`. Exit 6 is the held run's -- hart 1
# genuinely retired nothing, which test/dual_smoke.sh already establishes as
# the correct answer for `--hold-hart1` rather than a failure.
if [ "$contended_status" -ne 0 ]; then
  cat "$tmp/contended.log"
  echo "*** the contended run did not reach a PASS verdict (exit $contended_status)." >&2
  exit "$contended_status"
fi

if [ "$held_status" -ne 6 ]; then
  cat "$tmp/isolated.log"
  echo "*** the isolated run exited $held_status, not 6 -- hart 1 was not" >&2
  echo "*** actually silent, so this is not the isolated run it claims to be." >&2
  exit 1
fi

contended_cycles=$(awk '/^Cycles  *:/ { print $3; exit }' "$tmp/contended.log")
isolated_cycles=$(awk '/^Cycles  *:/ { print $3; exit }' "$tmp/isolated.log")
if [ -z "$contended_cycles" ] || [ -z "$isolated_cycles" ]; then
  echo "error: one of the two runs printed no 'Cycles' line -- was dhry_console" >&2
  echo "read at the wrong address, or did the report never finish printing?" >&2
  exit 1
fi

echo "== hart 0's Dhrystone report, contended =="
cat "$tmp/contended.log"
echo
echo "== hart 0's Dhrystone report, hart 1 held in reset =="
cat "$tmp/isolated.log"
echo

python3 - "$contended_cycles" "$isolated_cycles" <<'PY'
import sys
contended, isolated = (int(x) for x in sys.argv[1:3])
delta = contended - isolated
pct = 100.0 * delta / isolated
print("== the contention delta ==")
print(f"contended : {contended} cycles")
print(f"isolated  : {isolated} cycles")
print(f"delta     : {delta:+d} cycles, {pct:+.2f}%")
print()
print("Both numbers are hart 0's own 'Cycles' line -- the mcycle delta around")
print("Dhrystone's measured loop, read the same way 'make dhrystone' reads it --")
print("taken from two runs of ONE image on the DUAL harness. Neither is")
print("comparable to 'make dhrystone's own figure: that runs on littlesoc, which")
print("has no bus arbiter to register a grant through at all.")
PY
