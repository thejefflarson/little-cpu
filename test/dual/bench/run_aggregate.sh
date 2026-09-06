#!/bin/bash
# The aggregate measurement: two independently linked Dhrystone copies, one
# per hart, run concurrently on the DUAL harness. Each copy is the unmodified
# Dhrystone port (test/bench/dhry_1.c, dhry_2.c, dhry_port.c) compiled ONCE and
# then renamed twice with `objcopy --prefix-symbols` -- `main` becomes
# `h0_main`/`h1_main`, `tohost` becomes `h0_tohost`/`h1_tohost`, and so on --
# so the two can share one link with no line of Dhrystone's source touched and
# no symbol collision.
#
# test/dual/bench/dhry_dual.lds is the linker script this needs and
# test/bench/bench.lds is not: two stacks, and no shared `tohost` at `ram`'s
# origin for a runner to poll, because two harts finish at two different times
# under contention and stopping the run at the first one to finish would
# truncate whichever is still going. So this reads BOTH consoles after a fixed
# `--cycles` budget instead of waiting for a verdict.
#
# THE ROM BUDGET IS THE FIRST THING PRINTED, AND CHECKED BY THE LINKER: the
# combined image has to fit rtl/littledualsoc.v's real ROM_WORDS(2048), which
# `test/dual/bench/dhry_dual.lds` states as the SoC's real 8 KB, the same
# ceiling test/bench/bench.lds enforces for one copy. If two copies of
# Dhrystone plus this startup do not fit, the link fails with the overflow in
# bytes and that is the answer to report, not something to trim -- a Dhrystone
# built smaller than Dhrystone would not be Dhrystone, the same reason
# test/bench/run_dhrystone.sh gives for its own 8 KB budget.
#
# Usage: run_aggregate.sh <dual-sim-binary> <runs> <cycle-limit> <cflags>
#
# Set DUAL_DHRY_MHZ to the dual design's own placed clock -- from
# `make dual-ecp5-timing`, ECP5 only, never from 12 MHz -- to add the absolute
# DMIPS column. Left unset, only DMIPS/MHz is printed: the clock belongs to a
# placement, and a copy of it stored in this script would be a number that
# stops tracking the RTL, the same reason soc/compare/run_dhrystone.sh takes
# its own clock as an argument rather than a constant.
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: run_aggregate.sh <dual-sim-binary> <runs> <cycle-limit> <cflags>" >&2
  exit 1
fi

SIM=$1
RUNS=$2
CYCLE_LIMIT=$3
CFLAGS=$4
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)

lds_region_bytes() {
  awk -v region="$1" \
    '{ sub(/^[ \t]+/, "") }
     index($0, region "(") == 1 && match($0, /LENGTH = [0-9]+K/) {
       print substr($0, RSTART + 9, RLENGTH - 10) * 1024; exit
     }' "$HERE/dhry_dual.lds"
}
ROM_BUDGET=$(lds_region_bytes rom)
if [ -z "$ROM_BUDGET" ]; then
  echo "error: could not read the 'rom' region length out of $HERE/dhry_dual.lds." >&2
  exit 1
fi

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

tmp=$(mktemp -d "${TMPDIR:-/tmp}/dual-aggregate.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

# ONE compile, per unit -- the two copies are the same object code under two
# names, not two builds.
for unit in dhry_1 dhry_2 dhry_port; do
  # shellcheck disable=SC2086
  $CC $CFLAGS -DDHRY_RUNS="$RUNS" -DDHRY_FLAGS="\"$CFLAGS\"" \
    -c "$REPO/test/bench/$unit.c" -o "$tmp/$unit.o"
done

link_inputs=("$HERE/dhry_dual_boot.S" "$HERE/data_init.S")
for hart in h0 h1; do
  for unit in dhry_1 dhry_2 dhry_port; do
    renamed="$tmp/${hart}_$unit.o"
    $OBJCOPY --prefix-symbols="${hart}_" "$tmp/$unit.o" "$renamed"
    link_inputs+=("$renamed")
  done
done

elf="$tmp/aggregate.elf"
# shellcheck disable=SC2086
if ! $CC $CFLAGS -nostdlib -T "$HERE/dhry_dual.lds" -o "$elf" \
     "${link_inputs[@]}" 2> "$tmp/link.log"; then
  cat "$tmp/link.log" >&2
  echo "*** the two copies did not link. If the message above is a region" >&2
  echo "*** overflow on 'rom', two Dhrystone copies did not fit the dual" >&2
  echo "*** design's shared $ROM_BUDGET-byte text window. That is the answer" >&2
  echo "*** to report -- decline the aggregate measurement rather than trim" >&2
  echo "*** Dhrystone to fit." >&2
  exit 1
fi
if [ -s "$tmp/link.log" ]; then
  cat "$tmp/link.log" >&2
  echo "error: the link produced diagnostics; warnings are errors here." >&2
  exit 1
fi

rom_bytes=$($OBJCOPY -O binary --only-section=.text "$elf" "$tmp/rom.bin" \
  && wc -c < "$tmp/rom.bin" | tr -d ' ')
echo "== the combined image against the dual design's shared 8 KB text window =="
echo "rom (.text, both copies plus this startup): $rom_bytes of $ROM_BUDGET bytes"
if [ "$rom_bytes" -gt "$ROM_BUDGET" ]; then
  echo "*** over budget by $((rom_bytes - ROM_BUDGET)) bytes." >&2
  exit 1
fi
echo "                                             $((ROM_BUDGET - rom_bytes)) free"
echo

$OBJCOPY -O verilog --verilog-data-width=4 --only-section=.text "$elf" "$tmp/rom.hex"
$OBJCOPY -O verilog --verilog-data-width=4 -j .guard -j .tohost -j .data "$elf" "$tmp/ram.hex"

console_addrs=$($NM "$elf" | awk '
  $3 == "h0_dhry_console" { h0 = "0x" $1 }
  $3 == "h1_dhry_console" { h1 = "0x" $1 }
  END { print h0, h1 }')
read -r h0_addr h1_addr <<< "$console_addrs"
if [ -z "$h0_addr" ] || [ -z "$h1_addr" ]; then
  echo "error: one of the two renamed dhry_console symbols is missing from the" >&2
  echo "linked image, so that hart's report cannot be read back. Nothing was run." >&2
  exit 1
fi

set +e
"$SIM" --rom "$tmp/rom.hex" --ram "$tmp/ram.hex" --cycles "$CYCLE_LIMIT" \
  --console "$h0_addr" --console "$h1_addr" > "$tmp/run.log" 2>&1
sim_status=$?
set -e

# Exit 2 -- the cycle limit -- is THIS BUILD'S EXPECTED OUTCOME: there is no
# shared verdict to end the run early, by test/dual/bench/dhry_dual.lds'
# design (see its header), so both consoles are read out after a fixed budget
# rather than after a PASS. Anything else means a hart trapped, the per-retire
# monitor fired, or the two harts collided on the bus -- a real failure this
# script does not paper over.
if [ "$sim_status" -ne 2 ]; then
  cat "$tmp/run.log"
  echo "*** exited $sim_status, not 2 (the cycle limit). This build has no" >&2
  echo "*** shared verdict to reach, so anything but the cycle limit is a" >&2
  echo "*** real failure -- raise the cycle limit only if both 'RETIRES' lines" >&2
  echo "*** below are small relative to two full Dhrystone runs." >&2
  exit "$sim_status"
fi

grep '^HART[01] RETIRES' "$tmp/run.log"
echo

# The two consoles print back to back with nothing between them, in the order
# `--console` named them, so the second "Dhrystone Benchmark" header is where
# hart 0's report ends and hart 1's begins.
awk '/^Dhrystone Benchmark/ { n++ } { print > ("'"$tmp"'/report" n) }' "$tmp/run.log"
echo "== hart 0's Dhrystone report =="
cat "$tmp/report1"
echo
echo "== hart 1's Dhrystone report =="
cat "$tmp/report2"

h0_mhz=$(awk '/^DMIPS\/MHz/ { print $3; exit }' "$tmp/report1")
h1_mhz=$(awk '/^DMIPS\/MHz/ { print $3; exit }' "$tmp/report2")
if [ -z "$h0_mhz" ] || [ -z "$h1_mhz" ]; then
  echo "error: could not read two 'DMIPS/MHz' lines out of the run -- was one" >&2
  echo "console truncated before its report finished printing?" >&2
  exit 1
fi

python3 - "$h0_mhz" "$h1_mhz" "${DUAL_DHRY_MHZ:-}" <<'PY'
import sys
h0, h1 = (float(x) for x in sys.argv[1:3])
mhz = sys.argv[3]
combined = h0 + h1
print("== the aggregate ==")
print(f"hart 0 DMIPS/MHz : {h0:.3f}")
print(f"hart 1 DMIPS/MHz : {h1:.3f}")
print(f"combined         : {combined:.3f} DMIPS/MHz")
if mhz:
    f = float(mhz)
    print()
    print(f"at {f:g} MHz ({combined * f:.2f} DMIPS aggregate, "
          f"{h0 * f:.2f} + {h1 * f:.2f} per hart)")
else:
    print()
    print("No absolute figure: set DUAL_DHRY_MHZ to a placed clock from "
          "'make dual-ecp5-timing' (this configuration is ECP5-only) to add one.")
print()
print("This is a simulated concurrent run, not a placement: DMIPS/MHz is a")
print("property of the RTL and the two programs, and the megahertz above is")
print("whichever placement it is quoted with -- name the tree and the seed.")
PY
